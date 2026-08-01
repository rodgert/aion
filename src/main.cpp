// SPDX-License-Identifier: GPL-2.0-or-later

#include <nomos/rt/common_args.hpp>
#include <nomos/rt/event_scheduler.hpp>
#include <nomos/rt/heartbeat.hpp>
#include <nomos/rt/input_event.hpp>
#include <nomos/rt/modulator_engine.hpp>
#include <nomos/rt/rt_control_thread.hpp>
#include <nomos/rt/signal_handlers.hpp>
#include <nomos/rt/spsc_queue.hpp>

#include "aion_control_thread.hpp"
#include "audio_device.hpp"
#include "link_peer.hpp"
#include "midi_io.hpp"
#include "osc_server.hpp"
#include "routing_matrix.hpp"

#include <clap/events.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace aion {
std::string_view version() noexcept;
}

namespace {
std::atomic<bool> g_running{true};
void              on_signal(int) noexcept {
    g_running.store(false, std::memory_order_relaxed);
}
} // namespace

int main(int argc, char* argv[]) {
    constexpr double sample_rate = 48000.0;

    nomos::rt::common_args args;
    args.socket_path = "/tmp/aion.sock";
    args.db_path     = "aion.db";
    args.osc_port    = 9002;
    args.block_size  = 64;

    // aion has no process-specific flags beyond the common set.
    const auto rem = nomos::rt::parse_common_args(argc, argv, args);

    if (args.version) {
        std::cout << "aion v" << aion::version() << "\n";
        return EXIT_SUCCESS;
    }

    // --help handled after parse so defaults printed are the resolved values.
    for (std::size_t i = 0; i < rem.size(); ++i) {
        if (rem[i] == "--help") {
            std::cout << "Usage: aion [options]\n";
            nomos::rt::print_common_args_help(args, std::cout);
            return EXIT_SUCCESS;
        }
        std::cerr << "[aion] unknown argument: " << rem[i] << "\n";
    }

    if (args.list_audio_devs) {
        nomos::rt::audio_device::list_devices();
        return EXIT_SUCCESS;
    }

    nomos::rt::install_signal_handlers(on_signal);
    std::thread heartbeat_thread = nomos::rt::start_heartbeat_thread(args.heartbeat_ms, g_running);

    // Shared queues.
    nomos::rt::param_queue       param_queue;
    nomos::rt::input_event_queue ipc_in_queue;
    nomos::rt::input_event_queue hw_midi_in_queue;
    nomos::rt::input_event_queue osc_in_queue;

    nomos::rt::event_scheduler  scheduler;
    nomos::rt::modulator_engine mod_engine;
    aion::RoutingMatrix         routing;

    nomos::rt::midi_io midi;
    nomos::rt::midi_io::list_ports();
    if (args.midi_port >= 0)
        midi.open_port(static_cast<unsigned int>(args.midi_port));
    else if (!args.midi_port_name.empty())
        midi.open_port_by_name(args.midi_port_name);
    if (args.midi_in_port >= 0)
        midi.open_input_port(static_cast<unsigned int>(args.midi_in_port), hw_midi_in_queue);
    else if (!args.midi_in_port_name.empty())
        midi.open_input_port_by_name(args.midi_in_port_name, hw_midi_in_queue);

    // OSC endpoint — constructed before the control thread so it can receive
    // outbound msg_osc frames (external OSC send, mirroring .midi).
    nomos::rt::osc_server osc{args.osc_port, osc_in_queue};
    osc.start();

    aion::aion_control_thread ctrl{nomos::rt::rt_control_thread::config{
                                       .socket_path   = args.socket_path,
                                       .db_path       = args.db_path,
                                       .sched_staging = &scheduler.staging(),
                                       .mod_engine    = &mod_engine,
                                       .midi          = &midi,
                                       .osc           = &osc,
                                   },
                                   param_queue, ipc_in_queue, routing};
    ctrl.start();

    // Diagnostic tap: MSG-MIDI-DIAG frame pushed to nous after every MIDI send.
    midi.set_send_callback([&](const std::vector<uint8_t>& bytes) {
        std::string payload = "{:bytes [";
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            if (i > 0)
                payload += ' ';
            payload += std::to_string(static_cast<unsigned>(bytes[i]));
        }
        payload += "]}";
        ctrl.push_frame(nomos::rt::ipc::msg_midi_diag, payload);
    });

    nomos::rt::link_peer link{args.bpm};
    link.enable(true);

    nomos::rt::audio_device audio_dev;
    if (!args.no_audio) {
        nomos::rt::audio_device_config audio_cfg{
            .device_id     = args.audio_device,
            .out_channels  = args.audio_out_ch,
            .in_channels   = args.audio_in_ch,
            .sample_rate   = sample_rate,
            .buffer_frames = args.block_size,
        };
        const bool opened =
            audio_dev.open(audio_cfg, [](float** out, const float* const*, uint32_t out_ch,
                                         uint32_t, uint32_t nframes, double) {
                for (uint32_t c = 0; c < out_ch; ++c)
                    if (out && out[c])
                        std::fill_n(out[c], nframes, 0.0f);
            });
        if (opened) {
            if (!audio_dev.start())
                std::cerr << "[aion] audio device failed to start; no hardware clock discipline\n";
        } else {
            std::cerr << "[aion] audio device failed to open; no hardware clock discipline\n";
        }
    }

    std::thread event_thread{[&]() {
        nomos::rt::block_signals_on_this_thread();
        int64_t last_tick_n = -1;
        mod_engine.pre_warm_reader();

        while (g_running.load(std::memory_order_relaxed)) {
            bool did_work = false;

            const double  beat   = link.beat_at_time(link.now());
            const int64_t tick_n = static_cast<int64_t>(std::floor(beat * 24.0));
            const float   tick_rate_hz =
                static_cast<float>(sample_rate) / static_cast<float>(args.block_size);

            std::string mods_edn;
            mod_engine.tick(beat, tick_rate_hz,
                            [&](const std::string& id, const nomos::rt::modulator_output& out) {
                                routing.route_modulator(id, out, midi);
                                mods_edn += " :" + id + " {:cv " + std::to_string(out.cv) +
                                            " :aux " + std::to_string(out.aux) + " :gate " +
                                            (out.gate ? "true" : "false") + " :gate2 " +
                                            (out.gate2 ? "true" : "false") + "}";
                            });

            if (tick_n > last_tick_n) {
                last_tick_n = tick_n;
                std::string frame =
                    "{:beat " + std::to_string(beat) + " :tick-n " + std::to_string(tick_n);
                if (!mods_edn.empty())
                    frame += " :mods {" + mods_edn + "}";
                frame += "}";
                ctrl.push_frame(nomos::rt::ipc::msg_tick, frame);
                midi.realtime(0xF8);
                did_work = true;
            }

            scheduler.tick(beat, [&](const nomos::rt::clap_event_union& ev) {
                ipc_in_queue.push(ev);
                did_work = true;
            });

            while (auto ev = ipc_in_queue.pop()) {
                routing.route_event(*ev, aion::MidiRoute::Source::ipc, midi);
                did_work = true;
            }
            while (auto ev = hw_midi_in_queue.pop()) {
                routing.route_event(*ev, aion::MidiRoute::Source::midi_hw, midi);
                did_work = true;
            }
            while (auto ev = osc_in_queue.pop()) {
                routing.route_event(*ev, aion::MidiRoute::Source::osc, midi);
                did_work = true;
            }
            while (param_queue.pop())
                did_work = true;

            if (!did_work)
                std::this_thread::sleep_for(std::chrono::microseconds(250));
        }
    }};

    std::cerr << "[aion]  v" << aion::version() << "\n";
    std::cerr << "[aion]  socket=" << args.socket_path << " db=" << args.db_path << "\n";
    std::cerr << "[link]  enabled, bpm=" << args.bpm << "\n";
    std::cerr << "[osc]   listening on port " << args.osc_port << "\n";
    if (audio_dev.is_running())
        std::cerr << "[audio] hardware clock discipline active"
                  << " sample_rate=" << sample_rate << " block_size=" << args.block_size << "\n";
    else
        std::cerr << "[audio] no hardware clock discipline (software timer only)\n";

    while (g_running.load(std::memory_order_relaxed))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::cerr << "[aion] shutting down\n";
    audio_dev.stop();
    audio_dev.close();
    osc.stop();
    ctrl.stop();
    event_thread.join();
    if (heartbeat_thread.joinable())
        heartbeat_thread.join();
    midi.close();
    link.enable(false);

    return EXIT_SUCCESS;
}
