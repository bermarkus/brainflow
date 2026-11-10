#include <algorithm>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <vector>

#include "custom_cast.h"
#include "freeeeg.h"
#include "serial.h"
#include "timestamp.h"


constexpr int FreeEEG::start_byte;
constexpr int FreeEEG::end_byte;
constexpr int FreeEEG::classic_size;
constexpr int FreeEEG::extended_ppg_size;
constexpr int FreeEEG::extended_size;
constexpr double FreeEEG::accel_scale;
constexpr double FreeEEG::gyro_scale;
constexpr double FreeEEG::ads_gain;
constexpr double FreeEEG::ads_vref;


FreeEEG::FreeEEG (int board_id, struct BrainFlowInputParams params) : Board (board_id, params)
{
    serial = NULL;
    is_streaming = false;
    keep_alive = false;
    initialized = false;
    exact_frame_length = false;
    if (board_id == (int)BoardIds::FREEEEG32_BOARD)
    {
        min_package_size = 1 + 32 * 3;
        exact_frame_length = true;
    }
    if (board_id == (int)BoardIds::FREEEEG128_BOARD)
    {
        min_package_size = 1 + 128 * 3;
    }
}

FreeEEG::~FreeEEG ()
{
    skip_logs = true;
    release_session ();
}

int FreeEEG::prepare_session ()
{
    if (initialized)
    {
        safe_logger (spdlog::level::info, "Session already prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }
    if (params.serial_port.empty ())
    {
        safe_logger (spdlog::level::err, "serial port is empty");
        return (int)BrainFlowExitCodes::INVALID_ARGUMENTS_ERROR;
    }
    serial = Serial::create (params.serial_port.c_str (), this);
    int port_open = open_port ();
    if (port_open != (int)BrainFlowExitCodes::STATUS_OK)
    {
        delete serial;
        serial = NULL;
        return port_open;
    }

    int set_settings = set_port_settings ();
    if (set_settings != (int)BrainFlowExitCodes::STATUS_OK)
    {
        delete serial;
        serial = NULL;
        return set_settings;
    }

    initialized = true;
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int FreeEEG::start_stream (int buffer_size, const char *streamer_params)
{
    if (is_streaming)
    {
        safe_logger (spdlog::level::err, "Streaming thread already running");
        return (int)BrainFlowExitCodes::STREAM_ALREADY_RUN_ERROR;
    }
    int res = prepare_for_acquisition (buffer_size, streamer_params);
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    serial->flush_buffer ();

    keep_alive = true;
    streaming_thread = std::thread ([this] { this->read_thread (); });
    is_streaming = true;
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int FreeEEG::stop_stream ()
{
    if (is_streaming)
    {
        keep_alive = false;
        is_streaming = false;
        if (streaming_thread.joinable ())
        {
            streaming_thread.join ();
        }
        return (int)BrainFlowExitCodes::STATUS_OK;
    }
    else
    {
        return (int)BrainFlowExitCodes::STREAM_THREAD_IS_NOT_RUNNING;
    }
}

int FreeEEG::release_session ()
{
    if (initialized)
    {
        if (is_streaming)
        {
            stop_stream ();
        }
        free_packages ();
        initialized = false;
    }
    if (serial)
    {
        serial->close_serial_port ();
        delete serial;
        serial = NULL;
    }
    return (int)BrainFlowExitCodes::STATUS_OK;
}

void FreeEEG::read_thread ()
{
    int res;
    constexpr int max_size = 1000; // random value bigger than package size which is unknown
    unsigned char b[max_size] = {0};
    float eeg_scale = FreeEEG::ads_vref / float ((pow (2, 23) - 1)) / FreeEEG::ads_gain * 1000000.;
    int num_rows = board_descr["default"]["num_rows"];
    double *package = new double[num_rows];
    for (int i = 0; i < num_rows; i++)
    {
        package[i] = 0.0;
    }
    bool first_package_received = false;
    int learned_size = 0; // FreeEEG32: size of the last frame, for firmware with other sizes
    int extended_seen = 0; // FreeEEG32: the extended size this firmware sends, once seen
    int resyncs = 0;

    std::vector<int> eeg_channels = board_descr["default"]["eeg_channels"];
    std::vector<int> ppg_channels;
    std::vector<int> other_channels;
    std::vector<int> accel_channels;
    std::vector<int> gyro_channels;
    if (exact_frame_length)
    {
        ppg_channels = board_descr["default"]["ppg_channels"].get<std::vector<int>> ();
        other_channels = board_descr["default"]["other_channels"].get<std::vector<int>> ();
        accel_channels = board_descr["default"]["accel_channels"].get<std::vector<int>> ();
        gyro_channels = board_descr["default"]["gyro_channels"].get<std::vector<int>> ();
    }

    while (keep_alive)
    {
        int pos = 0;
        bool complete_package = false;
        while ((keep_alive) && (pos < max_size - 2))
        {
            res = serial->read_from_serial_port (b + pos, 1);
            if (res < 1)
            {
                continue; // timeout or error: no new byte at b[pos]
            }
            int prev_id = (pos <= 0) ? 0 : pos - 1;
            if ((b[pos] == FreeEEG::start_byte) && (b[prev_id] == FreeEEG::end_byte) &&
                is_frame_end (pos, learned_size, extended_seen, first_package_received))
            {
                complete_package = true;
                break;
            }
            pos += res;
            // FreeEEG32: no frame ended where one had to, so the frame boundaries are lost;
            // start over at the next 0xC0 0xA0 pair
            if (exact_frame_length && first_package_received &&
                pos > std::max (FreeEEG::extended_size, learned_size))
            {
                first_package_received = false;
                extended_seen = 0;
                resyncs++;
                safe_logger (spdlog::level::warn, "FreeEEG32 frame sync lost, resync {}", resyncs);
            }
        }
        if (complete_package)
        {
            // handle the case that we start reading in the middle of data stream
            if (!first_package_received)
            {
                first_package_received = true;
                continue;
            }
            learned_size = pos;
            if ((pos == FreeEEG::extended_ppg_size) || (pos == FreeEEG::extended_size))
            {
                extended_seen = pos;
            }
            package[board_descr["default"]["package_num_channel"].get<int> ()] = (double)b[0];
            for (unsigned int i = 0; i < eeg_channels.size (); i++)
            {
                package[eeg_channels[i]] = (double)eeg_scale * cast_24bit_to_int32 (b + 1 + 3 * i);
            }
            if (exact_frame_length)
            {
                // Extended frames, offsets counted after the header: 97-102 board ID (48 bit),
                // 103-111 latest PPG red/IR/green (24 bit), 112 PPG counter, 113 sync pulse
                // counter, 114-115 ping ID, 116-117 pulse offset in us after DRDY, 118 flags
                // (bit 5 motion stale). All big-endian, unsigned. 134-byte frames add 119-124
                // accel X/Y/Z, 125-130 gyro X/Y/Z (signed 16 bit, big-endian), 131 motion
                // counter. Classic frames carry none of it: rows stay 0, as do the motion rows
                // for 121-byte frames.
                bool extended =
                    (pos == FreeEEG::extended_ppg_size) || (pos == FreeEEG::extended_size);
                bool motion = (pos == FreeEEG::extended_size);
                for (int i = 0; i < 3; i++)
                {
                    package[accel_channels[i]] = motion
                        ? FreeEEG::accel_scale * (int16_t)((b[119 + 2 * i] << 8) | b[120 + 2 * i])
                        : 0.0;
                    package[gyro_channels[i]] = motion
                        ? FreeEEG::gyro_scale * (int16_t)((b[125 + 2 * i] << 8) | b[126 + 2 * i])
                        : 0.0;
                }
                package[other_channels[6]] = motion ? (double)b[131] : 0.0;
                for (int i = 0; i < 3; i++)
                {
                    const unsigned char *v = b + 103 + 3 * i;
                    package[ppg_channels[i]] =
                        extended ? (double)((v[0] << 16) | (v[1] << 8) | v[2]) : 0.0;
                }
                double board_id = 0.0;
                for (int i = 0; i < 6; i++)
                {
                    board_id = board_id * 256.0 + b[97 + i];
                }
                package[other_channels[0]] = extended ? (double)b[112] : 0.0;
                package[other_channels[1]] = extended ? (double)b[113] : 0.0;
                package[other_channels[2]] = extended ? (double)((b[114] << 8) | b[115]) : 0.0;
                package[other_channels[3]] = extended ? (double)((b[116] << 8) | b[117]) : 0.0;
                package[other_channels[4]] = extended ? (double)b[118] : 0.0;
                package[other_channels[5]] = extended ? board_id : 0.0;
            }
            package[board_descr["default"]["timestamp_channel"].get<int> ()] = get_timestamp ();
            push_package (package);
        }
        else
        {
            safe_logger (
                spdlog::level::trace, "stopped with pos: {}, keep_alive: {}", pos, keep_alive);
        }
    }
    delete[] package;
}

// Where a frame may end, given that b[pos] is 0xA0 and b[pos - 1] is 0xC0.
bool FreeEEG::is_frame_end (int pos, int learned_size, int extended_seen, bool synced) const
{
    if (!exact_frame_length)
    {
        return pos >= min_package_size;
    }
    // Before the first frame boundary any pair will do; a wrong one is caught when the next
    // frame doesn't end at a known size.
    if (!synced)
    {
        return pos >= 1;
    }
    // Firmware sends classic frames and one extended size (121 bytes before the motion sensor,
    // 134 since). Once one extended size was seen only that one counts: in a 134-byte frame
    // accel X = 0xC0A0 puts a pair where a 121-byte frame ends.
    if (pos == FreeEEG::classic_size)
    {
        return true;
    }
    if ((pos == FreeEEG::extended_ppg_size) || (pos == FreeEEG::extended_size))
    {
        return (extended_seen == 0) || (pos == extended_seen);
    }
    if ((learned_size > 0) && (pos == learned_size))
    {
        return true;
    }
    // firmware with another frame size: take the first frame after sync, then hold its size
    return (learned_size == 0) && (pos >= min_package_size);
}

int FreeEEG::open_port ()
{
    if (serial->is_port_open ())
    {
        safe_logger (spdlog::level::err, "port {} already open", serial->get_port_name ());
        return (int)BrainFlowExitCodes::PORT_ALREADY_OPEN_ERROR;
    }

    safe_logger (spdlog::level::info, "openning port {}", serial->get_port_name ());
    int res = serial->open_serial_port ();
    if (res < 0)
    {
        return (int)BrainFlowExitCodes::UNABLE_TO_OPEN_PORT_ERROR;
    }
    safe_logger (spdlog::level::trace, "port {} is open", serial->get_port_name ());
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int FreeEEG::set_port_settings ()
{
    int res = serial->set_serial_port_settings (1000, false);
    if (res < 0)
    {
        safe_logger (spdlog::level::err, "Unable to set port settings, res is {}", res);
#ifndef _WIN32
        return (int)BrainFlowExitCodes::SET_PORT_ERROR;
#endif
    }
    res = serial->set_custom_baudrate (921600);
    if (res < 0)
    {
        safe_logger (spdlog::level::err, "Unable to set custom baud rate, res is {}", res);
#ifndef _WIN32
        // Setting the baudrate may return an error on Windows for some serial drivers.
        // We do not throw an exception, because it will still work with USB.
        // Optical connection will fail, though.
        return (int)BrainFlowExitCodes::SET_PORT_ERROR;
#endif
    }
    safe_logger (spdlog::level::trace, "set port settings");
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int FreeEEG::config_board (std::string config, std::string &response)
{
    safe_logger (spdlog::level::err, "FreeEEG doesn't support board configuration.");
    return (int)BrainFlowExitCodes::UNSUPPORTED_BOARD_ERROR;
}
