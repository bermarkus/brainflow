#pragma once

#include <thread>

#include "board.h"
#include "board_controller.h"
#include "serial.h"


class FreeEEG : public Board
{

protected:
    volatile bool keep_alive;
    bool initialized;
    bool is_streaming;
    std::thread streaming_thread;
    Serial *serial;
    // dont know exact package size and it can be changed with new firmware versions, its >=
    // min_package_size and we can check start\stop bytes, and it depends on exact board
    int min_package_size;
    // FreeEEG32 only: a frame ends only at a known length (see read_thread), because the
    // 0xC0 0xA0 pair can also occur inside the aux bytes of extended frames
    bool exact_frame_length;

    int open_port ();
    int set_port_settings ();
    void read_thread ();
    bool is_frame_end (int pos, int learned_size, bool synced) const;

public:
    FreeEEG (int board_id, struct BrainFlowInputParams params);
    ~FreeEEG ();

    int prepare_session ();
    int start_stream (int buffer_size, const char *streamer_params);
    int stop_stream ();
    int release_session ();
    int config_board (std::string config, std::string &response);

    static constexpr int start_byte = 0xA0;
    static constexpr int end_byte = 0xC0;
    // FreeEEG32 frame sizes counted after the 0xA0 header: sequence number, 32 x 24-bit EEG,
    // 6 aux bytes, 0xC0 (105-byte frames), and extended frames with 16 more aux bytes before
    // the 0xC0: PPG red/IR/green, PPG counter, sync pulse counter, ping ID, pulse offset, flags
    static constexpr int classic_size = 1 + 32 * 3 + 6 + 1;
    static constexpr int extended_size = classic_size + 16;
    static constexpr double ads_gain = 8.0;
    static constexpr double ads_vref = 2.5;
};
