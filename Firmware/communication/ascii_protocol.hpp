#ifndef __ASCII_PROTOCOL_HPP
#define __ASCII_PROTOCOL_HPP

#include <fibre/async_stream.hpp>
#include <fibre/../../stream_utils.hpp>

#define MAX_LINE_LENGTH ((size_t)256)

class AsciiProtocol {
public:
    AsciiProtocol(fibre::AsyncStreamSource* rx_channel, fibre::AsyncStreamSink* tx_channel)
        : rx_channel_(rx_channel), sink_(*tx_channel) {}

    void start();

private:
    // Single axis
    void cmd_set_ref_get_feedback(char * pStr, uint8_t axis, uint8_t mode);

    void cmd_set_position(char * pStr, bool use_checksum);
    void cmd_set_position_wl(char * pStr, bool use_checksum);
    void cmd_set_velocity(char * pStr, bool use_checksum);
    void cmd_set_torque(char * pStr, bool use_checksum);
    void cmd_set_trapezoid_trajectory(char * pStr, bool use_checksum);
    void cmd_get_feedback(char * pStr, bool use_checksum);
    void cmd_help(char * pStr, bool use_checksum);
    void cmd_info_dump(char * pStr, bool use_checksum);
    void cmd_system_ctrl(char * pStr, bool use_checksum);
    void cmd_read_property(char * pStr, bool use_checksum);
    void cmd_write_property(char * pStr, bool use_checksum);
    void cmd_update_axis_wdg(char * pStr, bool use_checksum);
    void cmd_unknown(char * pStr, bool use_checksum);
    void cmd_encoder(char * pStr, bool use_checksum);

    // DUAL-AXIS CONTROL LOOP FUNCTION
    // mode = 0 : torque
    // mode = 1 : velocity
    // mode = 2 : position
    void control_loop(char * pStr, uint8_t mode);

    template<typename ... TArgs> void respond(bool include_checksum, const char * fmt, TArgs&& ... args);
    void respond_byte(bool include_checksum, uint8_t* tx_data, size_t count);
    void process_line(fibre::cbufptr_t buffer);
    void on_write_finished(fibre::WriteResult result);
    void on_read_finished(fibre::ReadResult result);

    fibre::AsyncStreamSource* rx_channel_ = nullptr;
    uint8_t* rx_end_ = nullptr; // non-zero if an RX operation has finished but wasn't handled yet because the TX channel was busy

    uint8_t rx_buf_[MAX_LINE_LENGTH];
    bool read_active_ = true;

    fibre::BufferedStreamSink<512> sink_;

    uint8_t tx_buf_[512]; // Byte array response buffer 

};


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum ascii85_errs_e
{
    ascii85_err_out_buf_too_small = -255,
    ascii85_err_in_buf_too_large,
    ascii85_err_bad_decode_char,
    ascii85_err_decode_overflow
};

int32_t encode_ascii85 (const uint8_t *inp, int32_t in_length, uint8_t *outp, int32_t out_max_length);

int32_t decode_ascii85 (const uint8_t *inp, int32_t in_length, uint8_t *outp, int32_t out_max_length);

int32_t ascii85_get_max_encoded_length (int32_t in_length);

int32_t ascii85_get_max_decoded_length (int32_t in_length);


#ifdef __cplusplus
}
#endif


#endif // __ASCII_PROTOCOL_HPP
