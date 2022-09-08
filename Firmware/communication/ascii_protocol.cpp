/*
* The ASCII protocol is a simpler, human readable alternative to the main native
* protocol.
* In the future this protocol might be extended to support selected GCode commands.
* For a list of supported commands see doc/ascii-protocol.md
*/

/* Includes ------------------------------------------------------------------*/

#include "odrive_main.h"
#include "communication.h"
#include "ascii_protocol.hpp"
#include <utils.hpp>
#include <fibre/cpp_utils.hpp>

#include "autogen/type_info.hpp"
#include "communication/interface_can.hpp"

using namespace fibre;

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
const size_t float_enc_size = ascii85_get_max_encoded_length(sizeof(float32_t));
const size_t float_dec_size = ascii85_get_max_decoded_length(float_enc_size);

/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)

/* Private variables ---------------------------------------------------------*/

#if HW_VERSION_MAJOR == 3
static Introspectable root_obj = ODrive3TypeInfo<ODrive>::make_introspectable(odrv);
#elif HW_VERSION_MAJOR == 4
static Introspectable root_obj = ODrive4TypeInfo<ODrive>::make_introspectable(odrv);
#endif

/* Private function prototypes -----------------------------------------------*/

/* Function implementations --------------------------------------------------*/

// @brief Sends a line on the specified output.
template<typename ... TArgs>
void AsciiProtocol::respond(bool include_checksum, const char * fmt, TArgs&& ... args) {
    char tx_buf[64];

    size_t len = snprintf(tx_buf, sizeof(tx_buf), fmt, std::forward<TArgs>(args)...);

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= tx_buf[i];
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "*%u\r\n", checksum);
    } else {
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "\r\n");
    }

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    sink_.write({(const uint8_t*)tx_buf, len});
    sink_.maybe_start_async_write();
}


// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void AsciiProtocol::process_line(cbufptr_t buffer) {

    static_assert(sizeof(char) == sizeof(uint8_t));
    
    // scan line to find beginning of checksum and prune comment
    // *** SKIP THIS as it is not used
    /*
    uint8_t checksum = 0;
    size_t checksum_start = SIZE_MAX;
    for (size_t i = 0; i < buffer.size(); ++i) {
        if (buffer.begin()[i] == ';') { // ';' is the comment start char
            buffer = buffer.take(i);
            break;
        }
        if (checksum_start > i) {
            if (buffer[i] == '*') {
                checksum_start = i + 1;
            } else {
                checksum ^= buffer[i];
            }
        }
    }
    */

    // copy everything into a local buffer so we can insert null-termination
    char cmd[MAX_LINE_LENGTH + 1];
    size_t len = std::min(buffer.size(), MAX_LINE_LENGTH);
    memcpy(cmd, buffer.begin(), len);
    cmd[len] = 0; // null-terminate

    // optional checksum validation
    // *** SKIP THIS as it is not used
    /*
    bool use_checksum = (checksum_start < len);
    if (use_checksum) {
        unsigned int received_checksum;
        int numscan = sscanf(&cmd[checksum_start], "%u", &received_checksum);
        if ((numscan < 1) || (received_checksum != checksum))
            return;
        len = checksum_start - 1; // prune checksum and asterisk
        cmd[len] = 0; // null-terminate
    }
    */
   const bool use_checksum = false;

    // check incoming packet type
    switch(cmd[0]) {
        case 'a': cmd_set_torque_get_feedback_0(cmd);                 break;  // custom: torque command + feedback (pos vel Iq)
        case 'b': cmd_set_torque_get_feedback_1(cmd);                 break;

        case 'p': cmd_set_position(cmd, use_checksum);                break;  // position control
        case 'q': cmd_set_position_wl(cmd, use_checksum);             break;  // position control with limits
        case 'v': cmd_set_velocity(cmd, use_checksum);                break;  // velocity control
        case 'c': cmd_set_torque(cmd, use_checksum);                  break;  // current control
        case 't': cmd_set_trapezoid_trajectory(cmd, use_checksum);    break;  // trapezoidal trajectory
        case 'f': cmd_get_feedback(cmd, use_checksum);                break;  // feedback
        case 'h': cmd_help(cmd, use_checksum);                        break;  // Help
        case 'i': cmd_info_dump(cmd, use_checksum);                   break;  // Dump device info
        case 's': cmd_system_ctrl(cmd, use_checksum);                 break;  // System
        case 'r': cmd_read_property(cmd,  use_checksum);              break;  // read property
        case 'w': cmd_write_property(cmd, use_checksum);              break;  // write property
        case 'u': cmd_update_axis_wdg(cmd, use_checksum);             break;  // Update axis watchdog. 
        case 'e': cmd_encoder(cmd, use_checksum);                     break;  // Encoder commands
        default : cmd_unknown(nullptr, use_checksum);                 break;
    }
}

void AsciiProtocol::cmd_set_torque_get_feedback_0(char* pStr) {

    if( pStr[0] == 'a' ) {

        odrv.n_evt_ascii_++;    // counter for debugging

        /* Get float setpoint */
        float32_t torque_setpoint = 0.0;
        decode_ascii85((uint8_t*) pStr+1, float_enc_size, (uint8_t*) &torque_setpoint, float_dec_size);
        Axis& axis = axes[0];

        /* Set torque */
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();

        /* Pack feedback data */
        uint32_t n_cb = odrv.n_evt_control_loop_;
        float32_t data[4];
        data[0] = (float32_t) axis.encoder_.pos_estimate_.any().value_or(0.0f);
        data[1] = (float32_t) axis.encoder_.vel_estimate_.any().value_or(0.0f);
        data[2] = (float32_t) axis.motor_.current_control_.Iq_measured_;
        data[3] = (float32_t) axis.torque_sensor_.torque_estimate_.any().value_or(0.0f);

        /* Encode feedback data */
        size_t enc_size = 0;
        enc_size += encode_ascii85((const uint8_t*) &n_cb, sizeof(uint32_t), &tx_buf_[0], 512);
        if( enc_size < 0 ) respond(false, "0 invalid uint32_t encoding");
        enc_size += encode_ascii85((const uint8_t*) data, 4*sizeof(float), &tx_buf_[enc_size], 512);
        if( enc_size < 0 ) respond(false, "0 invalid float encoding");

        tx_buf_[enc_size] = 0x0A; // terminator ('\n')

        /* Write over USB */
        sink_.write({(const uint8_t*) tx_buf_, 1+enc_size});    // should always be 26 bytes
        sink_.maybe_start_async_write();

    }
    else {
        respond(false, "0 invalid command ");
    }

}

void AsciiProtocol::cmd_set_torque_get_feedback_1(char* pStr) {

   if( pStr[0] == 'b' ) {

        odrv.n_evt_ascii_++;    // counter for debugging

        /* Get float setpoint */
        float32_t torque_setpoint = 0.0;
        decode_ascii85((uint8_t*) pStr+1, float_enc_size, (uint8_t*) &torque_setpoint, float_dec_size);
        Axis& axis = axes[1];

        /* Set torque */
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();

        /* Pack feedback data */
        uint32_t n_cb = odrv.n_evt_control_loop_;
        float32_t data[4];
        data[0] = (float32_t) axis.encoder_.pos_estimate_.any().value_or(0.0f);
        data[1] = (float32_t) axis.encoder_.vel_estimate_.any().value_or(0.0f);
        data[2] = (float32_t) axis.motor_.current_control_.Iq_measured_;
        data[3] = (float32_t) axis.torque_sensor_.torque_estimate_.any().value_or(0.0f);

        /* Encode feedback data */
        size_t enc_size = 0;
        enc_size += encode_ascii85((const uint8_t*) &n_cb, sizeof(uint32_t), &tx_buf_[0], 512);
        if( enc_size < 0 ) respond(false, "0 invalid uint32_t encoding");
        enc_size += encode_ascii85((const uint8_t*) data, 4*sizeof(float), &tx_buf_[enc_size], 512);
        if( enc_size < 0 ) respond(false, "0 invalid float encoding");

        tx_buf_[enc_size] = 0x0A; // terminator ('\n')

        /* Write over USB */
        sink_.write({(const uint8_t*) tx_buf_, 1+enc_size});    // should always be 26 bytes
        sink_.maybe_start_async_write();

    }
    else {
        respond(false, "1 invalid command ");
    }

}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_position(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_feed_forward, torque_feed_forward;

    int numscan = sscanf(pStr, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &torque_feed_forward);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = pos_setpoint;
        if (numscan >= 3)
            axis.controller_.input_vel_ = vel_feed_forward;
        if (numscan >= 4)
            axis.controller_.input_torque_ = torque_feed_forward;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the set position with current and velocity limit command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_position_wl(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_limit, torque_lim;

    int numscan = sscanf(pStr, "q %u %f %f %f", &motor_number, &pos_setpoint, &vel_limit, &torque_lim);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = pos_setpoint;
        if (numscan >= 3)
            axis.controller_.config_.vel_limit = vel_limit;
        if (numscan >= 4)
            axis.motor_.config_.torque_lim = torque_lim;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the set velocity command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_velocity(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float vel_setpoint, torque_feed_forward;
    int numscan = sscanf(pStr, "v %u %f %f", &motor_number, &vel_setpoint, &torque_feed_forward);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
        axis.controller_.input_vel_ = vel_setpoint;
        if (numscan >= 3)
            axis.controller_.input_torque_ = torque_feed_forward;
        axis.watchdog_feed();
    }
}

// @brief Executes the set torque control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_torque(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float torque_setpoint;

    if (sscanf(pStr, "c %u %f", &motor_number, &torque_setpoint) < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();
    }
}

// @brief Sets the encoder linear count
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_encoder(char * pStr, bool use_checksum) {
    if (pStr[1] == 's') {
        pStr += 2; // Substring two characters to the right (ok because we have guaranteed null termination after all chars)

        unsigned motor_number;
        int encoder_count;

        if (sscanf(pStr, "l %u %i", &motor_number, &encoder_count) < 2) {
            respond(use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis& axis = axes[motor_number];
            axis.encoder_.set_linear_count(encoder_count);
            axis.watchdog_feed();
            respond(use_checksum, "encoder set to %u", encoder_count);
        }
    } else {
        respond(use_checksum, "invalid command format");
    }
}

// @brief Executes the set trapezoid trajectory command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_trapezoid_trajectory(char* pStr, bool use_checksum) {
    unsigned motor_number;
    float goal_point;

    if (sscanf(pStr, "t %u %f", &motor_number, &goal_point) < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = goal_point;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the get position and velocity feedback command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_get_feedback(char * pStr, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "f %u", &motor_number) < 1) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        respond(use_checksum, "%f %f",
                (double)axis.encoder_.pos_estimate_.any().value_or(0.0f),
                (double)axis.encoder_.vel_estimate_.any().value_or(0.0f));
    }
}

// @brief Shows help text
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_help(char * pStr, bool use_checksum) {
    (void)pStr;
    respond(use_checksum, "Please see documentation for more details");
    respond(use_checksum, "");
    respond(use_checksum, "Available commands syntax reference:");
    respond(use_checksum, "Position: q axis pos vel-lim I-lim");
    respond(use_checksum, "Position: p axis pos vel-ff I-ff");
    respond(use_checksum, "Velocity: v axis vel I-ff");
    respond(use_checksum, "Torque: c axis T");
    respond(use_checksum, "");
    respond(use_checksum, "Properties start at odrive root, such as axis0.requested_state");
    respond(use_checksum, "Read: r property");
    respond(use_checksum, "Write: w property value");
    respond(use_checksum, "");
    respond(use_checksum, "Save config: ss");
    respond(use_checksum, "Erase config: se");
    respond(use_checksum, "Reboot: sr");
}

// @brief Gets the hardware, firmware and serial details
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_info_dump(char * pStr, bool use_checksum) {
    // respond(use_checksum, "Signature: %#x", STM_ID_GetSignature());
    // respond(use_checksum, "Revision: %#x", STM_ID_GetRevision());
    // respond(use_checksum, "Flash Size: %#x KiB", STM_ID_GetFlashSize());
    respond(use_checksum, "Hardware version: %d.%d-%dV", odrv.hw_version_major_, odrv.hw_version_minor_, odrv.hw_version_variant_);
    respond(use_checksum, "Firmware version: %d.%d.%d", odrv.fw_version_major_, odrv.fw_version_minor_, odrv.fw_version_revision_);
    respond(use_checksum, "Serial number: %s", serial_number_str);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_system_ctrl(char * pStr, bool use_checksum) {
    switch (pStr[1])
    {
        case 's':   odrv.save_configuration();  break;  // Save config
        case 'e':   odrv.erase_configuration(); break;  // Erase config
        case 'r':   odrv.reboot();              break;  // Reboot
        case 'c':   odrv.clear_errors();        break;  // clear all errors and rearm brake resistor if necessary
        default:    /* default */               break;
    }
}

// @brief Executes the read parameter command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_read_property(char * pStr, bool use_checksum) {
    char name[MAX_LINE_LENGTH];

    if (sscanf(pStr, "r %255s", name) < 1) {
        respond(use_checksum, "invalid command format");
    } else {
        Introspectable property = root_obj.get_child(name, sizeof(name));
        const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
        if (!type_info) {
            respond(use_checksum, "invalid property");
        } else {
            char response[10];
            bool success = type_info->get_string(property, response, sizeof(response));
            respond(use_checksum, success ? response : "not implemented");
        }
    }
}

// @brief Executes the set write position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_write_property(char * pStr, bool use_checksum) {
    char name[MAX_LINE_LENGTH];
    char value[MAX_LINE_LENGTH];

    if (sscanf(pStr, "w %255s %255s", name, value) < 1) {
        respond(use_checksum, "invalid command format");
    } else {
        Introspectable property = root_obj.get_child(name, sizeof(name));
        const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
        if (!type_info) {
            respond(use_checksum, "invalid property");
        } else {
            bool success = type_info->set_string(property, value, sizeof(value));
            if (!success) {
                respond(use_checksum, "not implemented");
            }
        }
    }
}

// @brief Executes the motor watchdog update command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_update_axis_wdg(char * pStr, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "u %u", &motor_number) < 1) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        axes[motor_number].watchdog_feed();
    }
}

// @brief Sends the unknown command response
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_unknown(char * pStr, bool use_checksum) {
    (void)pStr;
    respond(use_checksum, "unknown command");
}

void AsciiProtocol::on_read_finished(ReadResult result) {
    if (result.status != kStreamOk) {
        return;
    }

    for (;;) {
        uint8_t* end_of_line = std::find_if(rx_buf_, result.end, [](uint8_t c) {
            return c == '\r' || c == '\n' ;
        });

        if (end_of_line >= result.end) {
            break;
        }

        if (read_active_) {
            process_line({rx_buf_, end_of_line});
        } else {
            // Ignoring this line cause it didn't start at a new-line character
            read_active_ = true;
        }
        
        // Discard the processed bytes and shift the remainder to the beginning of the buffer
        size_t n_remaining = result.end - end_of_line - 1;
        memmove(rx_buf_, end_of_line + 1, n_remaining);
        result.end = rx_buf_ + n_remaining;
    }

    // No more new-line characters in buffer

    if (result.end >= rx_buf_ + sizeof(rx_buf_)) {
        // If the line becomes too long, reset buffer and wait for the next line
        result.end = rx_buf_;
        read_active_ = false;
    }

    TransferHandle dummy;
    rx_channel_->start_read({result.end, rx_buf_ + sizeof(rx_buf_)}, &dummy, MEMBER_CB(this, on_read_finished));
}

void AsciiProtocol::start() {
    TransferHandle dummy;
    rx_channel_->start_read(rx_buf_, &dummy, MEMBER_CB(this, on_read_finished));
}

static const uint8_t base_char = 33u; // '!' -- note that (85 + 33) < 128

static const int32_t ascii85_in_length_max = 65536;

static const bool ascii85_decode_z_for_zero  = false;
static const bool ascii85_encode_z_for_zero  = false;

static const bool ascii85_check_decode_chars = true;

#if 0
static inline bool ascii85_char_ok (uint8_t c)
{
    return ((c >= 33u) && (c <= 117u));
}
#endif

static inline bool ascii85_char_ng (uint8_t c)
{
    return ((c < 33u) || (c > 117u));
}

/*!
 * @brief encode_ascii85: encode binary input into Ascii85
 * @param[in] inp pointer to a buffer of unsigned bytes
 * @param[in] in_length the number of bytes at inp to encode
 * @param[in] outp pointer to a buffer for the encoded data
 * @param[in] out_max_length available space at outp in bytes; must be >=
 * ascii85_get_max_encoded_length(in_length)
 * @return number of bytes in the encoded value at outp if non-negative; error code from
 * ascii85_errs_e if negative
 * @par Possible errors include: ascii85_err_in_buf_too_large, ascii85_err_out_buf_too_small
 */
int32_t encode_ascii85 (const uint8_t *inp, int32_t in_length, uint8_t *outp, int32_t out_max_length)
{
    int32_t out_length = ascii85_get_max_encoded_length(in_length);

    if (out_length < 0)
    {
        // ascii85_get_max_encoded_length() already returned an error, so return that
    }
    else if (out_length > out_max_length)
    {
        out_length = (int32_t )ascii85_err_out_buf_too_small;
    }
    else
    {
        int32_t in_rover = 0;

        out_length = 0; // we know we can increment by 5 * ceiling(in_length/4)

        while (in_rover < in_length)
        {
            uint32_t chunk;
            int32_t chunk_len = in_length - in_rover;

            if (chunk_len >= 4)
            {
                chunk  = (((uint32_t )inp[in_rover++]) << 24u);
                chunk |= (((uint32_t )inp[in_rover++]) << 16u);
                chunk |= (((uint32_t )inp[in_rover++]) <<  8u);
                chunk |= (((uint32_t )inp[in_rover++])       );
            }
            else
            {
                chunk  =                           (((uint32_t )inp[in_rover++]) << 24u);
                chunk |= ((in_rover < in_length) ? (((uint32_t )inp[in_rover++]) << 16u) : 0u);
                chunk |= ((in_rover < in_length) ? (((uint32_t )inp[in_rover++]) <<  8u) : 0u);
                chunk |= ((in_rover < in_length) ? (((uint32_t )inp[in_rover++])       ) : 0u);
            }

            if (/*lint -e{506} -e{774}*/ascii85_encode_z_for_zero && (0u == chunk) && (chunk_len >= 4))
            {
                outp[out_length++] = (uint8_t )'z';
            }
            else
            {
                outp[out_length + 4] = (chunk % 85u) + base_char;
                chunk /= 85u;
                outp[out_length + 3] = (chunk % 85u) + base_char;
                chunk /= 85u;
                outp[out_length + 2] = (chunk % 85u) + base_char;
                chunk /= 85u;
                outp[out_length + 1] = (chunk % 85u) + base_char;
                chunk /= 85u;
                outp[out_length    ] = (uint8_t )chunk + base_char;
                // we don't need (chunk % 85u) on the last line since (((((2^32 - 1) / 85) / 85) / 85) / 85) = 82.278

                if (chunk_len >= 4)
                {
                    out_length += 5;
                }
                else
                {
                    out_length += (chunk_len + 1); // see note above re: Ascii85 length
                }
            }
        }
    }

    return out_length;
}

/*!
 * @brief decode_ascii85: decode Ascii85 input to binary output
 * @param[in] inp pointer to a buffer of Ascii85 encoded unsigned bytes
 * @param[in] in_length the number of bytes at inp to decode
 * @param[in] outp pointer to a buffer for the decoded data
 * @param[in] out_max_length available space at outp in bytes; must be >=
 * ascii85_get_max_decoded_length(in_length)
 * @return number of bytes in the decoded value at outp if non-negative; error code from
 * ascii85_errs_e if negative
 * @par Possible errors include: ascii85_err_in_buf_too_large, ascii85_err_out_buf_too_small,
 * ascii85_err_bad_decode_char, ascii85_err_decode_overflow
 */
int32_t decode_ascii85 (const uint8_t *inp, int32_t in_length, uint8_t *outp, int32_t out_max_length)
{
    int32_t out_length = ascii85_get_max_decoded_length(in_length);

    if (out_length < 0)
    {
        // get_max_decoded_length() already returned an error, so return that
    }
    else if (out_length > out_max_length)
    {
        out_length = (int32_t )ascii85_err_out_buf_too_small;
    }
    else
    {
        int32_t in_rover = 0;

        out_length = 0; // we know we can increment by 4 * ceiling(in_length/5)

        while (in_rover < in_length)
        {
            uint32_t chunk;
            int32_t chunk_len = in_length - in_rover;

            if (/*lint -e{506} -e{774}*/ascii85_decode_z_for_zero && ((uint8_t )'z' == inp[in_rover]))
            {
                in_rover += 1;
                chunk = 0u;
                chunk_len = 5; // to make out_length increment correct
            }
            else if (/*lint -e{506} -e{774}*/ascii85_check_decode_chars
                    && (                       ascii85_char_ng(inp[in_rover    ])
                        || ((chunk_len > 1) && ascii85_char_ng(inp[in_rover + 1]))
                        || ((chunk_len > 2) && ascii85_char_ng(inp[in_rover + 2]))
                        || ((chunk_len > 3) && ascii85_char_ng(inp[in_rover + 3]))
                        || ((chunk_len > 4) && ascii85_char_ng(inp[in_rover + 4]))))
            {
                out_length = (int32_t )ascii85_err_bad_decode_char;
                break; // leave while loop early to report error
            }
            else if (chunk_len >= 5)
            {
                chunk  = inp[in_rover++] - base_char;
                chunk *= 85u; // max: 84 * 85 = 7,140
                chunk += inp[in_rover++] - base_char;
                chunk *= 85u; // max: (84 * 85 + 84) * 85 = 614,040
                chunk += inp[in_rover++] - base_char;
                chunk *= 85u; // max: (((84 * 85 + 84) * 85) + 84) * 85 = 52,200,540
                chunk += inp[in_rover++] - base_char;
                // max: (((((84 * 85 + 84) * 85) + 84) * 85) + 84) * 85 = 4,437,053,040 oops! 0x108780E70
                if (chunk > (UINT32_MAX / 85u))
                {
                    // multiply would overflow
                    out_length = (int32_t )ascii85_err_decode_overflow; // bad input
                    break; // leave while loop early to report error
                }
                else
                {
                    uint8_t addend = inp[in_rover++] - base_char;

                    chunk *= 85u; // multiply will not overflow due to test above

                    if (chunk > (UINT32_MAX - addend))
                    {
                        /// add would overflow
                        out_length = (int32_t )ascii85_err_decode_overflow; // bad input
                        break; // leave while loop early to report error
                    }
                    else
                    {
                        chunk += addend;
                    }
                }
            }
            else
            {
                chunk  = inp[in_rover++] - base_char;
                chunk *= 85u; // max: 84 * 85 = 7,140
                chunk += ((in_rover < in_length) ? (inp[in_rover++] - base_char) : 84u);
                chunk *= 85u; // max: (84 * 85 + 84) * 85 = 614,040
                chunk += ((in_rover < in_length) ? (inp[in_rover++] - base_char) : 84u);
                chunk *= 85u; // max: (((84 * 85 + 84) * 85) + 84) * 85 = 52,200,540
                chunk += ((in_rover < in_length) ? (inp[in_rover++] - base_char) : 84u);
                // max: (((((84 * 85 + 84) * 85) + 84) * 85) + 84) * 85 = 4,437,053,040 oops! 0x108780E70
                if (chunk > (UINT32_MAX / 85u))
                {
                    // multiply would overflow
                    out_length = (int32_t )ascii85_err_decode_overflow; // bad input
                    break; // leave while loop early to report error
                }
                else
                {
                    uint8_t addend = (uint8_t )((in_rover < in_length) ? (inp[in_rover++] - base_char) : 84u);

                    chunk *= 85u; // multiply will not overflow due to test above

                    if (chunk > (UINT32_MAX - addend))
                    {
                        /// add would overflow
                        out_length = (int32_t )ascii85_err_decode_overflow; // bad input
                        break; // leave while loop early to report error
                    }
                    else
                    {
                        chunk += addend;
                    }
                }
            }

            outp[out_length + 3] = (chunk % 256u);
            chunk /= 256u;
            outp[out_length + 2] = (chunk % 256u);
            chunk /= 256u;
            outp[out_length + 1] = (chunk % 256u);
            chunk /= 256u;
            outp[out_length    ] = (uint8_t )chunk;
            // we don't need (chunk % 256u) on the last line since ((((2^32 - 1) / 256u) / 256u) / 256u) = 255

            if (chunk_len >= 5)
            {
                out_length += 4;
            }
            else
            {
                out_length += (chunk_len - 1); // see note above re: Ascii85 length
            }
        }
    }

    return out_length;
}

/*!
 * @brief ascii85_get_max_encoded_length: get the maximum length a block of data will encode to
 * @param[in] in_length the number of data bytes to encode
 * @return maximum number of bytes the encoded buffer could be if non-negative; error code from
 * ascii85_errs_e if negative
 * @par Possible errors include: ascii85_err_in_buf_too_large
 */
int32_t ascii85_get_max_encoded_length (int32_t in_length)
{
    int32_t out_length;

    if ((in_length < 0) || (in_length > ascii85_in_length_max))
    {
        out_length = (int32_t )ascii85_err_in_buf_too_large;
    }
    else
    {
        // (in_length + 3) will not overflow since ascii85_in_length_max
        // is < (INT32_MAX - 3), and similar reasoning for the final * 5
        out_length = ((in_length + 3) / 4) * 5; // ceiling
    }

    return out_length;
}

/*!
 * @brief ascii85_get_max_encoded_length: get the maximum length a block of data will decode to
 * @param[in] in_length the number of encoded bytes to decode
 * @return maximum number of bytes the decoded buffer could be if non-negative; error code from
 * ascii85_errs_e if negative
 * @par Possible errors include: ascii85_err_in_buf_too_large
 */
int32_t ascii85_get_max_decoded_length (int32_t in_length)
{
    int32_t out_length;

    if ((in_length < 0) || (in_length > ascii85_in_length_max))
    {
        out_length = (int32_t )ascii85_err_in_buf_too_large;
    }
    else if (/*lint -e{506} -e{774}*/ascii85_decode_z_for_zero)
    {
        out_length = in_length * 4;
    }
    else
    {
        // (in_length + 4) will not overflow since ascii85_in_length_max
        // is < (INT32_MAX - 4)
        out_length = ((in_length + 4) / 5) * 4; // ceiling
    }

    return out_length;
}
