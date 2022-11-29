
#include "odrive_main.h"
#include <Drivers/STM32/stm32_system.h>
#include <bitset>

AnalogEncoder::AnalogEncoder(Stm32Gpio adc_gpio) : 
        adc_gpio_(adc_gpio)
{
    enabled_ = true;
}

AnalogEncoder::AnalogEncoder(uint16_t gpio_num) :
        pin_number_(gpio_num)
{
    if( gpio_num > 0 && gpio_num < 9 ) {
        enabled_ = true;
        
    }
}

bool AnalogEncoder::apply_config() {

    config_.parent = this;
    return true;

}

void AnalogEncoder::setup() {

    // Do GPIO pin setting here
    adc_gpio_ = get_gpio(config_.gpio_pin); // FIX this: why do we need Stm32Gpio as input if we set it here from gpio_pin ?
    adc_gpio_.config(GPIO_MODE_INPUT, GPIO_MODE_ANALOG); 

    is_ready_ = true;

    // adc_gpio.subscribe() > set an ISR for the pin > do we need it? 
    // probably not...

}

void AnalogEncoder::set_error(void) {
    position_estimate_valid_ = false;
}

// bool AnalogEncoder::do_checks(){
//     return error_ == ERROR_NONE;
// }

// We probably need to implement this in a smarter way...
// This is pointless now...
bool AnalogEncoder::do_checks() {
    is_ready_ = enabled_;
    return enabled_;
}


// This function should only sample data when called by a high-priority ISR
void AnalogEncoder::sample_now() {

    pos_voltage_estimate_ = get_adc_voltage(get_gpio(config_.gpio_pin));

    // I think we don't need this, but let's leave it here for now...
    // Sample all GPIO digital input data registers, used for HALL sensors for example.
    // for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); ++i) {
    //     port_samples_[i] = ports_to_sample[i]->IDR;
    // }

}

// I think we don't need this, but let's leave it here for now...
// bool AnalogEncoder::read_sampled_gpio(Stm32Gpio gpio) {
//     for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); ++i) {
//         if (ports_to_sample[i] == gpio.port_) {
//             return port_samples_[i] & gpio.pin_mask_;
//         }
//     }
//     return false;
// }


// Update the torque estimate based on the newest sampled value
bool AnalogEncoder::update() {

    if( !config_.enable ) {
        pos_estimate_ = 0.0f;
        pos_estimate_filt_ = 0.0f;
        return false;
    } 

    // Output from AnalogEncoder to the Controller
    pos_estimate_ = config_.K_Volt_to_Pos * (pos_voltage_estimate_ - 1.65f); - config_.pos_offset;

    // Filtering
    analog_encoder_buf_[analog_enc_buf_count_ % 8] = (float) pos_estimate_.any().value_or(0.0f);
    // Try to estimate the velocity... this may suck.
    analog_encoder_buf_vel_[analog_enc_buf_count_ % 8] = (float) (analog_encoder_buf_[(analog_enc_buf_count_%8)] - analog_encoder_buf_[(analog_enc_buf_count_%8)-1])/(0.000125f);
    analog_enc_buf_count_++;
    // Compute the average only when the buffer is filled with values
    if( analog_enc_buf_count_ % 8 == 0 ) {
        enc_filt_ = 0.0f;
        vel_filt_ = 0.0f;
        for( uint8_t i = 0; i < 8; i++ ) {
            enc_filt_ += analog_encoder_buf_[i];
            vel_filt_ += analog_encoder_buf_vel_[i];
        }   
        enc_filt_ = enc_filt_ / 8.0f;
        vel_filt_ = vel_filt_ / 8.0f;
        pos_estimate_filt_ = enc_filt_;
        vel_estimate_filt_ = vel_filt_;
    }

    return enabled_;

}
