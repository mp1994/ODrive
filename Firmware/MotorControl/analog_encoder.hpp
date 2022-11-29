#ifndef __ANALOG_ENCODER_HPP
#define __ANALOG_ENCODER_HPP

class AnalogEncoder;

#include <board.h> // needed for arm_math.h
#include "utils.hpp"
#include <autogen/interfaces.hpp>
#include "component.hpp"


class AnalogEncoder {  
public:
    struct Config_t {

        bool enable = false;

        uint16_t gpio_pin = 5;            // GPIO pin for ADC readout of analog (pot) encoder
        AnalogEncoder* parent = nullptr;  // parent (init to none)

        float K_Volt_to_Pos = 1.0f;
        float pos_offset = 0.0f;

        // Custom setters
        void enable_analog_encoder(bool value) { enable = value; };
        void set_gpio_pin(uint16_t value) { gpio_pin = value; };
        void set_K(float value) { K_Volt_to_Pos = float(value); };
        void set_offset(float value) { pos_offset = float(value); };
        
    };

    AnalogEncoder(Stm32Gpio adc_gpio);
    AnalogEncoder(uint16_t gpio_num);

    // Attributes
    bool is_ready_ = false;
    // Reading from the ADC
    float pos_voltage_estimate_ = 0.0f;
    // Outputs --> read from Axis and Controller
    OutputPort<float> pos_estimate_ = 0.0f;
    OutputPort<float> pos_estimate_filt_ = 0.0f;
    
    bool apply_config();
    void setup();
    void set_error();
    bool do_checks();

    bool run_offset_calibration();
    void sample_now();
    bool read_sampled_gpio(Stm32Gpio gpio);
    bool update();

    uint16_t pin_number_;
    Stm32Gpio adc_gpio_;
    Axis* axis_ = nullptr; // set by Axis constructor
    bool enabled_ = false;
    Config_t config_;

    float* analog_encoder_buf_ = (float*) malloc(8*sizeof(float));
    volatile uint32_t analog_enc_buf_count_ = 0;
    volatile float enc_filt_ = 0.0f;

    bool position_estimate_valid_ = false;

};

#endif // __ANALOG_ENCODER_HPP
