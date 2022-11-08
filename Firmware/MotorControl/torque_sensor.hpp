#ifndef __TORQUE_SENSOR_HPP
#define __TORQUE_SENSOR_HPP

class TorqueSensor;

#include <board.h> // needed for arm_math.h
#include "utils.hpp"
#include <autogen/interfaces.hpp>
#include "component.hpp"


class TorqueSensor {       // Da capire cosa mettere qui al posto di ODriveIntf::EncoderIntf
public:
    struct Config_t {

        bool enable = true;

        uint16_t gpio_pin = 3;           // GPIO pin for ADC readout of torque sensor
        
        // Sensor gains
        float K_VtoX = 1.0f;              // Voltage to Displacement gain [m/V]
        float K_XtoM = 1.0f;              // Displacement to Torque gain [Nm/m]
        float K_gain = -0.12f;            // [Nm/V]

        uint8_t N_spring_pairs = 3;      // Number of spring pairs

        float torque_offset = 0.0f;      // Zero offset of the SEA [Nm]

        // Control loop gains
        float k_p = 1.0f;                // Proportional gain
        float k_i = 0.0f;                // Integral gain 

        TorqueSensor* parent = nullptr;  // parent (init to none)

        // Custom setters
        void set_gpio_pin(uint16_t value) { gpio_pin = value; };
        void set_Kgain(float32_t value) { K_gain = value; };
        void set_Pgain(float32_t value) { k_p = value; };
        void set_Igain(float32_t value) { k_i = value; };
        void set_Nsprings(uint8_t value) { N_spring_pairs = value; K_gain *= float(value)/3.0f; };
        void set_TorqueOffset(float32_t value) { torque_offset = value; };
        void enable_sea(bool value) { enable = value;}

    };

    TorqueSensor(Stm32Gpio adc_gpio);
    TorqueSensor(uint16_t gpio_num);
    
    bool apply_config(ODriveIntf::MotorIntf::MotorType motor_type);
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

    bool is_ready_ = false;

    // TorqueSensor variables
    float torque_nm_estimate_  = 0.0f;    // Torque [Nm]
    float torque_voltage_meas_ = 0.0f;    // ADC reading [V]
    float torque_dx_estimate_  = 0.0f;    // Displacement [m]
    
    float torque_error_ = 0.0f;             // Torque error   [Nm]
    float torque_error_integral = 0.0f;     // Integral torque error
    float prev_error_ = 0.0f;               // Previous torque error [Nm]

    float* sea_buf_ = (float*) malloc(8*sizeof(float));
    volatile uint32_t sea_buf_count_ = 0;
    volatile float sea_filt_ = 0.0f;

    // Outputs --> read from Axis and Controller
    OutputPort<float> torque_estimate_ = 0.0f;
    OutputPort<float> torque_estimate_filt_ = 0.0f;

    bool torque_estimate_valid_ = false;

};

#endif // __TORQUE_SENSOR_HPP
