#ifndef PROJECT_CURRENT_SENSE_H
#define PROJECT_CURRENT_SENSE_H

#include "periph/i2c.h"
#include "etl/numerics.h"
#include "etl/getter_setter.h"
#include "etl/function.h"

namespace Project {

    /// current sense using IC INA219AID
    class CurrentSense {
        template <typename T>
        using Getter = etl::Getter<T, etl::Function<T(), CurrentSense*>>;

        template <typename T>
        using GetterSetter = etl::GetterSetter<T, etl::Function<T(), CurrentSense*>, etl::Function<void(T), CurrentSense*>>;

        inline static constexpr uint32_t timeout = 100;
        inline static constexpr uint8_t defaultAddress = 0x40;

        uint16_t config_ = {};
        float resistor_;
    
    public:
        periph::I2C& i2c;
        uint8_t address;

        struct ConstructorArgs {
            periph::I2C& i2c;
            float resistor;
            uint8_t address = defaultAddress;
        };

        /// default constructor
        /// @param args 
        ///     - .i2c reference to periph::I2C object
        ///     - .resistor resistor value
        ///     - .address device address
        explicit constexpr CurrentSense(ConstructorArgs args) 
            : resistor_(args.resistor)
            , i2c(args.i2c)
            , address(args.address) {}

        /// set default configuartion
        void init();

        /// set and get configuration
        const GetterSetter<uint16_t> config = {
            etl::bind<&CurrentSense::getConfig>(this),
            etl::bind<&CurrentSense::setConfig>(this)
        };

        /// get current in Ampere
        const Getter<float> current = {
            etl::bind<&CurrentSense::readCurrent>(this),
        };

        /// get voltage in Volt
        const Getter<float> voltage = {
            etl::bind<&CurrentSense::readVoltage>(this),
        };

        /// get shunt voltage different in Volt
        const Getter<float> shuntVoltage = {
            etl::bind<&CurrentSense::readShuntVoltage>(this),
        };

        /// get maximum current of the configuration
        const Getter<float> currentMax = {
                etl::bind<&CurrentSense::getCurrentMax>(this),
        };

        /// get maximum voltage of the configuration
        const Getter<float> voltageMax = {
                etl::bind<&CurrentSense::getVoltageMax>(this),
        };

        /// get ADC resolution of shunt voltage in bit
        const Getter<uint16_t> shuntBitResolution = {
                etl::bind<&CurrentSense::getShuntBitResoulition>(this),
        };

        /// get ADC resolution of bus voltage in bit
        const Getter<uint16_t> busBitResolution = {
                etl::bind<&CurrentSense::getBusBitResoulition>(this),
        };

        /// get number of sample of shunt voltage ADC conversion
        const Getter<uint16_t> shuntNSamples = {
                etl::bind<&CurrentSense::getShuntNSamples>(this),
        };

        /// get number of sample of bus voltage ADC conversion
        const Getter<uint16_t> busNSamples = {
                etl::bind<&CurrentSense::getBusNSamples>(this),
        };

        /// get current resolution in A
        const Getter<float> currentResolution = {
                etl::bind<&CurrentSense::getCurrentResolution>(this),
        };

        /// get current resolution in V
        const Getter<float> voltageResolution = {
                etl::bind<&CurrentSense::getVoltageResolution>(this),
        };

        /// get ADC conversion time of shunt voltage in seconds
        const Getter<float> currentConversionTime = {
                etl::bind<&CurrentSense::getCurrentConversionTime>(this),
        };

        /// get ADC conversion time of bus voltage in seconds
        const Getter<float> voltageConversionTime = {
                etl::bind<&CurrentSense::getVoltageConversionTime>(this),
        };

        /// get resistor value in ohm
        const float& resistor = resistor_;

        enum : uint8_t {
            REG_CONFIG = 0x00,
            REG_SHUNTVOLTAGE,
            REG_BUS_VOLTAGE,
            REG_POWER,
            REG_CURRENT,
            REG_CALIB,
        };

        enum : uint16_t {
            CONFIG_RESET                    = 0x8000u,
            CONFIG_BUS_VOLTAGE_RANGE_16V    = 0x0000u,
            CONFIG_BUS_VOLTAGE_RANGE_32V    = 0x2000u,
            
            CONFIG_GAIN_1                   = 0x0000u,
            CONFIG_GAIN_2                   = 0x0800u,
            CONFIG_GAIN_4                   = 0x1000u,
            CONFIG_GAIN_8                   = 0x1800u,

            CONFIG_SHUNT_9BIT               = 0x0000u,
            CONFIG_SHUNT_10BIT              = 0x0008u,
            CONFIG_SHUNT_11BIT              = 0x0010u,
            CONFIG_SHUNT_12BIT              = 0x0018u,
            CONFIG_SHUNT_12BIT_2X           = 0x0048u,
            CONFIG_SHUNT_12BIT_4X           = 0x0050u,
            CONFIG_SHUNT_12BIT_8X           = 0x0058u,
            CONFIG_SHUNT_12BIT_16X          = 0x0060u,
            CONFIG_SHUNT_12BIT_32X          = 0x0068u,
            CONFIG_SHUNT_12BIT_64X          = 0x0070u,
            CONFIG_SHUNT_12BIT_128X         = 0x0078u,

            CONFIG_BUS_9BIT                 = 0x0000u,
            CONFIG_BUS_10BIT                = 0x0080u,
            CONFIG_BUS_11BIT                = 0x0100u,
            CONFIG_BUS_12BIT                = 0x0180u,
            CONFIG_BUS_12BIT_2X             = 0x0480u,
            CONFIG_BUS_12BIT_4X             = 0x0500u,
            CONFIG_BUS_12BIT_8X             = 0x0580u,
            CONFIG_BUS_12BIT_16X            = 0x0600u,
            CONFIG_BUS_12BIT_32X            = 0x0680u,
            CONFIG_BUS_12BIT_64X            = 0x0700u,
            CONFIG_BUS_12BIT_128X           = 0x0780u,
        };

        enum : uint16_t {
            MODE_POWERDOWN                      = 0B000,
            MODE_SHUNT_VOLT_TRIGGERED           = 0B001,
            MODE_BUS_VOLT_TRIGGERED             = 0B010,
            MODE_SHUNT_AND_BUS_VOLT_TRIGGERED   = 0B011,
            MODE_ADC_OFF                        = 0B100,
            MODE_SHUNT_VOLT_CONTINOUS           = 0B101,
            MODE_BUS_VOLT_CONTINOUS             = 0B110,
            MODE_SHUNT_AND_BUS_VOLT_CONTINOUS   = 0B111,
            MODE_MASK                           = 0B111,
        };
    
    private:
        void setConfig(uint16_t cfg);
        uint16_t getConfig();

        float readCurrent();
        float readVoltage();
        float readShuntVoltage();

        float getCurrentMax();
        float getVoltageMax();

        uint16_t getShuntBitResoulition();
        uint16_t getBusBitResoulition();

        uint16_t getShuntNSamples();
        uint16_t getBusNSamples();

        float getCurrentResolution();
        float getVoltageResolution();

        float getCurrentConversionTime();
        float getVoltageConversionTime();

        uint16_t read(uint8_t reg);
        void write(uint8_t reg, uint16_t value);
    };
}

#endif