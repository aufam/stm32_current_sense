#include "current_sense/current_sense.h"
#include "etl/array.h"
#include "etl/bit.h"
#include "etl/keywords.h"


fun CurrentSense::init() -> void {
    i2c.init();
    constexpr uint16_t cfg = 
        MODE_SHUNT_AND_BUS_VOLT_CONTINOUS | 
        CONFIG_BUS_VOLTAGE_RANGE_32V |
        CONFIG_GAIN_8 |
        CONFIG_SHUNT_12BIT_128X |
        CONFIG_BUS_12BIT_128X;
    setConfig(cfg);
}

fun CurrentSense::setConfig(uint16_t cfg) -> void {
    write(REG_CONFIG, config_);
    config_ = cfg;
}

fun CurrentSense::getConfig() -> uint16_t {
    config_ = read(REG_CONFIG);
    return config_;
}

fun CurrentSense::readCurrent() -> float {
    return readShuntVoltage() / resistor_;
}

fun CurrentSense::readVoltage() -> float {
    val raw = read(REG_BUS_VOLTAGE);
    return etl::safe_div<float>((raw >> 3) * 4, 1000);
}

fun CurrentSense::readShuntVoltage() -> float {
    val raw = read(REG_SHUNTVOLTAGE);
    val v = etl::bit_cast<int16_t>(raw);
    val mv = etl::safe_mul<float>(v, 0.01); // convert to mV
    return mv * 0.0001; // convert to V 
}

fun CurrentSense::getVoltageMax() -> float {
    return config_ & CONFIG_RESET ? NAN :
           config_ & CONFIG_BUS_VOLTAGE_RANGE_32V ? 32 :
           16;
}

fun CurrentSense::getCurrentMax() -> float {
    float v = config_ & CONFIG_RESET ? NAN :
              config_ & CONFIG_GAIN_8 ? 0.32 :
              config_ & CONFIG_GAIN_4 ? 0.16 :
              config_ & CONFIG_GAIN_2 ? 0.08 :
              0.04;
    return v / resistor_;
}

fun CurrentSense::getShuntBitResoulition() -> uint16_t {
    return config_ & CONFIG_RESET ? 0 :
           config_ & 0b1000000u ? 12 :
           config_ & CONFIG_SHUNT_12BIT ? 12 :
           config_ & CONFIG_SHUNT_11BIT ? 11 :
           config_ & CONFIG_SHUNT_10BIT ? 10 :
           9;
}

fun CurrentSense::getBusBitResoulition() -> uint16_t {
    return config_ & CONFIG_RESET ? 0 :
           config_ & 0b10000000000u ? 12 :
           config_ & CONFIG_BUS_12BIT ? 12 :
           config_ & CONFIG_BUS_11BIT ? 11 :
           config_ & CONFIG_BUS_10BIT ? 10 :
           9;
}

fun CurrentSense::getShuntNSamples() -> uint16_t {
    return config_ & CONFIG_RESET ? 0 :
           config_ & CONFIG_SHUNT_12BIT_128X ? 128 :
           config_ & CONFIG_SHUNT_12BIT_64X ? 64 :
           config_ & CONFIG_SHUNT_12BIT_32X ? 32 :
           config_ & CONFIG_SHUNT_12BIT_16X ? 16 :
           config_ & CONFIG_SHUNT_12BIT_8X ? 8 :
           config_ & CONFIG_SHUNT_12BIT_4X ? 4 :
           config_ & CONFIG_SHUNT_12BIT_2X ? 2 :
           1;
}

fun CurrentSense::getBusNSamples() -> uint16_t {
    return config_ & CONFIG_RESET ? 0 :
           config_ & CONFIG_BUS_12BIT_128X ? 128 :
           config_ & CONFIG_BUS_12BIT_64X ? 64 :
           config_ & CONFIG_BUS_12BIT_32X ? 32 :
           config_ & CONFIG_BUS_12BIT_16X ? 16 :
           config_ & CONFIG_BUS_12BIT_8X ? 8 :
           config_ & CONFIG_BUS_12BIT_4X ? 4 :
           config_ & CONFIG_BUS_12BIT_2X ? 2 :
           1;
}

fun CurrentSense::getCurrentConversionTime() -> float {
    return config_ & CONFIG_RESET ? NAN :
           config_ & CONFIG_SHUNT_12BIT_128X ? 0.068'1f :
           config_ & CONFIG_SHUNT_12BIT_64X ? 0.034'05f :
           config_ & CONFIG_SHUNT_12BIT_32X ? 0.017'02f :
           config_ & CONFIG_SHUNT_12BIT_16X ? 0.008'51f :
           config_ & CONFIG_SHUNT_12BIT_8X ? 0.004'26f :
           config_ & CONFIG_SHUNT_12BIT_4X ? 0.002'13f :
           config_ & CONFIG_SHUNT_12BIT_2X ? 0.001'06f :
           config_ & 0b1000000u ? 0.000'532f :
           config_ & CONFIG_SHUNT_12BIT ? 0.000'532f :
           config_ & CONFIG_SHUNT_11BIT ? 0.000'276f :
           config_ & CONFIG_SHUNT_10BIT ? 0.000'148f :
           0.000'084f;
}

fun CurrentSense::getVoltageConversionTime() -> float {
    return config_ & CONFIG_RESET ? NAN :
           config_ & CONFIG_BUS_12BIT_128X ? 0.068'1f :
           config_ & CONFIG_BUS_12BIT_64X ? 0.034'05f :
           config_ & CONFIG_BUS_12BIT_32X ? 0.017'02f :
           config_ & CONFIG_BUS_12BIT_16X ? 0.008'51f :
           config_ & CONFIG_BUS_12BIT_8X ? 0.004'26f :
           config_ & CONFIG_BUS_12BIT_4X ? 0.002'13f :
           config_ & CONFIG_BUS_12BIT_2X ? 0.001'06f :
           config_ & 0b10000000000u ? 0.000'532f :
           config_ & CONFIG_BUS_12BIT ? 0.000'532f :
           config_ & CONFIG_BUS_11BIT ? 0.000'276f :
           config_ & CONFIG_BUS_10BIT ? 0.000'148f :
           0.000'084f;
}

fun CurrentSense::getVoltageResolution() -> float {
    val bitres = getBusBitResoulition();
    return bitres ? etl::safe_div<float>(getVoltageMax(), 1 << bitres) : NAN;
}

fun CurrentSense::getCurrentResolution() -> float {
    val bitres = getShuntBitResoulition();
    return bitres ? etl::safe_div<float>(getCurrentMax(), 1 << bitres) : NAN;
}

fun CurrentSense::read(uint8_t reg) -> uint16_t {
    var buf = etl::array<uint8_t, 2>();
    HAL_I2C_Mem_Read(&i2c.hi2c, address << 1, reg, 1, buf.data(), buf.len(), timeout);
    return etl::byte_array_cast_back_be<uint16_t>(buf);
}

fun CurrentSense::write(uint8_t reg, uint16_t value) -> void {
    var buf = etl::byte_array_cast_be(value);
    HAL_I2C_Mem_Write(&i2c.hi2c, address << 1, reg, 1, buf.data(), buf.len(), timeout);
}


