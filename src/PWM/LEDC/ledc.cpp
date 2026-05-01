#include "ledc.h"

namespace Haptics {
namespace LEDC {
Logging::Logger logger("LEDC");

//frequency calculationss
const double_t UpdateFrequency = LEDC_FREQUENCY;
const unsigned long tickPeriod = (1./UpdateFrequency)*1000000; // microseconds
const unsigned long tockPeriod = tickPeriod / (1 << LEDC_RESOLUTION); // microseconds

volatile uint16_t phase = 0;

#ifdef ESP8266
// ESP8266 uses built-in analogWrite()
// No need for custom ISR implementation

void updateESP8266PWM() {
    // Limit to 8 channels for ESP8266
    const int maxChannels = min(8, (int)Haptics::Conf::conf.motor_map_ledc_num);
    
    for (int motor = 0; motor < maxChannels; ++motor) {
        uint8_t pin = Haptics::Conf::conf.motor_map_ledc[motor];
        // Convert from 8-bit (0-255) to 10-bit (0-1023) for ESP8266 analogWrite
        uint16_t pwmValue = (Haptics::globals.ledcMotorVals[motor] * 1023) / 255;
        analogWrite(pin, pwmValue);
    }
}

#else
// ESP32-specific implementation
void IRAM_ATTR pwm_isr()
{
    uint32_t setMask0 = 0, clrMask0 = 0; // GPIOs 0-31
    uint32_t setMask1 = 0, clrMask1 = 0; // GPIOs 32+

    for (int motor = 0; motor < Haptics::Conf::conf.motor_map_ledc_num; ++motor) {
        uint8_t pin = Haptics::Conf::conf.motor_map_ledc[motor];
        bool on = Haptics::globals.ledcMotorVals[motor] > phase;

        if (pin < 32) {
            uint32_t bit = 1UL << pin;
            if (on) setMask0 |= bit; else clrMask0 |= bit;
        } else {
            uint32_t bit = 1UL << (pin - 32);
            if (on) setMask1 |= bit; else clrMask1 |= bit;
        }
    }

    if (setMask0) REG_WRITE(GPIO_OUT_W1TS_REG, setMask0);
    if (clrMask0) REG_WRITE(GPIO_OUT_W1TC_REG, clrMask0);
    #if SOC_GPIO_PIN_COUNT > 32
    if (setMask1) REG_WRITE(GPIO_OUT1_W1TS_REG, setMask1);
    if (clrMask1) REG_WRITE(GPIO_OUT1_W1TC_REG, clrMask1);
    #endif

    if (++phase == 1 << LEDC_RESOLUTION) phase = 0;
}
#endif

int start(Haptics::Conf::Config *conf) {
    if (Haptics::Conf::conf.motor_map_ledc_num != 0) {
        // Update pins and declare pinmodes
#ifdef ESP8266
        // Limit to 8 channels for ESP8266
        const int maxChannels = min(8, (int)conf->motor_map_ledc_num);
        logger.info("Starting ESP8266 LEDC with %d channels (max 8) using analogWrite", maxChannels);
        
        for (int i = 0; i < maxChannels; i++) {
            uint8_t pin = conf->motor_map_ledc[i];
            pinMode(pin, OUTPUT);
            // Set PWM frequency for ESP8266 (default is 1000Hz, we want to match LEDC_FREQUENCY)
            analogWriteFreq(LEDC_FREQUENCY);
            analogWrite(pin, 0); // Start with PWM off
        }
        
        // No timer setup needed for ESP8266 - analogWrite handles it
#else
        // ESP32 implementation
        for (const auto& pin : conf->motor_map_ledc) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH); //Cycling this seems to get it to work, idk why, pinmode should just work
            digitalWrite(pin, LOW);
        }

        // 1 MHz hardware timer -> 39 µs alarms
        hw_timer_t *timer = timerBegin(LEDC_TIMER, 80, true);// 80 MHz / 80 = 1 MHz
        timerAttachInterrupt(timer, &pwm_isr, true);
        timerAlarmWrite(timer, tockPeriod, true);   // fire every 39 µs
        timerAlarmEnable(timer);
#endif
    }

    return 0;
}

#ifdef ESP8266
void tick() {
    // For ESP8266, we update PWM values directly when they change
    updateESP8266PWM();
}

inline int setChannel(const uint8_t channel, const uint16_t duty) {
    // Limit to 8 channels for ESP8266
    if (channel >= 8 || channel >= Haptics::Conf::conf.motor_map_ledc_num) {
        return -1;
    }
    
    // Store the value in globals for consistency
    Haptics::globals.ledcMotorVals[channel] = duty >> 8; // Convert 16-bit to 8-bit
    
    // Update PWM immediately for ESP8266
    uint8_t pin = Haptics::Conf::conf.motor_map_ledc[channel];
    uint16_t pwmValue = (Haptics::globals.ledcMotorVals[channel] * 1023) / 255;
    analogWrite(pin, pwmValue);
    
    return 0;
}

int setAllTo(const uint16_t duty) {
    const int maxChannels = min(8, (int)Haptics::Conf::conf.motor_map_ledc_num);
    for (int i = 0; i < maxChannels; i++) {
        setChannel(i, duty);
    }
    return 0;
}

#else
inline void tick(); // esp32's have timer shenanigans

inline int setChannel(const uint8_t channel, const uint16_t duty) {
    if (channel >= Haptics::Conf::conf.motor_map_ledc_num) {
        return -1;
    }
    Haptics::globals.ledcMotorVals[channel] = duty >> 8; // Convert 16-bit to 8-bit
    return 0;
}

int setAllTo(const uint16_t duty) {
    for (int i = 0; i < Haptics::Conf::conf.motor_map_ledc_num; i++) {
        setChannel(i, duty);
    }
    return 0;
}
#endif

} // namespace LEDC
} // namespace Haptics