//todo: put this note from https://wiki.seeedstudio.com/XIAO_BLE/ Question Q3 into the compilation warning: 
// When P0.14 (D14) turns off the ADC function at a high level of 3.3V, P0.31 will be at the input voltage limit of 3.6V. There is a risk of burning out the P0.31 pin.
// Currently for this issue, we recommend that users do not turn off the ADC function of P0.14 (D14) or set P0.14 (D14) to high during battery charging.

#include <Arduino.h>

#ifndef SEEDNRF52480BATTERY_H
#define SEEDNRF52480BATTERY_H

//#define PIN_BATTERY_CURRENT_PIN P0_13
//#define PIN_CHARGING_INV P0_17

#if !defined(PIN_CHARGING_INV)
  #define PIN_CHARGING_INV PIN_BATTERY_CHARGE_PIN
#endif

#if !defined(PIN_BATTERY_CURRENT_PIN)
  // Prefer the variant’s name if it exists
  #if defined(PIN_CHARGING_CURRENT)
    #define PIN_BATTERY_CURRENT_PIN PIN_CHARGING_CURRENT
  #else
    #define PIN_BATTERY_CURRENT_PIN A0
  #endif
#endif

#if !defined(PIN_BATTERY_CHARGE_PIN)
  // If your variant has a PIN_CHARGING / PIN_CHARGING_STATE, prefer that here.
  #ifdef PIN_CHARGING
    #define PIN_BATTERY_CHARGE_PIN PIN_CHARGING
  #elif defined(PIN_CHARGING_STATE)
    #define PIN_BATTERY_CHARGE_PIN PIN_CHARGING_STATE
  #else
    #define PIN_BATTERY_CHARGE_PIN 17   // matches earlier mapping for P0.17
  #endif
#endif

// --- Normalize pin macro names used across different cores / forks ---

// Some versions use these alternate names:
#ifndef PIN_CHARGING_INV
  // Use the same physical charge-status pin (active-low on most XIAO boards)
  #define PIN_CHARGING_INV PIN_BATTERY_CHARGE_PIN
#endif

// if the variant already defines it, DO NOT override
#if !defined(PIN_CHARGING_CURRENT)
// map to whatever the library expects if the variant doesn't provide it
  #if defined(PIN_BATTERY_CURRENT_PIN)
    #define PIN_CHARGING_CURRENT PIN_BATTERY_CURRENT_PIN
  #else
    // last-resort fallback (rarely used)
    #define PIN_CHARGING_CURRENT 22
  #endif
#endif

// If code cares about active level, define it explicitly.
// On XIAO nRF52840 the charge-status (CHG) line from the charger is ACTIVE-LOW.
#ifndef CHARGING_PIN_ACTIVE_LOW
  #define CHARGING_PIN_ACTIVE_LOW 1
#endif

// Fallbacks for missing raw pin names
#ifndef P0_13
  #define P0_13 13
#endif
#ifndef P0_17
  #define P0_17 17
#endif
#ifndef P0_31
  #ifdef A6
    #define P0_31 A6
  #else
    #define P0_31 30
  #endif
#endif





class SeeedNrf52480Battery {
    public:

        SeeedNrf52480Battery(bool disableVoltageReading = false, bool useP0_31 = false);

        bool isCharging();
        void setChargeCurrent100mA();
        void setChargeCurrent50mA();
        
        void enableVoltageReading();
        void setSampleSize(uint8_t adcSampleSize);

        float getVoltage();
        float getPercentage();
        void setVoltageDividerRatio(float voltageDividerRatio);
        float getVoltageDividerRatio();
        int updateADCReading();
        void setMaxVoltage(float maxVolts);
        void setMinVoltage(float minVolts);

    private:

        void chargingInterrupt();
        void disableVoltageReading();
        //battery performance
        //according to the texas instruments BQ25100 documentation 4.2V is the maximum charge voltage
        //const float maxVoltage = 4.2;
        float maxVoltage = 4.2;
        //3.2V is just a reasonable discharged voltage for Lithium-(X) cells
        float minVoltage = 3.2;

        //number of samples of adc already collected
        int collectedADCSamples = 0;
        
        //rolling average of inputs
        float batteryADCRollingAvg = 0;

        //number of samples used to smooth fluctuations
        uint8_t adcSampleSize = 10;

        //disable ADC and voltage pin
        bool voltageReadingDisabled = false;

        //pin used to read input voltage
        uint8_t adcInputPin = PIN_VBAT;

        //ratio of Voltage divider: 1.0 +  R16 (nominally 1MOhm +- 1%)  / R17 (nominally 510kOhm +-1%), default 1+ 1 000 000/500 000 = 2.960784314
        float voltageDividerRatio = 2.960784314;

        //reference Voltage for calculations
        float referenceVoltage = 2.40;

        //refernce voltage setting of ADC
        //uint8_t adcReferenceMode = AR_INTERNAL2V4;
        uint8_t adcReferenceMode = AR_INTERNAL_2_4;
};

#endif