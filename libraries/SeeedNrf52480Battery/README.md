# SeeedNrf52480Battery

Arduino library for reading voltage of the Seeed Studio NRF52480 BLE (Sense) battery and set the charge current

This library allows to use the integrated battery voltage monitor with ease.

# Usage

#### Initialize 

```
#include <Nrf52Bat.h>

Nrf52Bat battery = Nrf52Bat();
```
#### Read battery voltage, percentage* and charge status

*percentage is currently only a very rough linear estimation

```
float voltage = battery.getVoltage();
float percentage = battery.getPercentage();
bool charging = battery.isCharging();
```

#### Set Battery Parameters I - charge current

you can select high and low charge current, which is best depends on the capacity and charge rating of your battery and use case

```
battery.setChargeCurrent100mA();
battery.setChargeCurrent50mA();
```

#### Set Battery Parameters II - minimum and maximum voltage refernce

**Important note: these setting are purely there for the battery level calculation, they do not change the behaviour of charging and discharging**

```
//set the voltage where the battery is considered empty/0% to 3.1V, note the battery charge ic (BQ25101) will lock out at 3.3V (typ)
battery.setMinVoltage(3.1)
//set the voltage that will be considered 100% to 4.3V, note: the battery charge IC (BQ25101) will charge the battery up to 4.2V (typ)
battery.setMaxVoltage(4.3);
//
```

### See more
For a complete Example, see examples/Nrf52Bat.ino

