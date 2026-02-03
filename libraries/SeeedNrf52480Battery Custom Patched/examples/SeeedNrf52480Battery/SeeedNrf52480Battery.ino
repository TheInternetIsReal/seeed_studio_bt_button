#include <SeeedNrf52480Battery.h>

SeeedNrf52480Battery battery = SeeedNrf52480Battery();
unsigned long lastChange = 0;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  Serial.println("Seeed Studio NRF52480 Battery Demo\n");
  battery.setChargeCurrent100mA();
  attachInterrupt(digitalPinToInterrupt(PIN_CHARGING_INV), chargeUpdate, CHANGE);
}

void chargeUpdate(){
  lastChange = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("\n\nVoltage:\t\t");
  Serial.println(battery.getVoltage());
  Serial.print("Percentage:\t\t");
  Serial.println(battery.getPercentage());
  Serial.print("Status: ");
  if(battery.isCharging()){
    Serial.print("charging since: ");
  }else{
    Serial.print("discharging since: ");
  }
  Serial.print(millis()-lastChange);
  Serial.println("ms");
  delay(1000);
}
