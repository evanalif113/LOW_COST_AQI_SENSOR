#include <Wire.h>
#include <Tomoto_HM330X.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

Tomoto_HM330X sensor;
BluetoothSerial SerialBT;
// Tomoto_HM330X sensor(Wire1); // to use the alternative wire

unsigned long previousMillis = 0;
const long interval = 5000; // Interval at which to read the sensor (milliseconds)



void setup() {
  Serial.begin(115200);
  SerialBT.begin("Air Quality Monitor"); // Bluetooth device name
  Wire.begin(21, 22); // Specify the I2C pins

  if (!sensor.begin()) {
    Serial.println("Failed to initialize HM330X");
    while (1);
  }
  Serial.println("HM330X initialized");
  Serial.println();
}

void printValue(const char* label, int value) {
  SerialBT.print(label);
  SerialBT.print(": ");
  SerialBT.println(value);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (!sensor.readSensor()) {
      Serial.println("Failed to read HM330X");
    } else {
      printValue("Sensor number", sensor.getSensorNumber());

      Serial.println("Concentration based on CF=1 standard particulate matter (ug/m^3) --");
      printValue("PM1.0", sensor.std.getPM1());
      printValue("PM2.5", sensor.std.getPM2_5());
      printValue("PM10", sensor.std.getPM10());

      Serial.println("Concentration based on atmospheric environment (ug/m^3) --");
      printValue("PM1.0", sensor.atm.getPM1());
      printValue("PM2.5", sensor.atm.getPM2_5());
      printValue("PM10", sensor.atm.getPM10());

      // Supported or not, depending on the sensor model
      Serial.println("Number of particles with diameter of (/0.1L) --");
      printValue(">=0.3um", sensor.count.get0_3());
      printValue(">=0.5um", sensor.count.get0_5());
      printValue(">=1.0um", sensor.count.get1());
      printValue(">=2.5um", sensor.count.get2_5());
      printValue(">=5.0um", sensor.count.get5());
      printValue(">=10um", sensor.count.get10());

      Serial.println();
    }
  }
}
