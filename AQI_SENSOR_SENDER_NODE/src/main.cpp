#include <Wire.h>
#include <Tomoto_HM330X.h>

Tomoto_HM330X sensor;

// Tomoto_HM330X sensor(Wire1); // to use the alternative wire

unsigned long previousMillis = 0;
const long interval = 5000; // Interval at which to read the sensor (milliseconds)


void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Specify the I2C pins

  if (!sensor.begin()) {
    Serial.println("Failed to initialize HM330X");
    while (1);
  }
  Serial.println("HM330X initialized");
  Serial.println();
}

void printValue(const char* label, int value) {
  Serial.print(label);
  Serial.print(": ");
  Serial.println(value);
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

      Serial.println();
    }
  }
}
