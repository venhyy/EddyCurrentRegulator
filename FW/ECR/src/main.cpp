#include <Arduino.h>
#include <frequency_counter_PCI.h>
#include <ArduinoJson.h>

unsigned long freq;
String inData;
StaticJsonDocument<200> doc;

void setup()
{

  pinMode(PIN_PD3, INPUT);
  pinMode(PIN_PB6, OUTPUT);
  pinMode(PIN_PD6, OUTPUT);
  pinMode(PIN_PB7, INPUT);
  Serial.begin(9600);
}

void loop()
{

  freq = count_frequency(PIN_PD3);

  if (Serial.available() > 0)
  {

    inData = Serial.readStringUntil('\n');
    Serial.println("data: " + inData);

    DeserializationError error = deserializeJson(doc, inData);

    // Test if parsing succeeds.
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else
    {
      long time = doc["time"];
      Serial.println(time);
    }

    inData = "";
  }
}
