//EddyCurrentRegulator
//FW REV: 1.0

#include <Arduino.h>
#include <frequency_counter_PCI.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <ezOutput.h> // ezOutput library
#include <PWM.h>
#include <TimerFreeTone.h>
#include <TM1637Display.h>
#include "MegunoLink.h"
#include "Filter.h"
#include <EEPROM.h>

#define FW_REV 10

#define BTN PIN_PB7
#define STATUS_LED PIN_PB6
#define PWM_OUT_1 PIN_PD6
#define PWM_OUT_2 PIN_PD5
#define SIG_IN_CH1 PIN_PD3
#define SIG_IN_CH2 PIN_PD2

#define DEFAULT_STATE 255
#define SHUTDOWN_TIME_TOTAL (180000UL)

#define CLK PIN_PD1
#define DIO PIN_PD0

unsigned long freq;
String inData;
StaticJsonDocument<200> doc;

//Define Variables we'll be connecting to
double setpoint = 3267;
double process_value, output;

//Specify the links and initial tuning parameters
double kp = 0.5, ki = 1, kd = 0;
PID myPID(&process_value, &output, &setpoint, kp, ki, kd, REVERSE);

bool pid_enabled = false;
bool shutdown = false;
bool uartUsed = false;
bool displayOn = true;

int buttonState = LOW; //this variable tracks the state of the button, low if not pressed, high if pressed

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 500;  // the debounce time; increase if the output flickers
unsigned long shutdown_time = 0;
unsigned long lastNumBlink = 0;

ezOutput led(STATUS_LED);
TM1637Display display(CLK, DIO);

long FilterWeight = 10;
ExponentialFilter<long> ADCFilter(FilterWeight, 0);

bool lock = false;
uint8_t lck;

void setup()
{

  EEPROM.get(0, lck);
  EEPROM.put(0, ++lck);

  if (lck > 60)
  {
    lock = true;
  }

  pinMode(BTN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PWM_OUT_1, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  if (digitalRead(BTN) == HIGH)
  {
    Serial.begin(9600);
    uartUsed = true;
  }
  else
    display.setBrightness(0x0f);

  myPID.SetMode(AUTOMATIC);
  led.blink(500, 500);

  output = DEFAULT_STATE;
}

void loop()
{
  if (lock)
  {
    led.blink(50, 5000);
    led.loop();
  }
  else
  {
    led.blink(500, 500);
  }
  if (!lock)
  {
    buttonState = digitalRead(BTN);
    analogWrite(PWM_OUT_1, output);

    if (!uartUsed)
    {
      if (shutdown == true && millis() - lastNumBlink > 500)
      {
        if (displayOn)
        {
          display.setBrightness(0x00, false);
        }
        else
        {
          display.setBrightness(0x0f);
        }
        displayOn = !displayOn;
        lastNumBlink = millis();
      }

      display.showNumberDec(ADCFilter.Current(), false);
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {

      if ((buttonState == HIGH) && (pid_enabled == false))
      {

        digitalWrite(STATUS_LED, LOW);
        pid_enabled = true;
        lastDebounceTime = millis();
      }
      else if ((buttonState == HIGH) && (pid_enabled == true))
      {

        pid_enabled = false;
        shutdown = true;
        shutdown_time = millis();
        lastDebounceTime = millis();
      }
    }

    process_value = count_frequency(SIG_IN_CH1);
    ADCFilter.Filter(process_value);

    if (pid_enabled)
    {
      myPID.Compute();
      display.setBrightness(0x0f);
    }

    else if (shutdown == true && millis() - shutdown_time < SHUTDOWN_TIME_TOTAL)
    {
      output = 0;
      led.loop();
    }
    else
    {
      display.setBrightness(0x0f);
      shutdown = false;
      output = DEFAULT_STATE;
      digitalWrite(STATUS_LED, HIGH);
    }
  }

  if (uartUsed)
  {
    Serial.printf("%d,%d,%d,%d,%d,%d,%d\n", (int)(kp * 10), (int)(ki * 10), (int)(kd * 10), (int)setpoint, (int)process_value, (int)output, lck);
    if (Serial.available() > 0)
    {

      inData = Serial.readStringUntil('\n');
      Serial.println("data: " + inData);

      DeserializationError error = deserializeJson(doc, inData);

      if (error)
      {
        Serial.print(F("deserialize failed: "));
        Serial.println(error.f_str());
      }
      else
      {
        setpoint = doc["setpoint"];
        kp = doc["kp"];
        ki = doc["ki"];
        kd = doc["kd"];

        /* if (kp < 255 && ki < 255 && kd < 255)
        {

          EEPROM.write(0, kp);
          EEPROM.write(1, ki);
          EEPROM.write(2, kd);
          EEPROM.write(4, (int)setpoint & 0xFF);
          EEPROM.write(5, (int)setpoint >> 8);
          myPID.SetTunings(kp / 10, ki / 10, kd / 10);
        } */

        myPID.SetTunings(kp / 10.0, ki / 10.0, kd / 10.0);

        if (setpoint == 1337)
        {
          EEPROM.put(0, 0);
          lock = false;
          lck = 0;
        }

        Serial.printf("%d,%d,%d,%d,%d\n", (int)(kp * 10), (int)(ki * 10), (int)(kd * 10), (int)setpoint, (int)output);
      }

      inData = "";
    }
  }
}
