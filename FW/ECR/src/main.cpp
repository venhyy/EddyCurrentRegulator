//EddyCurrentRegulator
//FW REV: 1.0

#include <Arduino.h>
#include <frequency_counter_PCI.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <ezOutput.h> // ezOutput library
#include <PWM.h>
#include <TimerFreeTone.h>

#define FW_REV 10

#define BTN PIN_PB7
#define STATUS_LED PIN_PB6
#define PWM_OUT_1 PIN_PD6
#define PWM_OUT_2 PIN_PD5
#define SIG_IN_CH1 PIN_PD3
#define SIG_IN_CH2 PIN_PD2

#define DEFAULT_STATE 255
#define SHUTDOWN_TIME_TOTAL (180000UL)

unsigned long freq;
String inData;
StaticJsonDocument<200> doc;

//Define Variables we'll be connecting to
double setpoint = 3333;
double process_value, output;

//Specify the links and initial tuning parameters
double kp = 2, ki = 5, kd = 0;
PID myPID(&process_value, &output, &setpoint, kp, ki, kd, REVERSE);

bool pid_enabled = false;
bool shutdown = false;

int buttonState = LOW; //this variable tracks the state of the button, low if not pressed, high if pressed

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 500;  // the debounce time; increase if the output flickers
unsigned long shutdown_time = 0;

unsigned long led_freq = 0;
unsigned long period;

void setup()
{
  pinMode(BTN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PWM_OUT_1, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{

  buttonState = digitalRead(BTN);

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

      digitalWrite(STATUS_LED, HIGH);
      pid_enabled = false;
      shutdown = true;
      shutdown_time = millis();
      lastDebounceTime = millis();
    }
  }

  process_value = count_frequency(SIG_IN_CH1);

  if (pid_enabled)
  {
    myPID.Compute();
  }

  else if (shutdown == true && millis() - shutdown_time < SHUTDOWN_TIME_TOTAL)
  {
    output = 0;
  }
  else
  {
    output = DEFAULT_STATE;
  }

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
      process_value = doc["process_value"];

      myPID.SetTunings(kp, ki, kd);
      Serial.printf("%d,%d,%d,%d,%d\n", (int)kp, (int)ki, (int)kd, (int)setpoint, (int)output);
    }

    inData = "";
  }

  Serial.printf("%d,%d,%d,%d,%d,%d,%lu,%lu\n", (int)kp, (int)ki, (int)kd, (int)setpoint, (int)process_value, (int)output, period, led_freq);
}
