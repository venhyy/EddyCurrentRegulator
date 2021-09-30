#include <Arduino.h>
#include <frequency_counter_PCI.h>
#include <ArduinoJson.h>
#include <PID_v1.h>

#define BTN PIN_PB7
#define STATUS_LED PIN_PB6
#define PWM_OUT_1 PIN_PD6
#define PWM_OUT_2 PIN_PD5
#define SIG_IN_CH1 PIN_PD3
#define SIG_IN_CH2 PIN_PD2

#define DEFAULT_STATE 125

unsigned long freq;
String inData;
StaticJsonDocument<200> doc;

//Define Variables we'll be connecting to
double setpoint, process_value, output;

//Specify the links and initial tuning parameters
double kp = 2, ki = 5, kd = 0;
PID myPID(&process_value, &output, &setpoint, kp, ki, kd, REVERSE);

bool pid_enabled = false;

int buttonState = LOW; //this variable tracks the state of the button, low if not pressed, high if pressed
int ledState = -1;     //this variable tracks the state of the LED, negative if off, positive if on

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 100;  // the debounce time; increase if the output flickers

void setup()
{

  pinMode(BTN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PWM_OUT_1, OUTPUT);

  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{

  buttonState = digitalRead(BTN);

  freq = count_frequency(SIG_IN_CH1);
  if (pid_enabled)
  {
    myPID.Compute();
  }
  else
    output = DEFAULT_STATE;

  Serial.printf("%d,%d,%d,%d,%d,%d\n", (int)kp, (int)ki, (int)kd, (int)setpoint, (int)process_value, (int)output);

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
      lastDebounceTime = millis();
    }
  }

  if (Serial.available() > 0)
  {

    inData = Serial.readStringUntil('\n');
    Serial.println("data: " + inData);

    DeserializationError error = deserializeJson(doc, inData);

    // Test if parsing succeeds.
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
}
