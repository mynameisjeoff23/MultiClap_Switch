#include <Arduino.h>

constexpr int THRESHOLD = 3000;                // 12 bit adc 0-4095
constexpr int TIME_WINDOW = 2000;              // time window to detect multiple claps (ms)
constexpr int AUDIO_PIN = 15;
constexpr int LED_PIN = 22;
unsigned long clapBegin = millis();
bool clapDetected = false;
bool ledToggle = false;
int clapCount = 0;

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(AUDIO_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(AUDIO_PIN);
  if (!clapDetected){

    if (sensorValue > THRESHOLD){
      clapBegin = millis();
      clapDetected = true;
      clapCount++;
      Serial.println("Clap detected!");

      delay(50);
      sensorValue = analogRead(AUDIO_PIN);
      if (sensorValue > THRESHOLD){                 // sound source is not a clap
        clapDetected = false;
        clapCount = 0;
      }
    }

  }
  // read for 2s at 50ms intervals to see 
  // if continuous sound or if there are dips.
  // if there are dips in sound, it may be a clap.
  else {
    if (millis() - clapBegin < TIME_WINDOW){
      delay(50);
      int sensorValue = analogRead(AUDIO_PIN);
      if (sensorValue > THRESHOLD){
        
        clapCount++;
        delay(50);
        sensorValue = analogRead(AUDIO_PIN);

        if (sensorValue > THRESHOLD){                 // clap debounce
          clapDetected = false;
          clapCount = 0;
        }
      }
    }
    else {
      Serial.print("Total claps detected: ");
      Serial.println(clapCount);
      clapDetected = false;
      clapCount = 0;
      ledToggle = ledToggle ? false : true;
      Serial.print("Led On: ");
      Serial.println(ledToggle ? "True" : "False");
      digitalWrite(LED_PIN, ledToggle ? HIGH : LOW);
    }
  }
}