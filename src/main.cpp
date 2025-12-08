#include <Arduino.h>

//#define DEBUG_ENABLE
#define DYNAMIC_THRESHOLD_ENABLE

int THRESHOLD = 2500;                           // 12 bit adc 0-4095
constexpr int TIME_WINDOW = 2000;               // time window to detect multiple claps (ms)
constexpr int AUDIO_PIN = 15;
constexpr int LED_PIN = 22;
unsigned long clapBegin = millis();
bool clapDetected = false;
bool ledToggle = false;
int clapCount = 0;

#ifdef DEBUG_ENABLE
// audio print timer for debugging
// for now, every 200ms
constexpr int PRINT_INTERVAL = 200;           // in ms
unsigned long lastPrint = millis();
#endif

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(AUDIO_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {

  #ifdef DEBUG_ENABLE
  if (millis() - lastPrint > PRINT_INTERVAL) {      // print analog audio strength every PRINT_INTERVAL ms
    lastPrint = millis();
    uint16_t sensorValue = analogRead(AUDIO_PIN);
    Serial.print("Audio Pin Value: ");
    Serial.println(sensorValue);
  }
  #endif

  #ifdef DYNAMIC_THRESHOLD_ENABLE
  if (Serial.available()){                            // dynamic threshold changing
    String command = Serial.readStringUntil('\n');
    int newThreshold = command.toInt();
    if (newThreshold > 0 && newThreshold <= 4095){
      Serial.print("Setting new threshold to: ");
      Serial.println(newThreshold);
      THRESHOLD = newThreshold;
    }
  }
  #endif


  uint16_t sensorValue = analogRead(AUDIO_PIN);
  if (!clapDetected){

    if (sensorValue > THRESHOLD){
      clapBegin = millis();
      clapDetected = true;
      clapCount++;
      
      delay(100);
      sensorValue = analogRead(AUDIO_PIN);
      if (sensorValue > THRESHOLD){                 // sound source is not a clap
        clapDetected = false;
        clapCount = 0;
      }
      else {
        Serial.println("Clap detected!");
      }
    }

  }
  // read for 2s to see 
  // if continuous sound or if there are dips.
  // if there are dips in sound, it may be a clap.
  else {
    if (millis() - clapBegin < TIME_WINDOW){
      int sensorValue = analogRead(AUDIO_PIN);
      if (sensorValue > THRESHOLD){
        
        clapCount++;
        delay(100);
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
      
      if (clapCount > 1) {
        ledToggle = ledToggle ? false : true;
        Serial.print("Led On: ");
        Serial.println(ledToggle ? "True" : "False");
        digitalWrite(LED_PIN, ledToggle ? HIGH : LOW);
      }
      clapCount = 0;
    }
  }
}