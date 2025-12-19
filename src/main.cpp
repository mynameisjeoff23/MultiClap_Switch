#ifdef ARDUINO_ARCH_ESP32
  #define IR_SEND_PIN 22                    // can be anything on esp32
#elif defined(__AVR_ATmega328P__)
  #define IR_SEND_PIN 3                     // default IR send pin for Arduino Uno
#elif defined(__AVR_ATmega2560__)
  #define IR_SEND_PIN 9                     // default IR send pin for Arduino Mega
#endif

#include <Arduino.h>
#include <IRremote.hpp>

/*Note to self:
  Fan: 32 bits Address: FF00
    Power: 0xEA15FF00 or 0x15
    Timer: 0xF609FF00 or 0x09

  Led: 32 bits Address: EF00
    Off: 0xFD02EF00 or 0x02
    On: 0xFC03EF00 or 0x03
*/

#define DEBUG_ENABLE
#define SERIAL_TOGGLE_ENABLE
#define DYNAMIC_THRESHOLD_ENABLE

#ifdef ARDUINO_ARCH_ESP32
  int THRESHOLD = 2500;                             // 12 bit adc 0-4095
  constexpr int AUDIO_PIN = 15;
#else
  int THRESHOLD = 500;                              // 10 bit adc 0-1023
  constexpr int AUDIO_PIN = A0;
#endif

constexpr int TIME_WINDOW = 2000;                   // time window to detect multiple claps (ms)
//constexpr int LED_PIN = 1;
IRsend irsend;

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

void turnOnFan() {
  uint16_t fanAddress = 0xFF00;
  uint32_t power =    0x15;
  uint32_t timer =    0x09;
  irsend.sendNEC(fanAddress, power, 0);
  delay(200);
  irsend.sendNEC(fanAddress, timer, 0);
  delay(200);
}

void toggle() {
  ledToggle = ledToggle ? false : true;
  Serial.println("Fan Toggled");
  //digitalWrite(LED_PIN, ledToggle ? HIGH : LOW);            // IRRemote library uses LED_BUILTIN for feedback
  turnOnFan();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(AUDIO_PIN, INPUT);
  //pinMode(LED_PIN, OUTPUT);
  irsend.begin();
  Serial.begin(115200);

}

void loop() {

  #ifdef DEBUG_ENABLE
  if (millis() - lastPrint > PRINT_INTERVAL) {                // print analog audio strength every PRINT_INTERVAL ms
    lastPrint = millis();
    uint16_t sensorValue = analogRead(AUDIO_PIN);
    Serial.print("Audio Pin Value: ");
    Serial.println(sensorValue);
  }
  #endif

  #ifdef DYNAMIC_THRESHOLD_ENABLE
  if (Serial.available()){                                    // dynamic threshold changing
    String command = Serial.readStringUntil('\n');

    #ifdef SERIAL_TOGGLE_ENABLE
    if (command == "q") {
      toggle();
    } 
    else {
      int newThreshold = command.toInt();
      if (newThreshold > 0 && newThreshold <= 4095){
        Serial.print("Setting new threshold to: ");
        Serial.println(newThreshold);
        THRESHOLD = newThreshold;
      }
    }
    #else
    
    int newThreshold = command.toInt();
    //TODO: change 4095 to be platform specific
    #ifdef ARDUINO_ARCH_ESP32
      if (newThreshold > 0 && newThreshold <= 4095){
    #else
      if (newThreshold > 0 && newThreshold <= 1023){
    #endif
        Serial.print("Setting new threshold to: ");
        Serial.println(newThreshold);
        THRESHOLD = newThreshold;
      }
    }
    #endif
  }
  #endif


  uint16_t sensorValue = analogRead(AUDIO_PIN);
  uint16_t originalValue = sensorValue;
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
        #ifdef DEBUG_ENABLE
        Serial.println(originalValue);
        #endif
      }
    }

  }
  // read for 2s to see 
  // if continuous sound or if there are dips.
  // if there are dips in sound, it may be a clap.
  else {
    if (millis() - clapBegin < TIME_WINDOW){
      int sensorValue = analogRead(AUDIO_PIN);
      int originalValue = sensorValue;
      if (sensorValue > THRESHOLD){
        
        clapCount++;
        delay(100);
        sensorValue = analogRead(AUDIO_PIN);

        if (sensorValue > THRESHOLD){                 // clap debounce
          clapDetected = false;
          clapCount = 0;
        }
        #ifdef DEBUG_ENABLE
        else {
          Serial.println(originalValue);
        }
        #endif
      }
    }
    else {
      Serial.print("Total claps detected: ");
      Serial.println(clapCount);
      clapDetected = false;
      
      if (clapCount > 1) {
        toggle();
      }
      clapCount = 0;
    }
  }
}