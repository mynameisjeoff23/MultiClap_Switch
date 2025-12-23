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

#ifdef ARDUINO_ARCH_ESP32
  constexpr int AUDIO_PIN = 15;
#else
  constexpr int AUDIO_PIN = A0;
#endif

constexpr int TIME_WINDOW =             2000;       // time window to detect multiple claps (ms)
constexpr int CLAP_DEBOUNCE =           30;         // time for a clap to fade (ms)
constexpr float PERCENT_ABOVE_AVERAGE = 30/100.0f;  // threshold percentage above ema
constexpr double ALPHA =                0.0000833;  // smoothing factor for ema                 
double ema =                            0.0f;       // ~ one second of ema at 12kHz sample rate
IRsend irsend;

unsigned long clapBegin =               millis();
bool clapDetected =                     false;
bool ledToggle =                        false;
int clapCount =                         0;
long accumulator =                      0;

#ifdef DEBUG_ENABLE
constexpr int PRINT_INTERVAL =          200;           // in ms
constexpr int COUNT_INTERVAL =          10000;         // in ms
unsigned long lastPrint =               clapBegin;
unsigned long lastCountPrint =          clapBegin;
unsigned long cycles =                  0;
#endif

// put function declarations here:

void EMAInit() {
  //set ema to real average
  uint64_t total = 0;
  constexpr int SAMPLES = 10000;

  for (int i = 0; i < SAMPLES; i++)
  {
    total += analogRead(AUDIO_PIN);
  }
  ema = total / (double)SAMPLES;
}

void turnOnFan() {
  uint16_t fanAddress = 0xFF00;
  uint32_t power =      0x15;
  uint32_t timer =      0x09;

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
  EMAInit();
}

void loop() {

  uint16_t sensorValue = analogRead(AUDIO_PIN);
  uint16_t originalValue = sensorValue;
  
  ema += ALPHA * (sensorValue - ema);                       // exponential moving average
  const int THRESHOLD = ema * (1 + PERCENT_ABOVE_AVERAGE);  // dynamic threshold based on ema

  #ifdef DEBUG_ENABLE
  unsigned long currentTime = millis();
  cycles++;
  if (currentTime - lastCountPrint > COUNT_INTERVAL) {                // print cycles every COUNT_INTERVAL ms
    lastCountPrint = currentTime;
    Serial.print("Cycles in last ");
    Serial.print(COUNT_INTERVAL/1000);
    Serial.print(" seconds: ");
    Serial.println(cycles);
    cycles = 0;
  }
  if (currentTime - lastPrint > PRINT_INTERVAL) {                // print analog audio strength every PRINT_INTERVAL ms
    lastPrint = currentTime;
    Serial.print("Audio Pin Value: ");
    Serial.println(sensorValue);
    Serial.print("EMA: ");
    Serial.println(ema);
  }
  #endif

  #ifdef SERIAL_TOGGLE_ENABLE
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    if (command == "q") {
      toggle();
    } 
  }
  #endif


  if (!clapDetected)
  {
    if (sensorValue > THRESHOLD) // possible clap detected
    {
      clapBegin = millis();
      
      delay(CLAP_DEBOUNCE);
      sensorValue = analogRead(AUDIO_PIN);
      if (sensorValue > THRESHOLD){                 // sound source is not a clap
        clapDetected = false;
        clapCount = 0;
      }
      else {
        clapDetected = true;
        clapCount++;
        #ifdef DEBUG_ENABLE
        Serial.println("Clap detected!");
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
        
        delay(CLAP_DEBOUNCE);
        sensorValue = analogRead(AUDIO_PIN);
        if (sensorValue > THRESHOLD){                 // clap debounce
          clapDetected = false;
          clapCount = 0;
        }
        else {
          clapCount++;
          #ifdef DEBUG_ENABLE
          Serial.println(originalValue);
          #endif
        }
      }
    }
    else {
      #ifdef DEBUG_ENABLE
      Serial.print("Total claps detected: ");
      Serial.println(clapCount);
      #endif
      clapDetected = false;
      
      if (clapCount > 1) {
        toggle();
      }
      clapCount = 0;
    }
  }
}