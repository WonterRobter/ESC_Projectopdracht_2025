/* === Bronnenlijst ===
Arduino documentatie:
digitalRead() functie: https://docs.arduino.cc/language-reference/en/functions/digital-io/digitalread/ (05/10/2025)
millis() functie: https://docs.arduino.cc/language-reference/en/functions/time/millis/ (05/10/2025)
Blink Without Delay voorbeeld: https://docs.arduino.cc/built-in-examples/digital/BlinkWithoutDelay/ (05/10/2025)
String startsWith() voorbeeld: https://docs.arduino.cc/built-in-examples/strings/StringStartsWithEndsWith/ (05/10/2025)

ESP32 libraries:
toneESP32 library: https://docs.arduino.cc/libraries/toneesp32/ (01/10/2025)
PWM aansturing op ESP32: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/ (05/10/2025)

Hardware tutorials:
Sirene met buzzer: https://docs.sunfounder.com/projects/beginners-lab-kit/en/latest/21_siren.html (05/10/2025)
DHT sensor library (Adafruit): https://github.com/adafruit/DHT-sensor-library (05/10/2025)

Kleur en licht:
RGB kleurmenging uitleg: https://www.physicsclassroom.com/interactive/light-waves-and-colors/rgb-color-addition/launch (05/10/2025)

Seriële debug en inputverwerking:
Seriële input verwerken via monitor: https://www.circuitbasics.com/how-to-read-user-input-from-the-arduino-serial-monitor/ (05/10/2025)

AI-assistent: voor opmaak en error searching
Microsoft Copilot: https://copilot.github.com/ (05/10/2025)

*/

#include <Arduino.h>
#include "config.h"
#include "DHT.h"

DHT dht(DHT_PIN, DHTTYPE);

// Tijdregistratie
unsigned long previousMillis = 0;
unsigned long lastRedBlinkTime = 0;
unsigned long lastBluePhaseTime = 0;
unsigned long lastToneStepTime = 0;
unsigned long lastDebounceTime = 0;

// Statusvariabelen
bool redBlinkState = false;
bool bluePhaseState = true;
bool alarmDisabled = false;
bool wasBelowThreshold = false;

bool lastButtonState = HIGH;
const unsigned long debounceDelay = 50;

int sirenFreq = 500;
int sirenDir = 1;

// Debugmodus
bool debugMode = false;        // start in debugmodus (true=aan false=uit)
float debugTemp = DEFAULT_DEBUG_TEMP;
float debugHum = DEFAULT_DEBUG_HUM;

// Seriële debugfunctie
void handleSerialDebug() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("T=")) {
      debugTemp = input.substring(2).toFloat();
      debugMode = true;
      Serial.print("[DEBUG] Temperatuur ingesteld op: ");
      Serial.println(debugTemp);
    } else if (input.startsWith("H=")) {
      debugHum = input.substring(2).toFloat();
      debugMode = true;
      Serial.print("[DEBUG] Vochtigheid ingesteld op: ");
      Serial.println(debugHum);
    } else if (input == "LIVE") {
      debugMode = false;
      Serial.println("[DEBUG] Live sensorwaarden actief");
    }
  }
}

void stopBuzzer() {
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

// RGB-LED aansturen (common anode: HIGH = uit, LOW = aan)
void setRgb(bool r, bool g, bool b) {
  digitalWrite(RED_PIN, r ? LOW : HIGH);
  digitalWrite(GREEN_PIN, g ? LOW : HIGH);
  digitalWrite(BLUE_PIN, b ? LOW : HIGH);
}

// Sensorwaarden naar seriële monitor printen
void printData(float temp, float hum) {
  Serial.print("Temperatuur: ");
  Serial.print(isnan(temp) ? "n/a" : String(temp, 1));
  Serial.print(" °C  |  Vochtigheid: ");
  Serial.print(isnan(hum) ? "n/a" : String(hum, 1));
  Serial.println(" %");
}

// Tornado-sirene (frequentie sweep)
void tornadoSiren() {
  unsigned long now = millis();

  if (now - lastToneStepTime >= 8) {
    ledcWriteTone(BUZZER_CHANNEL, sirenFreq);
    sirenFreq += sirenDir * 5;

    if (sirenFreq >= 2000) {
      sirenFreq = 2000;
      sirenDir = -1;
    }
    if (sirenFreq <= 500) {
      sirenFreq = 500;
      sirenDir = 1;
    }

    lastToneStepTime = now;
  }
}

// Rood knipperen bij alarm
void blinkRed(unsigned long interval) {
  unsigned long currentMillis = millis();

  if (currentMillis - lastRedBlinkTime >= interval) {
    lastRedBlinkTime = currentMillis;

    if (redBlinkState == false) {
      redBlinkState = true;
      setRgb(1, 0, 0); // rood aan
    } else {
      redBlinkState = false;
      setRgb(0, 0, 0); // alles uit
    }
  }
}

// Blauw knipperen bij < 0°C
void blinkBluePhase(unsigned long onTimeMs, unsigned long offTimeMs) {
  unsigned long now = millis();
  unsigned long interval = bluePhaseState ? onTimeMs : offTimeMs;

  if (now - lastBluePhaseTime >= interval) {
    bluePhaseState = !bluePhaseState;
    lastBluePhaseTime = now;
  }

  setRgb(0, 0, bluePhaseState ? 1 : 0);
}

// Leest sensor elke READ_INTERVAL en print waardes wanneer nodig
void sensorTask(unsigned long now) {
  if (now - previousMillis >= READ_INTERVAL) {
    previousMillis = now;

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float temp = isnan(t) || debugMode ? debugTemp : t;
    float hum = isnan(h) || debugMode ? debugHum : h;

    printData(temp, hum);
  }
}

// Lees temperatuur voor beslislogica (houdt gedrag van origineel aan - kan sensor opnieuw uitlezen)
float readTempForDecision() {
  float t = dht.readTemperature();
  float temp = isnan(t) || debugMode ? debugTemp : t;
  return temp;
}

// Verwerk knop- en alarm in-/uitschakel-logica
void checkButton(float temp) {
  // Knop direct controleren (actief LOW)
  if (digitalRead(BUTTON_PIN) == LOW && !alarmDisabled && temp >= 50) {
    alarmDisabled = true;
    wasBelowThreshold = false;
    Serial.println("[ALARM] Alarm handmatig uitgeschakeld");
  }

  // Resetlogica: als temperatuur onder 50°C zakt, reset de blokkering
  if (temp < 50) {
    wasBelowThreshold = true;
  }

  if (wasBelowThreshold && temp >= 50 && alarmDisabled) {
    alarmDisabled = false;
    wasBelowThreshold = false;
    Serial.println("[ALARM] Alarm opnieuw geactiveerd");
  }
}

// Bepaal outputs (LED + buzzer) op basis van temperatuur
void outputsTask(float temp) {
  if (isnan(temp)) {
    setRgb(COLOR_OFF);
    stopBuzzer();
  } else if (temp >= 50 && !alarmDisabled) {
    blinkRed(200);
    tornadoSiren();
  } else if (temp >= 40) {
    setRgb(COLOR_RED);
    stopBuzzer();
  } else if (temp >= 20) {
    setRgb(COLOR_GREEN);
    stopBuzzer();
  } else if (temp >= 10) {
    setRgb(COLOR_CYAN);
    stopBuzzer();
  } else if (temp >= 0) {
    setRgb(COLOR_BLUE);
    stopBuzzer();
  } else {
    blinkBluePhase(1000, 500);
    stopBuzzer();
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // knop schakelt naar GND

  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  ledcSetup(BUZZER_CHANNEL, BUZZER_BASE_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
}

void loop() {
  // Kort overzichtelijke loop: seriële input, periodieke sensors, knoppen en outputs
  handleSerialDebug();
  unsigned long now = millis();

  // Sensor lezen en printen als READ_INTERVAL verstreken is
  sensorTask(now);

  // Temperatuur voor beslislogica ophalen
  float temp = readTempForDecision();

  // Knop en alarmlogica
  checkButton(temp);

  // LED en buzzer gedrag
  outputsTask(temp);
}