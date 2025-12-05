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

DEEL2: Bronnen
PubSubClient Documentatie:
https://registry.platformio.org/libraries/knolleary/PubSubClient (05/12/2025)
https://github.com/knolleary/pubsubclient (05/12/2025)
  https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/ (05/12/2025)


https://docs.platformio.org/en/latest/librarymanager/

WiFi Connectie (Espressif): https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi

=== Einde bronnenlijst ===
*/

#include <Arduino.h>
#include <WiFi.h>           
#include <PubSubClient.h>   
#include <DHT.h>            
#include "secrets.h"        // Bevat netwerkgegevens
#include "config.h"         // Bevat hardware pins

// --- Objecten Initialisatie ---
DHT dht(DHT_PIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// --- Tijdregistratie ---
unsigned long previousMillis = 0;
unsigned long lastRedBlinkTime = 0;
unsigned long lastBluePhaseTime = 0;
unsigned long lastToneStepTime = 0;
unsigned long lastDebounceTime = 0;

// --- Statusvariabelen ---
bool redBlinkState = false;
bool bluePhaseState = true;
bool alarmDisabled = false;
bool wasBelowThreshold = false;

bool lastButtonState = HIGH; // PULLUP: default HIGH
int sirenFreq = 500;
int sirenDir = 1;

// Zet dit bij je statusvariabelen
bool forceLedsOff = false; // false = AUTO, true = ALLES UIT

// --- Debugmodus ---
bool debugMode = false;        // start in debugmodus (true=aan false=uit)
float debugTemp = DEFAULT_DEBUG_TEMP;
float debugHum = DEFAULT_DEBUG_HUM;

// ==========================================
//               WIFI & MQTT FUNCTIES
// ==========================================

void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Verbinden met ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi verbonden");
  Serial.print("IP adres: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop totdat we verbonden zijn
  while (!client.connected()) {
    Serial.print("Proberen MQTT verbinding te maken...");
    // Probeer te verbinden met een unieke Client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("verbonden");
      // === HIER ABONNEREN WE OP DE TOPICS ===
      client.subscribe("esp32/leds");
      client.subscribe("esp32/alarm");

    } else {
      Serial.print("mislukt, rc=");
      Serial.print(client.state());
      Serial.println(" probeer opnieuw in 5 seconden");
      delay(5000); // Blocking delay bij reconnect (toegestaan in simpele setup)
    }
  }
}

void publishData(float temp, float hum) {
  // Converteer waarden naar String (of char array) voor MQTT
  char tempStr[8];
  char humStr[8];
  dtostrf(temp, 1, 2, tempStr);
  dtostrf(hum, 1, 2, humStr);

  client.publish("esp32/temperatuur", tempStr);
  client.publish("esp32/luchtvochtigheid", humStr);
  
  if(debugMode) {
    Serial.println("[MQTT] Data gepubliceerd");
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Bericht ontvangen op topic: ");
  Serial.print(topic);
  Serial.print(". Bericht: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Actie 1: LED Besturing (Topic: esp32/leds)
  if (String(topic) == "esp32/leds") {
    if(messageTemp == "OFF"){
      Serial.println("[REMOTE] LEDs geforceerd UIT");
      forceLedsOff = true;
    }
    else if(messageTemp == "AUTO"){
      Serial.println("[REMOTE] LEDs terug op AUTO");
      forceLedsOff = false;
    }
  }

  // Actie 2: Alarm Besturing (Topic: esp32/alarm)
  if (String(topic) == "esp32/alarm") {
    if(messageTemp == "OFF"){
      Serial.println("[REMOTE] Alarm uitgeschakeld via Dashboard");
      alarmDisabled = true; // We hergebruiken je bestaande variabele!
      wasBelowThreshold = false; // Reset de logica zodat hij niet meteen weer aanspringt
    }
  }
}

// ==========================================
//            BESTAANDE LOGICA (DEEL1)
// ==========================================


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

    // 1. Printen naar Serieel (lokaal)
    printData(temp, hum);

    // 2. Sturen naar MQTT (cloud/dashboard)
    if (client.connected()) {
        publishData(temp, hum);
    }
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
  if (!isnan(temp) && temp >= 50 && !alarmDisabled) {
    tornadoSiren();
  } else {
    // Als het veilig is (of alarm disabled), buzzer stil.
    stopBuzzer();
  }
  if (forceLedsOff) {
    setRgb(COLOR_OFF);
    return;
  }


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

// ==========================================
//               SETUP & LOOP
// ==========================================

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
  // Netwerk setup
  setupWifi();
  client.setServer(MQTT_SERVER, 1883); // Poort 1883 is standaard voor onbeveiligde MQTT
  client.setCallback(callback);
}

void loop() {
  // Controleer MQTT verbinding
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Houdt de MQTT client levend

  handleSerialDebug();
  unsigned long now = millis();

  sensorTask(now);                    // Sensor lezen en printen als READ_INTERVAL verstreken is
  float temp = readTempForDecision(); // Temperatuur voor beslislogica ophalen
  checkButton(temp);                  // Knop en alarmlogica
  outputsTask(temp);                  // LED en buzzer gedrag
}