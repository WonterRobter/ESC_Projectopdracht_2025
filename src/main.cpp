/*
=== BRONNENLIJST ===

Deel 1: Basis & Hardware (Geraadpleegd rond 05/10/2025)
-------------------------------------------------------
1.  digitalRead() functie
    Bron: https://docs.arduino.cc/language-reference/en/functions/digital-io/digitalread/
    Datum: 05/10/2025
    Gebruikt voor: Het uitlezen van de fysieke drukknop.

2.  millis() functie
    Bron: https://docs.arduino.cc/language-reference/en/functions/time/millis/
    Datum: 05/10/2025
    Gebruikt voor: Timers maken zonder delay() te gebruiken

3.  Sirene met buzzer maken
    Bron: https://docs.sunfounder.com/projects/beginners-lab-kit/en/latest/21_siren.html
    Datum: 05/10/2025
    Gebruikt voor: Het idee voor de 'tornado' sirene (frequentie sweep).

4.  DHT sensor library (Adafruit)
    Bron: https://github.com/adafruit/DHT-sensor-library
    Datum: 05/10/2025
    Gebruikt voor: Aansturing van de DHT22 temperatuursensor.

5.  RGB kleurmenging & Licht
    Bron: https://www.physicsclassroom.com/interactive/light-waves-and-colors/rgb-color-addition/launch
    Datum: 05/10/2025
    Gebruikt voor: Begrijpen welke combinaties (R, G, B) welke kleur geven (Cyan, etc).

6.  Seriële input verwerken
    Bron: https://www.circuitbasics.com/how-to-read-user-input-from-the-arduino-serial-monitor/
    Datum: 05/10/2025
    Gebruikt voor: De debug commando's (zoals "LIVE" of "T=25") inlezen.

7.  Microsoft Copilot
    Bron: https://copilot.github.com/
    Datum: 05/10/2025
    Gebruikt voor: Hulp bij de initiële opzet en syntax foutjes zoeken in deel 1.


Deel 2: Connectiviteit & Dashboard (Geraadpleegd rond 05/12/2025)
-----------------------------------------------------------------
8.  PubSubClient Documentatie
    Bron: https://github.com/knolleary/pubsubclient
    Datum: 05/12/2025
    Gebruikt voor: De functies om berichten te sturen (publish) en ontvangen (subscribe) via MQTT.

9.  ESP32 MQTT Tutorial
    Bron: https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
    Datum: 05/12/2025
    Gebruikt voor: Uitleg over hoe je de 'callback' functie moet schrijven en de WiFi verbinding opzet.

10. WiFi Library (Espressif)
    Bron: https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi
    Datum: 05/12/2025
    Gebruikt voor: Verbinding maken met het lokale netwerk.

11. Node-RED Dashboard
    Bron: https://flows.nodered.org/node/node-red-dashboard
    Datum: 05/12/2025
    Gebruikt voor: Uitzoeken hoe de 'Button' node werkt en string payloads stuurt.

12. Google Gemini
    Datum: 05/12/2025
    Gebruikt voor: 
    - Oplossen van de bug waarbij de sensor "0.0" aangaf (foutafhandeling).
    - Advies over veiligheidslogica: zorgen dat het alarm altijd werkt, ook als LEDs uit staan.
    - Opschonen Bronnenlijst.

=== EINDE BRONNENLIJST ===

// Datum = laatst Geraadpleegd
*/

#include <Arduino.h>
#include <WiFi.h>           
#include <PubSubClient.h>   
#include <DHT.h>            
#include "secrets.h"        
#include "config.h"         

// Objecten aanmaken voor sensor en netwerk
DHT dht(DHT_PIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// Variabelen voor millis() timers (geen delay gebruiken!)
unsigned long previousMillis = 0;
unsigned long lastRedBlinkTime = 0;
unsigned long lastBluePhaseTime = 0;
unsigned long lastToneStepTime = 0;

// Status bijhouden
bool redBlinkState = false;
bool bluePhaseState = true;
bool alarmDisabled = false;      // Als ik op de knop druk, gaat het alarm uit
bool wasBelowThreshold = false;  // Nodig om te weten of de temp al gezakt is
bool forceLedsOff = false;       // Voor de 'stealth mode' via het dashboard

// Variabelen voor de sirene
int sirenFreq = 500;
int sirenDir = 1;

// Debug modus variabelen
bool debugMode = false;        
float debugTemp = DEFAULT_DEBUG_TEMP;
float debugHum = DEFAULT_DEBUG_HUM;

// ==========================================
//               WIFI & MQTT
// ==========================================

void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Verbinden met ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wachten tot er verbinding is
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi verbonden!");
  Serial.print("Mijn IP adres is: ");
  Serial.println(WiFi.localIP());
}

// Deze functie wordt uitgevoerd als Node-RED een bericht stuurt
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Bericht ontvangen op topic: ");
  Serial.print(topic);
  Serial.print(" | Inhoud: ");
  
  // Het bericht komt binnen als losse bytes, die zet ik hier om naar tekst
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // Actie 1: Knoppen voor de LEDs (Stealth mode)
  if (String(topic) == "esp32/leds") {
    if(messageTemp == "OFF"){
      Serial.println("[REMOTE] Ik zet de LEDs nu uit");
      forceLedsOff = true;
    }
    else if(messageTemp == "AUTO"){
      Serial.println("[REMOTE] LEDs gaan weer op automatisch");
      forceLedsOff = false;
    }
  }

  // Actie 2: Alarm uitzetten via dashboard
  if (String(topic) == "esp32/alarm") {
    if(messageTemp == "OFF"){
      Serial.println("[REMOTE] Alarm uitgezet via Dashboard knop");
      alarmDisabled = true; 
      wasBelowThreshold = false; // Resetten zodat hij niet meteen weer aan gaat
    }
  }
}

void reconnect() {
  // Blijven proberen tot we verbonden zijn met MQTT
  while (!client.connected()) {
    Serial.print("Verbinden met MQTT...");
    // Random ID genereren zodat de server niet in de war raakt
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("verbonden!");
      // Hier zeg ik tegen de server naar welke topics ik wil luisteren
      client.subscribe("esp32/leds");
      client.subscribe("esp32/alarm");
    } else {
      Serial.print("mislukt, code=");
      Serial.print(client.state());
      Serial.println(" ik probeer het over 5 sec opnieuw");
      delay(5000); 
    }
  }
}

void publishData(float temp, float hum) {
  // MQTT wil tekst (char array) hebben, geen getallen. Hier converteer ik dat.
  char tempStr[8];
  char humStr[8];
  dtostrf(temp, 1, 2, tempStr); // 1 cijfer voor de komma, 2 erna
  dtostrf(hum, 1, 2, humStr);

  client.publish("esp32/temperatuur", tempStr);
  client.publish("esp32/luchtvochtigheid", humStr);
}

// ==========================================
//            HARDWARE AANSTURING
// ==========================================

void setRgb(bool r, bool g, bool b) {
  // Common anode: LOW is aan
  digitalWrite(RED_PIN, r ? LOW : HIGH);
  digitalWrite(GREEN_PIN, g ? LOW : HIGH);
  digitalWrite(BLUE_PIN, b ? LOW : HIGH);
}

void stopBuzzer() {
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

// Functie voor het loeiende geluid (sweep)
void tornadoSiren() {
  unsigned long now = millis();
  if (now - lastToneStepTime >= 8) {
    ledcWriteTone(BUZZER_CHANNEL, sirenFreq);
    sirenFreq += sirenDir * 5;
    
    // Als frequentie te hoog of te laag wordt, draai ik de richting om
    if (sirenFreq >= 2000) { sirenFreq = 2000; sirenDir = -1; }
    if (sirenFreq <= 500) { sirenFreq = 500; sirenDir = 1; }
    
    lastToneStepTime = now;
  }
}

void blinkRed(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - lastRedBlinkTime >= interval) {
    lastRedBlinkTime = currentMillis;
    redBlinkState = !redBlinkState;
    if (redBlinkState) setRgb(1, 0, 0); 
    else setRgb(0, 0, 0); 
  }
}

void blinkBluePhase(unsigned long onTimeMs, unsigned long offTimeMs) {
  unsigned long now = millis();
  // Hier kijk ik of hij aan of uit staat om de interval te bepalen
  unsigned long interval = bluePhaseState ? onTimeMs : offTimeMs;
  
  if (now - lastBluePhaseTime >= interval) {
    bluePhaseState = !bluePhaseState;
    lastBluePhaseTime = now;
  }
  setRgb(0, 0, bluePhaseState ? 1 : 0);
}

// ==========================================
//             LOGICA & TAKEN
// ==========================================

void handleSerialDebug() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("T=")) {
      debugTemp = input.substring(2).toFloat();
      debugMode = true;
      Serial.printf("[DEBUG] Test temperatuur gezet op: %.1f\n", debugTemp);
    } else if (input == "LIVE") {
      debugMode = false;
      Serial.println("[DEBUG] Weer live sensoren gebruiken");
    }
  }
}

void sensorTask(unsigned long now) {
  if (now - previousMillis >= READ_INTERVAL) {
    previousMillis = now;

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    // Als ik aan het testen ben (debugMode), negeer ik de echte sensor
    float temp = debugMode ? debugTemp : t;
    float hum = debugMode ? debugHum : h;

    // Fix voor de "0.0 bug": Als de waarde NaN is, print ik 0.0 anders crasht de print
    Serial.printf("Temperatuur: %.1f °C | Vochtigheid: %.1f %%\n", isnan(temp) ? 0.0 : temp, isnan(hum) ? 0.0 : hum);

    if (client.connected()) {
        publishData(isnan(temp) ? 0.0 : temp, isnan(hum) ? 0.0 : hum);
    }
  }
}

// Helper om snel de juiste temp op te halen (echt of debug)
float getDecisionTemp() {
  float t = dht.readTemperature();
  return debugMode ? debugTemp : t;
}

void checkButton(float temp) {
  // De knop is LOW als ik hem indruk
  if (digitalRead(BUTTON_PIN) == LOW && !alarmDisabled && temp >= 50) {
    alarmDisabled = true;
    wasBelowThreshold = false;
    Serial.println("[ALARM] Ik heb het alarm handmatig uitgezet.");
  }
  
  // Als de temperatuur weer zakt, reset ik de blokkering
  if (temp < 50) {
    wasBelowThreshold = true;
  }
  
  // Als het daarna weer heet wordt, moet het alarm wel weer afgaan
  if (wasBelowThreshold && temp >= 50 && alarmDisabled) {
    alarmDisabled = false;
    wasBelowThreshold = false;
    Serial.println("[ALARM] Temperatuur stijgt weer, alarm gaat weer aan.");
  }
}

void outputsTask(float temp) {
  // 1. VEILIGHEID EERST
  // Het alarm moet altijd afgaan bij gevaar, ook als ik de leds uit heb gezet.
  if (!isnan(temp) && temp >= 50 && !alarmDisabled) {
    tornadoSiren();
  } else {
    stopBuzzer();
  }

  // 2. VISUEEL (De LEDs)
  // Als ik via het dashboard op "UIT" heb gedrukt, stoppen we hier.
  if (forceLedsOff) {
    setRgb(COLOR_OFF);
    return; // Stop de functie, lampjes blijven uit.
  }

  // De normale kleuren logica
  if (isnan(temp)) {
    setRgb(COLOR_OFF);
  } else if (temp >= 50 && !alarmDisabled) {
    blinkRed(200);
  } else if (temp >= 40) {
    setRgb(COLOR_RED);
  } else if (temp >= 20) {
    setRgb(COLOR_GREEN);
  } else if (temp >= 10) {
    setRgb(COLOR_CYAN);
  } else if (temp >= 0) {
    setRgb(COLOR_BLUE);
  } else {
    blinkBluePhase(1000, 500);
  }
}

// ==========================================
//               MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Pinnen instellen
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 

  // Alles uit bij opstarten
  setRgb(COLOR_OFF);
  stopBuzzer();

  // Buzzer instellen
  ledcSetup(BUZZER_CHANNEL, BUZZER_BASE_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  setupWifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback); // Zorgen dat we berichten kunnen ontvangen
}

void loop() {
  // Checken of MQTT nog verbonden is
  if (!client.connected()) reconnect();
  client.loop(); // Belangrijk voor ontvangen van berichten!

  handleSerialDebug();
  unsigned long now = millis();

  // Taken uitvoeren
  sensorTask(now);
  float temp = getDecisionTemp();
  checkButton(temp);
  outputsTask(temp);
}