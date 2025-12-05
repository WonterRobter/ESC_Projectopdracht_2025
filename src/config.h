#ifndef CONFIG_H
#define CONFIG_H

// --- Hardware Pin Definities ---

// Pinconfiguratie
#define DHT_PIN 25      // Pin DHT22
#define RED_PIN 16      // R van RGB LED
#define GREEN_PIN 17    // G van RGB LED
#define BLUE_PIN 4      // B van RGB LED
#define BUZZER_PIN 26   // Buzzer pin
#define BUTTON_PIN 14   // fysieke knop om alarm te onderdrukken

#define DHTTYPE DHT22 

// Buzzer instellingen
#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8
#define BUZZER_BASE_FREQ 500

// Debugwaarden
#define DEFAULT_DEBUG_TEMP 0
#define DEFAULT_DEBUG_HUM 0

// Kleuren
#define COLOR_RED     1, 0, 0
#define COLOR_GREEN   0, 1, 0
#define COLOR_BLUE    0, 0, 1
#define COLOR_CYAN    0, 1, 1
#define COLOR_OFF     0, 0, 0
// RGB-kleuren voor common anode LED (LOW = aan, HIGH = uit)

// Interval voor uitlezen
const unsigned long READ_INTERVAL = 2000; // elke 2 seconden

// --- MQTT Topics (Namen voor de communicatie) ---

// Waarden die we publiceren (ESP32 -> Node-RED)
const char* TOPIC_TEMP = "wouter/data/temperatuur";
const char* TOPIC_HUM  = "wouter/data/luchtvochtigheid";

// Commando's die we ontvangen (Node-RED -> ESP32)
const char* TOPIC_LED  = "wouter/command/ledstatus"; 
const char* TOPIC_BEEP = "wouter/command/buzzeralarm";

// --- Timing en Drempelwaarden ---

const long SENSOR_INTERVAL_MS = 10000; // Meet interval in milliseconden (10 seconden)
const long MQTT_RECONNECT_DELAY_MS = 5000; // Wachtijd bij mislukte MQTT connectie

#endif