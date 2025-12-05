#ifndef CONFIG_H
#define CONFIG_H

// --- Hardware Pinnen ---
// Ik heb hier defines gebruikt zodat ik de pinnen makkelijk kan aanpassen
#define DHT_PIN 25       
#define RED_PIN 16       
#define GREEN_PIN 17     
#define BLUE_PIN 4       
#define BUZZER_PIN 26    
#define BUTTON_PIN 14    // Knop zit aan GND, dus interne pullup nodig

// Instellingen voor de sensor
#define DHTTYPE DHT22    

// Instellingen voor de Buzzer (PWM)
#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8
#define BUZZER_BASE_FREQ 500

// --- Tijd Instellingen ---
const unsigned long READ_INTERVAL = 2000;     // Elke 2 seconden meten
const unsigned long DEBOUNCE_DELAY = 50;      // Tegen het 'stuiteren' van de knop

// --- Debug Standaardwaarden ---
// Als de sensor stuk is, gebruiken we deze waarden in debug modus
#define DEFAULT_DEBUG_TEMP 0.0
#define DEFAULT_DEBUG_HUM 0.0

// --- Kleuren (Common Anode) ---
// Let op: Bij Common Anode is LOW aan en HIGH uit!
#define COLOR_RED     1, 0, 0
#define COLOR_GREEN   0, 1, 0
#define COLOR_BLUE    0, 0, 1
#define COLOR_CYAN    0, 1, 1
#define COLOR_OFF     0, 0, 0 

#endif