#ifndef CONFIG_H
#define CONFIG_H

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

#endif