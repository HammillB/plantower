// Source: https://www.hackster.io/chshammill/pms-5003-laser-air-sensor-with-photon-2e8e05
// This code has been refactored for readability, maintainability, and robustness.

#include <Adafruit_DHT.h>

// --- Pin Definitions ---
const int RED_LED_PIN = D1;
const int GREEN_LED_PIN = D2;
const int BLUE_LED_PIN = D3;
const int LED_POWER_PIN = D0;
const int STATUS_LED_PIN = D7; // D7 is the blue LED on the Photon
const int DHT_PIN = 5;

// --- Sensor & API Constants ---
#define DHT_TYPE DHT22

// --- Plantower PMS5003 Commands ---
const uint8_t PMS_CMD_WAKE[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
const uint8_t PMS_CMD_SLEEP[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };

// --- Timing Constants (in milliseconds) ---
const unsigned long SENSOR_READ_INTERVAL = 120000; // Time between sensor readings
const unsigned long PMS_WAKE_TIME = 30000;         // Time for PMS sensor to warm up
const unsigned long PARTICLE_PUBLISH_DELAY = 2000; // Delay between Particle.publish calls

// --- LED Brightness ---
const int LED_BRIGHTNESS = 40; // 0 (off) to 255 (full brightness)

// --- Data Structures ---
struct PlantowerData {
    uint16_t pm10_standard;
    uint16_t pm25_standard;
    uint16_t pm100_standard;
};

struct SensorData {
    float temperature_f;
    float humidity;
    PlantowerData pm_data;
    long filter_status;
    const char* led_color;
};

// --- State Machine ---
enum State {
    STATE_IDLE,
    STATE_WAKE_SENSOR,
    STATE_WARMUP,
    STATE_READ_SENSORS,
    STATE_PUBLISH,
    STATE_SLEEP_SENSOR
};
State current_state = STATE_IDLE;
unsigned long last_state_change = 0;


// --- Global Variables ---
DHT dht(DHT_PIN, DHT_TYPE);
SensorData current_sensor_data = {0};

// --- Function Prototypes ---
void readDhtSensor(SensorData& data);
bool readPmSensor(PlantowerData& data);
void setLedColor(uint16_t pm25_value, const char*& color_string);
void publishData(const SensorData& data);

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600); // For PMS5003 sensor

    dht.begin();

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(LED_POWER_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    analogWrite(LED_POWER_PIN, LED_BRIGHTNESS); // Power the status LED
    digitalWrite(STATUS_LED_PIN, LOW); // Start with status LED off (asleep)

    current_state = STATE_SLEEP_SENSOR; // Start in sleep state
    last_state_change = millis();
}

void loop() {
    unsigned long now = millis();

    switch (current_state) {
        case STATE_IDLE:
            // This state is effectively the long sleep interval
            if (now - last_state_change >= SENSOR_READ_INTERVAL) {
                current_state = STATE_WAKE_SENSOR;
                last_state_change = now;
            }
            break;

        case STATE_WAKE_SENSOR:
            digitalWrite(STATUS_LED_PIN, HIGH); // Turn on status LED (awake)
            Serial.println("Waking up sensor...");
            Serial1.write(PMS_CMD_WAKE, sizeof(PMS_CMD_WAKE));
            current_state = STATE_WARMUP;
            last_state_change = now;
            break;

        case STATE_WARMUP:
            if (now - last_state_change >= PMS_WAKE_TIME) {
                current_state = STATE_READ_SENSORS;
                last_state_change = now;
            }
            break;

        case STATE_READ_SENSORS:
            Serial.println("Reading sensor data...");
            if (readPmSensor(current_sensor_data.pm_data)) {
                readDhtSensor(current_sensor_data);
                setLedColor(current_sensor_data.pm_data.pm25_standard, current_sensor_data.led_color);

                // Update filter status
                if (current_sensor_data.pm_data.pm25_standard > 15 && current_sensor_data.filter_status == 0) {
                    current_sensor_data.filter_status = 1;
                } else if (current_sensor_data.pm_data.pm25_standard < 10 && current_sensor_data.filter_status == 1) {
                    current_sensor_data.filter_status = 0;
                }

                current_state = STATE_PUBLISH;
            } else {
                Serial.println("Failed to read from PMS5003 sensor. Retrying after interval.");
                current_state = STATE_SLEEP_SENSOR;
            }
            last_state_change = now;
            break;

        case STATE_PUBLISH:
            publishData(current_sensor_data);
            current_state = STATE_SLEEP_SENSOR;
            last_state_change = now;
            break;

        case STATE_SLEEP_SENSOR:
            Serial.println("Putting sensor to sleep...");
            Serial1.write(PMS_CMD_SLEEP, sizeof(PMS_CMD_SLEEP));
            digitalWrite(STATUS_LED_PIN, LOW); // Turn off status LED (asleep)
            current_state = STATE_IDLE;
            last_state_change = now;
            break;
    }
}

/**
 * @brief Reads temperature and humidity from the DHT sensor.
 * @param data The SensorData struct to store the readings in.
 */
void readDhtSensor(SensorData& data) {
    data.humidity = dht.getHumidity();
    // Read temperature as Celsius and convert to Fahrenheit
    float temp_c = dht.getTempCelcius();
    data.temperature_f = (temp_c * 1.8) + 32;
}

/**
 * @brief Reads and parses data from the Plantower PMS5003 sensor with checksum validation.
 * @param data The PlantowerData struct to store the readings in.
 * @return True if data was read successfully, false otherwise.
 */
bool readPmSensor(PlantowerData& data) {
    if (!Serial1.available()) {
        return false;
    }

    // Look for the start of the data frame
    if (Serial1.peek() != 0x42) {
        Serial1.read();
        return false;
    }

    // Check if there are enough bytes for a full frame
    if (Serial1.available() < 32) {
        return false;
    }

    uint8_t buffer[32];
    Serial1.readBytes((char *)buffer, 32);

    // Verify the start bytes
    if (buffer[0] != 0x42 || buffer[1] != 0x4D) {
        Serial.println("Invalid start bytes");
        return false;
    }

    // Calculate the checksum
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) {
        sum += buffer[i];
    }

    // Get the checksum from the data frame
    uint16_t checksum = (buffer[30] << 8) | buffer[31];

    // Verify the checksum
    if (sum != checksum) {
        Serial.println("Checksum failure");
        return false;
    }

    // The data is in big-endian format (high byte first)
    data.pm10_standard = (buffer[4] << 8) | buffer[5];
    data.pm25_standard = (buffer[6] << 8) | buffer[7];
    data.pm100_standard = (buffer[8] << 8) | buffer[9];

    // Note: The original code had confusing variable names.
    // pm10 was PM1.0, pm25 was PM2.5, pm100 was PM10.0.
    // This has been corrected. We are reading the "Standard PM" values.
    // The sensor also provides "Atmospheric Environment" values if needed.

    Serial.print("{ ");
    Serial.print("\"pm1.0\": "); Serial.print(data.pm10_standard); Serial.print(", ");
    Serial.print("\"pm2.5\": "); Serial.print(data.pm25_standard); Serial.print(", ");
    Serial.print("\"pm10.0\": "); Serial.print(data.pm100_standard);
    Serial.println(" }");

    // Clear any remaining bytes in the buffer, although there shouldn't be any
    // if the frame was read correctly.
    while(Serial1.available()) Serial1.read();

    return true;
}

/**
 * @brief Sets the RGB LED color based on the PM2.5 value.
 * @param pm25_value The current PM2.5 reading.
 * @param color_string A reference to a const char* to store the name of the color.
 */
void setLedColor(uint16_t pm25_value, const char*& color_string) {
    // Note: HIGH is off, LOW is on for the LED wiring.
    if (pm25_value < 10) { // Green
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, HIGH);
        color_string = "green";
    } else if (pm25_value < 20) { // Cyan
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, LOW);
        color_string = "cyan";
    } else if (pm25_value < 30) { // Blue
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, LOW);
        color_string = "blue";
    } else if (pm25_value < 40) { // Magenta
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, LOW);
        color_string = "magenta";
    } else if (pm25_value < 100) { // Red
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, HIGH);
        color_string = "red";
    } else { // White
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, LOW);
        color_string = "white";
    }
}

/**
 * @brief Publishes sensor data to the Particle Cloud.
 * @param data The SensorData struct containing the data to publish.
 */
void publishData(const SensorData& data) {
    Particle.publish("temperature", String(data.temperature_f) + " °F", PRIVATE);
    delay(PARTICLE_PUBLISH_DELAY);
    Particle.publish("humidity", String(data.humidity) + "%", PRIVATE);
    delay(PARTICLE_PUBLISH_DELAY);
    // Correcting the confusing naming from the original code
    Particle.publish("PM1.0", String(data.pm_data.pm10_standard) + " ug/m3", PRIVATE);
    delay(PARTICLE_PUBLISH_DELAY);
    Particle.publish("PM2.5", String(data.pm_data.pm25_standard) + " ug/m3", PRIVATE);
    delay(PARTICLE_PUBLISH_DELAY);
    Particle.publish("PM10.0", String(data.pm_data.pm100_standard) + " ug/m3", PRIVATE);
    delay(PARTICLE_PUBLISH_DELAY);
    Particle.publish("led_color", data.led_color, PRIVATE);
    delay(PARTICLE_PUBLISH_DELAY);
    Particle.publish("filter_status", String(data.filter_status), PRIVATE);
}
