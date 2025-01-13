#include <esp_now.h>
#include <WiFi.h>

// Pin Definitions
#define BTN_FORWARD    16  // Forward button
#define BTN_BACKWARD   17  // Backward button
#define BTN_LEFT       18  // Left button
#define BTN_RIGHT      19  // Right button
#define GAS_LED_P1    22  // Gas LED Red pin
#define GAS_LED_P2    23  // Gas LED Green pin
#define WARNING_LED    21  // Yellow warning LED

// Constants
#define BLINK_INTERVAL 500  // LED blink interval in ms
#define GAS_THRESHOLD  2000 // Adjust based on your MQ135 calibration

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Global variables
unsigned long lastBlinkTime = 0;
bool ledState = false;
bool obstacleDetected = false;
bool forwardPressed = false;
//bot mac 88:13:bf:62:d3:30
uint8_t botAddress[] = {0x88, 0x13, 0xbf, 0x62, 0xd3, 0x30};

// Structure for receiving sensor data
// Structure for sending sensor data
struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
    int lightLevel;
};

// Structure for sending commands
struct CommandData {
    Direction direction;
};

void onDataReceived(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    Serial.println("Data received");
    if (data_len == sizeof(SensorData)) {
        SensorData sensorData;
        memcpy(&sensorData, data, sizeof(sensorData));
        Serial.printf("Temp: %.2f, Humidity: %.2f, AirQuality: %d, Obstacle: %d, LightLevel: %d\n",
            sensorData.temperature, sensorData.humidity, sensorData.airQuality, sensorData.obstacle, sensorData.lightLevel);
        processSensorData(sensorData);
    } else {
        Serial.printf("Unexpected data length: %d\n", data_len);
    }
}



void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(BTN_FORWARD, INPUT_PULLUP);
    pinMode(BTN_BACKWARD, INPUT_PULLUP);
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(GAS_LED_P1, OUTPUT);
    pinMode(GAS_LED_P2, OUTPUT);
    pinMode(WARNING_LED, OUTPUT);
    
    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    
    // Register callback
    esp_now_register_recv_cb(onDataReceived);
    
    // Add bot as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, botAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer added successfully");
    } else {
        Serial.println("Failed to add peer");
    }
}

void loop() {
    // Check buttons and send commands
    checkButtons();
    // Handle warning LED
    handleWarningLED();
}

void checkButtons() {
    CommandData command;
    command.direction = Direction::Stop;  // Default to stop
    
    // Check each button and set appropriate direction
    if (!digitalRead(BTN_FORWARD)) {
        command.direction = Direction::Forward;
        //Serial.println("Forward");
        forwardPressed = true;
    }
    else if (!digitalRead(BTN_BACKWARD)) {
        command.direction = Direction::Backward;
        //Serial.println("Backward");
        forwardPressed = false;
    }
    else if (!digitalRead(BTN_LEFT)) {
        command.direction = Direction::Left;
        //Serial.println("Turn left");
        forwardPressed = false;
    }
    else if (!digitalRead(BTN_RIGHT)) {
        command.direction = Direction::Right;
        //Serial.println("Turn right");
        forwardPressed = false;
    }
    else {
        forwardPressed = false;
    }
    
    // Send command via ESP-NOW
    esp_now_send(botAddress, (uint8_t*)&command, sizeof(command));
}

void handleWarningLED() {
    if (obstacleDetected) {
        if (forwardPressed) {
            // Blink LED if trying to move forward with obstacle
            if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
                ledState = !ledState;
                digitalWrite(WARNING_LED, ledState);
                lastBlinkTime = millis();
            }
        } else {
            // Solid LED if obstacle detected but not moving
            digitalWrite(WARNING_LED, HIGH);
        }
    } else {
        digitalWrite(WARNING_LED, LOW);
    }
}

void processSensorData(const SensorData &data) {
    // Update obstacle status
    obstacleDetected = data.obstacle;
    
    // Update gas warning LED
    if (data.airQuality > GAS_THRESHOLD) {
        digitalWrite(GAS_LED_P2, LOW);
        digitalWrite(GAS_LED_P1, HIGH);
    } else {
        digitalWrite(GAS_LED_P2, HIGH);
        digitalWrite(GAS_LED_P1, LOW);
    }
}
