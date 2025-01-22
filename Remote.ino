#include <esp_now.h>
#include <WiFi.h>

// Pin Definitions
#define JOYSTICK_X    32  // Joystick X-axis
#define JOYSTICK_Y    33  // Joystick Y-axis
#define WARNING_LED   2

// Constants
#define WARNING_LED_BLINK_INTERVAL 500  // LED blink interval in ms
#define GAS_THRESHOLD  2000  // Adjust based on your MQ135 calibration
#define JOYSTICK_THRESHOLD 2048 // Center value of joystick
#define JOYSTICK_DEADZONE 500  // Deadzone for joystick. At what point should the bot start moving

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Global variables
unsigned long warningLEDlastBlinkTime = 0;
bool warningLEDState = false;
bool obstacleDetected = false;
bool forwardPressed = false;

// Bot MAC address
uint8_t botAddress[] = {0x2C, 0xBC, 0xBB, 0x0D, 0xCE, 0xD0};

// Structure for receiving sensor data
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
    pinMode(JOYSTICK_X, INPUT);
    pinMode(JOYSTICK_Y, INPUT);
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
    // Check joystick and send commands
    checkJoystick();
    // Handle warning LED
    handleWarningLED();
}

void checkJoystick() {
    CommandData command;
    command.direction = Direction::Stop;  // Default to stop
    
    int xValue = analogRead(JOYSTICK_X);
    int yValue = analogRead(JOYSTICK_Y);
    
// Determine direction based on joystick position
        if (abs(xValue - JOYSTICK_THRESHOLD) > JOYSTICK_DEADZONE || 
            abs(yValue - JOYSTICK_THRESHOLD) > JOYSTICK_DEADZONE) {
            
            if (abs(xValue - JOYSTICK_THRESHOLD) > abs(yValue - JOYSTICK_THRESHOLD)) {
                // Horizontal movement takes priority
                if (xValue > JOYSTICK_THRESHOLD + JOYSTICK_DEADZONE) {
                    command.direction = Direction::Right;
                    forwardPressed = false;
                } else if (xValue < JOYSTICK_THRESHOLD - JOYSTICK_DEADZONE) {
                    command.direction = Direction::Left;
                    forwardPressed = false;
                }
            } else {
                // Vertical movement
                if (yValue > JOYSTICK_THRESHOLD + JOYSTICK_DEADZONE) {
                    command.direction = Direction::Backward;
                    forwardPressed = false;
                } else if (yValue < JOYSTICK_THRESHOLD - JOYSTICK_DEADZONE) {
                    command.direction = Direction::Forward;
                    forwardPressed = true;
                }
            }
        } else {
            forwardPressed = false;
        }

    
    // Send command via ESP-NOW
    esp_now_send(botAddress, (uint8_t*)&command, sizeof(command));
}

void handleWarningLED() {
    if (obstacleDetected) {
        if (forwardPressed) {
            // Blink LED if trying to move forward with obstacle
            if (millis() - warningLEDlastBlinkTime >= WARNING_LED_BLINK_INTERVAL) {
                warningLEDState = !warningLEDState;
                digitalWrite(WARNING_LED, warningLEDState);
                warningLEDlastBlinkTime = millis();
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
}
