#include <esp_now.h>
#include <WiFi.h>

namespace BotController {

// Pin Definitions
constexpr int JOYSTICK_X = 32;    // VRX pin
constexpr int JOYSTICK_Y = 33;    // VRY pin
constexpr int JOYSTICK_SW = 35;   // Switch pin
constexpr int WARNING_LED = 2;

// Constants
constexpr unsigned long BLINK_INTERVAL = 500;
constexpr int JOYSTICK_THRESHOLD = 2048;  // Mid-point for analog reading (4096/2)
constexpr int JOYSTICK_DEADZONE = 500;    // Deadzone to prevent drift. This means the joystick must move at least 500 units from the center to register a movement

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Command types
enum class CommandType : uint8_t {
    Movement = 0,
    CheckConnection
};

// Sensor data structure
struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
    int lightLevel;
};

// Command data structure
struct CommandData {
    CommandType type;
    Direction direction;
};

class Controller {
private:
    unsigned long lastBlinkTime = 0;
    bool ledState = false;
    bool obstacleDetected = false;
    bool forwardPressed = false;
    // MAC: 2c:bc:bb:0d:ce:d0
    uint8_t botAddress[6] = {0x2c, 0xbc, 0xbb, 0x0d, 0xce, 0xd0};
    unsigned long lastButtonPress = 0;
    constexpr static unsigned long DEBOUNCE_DELAY = 200;

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
    }

    void checkJoystick() {
        CommandData command{CommandType::Movement, Direction::Stop};
        
        // Read joystick values
        int xValue = analogRead(JOYSTICK_X);
        int yValue = analogRead(JOYSTICK_Y);
        bool buttonPressed = !digitalRead(JOYSTICK_SW);  // Active LOW

        // Handle button press with debounce
        if (buttonPressed && (millis() - lastButtonPress > DEBOUNCE_DELAY)) {
            command.type = CommandType::CheckConnection;
            lastButtonPress = millis();
            esp_now_send(botAddress, reinterpret_cast<uint8_t*>(&command), sizeof(command));
            return;
        }

        // Process joystick movement
        command.type = CommandType::Movement;

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

        //Serial.printf("Direction: %d\n", static_cast<int>(command.direction));

        esp_now_send(botAddress, reinterpret_cast<uint8_t*>(&command), sizeof(command));
    }

    static void onDataReceived(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
        if (data_len == sizeof(SensorData)) {
            SensorData sensorData;
            memcpy(&sensorData, data, sizeof(sensorData));
            Serial.printf("Temp: %.2f, Humidity: %.2f, AirQuality: %d, Obstacle: %d, LightLevel: %d\n",
                          sensorData.temperature, sensorData.humidity, sensorData.airQuality, sensorData.obstacle, sensorData.lightLevel);
            instance().processSensorData(sensorData);
        } else {
            Serial.printf("Unexpected data length: %d\n", data_len);
        }
    }

    static Controller& instance() {
        static Controller controllerInstance;
        return controllerInstance;
    }

public:
    void setup() {
        Serial.begin(115200);

        // Configure joystick pins
        pinMode(JOYSTICK_X, INPUT);
        pinMode(JOYSTICK_Y, INPUT);
        pinMode(JOYSTICK_SW, INPUT_PULLUP);
        
        pinMode(WARNING_LED, OUTPUT);

        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK) {
            Serial.println("ESP-NOW init failed");
            return;
        }

        esp_now_register_recv_cb(onDataReceived);

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
        checkJoystick();
        handleWarningLED();
    }
};

}  // namespace BotController

BotController::Controller controller;

void setup() {
    controller.setup();
}

void loop() {
    controller.loop();
}