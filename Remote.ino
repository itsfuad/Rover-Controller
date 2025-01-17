#include <esp_now.h>
#include <WiFi.h>

namespace BotController {

// Pin Definitions
constexpr int BTN_FORWARD = 16;
constexpr int BTN_BACKWARD = 17;
constexpr int BTN_LEFT = 18;
constexpr int BTN_RIGHT = 19;
constexpr int GAS_LED_P1 = 22;
constexpr int GAS_LED_P2 = 23;
constexpr int WARNING_LED = 21;

// Constants
constexpr unsigned long BLINK_INTERVAL = 500;
constexpr int GAS_THRESHOLD = 2000;

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
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
    Direction direction;
};

class Controller {
private:
    unsigned long lastBlinkTime = 0;
    bool ledState = false;
    bool obstacleDetected = false;
    bool forwardPressed = false;
    uint8_t botAddress[6] = {0x88, 0x13, 0xBF, 0x62, 0xD3, 0x30};

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

    void checkButtons() {
        CommandData command{Direction::Stop};

        if (!digitalRead(BTN_FORWARD)) {
            command.direction = Direction::Forward;
            forwardPressed = true;
        } else if (!digitalRead(BTN_BACKWARD)) {
            command.direction = Direction::Backward;
            forwardPressed = false;
        } else if (!digitalRead(BTN_LEFT)) {
            command.direction = Direction::Left;
            forwardPressed = false;
        } else if (!digitalRead(BTN_RIGHT)) {
            command.direction = Direction::Right;
            forwardPressed = false;
        } else {
            forwardPressed = false;
        }

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

        pinMode(BTN_FORWARD, INPUT_PULLUP);
        pinMode(BTN_BACKWARD, INPUT_PULLUP);
        pinMode(BTN_LEFT, INPUT_PULLUP);
        pinMode(BTN_RIGHT, INPUT_PULLUP);
        pinMode(GAS_LED_P1, OUTPUT);
        pinMode(GAS_LED_P2, OUTPUT);
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
        checkButtons();
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
