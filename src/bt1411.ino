
/*
 * FALL DETECTION SYSTEM
 * Ho ten:Trịnh Trường Phúc
 * Lop: N01
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <math.h>

// =====================================================
// 1. CAU HINH CHAN
// =====================================================

#define SDA_PIN       21
#define SCL_PIN       22
#define LED_PIN       25
#define BUZZER_PIN    26

// =====================================================
// 2. WIFI
// =====================================================

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""

// =====================================================
// 3. THINGSBOARD MQTT
// =====================================================

#define MQTT_SERVER "mqtt.thingsboard.cloud"
#define MQTT_PORT   1883

// Thay bang Access Token cua Device ThingsBoard
#define MQTT_TOKEN "J6UE4qUISMQAxC8nFpRb"

#define MQTT_CLIENT_ID "ESP32_Fall_Detection"

#define TELEMETRY_TOPIC "v1/devices/me/telemetry"
#define RPC_TOPIC       "v1/devices/me/rpc/request/+"

// =====================================================
// 4. DOI TUONG
// =====================================================

WiFiClient espClient;
PubSubClient mqtt(espClient);

Adafruit_MPU6050 mpu;

// =====================================================
// 5. CAU TRUC DU LIEU CAM BIEN
// =====================================================

struct SensorData {
  float ax;
  float ay;
  float az;

  float magnitude;
  float roll;
  float pitch;

  bool fallDetected;
};

// =====================================================
// 6. QUEUE FREERTOS
// =====================================================

QueueHandle_t dataQueue;
QueueHandle_t dataAlert;

// =====================================================
// 7. BIEN DIEU KHIEN
// =====================================================

// Lenh dieu khien thu cong tu ThingsBoard
bool manualLed = false;
bool manualBuzzer = false;

// Trang thai dau ra thuc te
bool actualLedState = false;
bool actualBuzzerState = false;

// Bao ve bien dung chung giua cac task
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// 8. KHAI BAO TASK
// =====================================================

void Task_Read(void *pvParameters);
void Task_Transmit(void *pvParameters);
void Task_Alert(void *pvParameters);

// =====================================================
// 9. MQTT CALLBACK - NHAN LENH RPC
// =====================================================

void mqttCallback(char *topic, byte *payload, unsigned int length) {

  Serial.println("\n===== MQTT RPC RECEIVED =====");

  char message[256];

  if (length >= sizeof(message)) {
    Serial.println("RPC message too long");
    return;
  }

  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.println(message);

  StaticJsonDocument<256> doc;

  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.println("JSON parsing failed");
    return;
  }

  const char *method = doc["method"] | "";
  JsonVariant params = doc["params"];

  bool value = false;
  bool validValue = false;

  // Cho phep params la true/false
  if (params.is<bool>()) {
    value = params.as<bool>();
    validValue = true;
  }

  // Hoac params la {"value": true/false}
  else if (params.is<JsonObject>() &&
           params["value"].is<bool>()) {

    value = params["value"].as<bool>();
    validValue = true;
  }

  if (!validValue) {
    Serial.println("Invalid RPC params");
    return;
  }

  bool success = false;

  portENTER_CRITICAL(&controlMux);

  if (strcmp(method, "setLed") == 0) {

    manualLed = value;
    success = true;

    Serial.printf("Manual LED: %s\n",
                  value ? "ON" : "OFF");
  }

  else if (strcmp(method, "setBuzzer") == 0) {

    manualBuzzer = value;
    success = true;

    Serial.printf("Manual BUZZER: %s\n",
                  value ? "ON" : "OFF");
  }

  portEXIT_CRITICAL(&controlMux);

  if (!success) {
    Serial.println("Unknown RPC method");
    return;
  }

  // Gui phan hoi RPC
  char responseTopic[100];

  const char *lastSlash = strrchr(topic, '/');

  if (lastSlash != nullptr) {

    snprintf(
      responseTopic,
      sizeof(responseTopic),
      "v1/devices/me/rpc/response/%s",
      lastSlash + 1
    );

    mqtt.publish(responseTopic, "{\"success\":true}");
  }
}

// =====================================================
// 10. TASK DOC CAM BIEN MPU6050
// =====================================================

void Task_Read(void *pvParameters) {

  Serial.println("Task_Read started on Core 1");

  SensorData data;

  unsigned long lastImpactTime = 0;
  unsigned long fallTime = 0;

  bool impactDetected = false;
  bool fallDetected = false;

  const float IMPACT_THRESHOLD = 2.3;
  const float ANGLE_THRESHOLD = 45.0;

  while (1) {

    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    // Gia toc truc X, Y, Z (m/s2)
    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;

    // Do lon gia toc (g)
    data.magnitude =
      sqrt(
        data.ax * data.ax +
        data.ay * data.ay +
        data.az * data.az
      ) / 9.80665;

    // Tinh Roll va Pitch
    data.roll =
      atan2(data.ay, data.az) * 180.0 / PI;

    data.pitch =
      atan2(
        -data.ax,
        sqrt(data.ay * data.ay +
             data.az * data.az)
      ) * 180.0 / PI;

    unsigned long now = millis();

    // Phat hien va cham
    if (data.magnitude > IMPACT_THRESHOLD) {

      impactDetected = true;
      lastImpactTime = now;

      Serial.println("IMPACT DETECTED!");
    }

    // Kiem tra goc nghieng sau va cham
    if (impactDetected &&
        now - lastImpactTime <= 3000) {

      if (fabs(data.roll) > ANGLE_THRESHOLD ||
          fabs(data.pitch) > ANGLE_THRESHOLD) {

        fallDetected = true;
        fallTime = now;

        impactDetected = false;

        Serial.println("!!! FALL DETECTED !!!");
      }
    }

    // Het thoi gian kiem tra
    if (impactDetected &&
        now - lastImpactTime > 3000) {

      impactDetected = false;
    }

    // Giu trang thai canh bao trong 10 giay
    if (fallDetected && now - fallTime > 10000) {
      fallDetected = false;
      Serial.println("Fall alarm cleared");
    }

    data.fallDetected = fallDetected;

    // Dua du lieu vao Queue truyen MQTT
    if (xQueueSend(dataQueue, &data, 0) != pdPASS) {

      SensorData oldData;

      xQueueReceive(dataQueue, &oldData, 0);
      xQueueSend(dataQueue, &data, 0);
    }

    // Cap nhat Queue canh bao
    xQueueOverwrite(dataAlert, &data);

    // In thong tin moi 1 giay
    static unsigned long lastPrint = 0;

    if (now - lastPrint >= 1000) {

      lastPrint = now;

      Serial.println("----- MPU6050 DATA -----");

      Serial.printf("AX: %.2f\n", data.ax);
      Serial.printf("AY: %.2f\n", data.ay);
      Serial.printf("AZ: %.2f\n", data.az);

      Serial.printf("Magnitude: %.2f g\n",
                    data.magnitude);

      Serial.printf("Roll: %.2f\n", data.roll);
      Serial.printf("Pitch: %.2f\n", data.pitch);

      Serial.printf("Fall: %s\n",
                    data.fallDetected ? "YES" : "NO");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =====================================================
// 11. TASK DIEU KHIEN LED VA BUZZER
// =====================================================

void Task_Alert(void *pvParameters) {

  Serial.println("Task_Alert started on Core 1");

  SensorData alertData;

  while (1) {

    if (xQueueReceive(
          dataAlert,
          &alertData,
          portMAX_DELAY
        ) == pdPASS) {

      bool ledManual;
      bool buzzerManual;

      // Doc lenh dieu khien thu cong
      portENTER_CRITICAL(&controlMux);

      ledManual = manualLed;
      buzzerManual = manualBuzzer;

      portEXIT_CRITICAL(&controlMux);

      // Ket hop canh bao tu dong va dieu khien thu cong
      bool ledState =
        alertData.fallDetected || ledManual;

      bool buzzerState =
        alertData.fallDetected || buzzerManual;

      // Dieu khien LED
      digitalWrite(
        LED_PIN,
        ledState ? HIGH : LOW
      );

      // Dieu khien coi
      if (buzzerState) {
        tone(BUZZER_PIN, 2000);
      }
      else {
        noTone(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
      }

      // Cap nhat trang thai dau ra thuc te
      portENTER_CRITICAL(&controlMux);

      actualLedState = ledState;
      actualBuzzerState = buzzerState;

      portEXIT_CRITICAL(&controlMux);
    }
  }
}

// =====================================================
// 12. TASK WIFI + MQTT + GUI TELEMETRY
// =====================================================

void Task_Transmit(void *pvParameters) {

  Serial.println("Task_Transmit started on Core 0");

  bool wifiStarted = false;

  uint32_t lastWifiAttempt = 0;
  uint32_t lastMqttAttempt = 0;

  while (1) {

    // ---------------- WIFI ----------------

    if (WiFi.status() != WL_CONNECTED) {

      if (!wifiStarted ||
          millis() - lastWifiAttempt > 10000) {

        Serial.println("Connecting WiFi...");

        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        wifiStarted = true;
        lastWifiAttempt = millis();
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ---------------- MQTT ----------------

    if (!mqtt.connected()) {

      if (millis() - lastMqttAttempt >= 3000) {

        lastMqttAttempt = millis();

        Serial.println("Connecting MQTT...");

        if (mqtt.connect(
              MQTT_CLIENT_ID,
              MQTT_TOKEN,
              nullptr
            )) {

          Serial.println("MQTT Connected!");

          mqtt.subscribe(RPC_TOPIC);

          Serial.println("Subscribed RPC topic");
        }

        else {

          Serial.printf(
            "MQTT failed, state=%d\n",
            mqtt.state()
          );
        }
      }

      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Xu ly MQTT RPC
    mqtt.loop();

    // ---------------- QUEUE ----------------

    SensorData data;

    if (xQueueReceive(
          dataQueue,
          &data,
          pdMS_TO_TICKS(50)
        ) == pdPASS) {

      bool ledState;
      bool buzzerState;

      // Lay trang thai dau ra thuc te
      portENTER_CRITICAL(&controlMux);

      ledState = actualLedState;
      buzzerState = actualBuzzerState;

      portEXIT_CRITICAL(&controlMux);

      char json[500];

      snprintf(
        json,
        sizeof(json),

        "{"
        "\"ax\":%.3f,"
        "\"ay\":%.3f,"
        "\"az\":%.3f,"
        "\"magnitude\":%.3f,"
        "\"roll\":%.2f,"
        "\"pitch\":%.2f,"
        "\"fallDetected\":%s,"
        "\"ledState\":%s,"
        "\"buzzerState\":%s"
        "}",

        data.ax,
        data.ay,
        data.az,
        data.magnitude,
        data.roll,
        data.pitch,

        data.fallDetected ? "true" : "false",

        ledState ? "true" : "false",
        buzzerState ? "true" : "false"
      );

      if (mqtt.publish(TELEMETRY_TOPIC, json)) {

        Serial.println("MQTT DATA SENT:");
        Serial.println(json);
      }

      else {

        Serial.println("MQTT publish failed");
      }
    }

    mqtt.loop();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =====================================================
// 13. SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println("\n==============================");
  Serial.println(" ESP32 FALL DETECTION SYSTEM ");
  Serial.println("==============================");

  // Khoi tao chan
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  noTone(BUZZER_PIN);

  // Khoi tao I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Khoi tao MPU6050
  if (!mpu.begin()) {

    Serial.println("MPU6050 NOT FOUND!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 initialized!");

  mpu.setAccelerometerRange(
    MPU6050_RANGE_8_G
  );

  mpu.setGyroRange(
    MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
    MPU6050_BAND_21_HZ
  );

  // Cau hinh MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);

  // Tao Queue
  dataQueue = xQueueCreate(
    10,
    sizeof(SensorData)
  );

  dataAlert = xQueueCreate(
    1,
    sizeof(SensorData)
  );

  if (dataQueue == NULL || dataAlert == NULL) {

    Serial.println("QUEUE CREATION FAILED!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("FreeRTOS queues created!");

  // Tao Task doc cam bien
  xTaskCreatePinnedToCore(
    Task_Read,
    "Task_Read",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  // Tao Task truyen MQTT
  xTaskCreatePinnedToCore(
    Task_Transmit,
    "Task_Transmit",
    6144,
    NULL,
    1,
    NULL,
    0
  );

  // Tao Task canh bao
  xTaskCreatePinnedToCore(
    Task_Alert,
    "Task_Alert",
    3072,
    NULL,
    2,
    NULL,
    1
  );

  Serial.println("All FreeRTOS tasks started!");
}

// =====================================================
// 14. LOOP
// =====================================================

void loop() {

  vTaskDelay(pdMS_TO_TICKS(1000));
}
