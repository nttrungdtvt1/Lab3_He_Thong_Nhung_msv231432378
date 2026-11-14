/*
 * Lab 3 - Giám sát vận động viên và cảnh báo
 * Mạch: ESP32 DevKit C V4
 * Cảm biến: BMP180 qua I2C, địa chỉ 0x77
 *
 * Sơ đồ chân, lấy từ diagram.json:
 *   BMP180 VCC -> 3V3
 *   BMP180 GND -> GND.1
 *   BMP180 SDA -> GPIO 21
 *   BMP180 SCL -> GPIO 22
 *   Anode LED  -> GPIO 26 qua điện trở 1k, cathode -> GND
 *   Chân dương còi -> GPIO 25, chân âm -> GND
 *
 * Broker MQTT công cộng EMQX, không cần tài khoản:
 *   ESP32 dùng TCP cổng 1883
 *   Website dùng WebSocket ws://broker.emqx.io:8083/mqtt
 *
 * Website đọc payload cảm biến dạng số thập phân và dòng trạng thái.
 * JSON chỉ dùng cho log Serial. Đăng JSON lên topic cảm biến sẽ làm dashboard lỗi.
 *
 * Task:
 *   Task_ReadData     core 1  đọc BMP180, gửi dataQueue và dataAlert
 *   Task_TransmitData core 0  Wi-Fi, MQTT, gửi telemetry và trạng thái
 *   Task_Alert        core 1  phân loại nhiệt độ và biến thiên cao độ khoảng 5 s
 *   Task_MQTTControl  core 1  lệnh LED/còi ON, OFF, AUTO
 * Chỉ module đầu ra được ghi GPIO, và phải giữ outputMutex.
 */

#include <Arduino.h>                 // Hàm Arduino: Serial, pinMode, millis, tone
#include <math.h>                    // pow, isnan cho công thức cao độ
#include <stdio.h>                   // snprintf để tạo chuỗi log và payload
#include <string.h>                  // memcpy, strcmp, strlen, memset
#include <Wire.h>                    // Bus I2C nối ESP32 với BMP180
#include <WiFi.h>                    // Wi-Fi station của ESP32
#include <PubSubClient.h>            // Client MQTT
#include <Adafruit_BMP085.h>         // Thư viện đọc BMP180, tương thích BMP085
#include <freertos/FreeRTOS.h>       // Kernel FreeRTOS
#include <freertos/task.h>           // Tạo task, delay theo tick
#include <freertos/queue.h>          // Hàng đợi trao đổi dữ liệu giữa các task
#include <freertos/semphr.h>         // Mutex bảo vệ Serial và GPIO
#include "alert_logic.h"             // Ngưỡng cảnh báo, lọc cao độ, ghép LED/còi

// Phần cứng
const int PIN_SDA = 21;              // Chân SDA của BMP180
const int PIN_SCL = 22;              // Chân SCL của BMP180
const int PIN_LED = 26;              // Chân bật LED cảnh báo
const int PIN_BUZZER = 25;           // Chân phát tiếng còi
const int BUZZER_FREQ_HZ = 2000;     // Tần số còi, đơn vị Hz

// Wi-Fi mạng khách của Wokwi. Kênh 6 giúp bỏ qua bước quét mạng.
const char* WIFI_SSID = "Wokwi-GUEST"; // Tên mạng Wi-Fi
const char* WIFI_PASSWORD = "";         // Mạng khách không có mật khẩu
const int WIFI_CHANNEL = 6;             // Kênh Wi-Fi cố định
const uint32_t WIFI_RETRY_MS = 10000;   // Chờ 10 s giữa hai lần nối lại Wi-Fi

// MQTT
const char* MQTT_HOST = "broker.emqx.io"; // Địa chỉ broker
const uint16_t MQTT_PORT = 1883;          // Cổng TCP của MQTT
const uint32_t MQTT_RETRY_MS = 4000;      // Chờ 4 s giữa hai lần nối lại MQTT
const uint32_t TELEMETRY_INTERVAL_MS = 2000; // Chu kỳ gửi số liệu cảm biến

const char* TOPIC_TEMPERATURE = "runner/lab 3/temperature";       // Topic nhiệt độ
const char* TOPIC_PRESSURE = "runner/lab 3/pressure";             // Topic áp suất
const char* TOPIC_ALTITUDE = "runner/lab 3/altitude";             // Topic cao độ
const char* TOPIC_WIFI = "runner/lab 3/wifi";                     // Topic trạng thái Wi-Fi
const char* TOPIC_STATUS = "runner/lab 3/status";                 // Topic dòng trạng thái tổng hợp
const char* TOPIC_ALARM_TEMP = "runner/lab 3/alarm/temperature";  // Topic cảnh báo nhiệt độ
const char* TOPIC_ALARM_ALT = "runner/lab 3/alarm/altitude";      // Topic cảnh báo cao độ
const char* TOPIC_CONTROL_LED = "runner/lab 3/control/led";       // Topic lệnh điều khiển LED
const char* TOPIC_CONTROL_BUZZER = "runner/lab 3/control/buzzer"; // Topic lệnh điều khiển còi

const float SEA_LEVEL_PRESSURE_PA = 101325.0f; // Áp suất mực nước biển, đơn vị Pa
const uint32_t SENSOR_INTERVAL_MS = 1000;      // Chu kỳ đọc BMP180
const uint32_t BMP_RETRY_MS = 3000;            // Chờ 3 s giữa hai lần khởi tạo BMP180
const int BMP_FAILS_BEFORE_REINIT = 3;         // Ba mẫu lỗi liên tiếp thì khởi tạo lại cảm biến

const int QUEUE_DEPTH = 8;             // Mỗi queue chứa tối đa 8 phần tử
const UBaseType_t TASK_PRIORITY = 2;   // Mức ưu tiên chung của bốn task

enum ControlDevice { CONTROL_LED = 0, CONTROL_BUZZER = 1 };          // Thiết bị trong lệnh điều khiển
enum ControlAction { CONTROL_ON = 0, CONTROL_OFF = 1, CONTROL_AUTO = 2 }; // Hành động ON, OFF hoặc AUTO

struct ControlCommand {              // Một lệnh điều khiển đưa qua commandQueue
  uint8_t device;                    // CONTROL_LED hoặc CONTROL_BUZZER
  uint8_t action;                    // CONTROL_ON, CONTROL_OFF hoặc CONTROL_AUTO
};

Adafruit_BMP085 bmp;                 // Đối tượng đọc cảm biến BMP180
WiFiClient wifiClient;               // Socket TCP dùng cho MQTT
PubSubClient mqttClient(wifiClient); // Client MQTT gắn trên socket Wi-Fi

QueueHandle_t dataQueue = NULL;      // Mẫu cảm biến đầy đủ, từ Task_ReadData sang Task_TransmitData
QueueHandle_t dataAlert = NULL;      // Nhiệt độ và cao độ, từ Task_ReadData sang Task_Alert
QueueHandle_t commandQueue = NULL;   // Lệnh LED/còi, từ callback MQTT sang Task_MQTTControl
QueueHandle_t statusMailbox = NULL;  // Một ảnh trạng thái mới nhất, ghi đè được
SemaphoreHandle_t serialMutex = NULL; // Khóa để chỉ một task ghi Serial tại một thời điểm
SemaphoreHandle_t outputMutex = NULL; // Khóa trạng thái LED, còi và mức cảnh báo

struct OutputState {                 // Trạng thái đầu ra dùng chung, phải giữ outputMutex khi sửa
  ActuatorControlMode ledMode;       // LED đang AUTO hay MANUAL
  ActuatorControlMode buzzerMode;    // Còi đang AUTO hay MANUAL
  bool ledManualOn;                  // Mức LED khi ở chế độ MANUAL
  bool buzzerManualOn;               // Mức còi khi ở chế độ MANUAL
  TempLevel tempLevel;               // Mức cảnh báo nhiệt độ hiện tại
  AltLevel altLevel;                 // Mức cảnh báo cao độ đã chốt
  bool ledOn;                        // LED đang sáng hay tắt
  bool buzzerOn;                     // Còi đang kêu hay tắt
  uint32_t sequence;                 // Số thứ tự tăng mỗi lần cập nhật đầu ra
  DeviceSnapshot lastQueued;         // Ảnh trạng thái đã ghi vào mailbox lần gần nhất
  bool haveQueued;                   // Đã từng ghi mailbox hay chưa
};

OutputState outputState = {          // Giá trị ban đầu: cả hai đầu ra ở AUTO và tắt
  ACTUATOR_AUTO,                     // LED tự động theo cảnh báo
  ACTUATOR_AUTO,                     // Còi tự động theo cảnh báo
  false,                             // Lệnh tay của LED là tắt
  false,                             // Lệnh tay của còi là tắt
  TEMP_LEVEL_NORMAL,                 // Chưa có cảnh báo nhiệt độ
  ALT_LEVEL_NORMAL,                  // Chưa có cảnh báo cao độ
  false,                             // LED tắt
  false,                             // Còi tắt
  0,                                 // Chưa có lần cập nhật nào
  {0, 0, 0, 0, 0, 0, 0},             // Ảnh mailbox rỗng
  false                              // Chưa ghi mailbox
};

// Chức năng: in một dòng ra Serial. Các task phải xếp hàng qua serialMutex.
void logLine(const char* line) {
  if (serialMutex == NULL) {         // Mutex chưa được tạo thì in trực tiếp
    Serial.println(line);
    return;
  }
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Chờ khóa tối đa 100 ms
    Serial.println(line);            // In đúng một dòng
    xSemaphoreGive(serialMutex);     // Trả khóa cho task khác
  }
}

// Chức năng: đưa phần tử mới vào queue. Queue đầy thì bỏ phần tử cũ nhất rồi gửi lại.
bool sendLatest(QueueHandle_t queue, const void* item, size_t itemSize) {
  if (queue == NULL || item == NULL) return false; // Thiếu queue hoặc dữ liệu thì thất bại
  if (xQueueSend(queue, item, 0) == pdTRUE) return true; // Còn chỗ thì gửi ngay, không chờ

  // Queue đầy: lấy bỏ một phần tử cũ để giữ được mẫu mới nhất.
  uint8_t discarded[64];             // Vùng nhớ tạm để nhận phần tử bị loại
  if (itemSize > sizeof(discarded)) return false; // Phần tử lớn hơn 64 byte thì không loại được
  if (xQueueReceive(queue, discarded, 0) != pdTRUE) return false; // Không lấy được phần tử cũ
  return xQueueSend(queue, item, 0) == pdTRUE; // Gửi lại phần tử mới vào chỗ vừa trống
}

// Chức năng: chép trạng thái đầu ra thành một ảnh gửi đi. Hàm này yêu cầu đang giữ outputMutex.
DeviceSnapshot outputMakeSnapshotLocked() {
  DeviceSnapshot snap;               // Ảnh trạng thái sẽ đưa vào mailbox hoặc MQTT
  snap.ledOn = outputState.ledOn ? 1 : 0;             // 1 nếu LED đang sáng
  snap.buzzerOn = outputState.buzzerOn ? 1 : 0;       // 1 nếu còi đang kêu
  snap.ledMode = (uint8_t)outputState.ledMode;        // Chế độ AUTO hoặc MANUAL của LED
  snap.buzzerMode = (uint8_t)outputState.buzzerMode;  // Chế độ AUTO hoặc MANUAL của còi
  snap.tempLevel = (uint8_t)outputState.tempLevel;    // Mức cảnh báo nhiệt độ
  snap.altLevel = (uint8_t)outputState.altLevel;      // Mức cảnh báo cao độ
  snap.sequence = outputState.sequence;               // Số thứ tự của lần cập nhật này
  return snap;
}

// Chức năng: so hai ảnh theo những mục website thực sự hiển thị.
bool outputVisibleEqual(const DeviceSnapshot& a, const DeviceSnapshot& b) {
  if (a.ledOn != b.ledOn || a.buzzerOn != b.buzzerOn) return false; // LED hoặc còi đổi mức
  if (a.ledMode != b.ledMode || a.buzzerMode != b.buzzerMode) return false; // Đổi AUTO/MANUAL
  if (strcmp(tempLevelMqtt((TempLevel)a.tempLevel), tempLevelMqtt((TempLevel)b.tempLevel)) != 0) return false; // Chuỗi cảnh báo nhiệt độ MQTT khác nhau
  if (strcmp(altLevelMqtt((AltLevel)a.altLevel), altLevelMqtt((AltLevel)b.altLevel)) != 0) return false; // Chuỗi cảnh báo cao độ MQTT khác nhau
  return true;                       // Phần hiển thị giống nhau, không cần phát lại
}

// Chức năng: ghi GPIO khi mức LED hoặc còi thực sự đổi. Yêu cầu đang giữ outputMutex.
void outputApplyGpioLocked(bool ledOn, bool buzzerOn, bool* ledChanged, bool* buzzerChanged) {
  *ledChanged = (outputState.ledOn != ledOn);       // Đánh dấu LED có đổi mức không
  *buzzerChanged = (outputState.buzzerOn != buzzerOn); // Đánh dấu còi có đổi mức không

  if (*ledChanged) {                 // Chỉ ghi chân LED khi mức mới khác mức cũ
    outputState.ledOn = ledOn;       // Lưu mức LED mới
    digitalWrite(PIN_LED, ledOn ? HIGH : LOW); // Bật hoặc tắt LED
  }
  if (*buzzerChanged) {              // Chỉ điều khiển còi khi mức mới khác mức cũ
    outputState.buzzerOn = buzzerOn; // Lưu mức còi mới
    if (buzzerOn) tone(PIN_BUZZER, BUZZER_FREQ_HZ); // Phát tiếng còi 2000 Hz
    else noTone(PIN_BUZZER);         // Tắt tiếng còi
  }
}

// Chức năng: tính mức LED/còi, ghi GPIO và đưa ảnh mới vào mailbox nếu cần phát.
void outputCommitLocked(bool forcePublish, bool* ledChanged, bool* buzzerChanged) {
  AlertActuators alert = combineAlertActuators(outputState.tempLevel, outputState.altLevel); // OR nhu cầu LED/còi của hai loại cảnh báo
  AppliedActuators applied = applyActuatorModes( // AUTO lấy mức cảnh báo, MANUAL lấy mức lệnh tay
    alert,
    outputState.ledMode,
    outputState.ledManualOn,
    outputState.buzzerMode,
    outputState.buzzerManualOn
  );
  outputApplyGpioLocked(applied.ledOn, applied.buzzerOn, ledChanged, buzzerChanged); // Áp mức đã chọn ra chân GPIO
  outputState.sequence++;          // Tăng số thứ tự sau mỗi lần chốt trạng thái

  DeviceSnapshot snap = outputMakeSnapshotLocked(); // Tạo ảnh trạng thái vừa chốt
  bool changed = !outputState.haveQueued || !outputVisibleEqual(outputState.lastQueued, snap); // Lần đầu hoặc phần hiển thị đã đổi
  if ((forcePublish || changed) && statusMailbox != NULL) { // Lệnh tay luôn phát; cảnh báo chỉ phát khi có thay đổi
    outputState.lastQueued = snap;   // Nhớ ảnh vừa xếp hàng
    outputState.haveQueued = true;   // Đánh dấu đã có ảnh trong mailbox
    xQueueOverwrite(statusMailbox, &snap); // Ghi đè ảnh mới nhất, mailbox chỉ chứa 1 phần tử
  }
}

// Chức năng: ghi log sau khi đã nhả outputMutex, để không giữ khóa trong lúc in Serial.
void outputLogGpio(
  bool ledChanged,
  bool buzzerChanged,
  bool ledOn,
  bool buzzerOn,
  ActuatorControlMode ledMode,
  ActuatorControlMode buzzerMode
) {
  char line[80];                     // Bộ đệm một dòng log
  if (ledChanged) {                  // LED vừa đổi thì in mức và chế độ mới
    snprintf(
      line,
      sizeof(line),
      "INFO: LED: %s (%s)",
      ledOn ? "ON" : "OFF",
      ledMode == ACTUATOR_AUTO ? "AUTO" : "MANUAL"
    );
    logLine(line);
  }
  if (buzzerChanged) {               // Còi vừa đổi thì in mức và chế độ mới
    snprintf(
      line,
      sizeof(line),
      "INFO: Buzzer: %s (%s)",
      buzzerOn ? "ON" : "OFF",
      buzzerMode == ACTUATOR_AUTO ? "AUTO" : "MANUAL"
    );
    logLine(line);
  }
}

// Chức năng: Task_Alert gọi hàm này để cập nhật mức cảnh báo rồi dẫn ra LED và còi.
void outputApplyAlert(TempLevel temp, AltLevel alt) {
  if (outputMutex == NULL) return;   // Chưa có mutex thì không đụng trạng thái dùng chung
  if (xSemaphoreTake(outputMutex, pdMS_TO_TICKS(50)) != pdTRUE) { // Chờ khóa tối đa 50 ms
    logLine("ERROR: Alert output lock timeout"); // Hết hạn chờ khóa
    return;
  }
  outputState.tempLevel = temp;      // Lưu mức nhiệt độ mới
  outputState.altLevel = alt;        // Lưu mức cao độ đang được chốt
  bool ledChanged = false;           // outputCommitLocked sẽ gán lại cờ này
  bool buzzerChanged = false;
  outputCommitLocked(false, &ledChanged, &buzzerChanged); // Không ép phát nếu phần hiển thị không đổi
  bool ledOn = outputState.ledOn;    // Chép giá trị ra biến cục bộ trước khi nhả khóa
  bool buzzerOn = outputState.buzzerOn;
  ActuatorControlMode ledMode = outputState.ledMode;
  ActuatorControlMode buzzerMode = outputState.buzzerMode;
  xSemaphoreGive(outputMutex);       // Nhả khóa trước khi ghi log
  outputLogGpio(ledChanged, buzzerChanged, ledOn, buzzerOn, ledMode, buzzerMode); // In thay đổi GPIO
}

// Chức năng: Task_MQTTControl gọi hàm này để áp lệnh ON, OFF hoặc AUTO.
void outputApplyCommand(uint8_t device, uint8_t action) {
  if (outputMutex == NULL) return;   // Chưa có mutex thì bỏ lệnh
  if (xSemaphoreTake(outputMutex, pdMS_TO_TICKS(50)) != pdTRUE) { // Chờ khóa tối đa 50 ms
    logLine("ERROR: Command output lock timeout");
    return;
  }

  ActuatorControlMode* mode = (device == CONTROL_LED) ? &outputState.ledMode : &outputState.buzzerMode; // Trỏ tới chế độ của thiết bị được lệnh
  bool* manualOn = (device == CONTROL_LED) ? &outputState.ledManualOn : &outputState.buzzerManualOn; // Trỏ tới mức tay tương ứng
  if (action == CONTROL_ON) {        // Lệnh ON: khóa tay và bật
    *mode = ACTUATOR_MANUAL;
    *manualOn = true;
  } else if (action == CONTROL_OFF) { // Lệnh OFF: khóa tay và tắt
    *mode = ACTUATOR_MANUAL;
    *manualOn = false;
  } else {                           // Lệnh AUTO: trả thiết bị về mức cảnh báo
    *mode = ACTUATOR_AUTO;
  }

  bool ledChanged = false;
  bool buzzerChanged = false;
  outputCommitLocked(true, &ledChanged, &buzzerChanged); // Ép ghi mailbox dù mức GPIO không đổi
  bool ledOn = outputState.ledOn;    // Chép trạng thái ra ngoài vùng khóa
  bool buzzerOn = outputState.buzzerOn;
  ActuatorControlMode ledMode = outputState.ledMode;
  ActuatorControlMode buzzerMode = outputState.buzzerMode;
  xSemaphoreGive(outputMutex);       // Nhả khóa
  outputLogGpio(ledChanged, buzzerChanged, ledOn, buzzerOn, ledMode, buzzerMode); // Log nếu GPIO đổi

  char line[64];
  const char* name = (device == CONTROL_LED) ? "LED" : "Buzzer"; // Tên thiết bị vừa nhận lệnh
  const char* modeName = (device == CONTROL_LED)
    ? (ledMode == ACTUATOR_AUTO ? "AUTO" : "MANUAL")
    : (buzzerMode == ACTUATOR_AUTO ? "AUTO" : "MANUAL"); // Chế độ sau khi áp lệnh
  snprintf(line, sizeof(line), "INFO: %s mode: %s", name, modeName); // Soạn dòng log chế độ
  logLine(line);
}

// Chức năng: đọc một ảnh trạng thái an toàn từ task khác, không giữ khóa lâu.
void outputRead(DeviceSnapshot* snap) {
  if (snap == NULL || outputMutex == NULL) return; // Không có nơi nhận hoặc chưa có khóa
  if (xSemaphoreTake(outputMutex, pdMS_TO_TICKS(50)) != pdTRUE) return; // Không lấy được khóa thì giữ nguyên snap
  *snap = outputMakeSnapshotLocked(); // Chép ảnh hiện tại
  xSemaphoreGive(outputMutex);        // Nhả khóa ngay
}

// Chức năng: callback MQTT. Chỉ kiểm tra lệnh rồi xếp vào commandQueue, không ghi GPIO tại đây.
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  char message[24];                  // Payload lệnh ngắn: ON, OFF hoặc AUTO
  unsigned int count = length;       // Số byte thực nhận
  if (count > sizeof(message) - 1) count = sizeof(message) - 1; // Chừa một byte cho ký tự kết thúc
  memcpy(message, payload, count);   // Chép payload sang chuỗi
  message[count] = '\0';             // Kết thúc chuỗi

  unsigned int start = 0;            // Vị trí ký tự đầu sau khoảng trắng
  while (message[start] == ' ') start++; // Bỏ khoảng trắng ở đầu
  if (start > 0) memmove(message, message + start, strlen(message + start) + 1); // Dồn chuỗi về đầu bộ đệm
  count = strlen(message);           // Độ dài sau khi cắt đầu
  while (count > 0 && (message[count - 1] == '\r' || message[count - 1] == '\n' || message[count - 1] == ' ')) {
    message[--count] = '\0';         // Cắt xuống dòng và khoảng trắng ở cuối
  }

  char line[96];
  snprintf(line, sizeof(line), "INFO: MQTT received topic=%s payload=%s", topic, message); // Log lệnh vừa nhận
  logLine(line);

  ControlCommand command;            // Lệnh sẽ chuyển sang Task_MQTTControl
  if (strcmp(topic, TOPIC_CONTROL_LED) == 0) command.device = CONTROL_LED; // Topic LED
  else if (strcmp(topic, TOPIC_CONTROL_BUZZER) == 0) command.device = CONTROL_BUZZER; // Topic còi
  else {
    logLine("WARNING: MQTT topic ignored"); // Topic khác hai topic điều khiển
    return;
  }

  if (strcmp(message, "ON") == 0) command.action = CONTROL_ON;
  else if (strcmp(message, "OFF") == 0) command.action = CONTROL_OFF;
  else if (strcmp(message, "AUTO") == 0) command.action = CONTROL_AUTO;
  else {
    snprintf(line, sizeof(line), "WARNING: Unknown %s command", command.device == CONTROL_LED ? "LED" : "Buzzer"); // Payload không hợp lệ
    logLine(line);
    return;
  }

  // Callback đang chạy bên trong mqttClient.loop() của Task_TransmitData.
  // Xếp lệnh sang task khác để không phát MQTT hoặc ghi GPIO ngay trong callback.
  if (!sendLatest(commandQueue, &command, sizeof(command))) {
    logLine("ERROR: Control queue full, command dropped"); // Không còn chỗ sau khi đã bỏ lệnh cũ
  }
}

// Chức năng: phát một chuỗi lên một topic MQTT. retained giữ bản tin cho client kết nối sau.
bool publishText(const char* topic, const char* payload, bool retained) {
  if (!mqttClient.connected()) return false; // Chưa nối broker thì không phát
  bool ok = mqttClient.publish(topic, payload, retained); // Gửi payload
  if (!ok) {
    char line[96];
    snprintf(line, sizeof(line), "ERROR: MQTT publish failed topic=%s", topic); // Broker từ chối hoặc bộ đệm đầy
    logLine(line);
  }
  return ok;
}

// Chức năng: phát cảnh báo nhiệt độ, cảnh báo cao độ và dòng trạng thái tổng hợp.
bool publishSnapshot(const DeviceSnapshot& snap, bool wifiReady, DeviceSnapshot* lastPublished, bool* havePublished) {
  char payload[192];                 // Bộ đệm dòng trạng thái
  bool ok = true;                    // Còn true khi mọi lần phát đều thành công

  const char* tempMqtt = tempLevelMqtt((TempLevel)snap.tempLevel); // LOW, HIGH hoặc NORMAL
  const char* altMqtt = altLevelMqtt((AltLevel)snap.altLevel);     // SUDDEN_CHANGE hoặc CLEAR
  bool alarmChanged = !(*havePublished) // Chưa phát lần nào
    || strcmp(tempLevelMqtt((TempLevel)lastPublished->tempLevel), tempMqtt) != 0 // Chuỗi nhiệt độ đổi
    || strcmp(altLevelMqtt((AltLevel)lastPublished->altLevel), altMqtt) != 0;    // Chuỗi cao độ đổi

  if (alarmChanged) {                // Chỉ phát hai topic cảnh báo khi chuỗi cảnh báo đổi
    ok = publishText(TOPIC_ALARM_TEMP, tempMqtt, true) && ok;
    ok = publishText(TOPIC_ALARM_ALT, altMqtt, true) && ok;
  }

  formatStatusPayload(payload, sizeof(payload), wifiReady, snap); // Tạo dòng wifi, led, buzzer, mode, alarm
  ok = publishText(TOPIC_STATUS, payload, true) && ok; // Luôn phát dòng trạng thái, retained
  if (ok) {
    char line[220];
    snprintf(line, sizeof(line), "INFO: Status %s", payload); // Log đúng payload đã phát
    logLine(line);
    *lastPublished = snap;           // Nhớ ảnh đã phát để lần sau so sánh
    *havePublished = true;
  }
  return ok;                         // false nếu bất kỳ lần publish nào thất bại
}

// Chức năng: log JSON trên Serial và phát nhiệt độ, áp suất, cao độ dạng số thập phân.
bool publishTelemetry(const SensorData& sample) {
  char json[192];                    // JSON chỉ để ghi log, không đưa lên topic cảm biến
  char number[16];                   // Một số dạng "12.34"
  formatSensorJson(json, sizeof(json), sample); // Tạo JSON từ mẫu cảm biến

  char line[220];
  snprintf(line, sizeof(line), "INFO: SensorData JSON %s", json);
  logLine(line);

  formatReading(number, sizeof(number), sample.temperature); // Hai chữ số thập phân
  if (!publishText(TOPIC_TEMPERATURE, number, true)) return false; // Nhiệt độ, retained
  formatReading(number, sizeof(number), sample.pressure);
  if (!publishText(TOPIC_PRESSURE, number, true)) return false;    // Áp suất, đơn vị hPa
  formatReading(number, sizeof(number), sample.altitude);
  if (!publishText(TOPIC_ALTITUDE, number, true)) return false;    // Cao độ, đơn vị mét
  return true;                       // Cả ba topic đều đã phát
}

// Chức năng: Task_ReadData đọc BMP180 mỗi 1 s và gửi mẫu mới nhất cho hai task tiêu thụ.
void taskReadData(void* arg) {
  (void)arg;                         // Task không dùng tham số tạo task
  char line[96];
  snprintf(line, sizeof(line), "INFO: Task_ReadData started on core %d", xPortGetCoreID()); // Ghi core thực tế
  logLine(line);

  bool bmpReady = false;             // Chưa khởi tạo cảm biến thành công
  uint32_t lastBmpAttempt = 0;       // Mốc lần khởi tạo gần nhất, 0 để lần đầu thử ngay
  int consecutiveFailures = 0;       // Số mẫu lỗi liên tiếp
  TickType_t lastWake = xTaskGetTickCount(); // Mốc để giữ đúng chu kỳ 1 s
  const TickType_t period = pdMS_TO_TICKS(SENSOR_INTERVAL_MS); // 1000 ms đổi sang tick

  for (;;) {                         // Vòng lặp vô hạn của task
    uint32_t now = millis();         // Thời gian hiện tại, dùng chung cho khởi tạo và mẫu đọc
    if (!bmpReady) {                 // Cảm biến chưa sẵn sàng
      if (now - lastBmpAttempt >= BMP_RETRY_MS) { // Đã qua 3 s kể từ lần thử trước
        lastBmpAttempt = now;        // Ghi mốc lần thử này
        logLine("INFO: BMP180 initializing...");
        Wire.begin(PIN_SDA, PIN_SCL); // Mở I2C trên GPIO 21 và GPIO 22
        if (!bmp.begin()) {          // Không thấy BMP180 tại địa chỉ 0x77
          logLine("ERROR: BMP180 not found. Check SDA=GPIO21, SCL=GPIO22, VCC=3V3, GND.");
        } else {
          bmpReady = true;           // Khởi tạo thành công
          consecutiveFailures = 0;   // Xóa bộ đếm lỗi trước khi đọc
          logLine("INFO: BMP180 OK");
        }
      }
    }

    if (bmpReady) {                  // Cùng vòng lặp vẫn đọc được ngay sau khi khởi tạo thành công
      float temperature = bmp.readTemperature(); // Nhiệt độ, đơn vị độ C
      int32_t pressurePa = bmp.readPressure();   // Áp suất của chính lần đo này, đơn vị Pa
      // Công thức cao độ của Adafruit, dùng mẫu áp suất vừa đọc, không đo lại lần hai.
      float altitude = 44330.0f * (1.0f - (float)pow((double)pressurePa / (double)SEA_LEVEL_PRESSURE_PA, 0.1903));
      bool valid = !isnan(temperature) // Loại giá trị không phải số
        && !isnan(altitude)
        && temperature >= -40.0f     // Dải nhiệt độ hợp lệ của BMP180
        && temperature <= 85.0f
        && pressurePa >= 30000       // Dải áp suất hợp lệ, đơn vị Pa
        && pressurePa <= 110000;

      if (!valid) {
        consecutiveFailures++;       // Tăng số lỗi liên tiếp
        snprintf(line, sizeof(line), "ERROR: BMP180 invalid reading (%d)", consecutiveFailures);
        logLine(line);
        if (consecutiveFailures >= BMP_FAILS_BEFORE_REINIT) { // Đủ 3 lỗi
          bmpReady = false;          // Vòng sau sẽ khởi tạo lại
          lastBmpAttempt = 0;        // Cho phép thử lại ngay, không chờ thêm 3 s
          logLine("ERROR: BMP180 stopped responding. Reinitializing.");
        }
      } else {
        consecutiveFailures = 0;     // Mẫu tốt làm bộ đếm lỗi về 0
        SensorData sample;           // Mẫu đầy đủ cho task truyền dữ liệu
        sample.temperature = temperature;
        sample.pressure = (float)pressurePa / 100.0f; // Đổi Pa sang hPa
        sample.altitude = altitude;
        sample.timestamp = now;      // Mốc thời gian của mẫu, đơn vị ms

        snprintf(                      // Log giá trị vừa đọc
          line,
          sizeof(line),
          "INFO: T=%.2f C P=%.2f hPa Alt=%.2f m t=%lu",
          sample.temperature,
          sample.pressure,
          sample.altitude,
          (unsigned long)sample.timestamp
        );
        logLine(line);

        if (!sendLatest(dataQueue, &sample, sizeof(sample))) { // Gửi mẫu telemetry, giữ mẫu mới nếu đầy
          logLine("ERROR: dataQueue send failed");
        }

        AlertData alert;             // Bản rút gọn cho task cảnh báo
        alert.temperature = sample.temperature;
        alert.altitude = sample.altitude;
        alert.timestamp = sample.timestamp;
        if (!sendLatest(dataAlert, &alert, sizeof(alert))) {
          logLine("ERROR: dataAlert send failed");
        }
      }
    }

    vTaskDelayUntil(&lastWake, period); // Ngủ đến đúng mốc chu kỳ 1 s
  }
}

// Chức năng: Task_Alert phân loại nhiệt độ, theo dõi cao độ khoảng 5 s và cập nhật LED/còi.
void taskAlert(void* arg) {
  (void)arg;
  char line[120];
  snprintf(line, sizeof(line), "INFO: Task_Alert started on core %d", xPortGetCoreID());
  logLine(line);

  AltitudeTracker tracker;           // Lịch sử cao độ, bộ lọc đột biến và mức đang chốt
  altitudeTrackerInit(&tracker);     // Xóa lịch sử trước mẫu đầu tiên
  TempLevel lastTemp = TEMP_LEVEL_NORMAL; // Mức nhiệt độ của lần log trước
  AltLevel lastAlt = ALT_LEVEL_NORMAL;    // Mức cao độ của lần log trước
  bool haveLevel = false;            // Chưa từng ghi nhận một cặp mức nào

  for (;;) {
    AlertData data;
    if (xQueueReceive(dataAlert, &data, pdMS_TO_TICKS(500)) != pdTRUE) continue; // Chờ mẫu tối đa 500 ms

    do {                             // Xử lý mẫu vừa nhận, sau đó rút nốt các mẫu đang nằm trong queue
      TempLevel temp = classifyTemperature(data.temperature); // LOW, NORMAL, WARNING hoặc HIGH
      AltitudeUpdate altitude = altitudeTrackerPush(&tracker, data.altitude, data.timestamp); // Lọc và chốt mức cao độ

      if (!altitude.accepted) {      // Mẫu nhảy quá 3 m, đang chờ mẫu kế xác nhận
        snprintf(line, sizeof(line), "INFO: Altitude spike held for confirmation, raw=%.2f m", data.altitude);
        logLine(line);
      }

      bool levelChanged = !haveLevel || temp != lastTemp || altitude.level != lastAlt; // Có mức mới cần ghi log
      if (levelChanged) {
        haveLevel = true;
        lastTemp = temp;             // Nhớ mức vừa log
        lastAlt = altitude.level;
        if (!altitude.accepted) {
          snprintf(line, sizeof(line), "INFO: Alarm temp=%s altitude=%s (altitude sample held)", tempLevelName(temp), altLevelName(altitude.level));
        } else if (!altitude.hasReference) { // Chưa có mẫu nằm trong cửa sổ 4,5 đến 6,5 s
          snprintf(
            line,
            sizeof(line),
            "INFO: Alarm temp=%s altitude=waiting for 5 s reference",
            tempLevelName(temp)
          );
        } else if (temp == TEMP_LEVEL_NORMAL && altitude.level == ALT_LEVEL_NORMAL) {
          snprintf(line, sizeof(line), "INFO: Alarm NORMAL delta=%.2f m/5s", altitude.deltaM); // Cả hai đều bình thường
        } else {
          snprintf(                        // Có ít nhất một cảnh báo
            line,
            sizeof(line),
            "WARNING: Alarm temp=%s altitude=%s delta=%.2f m/5s",
            tempLevelName(temp),
            altLevelName(altitude.level),
            altitude.deltaM
          );
        }
        logLine(line);
      }

      // Nhiệt độ vẫn được cập nhật khi mẫu cao độ đang chờ xác nhận.
      // Mức cao độ đã chốt được giữ nguyên trong trường hợp đó.
      outputApplyAlert(temp, altitude.level); // Dẫn mức cảnh báo ra LED và còi
    } while (xQueueReceive(dataAlert, &data, 0) == pdTRUE); // Lấy tiếp mẫu không cần chờ
  }
}

// Chức năng: Task_MQTTControl chờ lệnh trong commandQueue rồi áp dụng cho LED hoặc còi.
void taskMqttControl(void* arg) {
  (void)arg;
  char line[80];
  snprintf(line, sizeof(line), "INFO: Task_MQTTControl started on core %d", xPortGetCoreID());
  logLine(line);

  for (;;) {
    ControlCommand command;
    if (xQueueReceive(commandQueue, &command, portMAX_DELAY) != pdTRUE) continue; // Ngủ đến khi có lệnh
    outputApplyCommand(command.device, command.action); // ON, OFF hoặc AUTO
  }
}

// Chức năng: Task_TransmitData giữ Wi-Fi, MQTT, nhận lệnh điều khiển và phát dữ liệu ra broker.
void taskTransmitData(void* arg) {
  (void)arg;
  char line[96];
  snprintf(line, sizeof(line), "INFO: Task_TransmitData started on core %d", xPortGetCoreID());
  logLine(line);

  WiFi.mode(WIFI_STA);               // ESP32 chỉ đóng vai trò máy trạm
  WiFi.setSleep(false);              // Tắt ngủ Wi-Fi để MQTT không bị trễ
  mqttClient.setServer(MQTT_HOST, MQTT_PORT); // Trỏ client tới broker
  mqttClient.setCallback(onMqttMessage);      // Gọi khi có bản tin điều khiển
  mqttClient.setBufferSize(512);     // Bộ đệm MQTT đủ cho dòng trạng thái
  // Keepalive tính theo thời gian thực. Wokwi thường chạy chậm hơn 100%,
  // nên keepalive ngắn khiến broker ngắt phiên trước khi loop() kịp gửi ping.
  mqttClient.setKeepAlive(120);      // Giữ phiên MQTT trong 120 s
  mqttClient.setSocketTimeout(2);    // Mỗi thao tác socket chờ tối đa 2 s

  bool wifiReady = false;            // Chưa có Wi-Fi dùng được
  bool loggedOffline = false;        // Đã log một lần về việc chưa phát được
  uint32_t lastWifiAttempt = 0;      // Mốc lần gọi WiFi.begin gần nhất
  uint32_t lastMqttAttempt = 0;      // Mốc lần gọi MQTT connect gần nhất
  uint32_t lastTelemetryMs = 0;      // Mốc lần phát telemetry gần nhất
  SensorData pending;                // Mẫu cảm biến mới nhất đang chờ phát
  bool havePending = false;          // pending đã có dữ liệu hợp lệ
  DeviceSnapshot lastPublished;      // Ảnh trạng thái đã phát thành công
  bool havePublished = false;        // Chưa phát trạng thái lần nào trong phiên MQTT này
  memset(&pending, 0, sizeof(pending));             // Xóa mẫu chờ
  memset(&lastPublished, 0, sizeof(lastPublished)); // Xóa ảnh đã phát

  logLine("INFO: WiFi connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL); // Bắt đầu nối trước khi vào vòng lặp
  lastWifiAttempt = millis();        // Ghi mốc để lần mất mạng sau phải chờ đủ 10 s

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) { // Đang có liên kết Wi-Fi
        if (!wifiReady) {            // Đây là lần vừa chuyển từ mất mạng sang có mạng
        wifiReady = true;
        loggedOffline = false;       // Cho phép log lại nếu sau đó mất broker
        String ip = WiFi.localIP().toString(); // Địa chỉ được cấp
        snprintf(line, sizeof(line), "INFO: WiFi connected IP=%s", ip.c_str());
        logLine(line);
      }
    } else {                         // Wi-Fi chưa nối hoặc vừa rớt
      if (wifiReady) {               // Chỉ xử lý một lần tại thời điểm vừa mất mạng
        wifiReady = false;
        logLine("WARNING: WiFi lost. Sensor and alarm tasks keep running.");
        if (mqttClient.connected()) mqttClient.disconnect(); // Ngắt phiên MQTT vì đường truyền đã mất
      }
      uint32_t now = millis();
      if (now - lastWifiAttempt >= WIFI_RETRY_MS) { // Đủ 10 s từ lần begin trước
        lastWifiAttempt = now;
        logLine("INFO: WiFi connecting...");
        WiFi.disconnect();           // Xóa phiên cũ trước khi gọi begin lại
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
      }
    }

    if (wifiReady && !mqttClient.connected()) { // Có Wi-Fi nhưng MQTT chưa nối
      uint32_t now = millis();
      if (now - lastMqttAttempt >= MQTT_RETRY_MS) { // Đủ 4 s từ lần thử trước
        lastMqttAttempt = now;
        char clientId[32];
        snprintf(clientId, sizeof(clientId), "esp32-lab 3-%lu", (unsigned long)now); // ID đổi mỗi lần nối
        snprintf(line, sizeof(line), "INFO: MQTT connecting client=%s", clientId);
        logLine(line);
        bool connected = mqttClient.connect(clientId, TOPIC_WIFI, 0, true, "DISCONNECTED"); // Will: khi mất phiên thì topic Wi-Fi thành DISCONNECTED
        if (!connected) {
          snprintf(line, sizeof(line), "ERROR: MQTT connect failed rc=%d", mqttClient.state()); // Mã lỗi của PubSubClient
          logLine(line);
        } else {
          logLine("INFO: MQTT connected");
          havePublished = false;     // Phiên mới phải phát lại cảnh báo
          bool subLed = mqttClient.subscribe(TOPIC_CONTROL_LED);       // Nhận lệnh LED
          bool subBuzzer = mqttClient.subscribe(TOPIC_CONTROL_BUZZER); // Nhận lệnh còi
          if (!subLed || !subBuzzer) logLine("ERROR: MQTT subscribe failed");
          publishText(TOPIC_WIFI, "CONNECTED", true); // Báo dashboard là Wi-Fi/MQTT đang nối
          DeviceSnapshot snap;
          outputRead(&snap);         // Lấy trạng thái LED, còi và cảnh báo hiện tại
          if (!publishSnapshot(snap, true, &lastPublished, &havePublished)) {
            xQueueOverwrite(statusMailbox, &snap); // Phát lỗi thì giữ ảnh để vòng sau thử lại
          } else {
            DeviceSnapshot queued;
            if (xQueuePeek(statusMailbox, &queued, 0) == pdTRUE && queued.sequence <= snap.sequence) {
              xQueueReceive(statusMailbox, &queued, 0); // Bỏ ảnh mailbox cũ hơn hoặc bằng ảnh vừa phát
            }
          }
          lastTelemetryMs = 0;       // Cho phép phát telemetry ngay ở vòng đủ điều kiện
        }
      }
    }

    if (wifiReady) mqttClient.loop(); // Duy trì MQTT và gọi onMqttMessage khi có lệnh. Mất Wi-Fi thì bỏ qua.

    DeviceSnapshot snap;
    if (xQueueReceive(statusMailbox, &snap, 0) == pdTRUE) { // Có ảnh trạng thái mới, không chờ
      if (mqttClient.connected()) {
        if (!publishSnapshot(snap, wifiReady, &lastPublished, &havePublished)) {
          xQueueOverwrite(statusMailbox, &snap); // Phát lỗi thì đưa ảnh trở lại mailbox
        }
      } else {
        xQueueOverwrite(statusMailbox, &snap); // Chưa có broker thì giữ ảnh, không bỏ
        if (!loggedOffline) {
          logLine("WARNING: Status publish skipped, broker not connected");
          loggedOffline = true;     // Chỉ log một lần cho đến khi nối lại
        }
      }
    }

    SensorData sample;
    while (xQueueReceive(dataQueue, &sample, 0) == pdTRUE) { // Rút hết queue
      pending = sample;              // Mỗi lần ghi đè, nên pending là mẫu mới nhất
      havePending = true;
    }

    if (havePending && mqttClient.connected()) { // Có mẫu và broker đang nhận
      uint32_t now = millis();
      if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) { // Đã qua 2 s
        if (publishTelemetry(pending)) {
          havePending = false;       // Mẫu này đã phát xong
          lastTelemetryMs = now;
          loggedOffline = false;
        } else {
          lastTelemetryMs = now - TELEMETRY_INTERVAL_MS + 1000; // Thử lại sau khoảng 1 s
        }
      }
    } else if (havePending && !mqttClient.connected() && !loggedOffline) {
      logLine("WARNING: Telemetry held locally, broker not connected"); // Giữ mẫu cục bộ
      loggedOffline = true;
    }

    vTaskDelay(pdMS_TO_TICKS(20));   // Nhường CPU 20 ms rồi lặp lại từ bước kiểm tra Wi-Fi
  }
}

// Chức năng: khởi tạo Serial, GPIO, queue, mutex và bốn task FreeRTOS.
void setup() {
  Serial.begin(115200);              // Mở cổng log tốc độ 115200
  delay(200);                        // Chờ cổng Serial ổn định
  Serial.println();
  Serial.println("INFO: Runner monitor starting");

  pinMode(PIN_LED, OUTPUT);          // GPIO 26 là đầu ra
  digitalWrite(PIN_LED, LOW);        // Tắt LED lúc khởi động
  pinMode(PIN_BUZZER, OUTPUT);       // GPIO 25 là đầu ra
  digitalWrite(PIN_BUZZER, LOW);     // Đưa chân còi xuống mức thấp
  noTone(PIN_BUZZER);                // Bảo đảm chưa có sóng phát ra còi
  Serial.println("INFO: LED: OFF (AUTO)");
  Serial.println("INFO: Buzzer: OFF (AUTO)");

  serialMutex = xSemaphoreCreateMutex(); // Khóa dùng chung cho logLine
  outputMutex = xSemaphoreCreateMutex(); // Khóa dùng chung cho trạng thái đầu ra
  dataQueue = xQueueCreate(QUEUE_DEPTH, sizeof(SensorData));         // 8 mẫu cảm biến
  dataAlert = xQueueCreate(QUEUE_DEPTH, sizeof(AlertData));          // 8 mẫu cảnh báo
  commandQueue = xQueueCreate(QUEUE_DEPTH, sizeof(ControlCommand));  // 8 lệnh điều khiển
  statusMailbox = xQueueCreate(1, sizeof(DeviceSnapshot));           // Mailbox một ảnh trạng thái

  if (serialMutex == NULL || outputMutex == NULL || dataQueue == NULL
      || dataAlert == NULL || commandQueue == NULL || statusMailbox == NULL) {
    Serial.println("ERROR: FreeRTOS queue or mutex allocation failed"); // Hết heap, không tạo task
    return;
  }

  if (sizeof(SensorData) > 64 || sizeof(AlertData) > 64 || sizeof(ControlCommand) > 64) {
    Serial.println("ERROR: Queue item does not fit the drop-oldest buffer"); // sendLatest chỉ loại được phần tử tối đa 64 byte
    return;
  }

  xTaskCreatePinnedToCore(taskReadData, "Task_ReadData", 6144, NULL, TASK_PRIORITY, NULL, 1);         // Core 1, stack 6144 byte, ưu tiên 2
  xTaskCreatePinnedToCore(taskTransmitData, "Task_TransmitData", 10240, NULL, TASK_PRIORITY, NULL, 0); // Core 0 vì chạy Wi-Fi và MQTT
  xTaskCreatePinnedToCore(taskAlert, "Task_Alert", 5120, NULL, TASK_PRIORITY, NULL, 1);               // Core 1, xử lý cảnh báo
  xTaskCreatePinnedToCore(taskMqttControl, "Task_MQTTControl", 4096, NULL, TASK_PRIORITY, NULL, 1);   // Core 1, áp lệnh điều khiển
  Serial.println("INFO: FreeRTOS tasks started");
}

// Chức năng: vòng Arduino vẫn tồn tại sau khi scheduler chạy. Hàm này chỉ nhường CPU.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // Ngủ 1 s, không làm việc của các task
}
