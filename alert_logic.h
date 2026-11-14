#pragma once

// Luật cảnh báo thuần cho bộ giám sát vận động viên.
// File này không gọi Arduino, FreeRTOS hay GPIO, nên ESP32 và chương trình kiểm tra trên máy đều chạy cùng một luật.
//
// Tương thích dashboard:
// Website chỉ hiểu cảnh báo nhiệt độ là HIGH, LOW hoặc NORMAL.
// Website chỉ hiểu cảnh báo cao độ là SUDDEN_CHANGE hoặc CLEAR.
// Topic cảm biến được đọc bằng Number(text), nên payload phải là số thập phân thuần.
// WARNING_TEMP được phát thành HIGH. WARNING_ALTITUDE và HIGH_ALTITUDE đều phát thành SUDDEN_CHANGE.
// Trạng thái LED và còi vẫn cho biết đầu ra vật lý mà firmware đã áp dụng.

#include <math.h>                    // fabsf cho độ lệch cao độ
#include <stdio.h>                   // snprintf cho payload
#include <stdint.h>                  // uint32_t, uint8_t
#include <string.h>                  // strcmp khi so chuỗi mức cảnh báo

static const float TEMP_LOW_C = 10.0f;       // Dưới 10 độ C là LOW
static const float TEMP_WARNING_C = 32.0f;   // Đến 32 độ C vẫn là NORMAL
static const float TEMP_HIGH_C = 35.0f;      // Trên 32 đến 35 là WARNING, trên 35 là HIGH

static const float ALT_WARNING_M = 5.0f;         // Lệch từ 5 m thì vào WARNING
static const float ALT_HIGH_M = 10.0f;           // Lệch từ 10 m thì vào HIGH
static const float ALT_WARNING_RELEASE_M = 4.0f; // Dưới 4 m mới được xét thoát cảnh báo
static const float ALT_HIGH_RELEASE_M = 9.0f;    // Từ 9 m trở lên thì HIGH được giữ
static const float ALT_SPIKE_M = 3.0f;           // Nhảy hơn 3 m so với mẫu đã nhận thì tạm giữ
static const float ALT_SPIKE_CONFIRM_M = 1.0f;   // Mẫu sau phải nằm trong 1 m quanh mẫu đang chờ

static const uint32_t ALT_WINDOW_MS = 5000;      // Mốc lý tưởng để so cao độ: 5 s trước
static const uint32_t ALT_WINDOW_MIN_MS = 4500;  // Mẫu tham chiếu không được mới hơn 4,5 s
static const uint32_t ALT_WINDOW_MAX_MS = 6500;  // Mẫu tham chiếu không được cũ hơn 6,5 s
static const uint32_t ALT_HOLD_MS = 5000;        // Giữ mức cảnh báo ít nhất 5 s
static const int ALT_RELEASE_STREAK = 2;         // Cần 2 mẫu liên tiếp mới hạ mức
static const int ALT_HISTORY_LEN = 16;           // Tối đa 16 mẫu cao độ đã chấp nhận

enum TempLevel {                     // Mức nhiệt độ nội bộ, trước khi đổi sang chuỗi MQTT
  TEMP_LEVEL_NORMAL = 0,
  TEMP_LEVEL_WARNING = 1,
  TEMP_LEVEL_HIGH = 2,
  TEMP_LEVEL_LOW = 3
};

enum AltLevel {                      // Mức biến thiên cao độ nội bộ
  ALT_LEVEL_NORMAL = 0,
  ALT_LEVEL_WARNING = 1,
  ALT_LEVEL_HIGH = 2
};

enum ActuatorControlMode {           // Cách quyết định LED hoặc còi
  ACTUATOR_AUTO = 0,                 // Theo mức cảnh báo
  ACTUATOR_MANUAL = 1                // Theo lệnh ON/OFF
};

struct SensorData {                  // Một lần đọc đầy đủ của BMP180
  float temperature;                 // Nhiệt độ, độ C
  float pressure;                    // Áp suất, hPa
  float altitude;                    // Cao độ, mét
  uint32_t timestamp;                // Mốc millis lúc đọc
};

struct AlertData {                   // Phần dữ liệu Task_Alert cần từ mỗi lần đọc
  float temperature;
  float altitude;
  uint32_t timestamp;
};

struct DeviceSnapshot {              // Ảnh trạng thái đưa qua mailbox và MQTT
  uint8_t ledOn;                     // 1 khi LED sáng
  uint8_t buzzerOn;                  // 1 khi còi kêu
  uint8_t ledMode;                   // AUTO hoặc MANUAL
  uint8_t buzzerMode;
  uint8_t tempLevel;                 // Giá trị TempLevel
  uint8_t altLevel;                  // Giá trị AltLevel
  uint32_t sequence;                 // Số thứ tự lần cập nhật
};

struct AlertActuators {              // Nhu cầu bật LED và còi chỉ tính từ cảnh báo
  bool ledOn;
  bool buzzerOn;
};

struct AppliedActuators {            // Mức thực tế sau khi xét AUTO hoặc MANUAL
  bool ledOn;
  bool buzzerOn;
};

struct AltSample {                   // Một mẫu cao độ đã được bộ lọc chấp nhận
  float altitudeM;
  uint32_t timestampMs;
};

struct SpikeFilter {                 // Bộ lọc một mẫu cao độ nhảy bất thường
  bool hasAccepted;                  // Đã có mẫu được nhận
  bool hasPending;                   // Đang giữ một mẫu chờ xác nhận
  float acceptedM;                   // Mẫu hợp lệ gần nhất
  float pendingM;                    // Mẫu bị nghi ngờ
};

struct AltLatchState {               // Trạng thái chốt mức cao độ
  AltLevel level;                    // Mức đang giữ
  uint8_t clearStreak;               // Số mẫu liên tiếp đủ điều kiện về NORMAL
  uint8_t releaseStreak;             // Số mẫu liên tiếp đủ điều kiện từ HIGH xuống WARNING
  uint32_t holdUntilMs;              // Mức không được hạ trước mốc này
};

struct AltitudeTracker {             // Toàn bộ trạng thái theo dõi cao độ của Task_Alert
  AltSample samples[ALT_HISTORY_LEN];
  int count;                         // Số mẫu đang lưu
  SpikeFilter spike;
  AltLatchState latch;
  float lastDeltaM;                  // Độ lệch tuyệt đối gần nhất, mét trên khoảng 5 s
  bool lastHasReference;             // Lần gần nhất có tìm được mốc tham chiếu
};

struct AltitudeUpdate {              // Kết quả trả về cho Task_Alert sau một mẫu
  AltLevel level;                    // Mức đang chốt
  bool accepted;                     // false nếu mẫu bị giữ vì nghi đột biến
  bool hasReference;                 // true khi đã so được với mốc khoảng 5 s
  float deltaM;                      // Độ lệch tuyệt đối, mét
};

// Chức năng: phân loại nhiệt độ thành LOW, NORMAL, WARNING hoặc HIGH.
inline TempLevel classifyTemperature(float temperatureC) {
  if (temperatureC < TEMP_LOW_C) return TEMP_LEVEL_LOW;           // Dưới 10 độ C
  if (temperatureC <= TEMP_WARNING_C) return TEMP_LEVEL_NORMAL;   // Từ 10 đến 32 độ C
  if (temperatureC <= TEMP_HIGH_C) return TEMP_LEVEL_WARNING;     // Trên 32 đến 35 độ C
  return TEMP_LEVEL_HIGH;                                         // Trên 35 độ C
}

// Chức năng: phân loại độ lệch cao độ tức thời, chưa áp dụng thời gian giữ.
inline AltLevel classifyAltitudeDelta(float deltaM) {
  if (deltaM < ALT_WARNING_M) return ALT_LEVEL_NORMAL; // Dưới 5 m
  if (deltaM < ALT_HIGH_M) return ALT_LEVEL_WARNING;   // Từ 5 m đến dưới 10 m
  return ALT_LEVEL_HIGH;                               // Từ 10 m trở lên
}

// Chức năng: tên mức nhiệt độ dùng cho log Serial.
inline const char* tempLevelName(TempLevel level) {
  if (level == TEMP_LEVEL_WARNING) return "WARNING_TEMP";
  if (level == TEMP_LEVEL_HIGH) return "HIGH_TEMP";
  if (level == TEMP_LEVEL_LOW) return "LOW_TEMP";
  return "NORMAL";
}

// Chức năng: tên mức cao độ dùng cho log Serial.
inline const char* altLevelName(AltLevel level) {
  if (level == ALT_LEVEL_WARNING) return "WARNING_ALTITUDE";
  if (level == ALT_LEVEL_HIGH) return "HIGH_ALTITUDE";
  return "NORMAL";
}

// Chức năng: chuỗi nhiệt độ mà dashboard chấp nhận. WARNING và HIGH đều thành HIGH.
inline const char* tempLevelMqtt(TempLevel level) {
  if (level == TEMP_LEVEL_LOW) return "LOW";
  if (level == TEMP_LEVEL_WARNING || level == TEMP_LEVEL_HIGH) return "HIGH";
  return "NORMAL";
}

// Chức năng: chuỗi cao độ mà dashboard chấp nhận. Cả hai mức cảnh báo đều thành SUDDEN_CHANGE.
inline const char* altLevelMqtt(AltLevel level) {
  if (level == ALT_LEVEL_WARNING || level == ALT_LEVEL_HIGH) return "SUDDEN_CHANGE";
  return "CLEAR";
}

// Chức năng: mọi mức khác NORMAL đều yêu cầu bật LED vì nhiệt độ.
inline bool temperatureWantsLed(TempLevel level) {
  return level != TEMP_LEVEL_NORMAL;
}

// Chức năng: chỉ LOW và HIGH mới yêu cầu còi. WARNING chỉ bật LED.
inline bool temperatureWantsBuzzer(TempLevel level) {
  return level == TEMP_LEVEL_HIGH || level == TEMP_LEVEL_LOW;
}

// Chức năng: WARNING hoặc HIGH của cao độ đều yêu cầu bật LED.
inline bool altitudeWantsLed(AltLevel level) {
  return level == ALT_LEVEL_WARNING || level == ALT_LEVEL_HIGH;
}

// Chức năng: chỉ HIGH của cao độ mới yêu cầu còi.
inline bool altitudeWantsBuzzer(AltLevel level) {
  return level == ALT_LEVEL_HIGH;
}

// Chức năng: bật LED hoặc còi khi nhiệt độ hoặc cao độ có nhu cầu.
inline AlertActuators combineAlertActuators(TempLevel temp, AltLevel alt) {
  AlertActuators out;
  out.ledOn = temperatureWantsLed(temp) || altitudeWantsLed(alt);       // Một trong hai cảnh báo cần LED
  out.buzzerOn = temperatureWantsBuzzer(temp) || altitudeWantsBuzzer(alt); // Một trong hai cảnh báo cần còi
  return out;
}

// Chức năng: AUTO dùng mức cảnh báo, MANUAL dùng mức lệnh tay.
inline AppliedActuators applyActuatorModes(
  AlertActuators alert,
  ActuatorControlMode ledMode,
  bool ledManualOn,
  ActuatorControlMode buzzerMode,
  bool buzzerManualOn
) {
  AppliedActuators out;
  out.ledOn = (ledMode == ACTUATOR_AUTO) ? alert.ledOn : ledManualOn;             // Chọn nguồn mức LED
  out.buzzerOn = (buzzerMode == ACTUATOR_AUTO) ? alert.buzzerOn : buzzerManualOn; // Chọn nguồn mức còi
  return out;
}

// Chức năng: định dạng một số thực thành chuỗi hai chữ số thập phân.
inline int formatReading(char* out, int outSize, float value) {
  return snprintf(out, (size_t)outSize, "%.2f", value);
}

// Chức năng: tạo JSON của mẫu cảm biến cho log Serial.
inline int formatSensorJson(char* out, int outSize, const SensorData& data) {
  return snprintf(
    out,
    (size_t)outSize,
    "{\"temperature\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"timestamp\":%lu}",
    data.temperature,
    data.pressure,
    data.altitude,
    (unsigned long)data.timestamp
  );
}

// Chức năng: tạo dòng trạng thái đúng định dạng dashboard đọc.
inline int formatStatusPayload(
  char* out,
  int outSize,
  bool wifiConnected,
  const DeviceSnapshot& snap
) {
  return snprintf(
    out,
    (size_t)outSize,
    "wifi=%s;led=%s;buzzer=%s;led_mode=%s;buzzer_mode=%s;temp_alarm=%s;alt_alarm=%s",
    wifiConnected ? "CONNECTED" : "DISCONNECTED",                 // Trạng thái Wi-Fi
    snap.ledOn ? "ON" : "OFF",                                    // LED
    snap.buzzerOn ? "ON" : "OFF",                                 // Còi
    snap.ledMode == ACTUATOR_MANUAL ? "MANUAL" : "AUTO",          // Chế độ LED
    snap.buzzerMode == ACTUATOR_MANUAL ? "MANUAL" : "AUTO",       // Chế độ còi
    tempLevelMqtt((TempLevel)snap.tempLevel),                     // Cảnh báo nhiệt độ dạng MQTT
    altLevelMqtt((AltLevel)snap.altLevel)                         // Cảnh báo cao độ dạng MQTT
  );
}

// Chức năng: đưa bộ lọc đột biến về trạng thái chưa có mẫu.
inline void spikeFilterInit(SpikeFilter* filter) {
  filter->hasAccepted = false;      // Mẫu kế tiếp sẽ được nhận ngay
  filter->hasPending = false;       // Không có mẫu đang chờ
  filter->acceptedM = 0.0f;
  filter->pendingM = 0.0f;
}

// Chức năng: nhận mẫu cao độ hoặc giữ lại nếu nó nhảy hơn 3 m so với mẫu đã nhận.
// Mẫu đầu tiên luôn được nhận. Mẫu bị giữ chỉ được nhận khi mẫu sau nằm trong 1 m quanh nó.
inline bool spikeFilterAccept(SpikeFilter* filter, float sampleM, float* acceptedOut) {
  if (!filter->hasAccepted) {        // Chưa có mốc nào để so
    filter->hasAccepted = true;
    filter->hasPending = false;
    filter->acceptedM = sampleM;     // Nhận mẫu đầu tiên
    *acceptedOut = sampleM;
    return true;
  }

  if (fabsf(sampleM - filter->acceptedM) <= ALT_SPIKE_M) { // Lệch không quá 3 m so với mẫu đã nhận
    filter->hasPending = false;     // Hủy mẫu đang chờ vì mẫu này hợp lệ
    filter->acceptedM = sampleM;
    *acceptedOut = sampleM;
    return true;
  }

  if (filter->hasPending && fabsf(sampleM - filter->pendingM) <= ALT_SPIKE_CONFIRM_M) {
    filter->hasPending = false;     // Mẫu mới xác nhận giá trị đang chờ
    filter->acceptedM = sampleM;     // Nhận mẫu hiện tại, không nhận lại mẫu chờ
    *acceptedOut = sampleM;
    return true;
  }

  filter->pendingM = sampleM;        // Giữ mẫu nghi ngờ cho lần so kế tiếp
  filter->hasPending = true;
  return false;                      // Chưa cập nhật lịch sử và mức chốt
}

// Chức năng: đưa bộ chốt mức cao độ về NORMAL, chưa giữ thời gian.
inline void altitudeLatchInit(AltLatchState* state) {
  state->level = ALT_LEVEL_NORMAL;
  state->clearStreak = 0;            // Chưa có mẫu thoát về NORMAL
  state->releaseStreak = 0;          // Chưa có mẫu hạ từ HIGH xuống WARNING
  state->holdUntilMs = 0;            // 0 nghĩa là không còn thời gian giữ
}

// Chức năng: cho biết thời gian giữ tối thiểu 5 s còn hiệu lực hay không.
inline bool altitudeHoldActive(const AltLatchState* state, uint32_t nowMs) {
  if (state->holdUntilMs == 0) return false; // Không có mốc giữ
  return (int32_t)(nowMs - state->holdUntilMs) < 0; // nowMs vẫn nằm trước mốc giữ
}

// Chức năng: nâng mức ngay, nhưng chỉ hạ mức sau khi hết giữ 5 s và đủ 2 mẫu.
inline AltLevel altitudeLatchUpdate(
  AltLatchState* state,
  bool hasReference,
  float deltaM,
  uint32_t nowMs
) {
  if (!hasReference) return state->level; // Chưa có mốc 5 s thì giữ nguyên mức

  AltLevel instant = classifyAltitudeDelta(deltaM); // Mức tức thời của độ lệch này

  if (state->level == ALT_LEVEL_NORMAL) {
    state->clearStreak = 0;          // Đang bình thường nên không có chuỗi thoát
    state->releaseStreak = 0;
    if (instant != ALT_LEVEL_NORMAL) { // Từ 5 m trở lên
      state->level = instant;        // Lên WARNING hoặc lên thẳng HIGH
      state->holdUntilMs = nowMs + ALT_HOLD_MS; // Giữ mức mới ít nhất 5 s
    }
    return state->level;
  }

  if (instant == ALT_LEVEL_HIGH && state->level != ALT_LEVEL_HIGH) { // Đủ 10 m khi đang ở WARNING
    state->level = ALT_LEVEL_HIGH;   // Nâng ngay, không cần đủ 2 mẫu
    state->holdUntilMs = nowMs + ALT_HOLD_MS; // Tính lại thời gian giữ
    state->clearStreak = 0;
    state->releaseStreak = 0;
    return state->level;
  }

  bool hold = altitudeHoldActive(state, nowMs); // true thì chưa được hạ mức

  if (state->level == ALT_LEVEL_HIGH) {
    if (instant == ALT_LEVEL_HIGH || deltaM >= ALT_HIGH_RELEASE_M) { // Còn từ 9 m trở lên
      state->clearStreak = 0;        // Phá chuỗi thoát về NORMAL
      state->releaseStreak = 0;      // Phá chuỗi hạ xuống WARNING
      return state->level;           // Giữ HIGH
    }
    if (deltaM < ALT_WARNING_RELEASE_M) { // Dưới 4 m: xét thoát thẳng về NORMAL
      state->releaseStreak = 0;
      state->clearStreak++;
      if (!hold && state->clearStreak >= ALT_RELEASE_STREAK) { // Hết giữ và đủ 2 mẫu
        state->level = ALT_LEVEL_NORMAL;
        state->clearStreak = 0;
        state->holdUntilMs = 0;
      }
      return state->level;
    }
    state->clearStreak = 0;          // Dải 4 m đến dưới 9 m không đủ để về NORMAL
    state->releaseStreak++;          // Đếm mẫu để hạ HIGH xuống WARNING
    if (!hold && state->releaseStreak >= ALT_RELEASE_STREAK) {
      state->level = ALT_LEVEL_WARNING;
      state->releaseStreak = 0;
    }
    return state->level;
  }

  if (deltaM < ALT_WARNING_RELEASE_M) { // Đang WARNING và lệch đã xuống dưới 4 m
    state->clearStreak++;
    if (!hold && state->clearStreak >= ALT_RELEASE_STREAK) {
      state->level = ALT_LEVEL_NORMAL;
      state->clearStreak = 0;
      state->holdUntilMs = 0;
    }
  } else {
    state->clearStreak = 0;          // Lệch quay lại từ 4 m trở lên thì đếm lại từ đầu
  }
  return state->level;
}

// Chức năng: xóa lịch sử, bộ lọc đột biến và bộ chốt mức.
inline void altitudeTrackerInit(AltitudeTracker* tracker) {
  tracker->count = 0;                // Chưa lưu mẫu nào
  spikeFilterInit(&tracker->spike);
  altitudeLatchInit(&tracker->latch);
  tracker->lastDeltaM = 0.0f;
  tracker->lastHasReference = false;
  for (int i = 0; i < ALT_HISTORY_LEN; i++) { // Xóa toàn bộ mảng lịch sử
    tracker->samples[i].altitudeM = 0.0f;
    tracker->samples[i].timestampMs = 0;
  }
}

// Chức năng: lưu một mẫu đã chấp nhận. Cùng mốc thời gian thì ghi đè mẫu cuối.
inline void altitudeTrackerStore(AltitudeTracker* tracker, float altitudeM, uint32_t timestampMs) {
  if (tracker->count > 0 && tracker->samples[tracker->count - 1].timestampMs == timestampMs) {
    tracker->samples[tracker->count - 1].altitudeM = altitudeM; // Không tạo thêm mẫu trùng thời gian
    return;
  }
  if (tracker->count == ALT_HISTORY_LEN) { // Mảng đầy thì bỏ mẫu cũ nhất
    for (int i = 1; i < ALT_HISTORY_LEN; i++) {
      tracker->samples[i - 1] = tracker->samples[i];
    }
    tracker->count = ALT_HISTORY_LEN - 1;
  }
  tracker->samples[tracker->count].altitudeM = altitudeM;     // Ghi vào vị trí trống cuối
  tracker->samples[tracker->count].timestampMs = timestampMs;
  tracker->count++;
}

// Chức năng: tìm mẫu đã lưu có tuổi gần 5 s nhất và nằm trong cửa sổ 4,5 đến 6,5 s.
// Không so hai mẫu liên tiếp với nhau.
inline bool altitudeTrackerReference(
  const AltitudeTracker* tracker,
  uint32_t nowMs,
  float* currentOut,
  float* referenceOut
) {
  if (tracker->count < 2) return false; // Cần ít nhất mẫu hiện tại và một mẫu cũ

  int bestIndex = -1;                // Vị trí mẫu gần mốc 5 s nhất
  uint32_t bestError = 0xFFFFFFFFu;  // Sai lệch tuổi nhỏ nhất tìm được
  for (int i = 0; i < tracker->count; i++) {
    uint32_t age = nowMs - tracker->samples[i].timestampMs; // Tuổi của mẫu, ms
    if (age < ALT_WINDOW_MIN_MS || age > ALT_WINDOW_MAX_MS) continue; // Ngoài cửa sổ thì bỏ
    uint32_t error = (age > ALT_WINDOW_MS) ? (age - ALT_WINDOW_MS) : (ALT_WINDOW_MS - age); // Khoảng cách tới đúng 5 s
    if (error < bestError) {
      bestError = error;
      bestIndex = i;
    }
  }
  if (bestIndex < 0) return false;   // Không mẫu nào nằm trong cửa sổ
  if (bestIndex == tracker->count - 1) return false; // Không dùng chính mẫu mới nhất làm mốc của nó

  *currentOut = tracker->samples[tracker->count - 1].altitudeM; // Mẫu mới nhất
  *referenceOut = tracker->samples[bestIndex].altitudeM;        // Mẫu gần mốc 5 s nhất
  return true;
}

// Chức năng: nhận một mẫu cao độ thô, lọc đột biến, tính lệch khoảng 5 s và cập nhật mức chốt.
inline AltitudeUpdate altitudeTrackerPush(
  AltitudeTracker* tracker,
  float altitudeM,
  uint32_t timestampMs
) {
  AltitudeUpdate update;
  update.level = tracker->latch.level;       // Mặc định trả mức đang chốt
  update.accepted = false;                   // Chưa biết mẫu có được nhận không
  update.hasReference = false;
  update.deltaM = tracker->lastDeltaM;       // Giữ độ lệch cũ nếu mẫu bị loại

  float accepted = 0.0f;
  if (!spikeFilterAccept(&tracker->spike, altitudeM, &accepted)) {
    return update;                 // Mẫu đột biến bị giữ, không đổi latch
  }

  altitudeTrackerStore(tracker, accepted, timestampMs); // Đưa mẫu hợp lệ vào lịch sử
  update.accepted = true;

  float currentM = 0.0f;
  float referenceM = 0.0f;
  bool hasReference = altitudeTrackerReference(tracker, timestampMs, &currentM, &referenceM);
  float delta = hasReference ? fabsf(currentM - referenceM) : 0.0f; // Độ lệch tuyệt đối, mét
  update.hasReference = hasReference;
  update.deltaM = delta;
  update.level = altitudeLatchUpdate(&tracker->latch, hasReference, delta, timestampMs); // Nâng hoặc hạ mức

  tracker->lastHasReference = hasReference; // Nhớ kết quả cho lần gọi sau
  tracker->lastDeltaM = delta;
  return update;
}
