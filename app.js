// ============================================================
// RUNNER MONITORING SYSTEM – Web Dashboard
//
// MQTT Broker  : broker.emqx.io
// Transport    : WebSocket (ws:// port 8083, wss:// port 8084)
// ESP32 payload: plain-text strings, NOT JSON
// Topics       : runner/lab 3/#
//
// Architecture:
//   CONFIG  – broker & topic constants
//   STATE   – live application state
//   CLOCK   – real-time header clock
//   LOG     – activity log with types and XSS-safe rendering
//   CHARTS  – Chart.js lifecycle management
//   UI      – DOM update helpers (status, sensors, actuators, alarms)
//   MQTT    – connection, subscription, message dispatch
//   CTRL    – button bindings and command publishing
//   INIT    – boot sequence
// ============================================================

// ─────────────────────────────────────────────
// CONFIG
// ─────────────────────────────────────────────
const pageIsHttps = window.location.protocol === "https:";

const MQTT_CONFIG = {
  host:     "broker.emqx.io",
  wsPort:   pageIsHttps ? 8084 : 8083,
  path:     "/mqtt",
  protocol: pageIsHttps ? "wss" : "ws"
};

// All topics must stay in sync with firmware (sketch.ino).
// Do NOT rename topics without changing both sides.
const TOPICS = {
  temperature:    "runner/lab 3/temperature",
  pressure:       "runner/lab 3/pressure",
  altitude:       "runner/lab 3/altitude",
  wifi:           "runner/lab 3/wifi",
  status:         "runner/lab 3/status",
  alarmTemp:      "runner/lab 3/alarm/temperature",
  alarmAlt:       "runner/lab 3/alarm/altitude",
  controlLed:     "runner/lab 3/control/led",
  controlBuzzer:  "runner/lab 3/control/buzzer"
};

const MQTT_URL       = `${MQTT_CONFIG.protocol}://${MQTT_CONFIG.host}:${MQTT_CONFIG.wsPort}${MQTT_CONFIG.path}`;
const MAX_CHART_PTS  = 30;   // keep last N points per chart
const MAX_LOG        = 50;   // max log rows
const STALE_MS       = 30000; // mark sensor data stale after 30 s

// ─────────────────────────────────────────────
// STATE
// Application state reflects confirmed ESP32 state from MQTT feedback.
// Never mutate state to reflect a command before ESP32 confirms it.
// ─────────────────────────────────────────────
const State = {
  tempAlarm:    null, // "HIGH" | "LOW" | "NORMAL" | null
  altAlarm:     null, // "SUDDEN_CHANGE" | "CLEAR" | null
  led:          null, // "ON" | "OFF" | null
  ledMode:      null, // "AUTO" | "MANUAL" | null
  buzzer:       null, // "ON" | "OFF" | null
  buzzerMode:   null, // "AUTO" | "MANUAL" | null
  lastDataMs:   null  // timestamp of last telemetry message
};

// ─────────────────────────────────────────────
// DOM CACHE
// All getElementById calls in one place.
// ─────────────────────────────────────────────
const D = {
  // Header
  currentTime:       document.getElementById("current-time"),
  lastDataTime:      document.getElementById("last-data-time"),
  mqttHeaderBadge:   document.getElementById("mqtt-header-badge"),

  // Status cards + dots + card wrappers
  mqttStatus:        document.getElementById("mqtt-status"),
  mqttDot:           document.getElementById("mqtt-dot"),
  scMqtt:            document.getElementById("sc-mqtt"),

  wifiStatus:        document.getElementById("wifi-status"),
  wifiDot:           document.getElementById("wifi-dot"),
  scWifi:            document.getElementById("sc-wifi"),

  alarmStatus:       document.getElementById("alarm-status"),
  alarmDot:          document.getElementById("alarm-dot"),
  scAlarm:           document.getElementById("sc-alarm"),

  // Sensor values
  temperature:       document.getElementById("temperature"),
  pressure:          document.getElementById("pressure"),
  altitude:          document.getElementById("altitude"),

  // Sensor annotation
  tempAlarmBadge:    document.getElementById("temp-alarm-badge"),
  altAlarmBadge:     document.getElementById("alt-alarm-badge"),
  tempAlarmText:     document.getElementById("temp-alarm-text"),
  altAlarmText:      document.getElementById("alt-alarm-text"),

  // Sensor timestamps
  tempTime:          document.getElementById("temp-time"),
  presTime:          document.getElementById("pres-time"),
  altTime:           document.getElementById("alt-time"),

  // Sensor card articles (for class toggling)
  sensorTemp:        document.getElementById("sensor-temp"),
  sensorPres:        document.getElementById("sensor-pres"),
  sensorAlt:         document.getElementById("sensor-alt"),

  // Chart latest-value labels
  tempChartLatest:   document.getElementById("temp-chart-latest"),
  altChartLatest:    document.getElementById("alt-chart-latest"),

  // LED
  ledActual:         document.getElementById("led-actual"),
  ledModeVal:        document.getElementById("led-mode-val"),
  ledUpdated:        document.getElementById("led-updated"),
  ledStateBadge:     document.getElementById("led-state-badge"),
  btnLedOn:          document.getElementById("led-on"),
  btnLedOff:         document.getElementById("led-off"),
  btnLedAuto:        document.getElementById("led-auto"),

  // Buzzer
  buzzerActual:      document.getElementById("buzzer-actual"),
  buzzerModeVal:     document.getElementById("buzzer-mode-val"),
  buzzerUpdated:     document.getElementById("buzzer-updated"),
  buzzerStateBadge:  document.getElementById("buzzer-state-badge"),
  btnBuzzerOn:       document.getElementById("buzzer-on"),
  btnBuzzerOff:      document.getElementById("buzzer-off"),
  btnBuzzerAuto:     document.getElementById("buzzer-auto"),

  // Log
  eventLog:          document.getElementById("event-log"),
  logCount:          document.getElementById("log-count"),
  btnClearLog:       document.getElementById("clear-log")
};

// ─────────────────────────────────────────────
// CLOCK
// ─────────────────────────────────────────────
function tickClock() {
  if (D.currentTime) {
    D.currentTime.textContent = new Date().toLocaleTimeString("vi-VN", { hour12: false });
  }
}

function initClock() {
  tickClock();
  setInterval(tickClock, 1000);
}

// ─────────────────────────────────────────────
// LOG
// XSS-safe: message text is set via textContent, never innerHTML.
// ─────────────────────────────────────────────
const LOG_TYPE_LABEL = {
  info:    "INFO",
  success: "CONNCTD",
  warning: "WARN",
  error:   "ERROR",
  sent:    "SENT"
};

function addLog(type, message) {
  if (!D.eventLog) return;
  const time = new Date().toLocaleTimeString("vi-VN", { hour12: false });

  const entry = document.createElement("div");
  entry.className = `log-entry log-${type}`;

  const timeEl = document.createElement("span");
  timeEl.className = "log-time";
  timeEl.textContent = time;

  const typeEl = document.createElement("span");
  typeEl.className = "log-type";
  typeEl.textContent = LOG_TYPE_LABEL[type] || type.toUpperCase();

  const textEl = document.createElement("span");
  textEl.className = "log-text";
  textEl.textContent = message; // textContent → safe from XSS

  entry.appendChild(timeEl);
  entry.appendChild(typeEl);
  entry.appendChild(textEl);

  // Newest on top
  D.eventLog.prepend(entry);

  // Trim excess
  while (D.eventLog.children.length > MAX_LOG) {
    D.eventLog.removeChild(D.eventLog.lastChild);
  }

  // Update count
  if (D.logCount) {
    const n = D.eventLog.children.length;
    D.logCount.textContent = `${n} ${n === 1 ? "entry" : "entries"}`;
  }
}

function clearLog() {
  if (!D.eventLog) return;
  D.eventLog.innerHTML = "";
  if (D.logCount) D.logCount.textContent = "0 entries";
  addLog("info", "Log cleared.");
}

// ─────────────────────────────────────────────
// CHARTS
// ─────────────────────────────────────────────
let tempChart = null;
let altChart  = null;

function makeChart(canvasId, label, lineColor, unit) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return null;

  return new Chart(canvas, {
    type: "line",
    data: {
      labels: [],
      datasets: [{
        label: label,
        data: [],
        borderColor: lineColor,
        backgroundColor: lineColor + "18",
        borderWidth: 2.5,
        pointRadius: 3,
        pointHoverRadius: 5,
        pointBackgroundColor: lineColor,
        fill: true,
        tension: 0.3
      }]
    },
    options: {
      animation:          false,
      responsive:         true,
      maintainAspectRatio:false,
      interaction: { mode: "index", intersect: false },
      scales: {
        x: {
          ticks: {
            color: "#8FA0B8",
            maxRotation: 0,
            autoSkip: true,
            maxTicksLimit: 6,
            font: { size: 12, weight: "600", family: "'JetBrains Mono', monospace" }
          },
          grid: { color: "#1E3252" }
        },
        y: {
          ticks: { color: "#CBD5E1", font: { size: 13, weight: "600" } },
          grid: { color: "#1E3252" }
        }
      },
      plugins: {
        legend: {
          labels: {
            color: "#F1F5F9",
            font: { size: 13, weight: "700" },
            boxWidth: 12,
            usePointStyle: true
          }
        },
        tooltip: {
          backgroundColor: "#172338",
          borderColor: "#23385D",
          borderWidth: 1.5,
          titleColor: "#93C5FD",
          titleFont: { size: 13, weight: "700" },
          bodyColor: "#FFFFFF",
          bodyFont: { size: 14, weight: "700" },
          padding: 10,
          callbacks: {
            label: (ctx) => ` ${ctx.parsed.y.toFixed(2)} ${unit}`
          }
        }
      }
    }
  });
}

// Push one real-time data point. Never called with mock data.
function pushChartPoint(chart, value, latestEl, unit) {
  if (!chart) return;
  const stamp = new Date().toLocaleTimeString("vi-VN", { hour12: false });
  chart.data.labels.push(stamp);
  chart.data.datasets[0].data.push(value);

  if (chart.data.labels.length > MAX_CHART_PTS) {
    chart.data.labels.shift();
    chart.data.datasets[0].data.shift();
  }

  chart.update("none"); // skip animation for real-time performance

  if (latestEl) latestEl.textContent = `${value.toFixed(2)} ${unit}`;
}

function initCharts() {
  if (typeof Chart === "undefined") {
    addLog("error", "Chart.js did not load from CDN. Charts unavailable.");
    return;
  }
  Chart.defaults.color = "#94A3B8";
  tempChart = makeChart("temp-chart", "Temperature", "#EF4444", "°C");
  altChart  = makeChart("alt-chart",  "Altitude",    "#3B82F6", "m");

  window.addEventListener("resize", () => {
    if (tempChart) tempChart.resize();
    if (altChart) altChart.resize();
  });
}

// ─────────────────────────────────────────────
// UI HELPERS
// ─────────────────────────────────────────────
function nowTimeStr() {
  return new Date().toLocaleTimeString("vi-VN", { hour12: false });
}

// Header MQTT badge + Status card (sc-mqtt)
function setMqttUi(state) {
  // state: "connected" | "reconnecting" | "disconnected"
  const badge = D.mqttHeaderBadge;
  if (badge) {
    badge.className = "connection-badge " + {
      connected:    "badge-connected",
      reconnecting: "badge-reconnecting",
      disconnected: "badge-disconnected"
    }[state];
    const badgeState = badge.querySelector(".badge-state");
    if (badgeState) {
      badgeState.textContent = state.toUpperCase();
    }
  }

  const labels = { connected: "CONNECTED", reconnecting: "RECONNECTING...", disconnected: "DISCONNECTED" };
  const cls    = { connected: "sc-connected", reconnecting: "sc-reconnecting", disconnected: "sc-disconnected" };
  const dotCls = { connected: "sc-dot dot-on", reconnecting: "sc-dot dot-warn", disconnected: "sc-dot dot-off" };
  const cardCls = { connected: "status-card sc-ok", reconnecting: "status-card sc-warn", disconnected: "status-card sc-bad" };

  if (D.mqttStatus)  { D.mqttStatus.textContent = labels[state]; D.mqttStatus.className = "sc-value " + cls[state]; }
  if (D.mqttDot)     { D.mqttDot.className = dotCls[state]; }
  if (D.scMqtt)      { D.scMqtt.className = cardCls[state]; }
}

function setWifiUi(connected) {
  const label   = connected ? "CONNECTED"    : "DISCONNECTED";
  const cls     = connected ? "sc-connected" : "sc-disconnected";
  const dotCls  = connected ? "sc-dot dot-on" : "sc-dot dot-off";
  const cardCls = connected ? "status-card sc-ok" : "status-card sc-bad";

  if (D.wifiStatus)  { D.wifiStatus.textContent = label; D.wifiStatus.className = "sc-value " + cls; }
  if (D.wifiDot)     { D.wifiDot.className = dotCls; }
  if (D.scWifi)      { D.scWifi.className = cardCls; }
}

function renderAlarmCard() {
  const tempBad = State.tempAlarm === "HIGH" || State.tempAlarm === "LOW";
  const altBad  = State.altAlarm  === "SUDDEN_CHANGE";
  const hasData = State.tempAlarm !== null  || State.altAlarm  !== null;

  let label, cls, dotCls, cardCls;

  if (!hasData) {
    label = "WAITING"; cls = "sc-wait"; dotCls = "sc-dot dot-wait"; cardCls = "status-card sc-wait";
  } else if (tempBad || altBad) {
    const parts = [];
    if (tempBad) parts.push(State.tempAlarm === "HIGH" ? "TEMP HIGH" : "TEMP LOW");
    if (altBad)  parts.push("ALT CHANGE");
    label = "WARNING · " + parts.join(" + ");
    cls = "sc-warning"; dotCls = "sc-dot dot-warn"; cardCls = "status-card sc-warn";
  } else {
    label = "NORMAL"; cls = "sc-normal"; dotCls = "sc-dot dot-on"; cardCls = "status-card sc-ok";
  }

  if (D.alarmStatus)  { D.alarmStatus.textContent = label; D.alarmStatus.className = "sc-value " + cls; }
  if (D.alarmDot)     { D.alarmDot.className = dotCls; }
  if (D.scAlarm)      { D.scAlarm.className = cardCls; }
}

function renderTempAlarm() {
  const alarm = State.tempAlarm;
  const badge = D.tempAlarmBadge;
  const text  = D.tempAlarmText;
  const card  = D.sensorTemp;

  if (card) card.classList.remove("alarm-high", "alarm-low");
  if (badge) badge.classList.remove("hidden", "badge-high", "badge-low", "badge-normal");

  if (alarm === "HIGH") {
    if (badge) { badge.classList.add("badge-high"); badge.textContent = "HIGH"; }
    if (text)  text.textContent = "Vượt ngưỡng trên (> 30 °C)";
    if (card)  card.classList.add("alarm-high");
  } else if (alarm === "LOW") {
    if (badge) { badge.classList.add("badge-low"); badge.textContent = "LOW"; }
    if (text)  text.textContent = "Thấp hơn ngưỡng dưới (< 15 °C)";
    if (card)  card.classList.add("alarm-low");
  } else if (alarm === "NORMAL") {
    if (badge) { badge.classList.add("badge-normal"); badge.textContent = "NORMAL"; }
    if (text)  text.textContent = "Trong ngưỡng an toàn";
  } else {
    if (badge) badge.classList.add("hidden");
    if (text)  text.textContent = "Chưa nhận dữ liệu";
  }
}

function renderAltAlarm() {
  const alarm = State.altAlarm;
  const badge = D.altAlarmBadge;
  const text  = D.altAlarmText;
  const card  = D.sensorAlt;

  if (card) card.classList.remove("alarm-change");
  if (badge) badge.classList.remove("hidden", "badge-change", "badge-clear", "badge-normal");

  if (alarm === "SUDDEN_CHANGE") {
    if (badge) { badge.classList.add("badge-change"); badge.textContent = "CHANGE"; }
    if (text)  text.textContent = "Độ cao thay đổi đột ngột";
    if (card)  card.classList.add("alarm-change");
  } else if (alarm === "CLEAR") {
    if (badge) { badge.classList.add("badge-clear"); badge.textContent = "CLEAR"; }
    if (text)  text.textContent = "Không có bước nhảy";
  } else {
    if (badge) badge.classList.add("hidden");
    if (text)  text.textContent = "So với mực nước biển";
  }
}

// Device state badge helper
function applyDeviceBadge(badgeEl, state, mode) {
  if (!badgeEl) return;
  badgeEl.classList.remove("badge-on", "badge-off", "badge-auto");
  if (mode === "AUTO") {
    badgeEl.textContent = "AUTO";
    badgeEl.classList.add("badge-auto");
  } else if (state === "ON") {
    badgeEl.textContent = "ON";
    badgeEl.classList.add("badge-on");
  } else if (state === "OFF") {
    badgeEl.textContent = "OFF";
    badgeEl.classList.add("badge-off");
  } else {
    badgeEl.textContent = "--";
    badgeEl.classList.add("badge-off");
  }
}

// di-value state class helper
function diStateClass(value) {
  if (value === "ON")     return " state-on";
  if (value === "OFF")    return " state-off";
  if (value === "AUTO")   return " state-auto";
  if (value === "MANUAL") return " state-off";
  return "";
}

function renderActuators() {
  const { led, ledMode, buzzer, buzzerMode } = State;
  const time = led || buzzer ? nowTimeStr() : "--";

  // LED
  if (D.ledActual)   { D.ledActual.textContent = led || "--";     D.ledActual.className   = "di-value" + diStateClass(led); }
  if (D.ledModeVal)  { D.ledModeVal.textContent = ledMode || "--"; D.ledModeVal.className  = "di-value" + diStateClass(ledMode); }
  if (D.ledUpdated)  { D.ledUpdated.textContent = led ? time : "--"; D.ledUpdated.className = "di-value mono"; }
  applyDeviceBadge(D.ledStateBadge, led, ledMode);

  // LED button active states
  if (D.btnLedOn)   D.btnLedOn.classList.toggle("active",   led === "ON"  && ledMode === "MANUAL");
  if (D.btnLedOff)  D.btnLedOff.classList.toggle("active",  led === "OFF" && ledMode === "MANUAL");
  if (D.btnLedAuto) D.btnLedAuto.classList.toggle("active", ledMode === "AUTO");

  // Buzzer
  if (D.buzzerActual)  { D.buzzerActual.textContent = buzzer || "--";     D.buzzerActual.className   = "di-value" + diStateClass(buzzer); }
  if (D.buzzerModeVal) { D.buzzerModeVal.textContent = buzzerMode || "--"; D.buzzerModeVal.className  = "di-value" + diStateClass(buzzerMode); }
  if (D.buzzerUpdated) { D.buzzerUpdated.textContent = buzzer ? time : "--"; D.buzzerUpdated.className = "di-value mono"; }
  applyDeviceBadge(D.buzzerStateBadge, buzzer, buzzerMode);

  if (D.btnBuzzerOn)   D.btnBuzzerOn.classList.toggle("active",   buzzer === "ON"  && buzzerMode === "MANUAL");
  if (D.btnBuzzerOff)  D.btnBuzzerOff.classList.toggle("active",  buzzer === "OFF" && buzzerMode === "MANUAL");
  if (D.btnBuzzerAuto) D.btnBuzzerAuto.classList.toggle("active", buzzerMode === "AUTO");
}

function markSensorTimestamp(type) {
  const t = nowTimeStr();
  if      (type === "temperature") { if (D.tempTime) D.tempTime.textContent = t; }
  else if (type === "pressure")    { if (D.presTime) D.presTime.textContent = t; }
  else if (type === "altitude")    { if (D.altTime)  D.altTime.textContent  = t; }
}

function updateLastDataTime() {
  State.lastDataMs = Date.now();
  if (D.lastDataTime) D.lastDataTime.textContent = nowTimeStr();
}

// Stale data detection: runs every 5 s
function checkStaleData() {
  if (!State.lastDataMs) return;
  const stale = (Date.now() - State.lastDataMs) > STALE_MS;
  [D.sensorTemp, D.sensorPres, D.sensorAlt].forEach(el => {
    if (el) el.classList.toggle("stale", stale);
  });
}

// ─────────────────────────────────────────────
// PAYLOAD UTILS
// ─────────────────────────────────────────────
function payloadToText(payload) {
  if (typeof payload === "string")        return payload.trim();
  if (payload instanceof Uint8Array)      return new TextDecoder().decode(payload).trim();
  if (payload && payload.toString)        return payload.toString().trim();
  return "";
}

function parseStatus(text) {
  const map = {};
  text.split(";").forEach(part => {
    const idx = part.indexOf("=");
    if (idx > 0) map[part.slice(0, idx)] = part.slice(idx + 1);
  });
  return map;
}

// ─────────────────────────────────────────────
// MESSAGE HANDLER
// Routes incoming MQTT messages to the right handler.
// All payload → plain text via payloadToText().
// ─────────────────────────────────────────────
function onMessage(topic, rawPayload) {
  const text = payloadToText(rawPayload);
  if (!text) return;

  const isSensor = (
    topic === TOPICS.temperature ||
    topic === TOPICS.pressure    ||
    topic === TOPICS.altitude
  );

  // ── Sensor telemetry ────────────────────────
  if (isSensor) {
    const value = Number(text);
    if (!Number.isFinite(value)) {
      addLog("warning", `Bad numeric payload on ${topic}: "${text}"`);
      return;
    }

    updateLastDataTime();

    if (topic === TOPICS.temperature) {
      if (D.temperature) D.temperature.textContent = value.toFixed(2);
      markSensorTimestamp("temperature");
      pushChartPoint(tempChart, value, D.tempChartLatest, "°C");
    } else if (topic === TOPICS.pressure) {
      if (D.pressure) D.pressure.textContent = value.toFixed(2);
      markSensorTimestamp("pressure");
    } else if (topic === TOPICS.altitude) {
      if (D.altitude) D.altitude.textContent = value.toFixed(2);
      markSensorTimestamp("altitude");
      pushChartPoint(altChart, value, D.altChartLatest, "m");
    }
    return; // sensor telemetry: no log entry (high-frequency, would flood log)
  }

  // ── WiFi status (from ESP32 or LWT) ─────────
  if (topic === TOPICS.wifi) {
    const up = text === "CONNECTED";
    setWifiUi(up);
    addLog(up ? "success" : "warning", `WiFi (ESP32): ${text}`);
    return;
  }

  // ── Temperature alarm ───────────────────────
  if (topic === TOPICS.alarmTemp) {
    const changed = State.tempAlarm !== text;
    State.tempAlarm = text;
    renderTempAlarm();
    renderAlarmCard();
    if (changed) {
      addLog(text === "NORMAL" ? "info" : "warning", `Temperature alarm → ${text}`);
    }
    return;
  }

  // ── Altitude alarm ──────────────────────────
  if (topic === TOPICS.alarmAlt) {
    const changed = State.altAlarm !== text;
    State.altAlarm = text;
    renderAltAlarm();
    renderAlarmCard();
    if (changed) {
      addLog(text === "SUDDEN_CHANGE" ? "warning" : "info", `Altitude alarm → ${text}`);
    }
    return;
  }

  // ── Composite status from ESP32 ─────────────
  // Format: wifi=X;led=X;buzzer=X;led_mode=X;buzzer_mode=X;temp_alarm=X;alt_alarm=X
  if (topic === TOPICS.status) {
    const map = parseStatus(text);

    if (map.led)          State.led          = map.led;
    if (map.buzzer)       State.buzzer       = map.buzzer;
    if (map.led_mode)     State.ledMode      = map.led_mode;
    if (map.buzzer_mode)  State.buzzerMode   = map.buzzer_mode;

    // Status can carry alarm & wifi info too — sync state
    if (map.wifi) setWifiUi(map.wifi === "CONNECTED");

    if (map.temp_alarm) {
      State.tempAlarm = map.temp_alarm;
      renderTempAlarm();
    }
    if (map.alt_alarm) {
      State.altAlarm = map.alt_alarm;
      renderAltAlarm();
    }

    renderActuators();
    renderAlarmCard();
    addLog("info", `Status: LED=${map.led || "?"} (${map.led_mode || "?"}) · BZR=${map.buzzer || "?"} (${map.buzzer_mode || "?"})`);
    return;
  }

  // ── Unknown / ignored topic ──────────────────
  addLog("warning", `Unhandled topic: ${topic} = ${text}`);
}

// ─────────────────────────────────────────────
// MQTT
// ─────────────────────────────────────────────
let mqttClient = null;

function setControlsEnabled(enabled) {
  [
    D.btnLedOn, D.btnLedOff, D.btnLedAuto,
    D.btnBuzzerOn, D.btnBuzzerOff, D.btnBuzzerAuto
  ].forEach(btn => { if (btn) btn.disabled = !enabled; });
}

function connectMQTT() {
  if (typeof mqtt === "undefined") {
    addLog("error", "mqtt.js library failed to load from CDN. No MQTT connection.");
    setMqttUi("disconnected");
    return;
  }

  const clientId = "web-runner-" + Math.random().toString(16).slice(2, 10);
  addLog("info", `Connecting → ${MQTT_URL} [${clientId}]`);

  mqttClient = mqtt.connect(MQTT_URL, {
    clientId:        clientId,
    protocolVersion: 4,     // MQTT 3.1.1 (matches ESP32 PubSubClient)
    clean:           true,
    reconnectPeriod: 3000,
    connectTimeout:  8000
  });

  mqttClient.on("connect", () => {
    setMqttUi("connected");
    setControlsEnabled(true);
    addLog("success", `Connected to ${MQTT_URL} as ${clientId}`);

    // Subscribe to all telemetry / alarm / status / wifi topics.
    // Control topics are publish-only from the web side.
    const subTopics = Object.values(TOPICS).filter(t => !t.startsWith("runner/lab 3/control/"));
    subTopics.forEach(topic => {
      mqttClient.subscribe(topic, { qos: 0 }, err => {
        if (err) addLog("error", `Subscribe failed: ${topic} — ${err.message}`);
      });
    });
  });

  mqttClient.on("reconnect", () => {
    setMqttUi("reconnecting");
    setControlsEnabled(false);
    addLog("warning", "MQTT connection lost. Reconnecting...");
  });

  mqttClient.on("close", () => {
    setMqttUi("disconnected");
    setControlsEnabled(false);
  });

  mqttClient.on("error", err => {
    addLog("error", `MQTT error: ${err.message}`);
  });

  mqttClient.on("message", onMessage);
}

// ─────────────────────────────────────────────
// CONTROLS
// ─────────────────────────────────────────────

// Cooldown guard: prevent rapid duplicate commands (200 ms minimum between presses)
const _lastCmd = {};

function publishCommand(topic, payload) {
  if (!mqttClient || !mqttClient.connected) {
    addLog("error", "MQTT not connected. Command not sent.");
    return;
  }

  const key = topic + ":" + payload;
  const now = Date.now();
  if (_lastCmd[key] && now - _lastCmd[key] < 200) return; // debounce
  _lastCmd[key] = now;

  mqttClient.publish(topic, payload, { qos: 0 });
  addLog("sent", `${topic} ← ${payload}`);
  // Note: device state is NOT updated here.
  //       We wait for ESP32 to publish runner/lab 3/status to confirm.
}

function bindControls() {
  const bindings = [
    [D.btnLedOn,     TOPICS.controlLed,    "ON"],
    [D.btnLedOff,    TOPICS.controlLed,    "OFF"],
    [D.btnLedAuto,   TOPICS.controlLed,    "AUTO"],
    [D.btnBuzzerOn,  TOPICS.controlBuzzer, "ON"],
    [D.btnBuzzerOff, TOPICS.controlBuzzer, "OFF"],
    [D.btnBuzzerAuto,TOPICS.controlBuzzer, "AUTO"]
  ];

  bindings.forEach(([btn, topic, payload]) => {
    if (btn) btn.addEventListener("click", () => publishCommand(topic, payload));
  });

  if (D.btnClearLog) D.btnClearLog.addEventListener("click", clearLog);
}

// ─────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────
function init() {
  initClock();
  initCharts();
  bindControls();
  setControlsEnabled(false);
  renderActuators();
  renderAlarmCard();

  // Stale data check every 5 s
  setInterval(checkStaleData, 5000);

  if (typeof mqtt === "undefined" || typeof Chart === "undefined") {
    const missing = [
      typeof mqtt  === "undefined" ? "mqtt.js"   : null,
      typeof Chart === "undefined" ? "Chart.js"  : null
    ].filter(Boolean).join(", ");
    addLog("error", `Required CDN ${missing} failed to load. Check network.`);
    setMqttUi("disconnected");
    return;
  }

  connectMQTT();
}

init();
