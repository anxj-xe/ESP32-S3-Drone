/*
  ESP32-S3 Super Mini Coreless Quadcopter Firmware
  --------------------------------------------------
  - Reads MPU6050 over I2C (raw registers, no external IMU library needed)
  - Complementary filter for pitch/roll, gyro integration for yaw rate
  - PID stabilization (angle mode for pitch/roll, rate mode for yaw)
  - 4x brushed motor PWM output through SI2302 MOSFETs
  - ESP32 hosts its own WiFi (SoftAP) and serves a browser control page
  - Phone connects to WiFi, opens 192.168.4.1, controls via on-screen
    joysticks sent over WebSocket
  - Arm/disarm button on the page. Motors stay at 0 until armed.
  - Failsafe: if no WebSocket message arrives for 500ms, motors cut
    immediately and drone disarms.

  REQUIRED LIBRARIES (install via Arduino Library Manager):
  - ESPAsyncWebServer (by lacamera / me-no-dev fork)
  - AsyncTCP (by dvarrel or me-no-dev, ESP32 version)

  PIN MAP:
  MPU6050  SDA -> GPIO9   SCL -> GPIO8   AD0 -> GND
  Motor 1 (front-left)  gate -> GPIO11
  Motor 2 (front-right) gate -> GPIO7
  Motor 3 (rear-right)  gate -> GPIO1
  Motor 4 (rear-left)   gate -> GPIO13
  Battery ADC (reserved, future) -> GPIO4

  IMPORTANT: This is a STARTING POINT. PID gains (KP_*, KI_*, KD_*) are
  placeholder values and WILL need tuning on your specific frame/weight.
  Start with props OFF, confirm motor response direction is correct,
  THEN move to a tethered/hand-held test with props on before free flight.
*/

#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ---------- Pin definitions ----------
#define SDA_PIN 9
#define SCL_PIN 8
#define M1_PIN 11   // front-left
#define M2_PIN 7    // front-right
#define M3_PIN 1    // rear-right
#define M4_PIN 13   // rear-left

// If yaw response feels backwards during bench testing, flip the sign
// on YAW_M1..YAW_M4 below (do not rewire, just flip the sign in code).
// Diagonal motor pairs (M1/M3 and M2/M4) should spin opposite rotation
// directions to cancel reaction torque.
#define YAW_M1 -1
#define YAW_M2  1
#define YAW_M3 -1
#define YAW_M4  1

// ---------- MPU6050 I2C ----------
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float accelScale = 16384.0;   // for +-2g range
float gyroScale = 131.0;      // for +-250 dps range

float pitch = 0, roll = 0;
float yawRate = 0;
unsigned long lastIMUTime = 0;

// ---------- PID ----------
struct PID {
  float kp, ki, kd;
  float integral = 0;
  float prevError = 0;
};

// Placeholder gains - tune these during bench/hover testing
PID pitchPID = {2.0, 0.02, 0.8};
PID rollPID  = {2.0, 0.02, 0.8};
PID yawPID   = {1.5, 0.0,  0.0};

float pidUpdate(PID &p, float target, float measured, float dt) {
  float error = target - measured;
  p.integral += error * dt;
  p.integral = constrain(p.integral, -100, 100); // anti-windup clamp
  float derivative = (error - p.prevError) / dt;
  p.prevError = error;
  return p.kp * error + p.ki * p.integral + p.kd * derivative;
}

// ---------- Control inputs (from phone) ----------
volatile int inArmed = 0;
volatile int inThrottle = 0;   // 0-255
volatile int inYaw = 0;        // -100 to 100
volatile int inPitch = 0;      // -100 to 100
volatile int inRoll = 0;       // -100 to 100
unsigned long lastMsgTime = 0;
const unsigned long FAILSAFE_MS = 500;

// ---------- Motor output ----------
const int pwmFreq = 20000;
const int pwmResolution = 8;
const int MIN_ARM_THROTTLE = 15; // motors idle floor once armed & throttle > 0

// ---------- WiFi / WebSocket ----------
const char* AP_SSID = "Drone_ESP32";
const char* AP_PASS = "drone1234"; // must be 8+ characters
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char controlPage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<style>
  body { margin:0; background:#111; color:#eee; font-family:sans-serif; overflow:hidden; touch-action:none; text-align:center; }
  #header { position:fixed; top:10px; width:100%; z-index:10; }
  #armBtn { padding:14px 28px; font-size:18px; font-weight:bold; border:none; border-radius:8px;
    background:#c33; color:#fff; cursor:pointer; }
  #armBtn.armed { background:#2a2; }
  #status { margin-top:8px; font-size:13px; color:#aaa; }
  #telemetry { margin-top:4px; font-size:12px; color:#4af; font-family:monospace; }
  #warning { margin-top:4px; font-size:12px; color:#f55; min-height:16px; font-weight:bold; }
  .stick-container { position:fixed; bottom:30px; width:140px; text-align:center; }
  .stick-label { font-size:11px; color:#777; margin-bottom:6px; letter-spacing:1px; }
  .stick { width:140px; height:140px; border-radius:50%; background:#222; border:2px solid #555; position:relative; }
  #leftContainer { left:30px; }
  #rightContainer { right:30px; }
  .knob { position:absolute; width:60px; height:60px; border-radius:50%; background:#5af; }
  #leftKnob { left:40px; top:80px; } /* Resting at bottom (throttle = 0) */
  #rightKnob { left:40px; top:40px; } /* Resting at center */
</style>
</head>
<body>
<div id="header">
  <button id="armBtn">ARM</button>
  <div id="status">Connecting to drone...</div>
  <div id="telemetry">Throttle: 0% | State: DISARMED</div>
  <div id="warning"></div>
</div>
<div class="stick-container" id="leftContainer">
  <div class="stick-label">THROTTLE / YAW</div>
  <div class="stick" id="left"><div class="knob" id="leftKnob"></div></div>
</div>
<div class="stick-container" id="rightContainer">
  <div class="stick-label">PITCH / ROLL</div>
  <div class="stick" id="right"><div class="knob" id="rightKnob"></div></div>
</div>
<script>
let armed = 0, throttle = 0, yaw = 0, pitch = 0, roll = 0;
let ws;
function connect() {
  ws = new WebSocket("ws://" + location.hostname + "/ws");
  ws.onopen = () => { document.getElementById("status").innerText = "Connected"; };
  ws.onclose = () => { document.getElementById("status").innerText = "Disconnected - Retrying..."; setTimeout(connect, 1000); };
}
connect();

function updateTelemetry() {
  const pct = Math.round((throttle / 255) * 100);
  document.getElementById("telemetry").innerText = "Throttle: " + pct + "% (" + throttle + "/255) | " + (armed ? "ARMED" : "DISARMED");
}

setInterval(() => {
  if (ws && ws.readyState === 1) {
    ws.send(armed + "," + throttle + "," + yaw + "," + pitch + "," + roll);
  }
  updateTelemetry();
}, 50);

document.getElementById("armBtn").onclick = () => {
  if (!armed) {
    if (throttle > 15) {
      const warn = document.getElementById("warning");
      warn.innerText = "SAFETY: LOWER THROTTLE TO 0 BEFORE ARMING!";
      setTimeout(() => { warn.innerText = ""; }, 2500);
      return;
    }
    armed = 1;
  } else {
    armed = 0;
  }
  const btn = document.getElementById("armBtn");
  btn.classList.toggle("armed", armed);
  btn.innerText = armed ? "DISARM" : "ARM";
  document.getElementById("warning").innerText = "";
  updateTelemetry();
};

function constrainVal(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// Right Stick: auto-centers on release (pitch and roll = 0)
(function setupRightStick() {
  const stick = document.getElementById("right");
  const knob = document.getElementById("rightKnob");
  const radius = 40;
  let active = false;
  function handle(x, y) {
    const rect = stick.getBoundingClientRect();
    let dx = x - (rect.left + rect.width/2);
    let dy = y - (rect.top + rect.height/2);
    const dist = Math.min(Math.sqrt(dx*dx + dy*dy), radius);
    const angle = Math.atan2(dy, dx);
    dx = Math.cos(angle) * dist;
    dy = Math.sin(angle) * dist;
    knob.style.left = (40 + dx) + "px";
    knob.style.top = (40 + dy) + "px";
    roll = Math.round((dx / radius) * 100);
    pitch = Math.round((-dy / radius) * 100);
  }
  function reset() {
    knob.style.left = "40px";
    knob.style.top = "40px";
    roll = 0;
    pitch = 0;
  }
  stick.addEventListener("touchstart", (e) => { active = true; handle(e.touches[0].clientX, e.touches[0].clientY); e.preventDefault(); }, {passive:false});
  stick.addEventListener("touchmove", (e) => { if (!active) return; handle(e.touches[0].clientX, e.touches[0].clientY); e.preventDefault(); }, {passive:false});
  stick.addEventListener("touchend", (e) => { active = false; reset(); e.preventDefault(); }, {passive:false});
})();

// Left Stick: Throttle (Y) and Yaw (X)
// Throttle: bottom (y=80px) = 0%, top (y=0px) = 100%
// On release, yaw snaps to 0. Throttle resets to 0 (bottom) for bench safety!
(function setupLeftStick() {
  const stick = document.getElementById("left");
  const knob = document.getElementById("leftKnob");
  const radius = 40;
  let active = false;
  function handle(x, y) {
    const rect = stick.getBoundingClientRect();
    let dx = x - (rect.left + rect.width/2);
    let dy = y - (rect.top + rect.height/2);
    const dist = Math.min(Math.sqrt(dx*dx + dy*dy), radius);
    const angle = Math.atan2(dy, dx);
    dx = Math.cos(angle) * dist;
    dy = Math.sin(angle) * dist;
    knob.style.left = (40 + dx) + "px";
    knob.style.top = (40 + dy) + "px";
    yaw = Math.round((dx / radius) * 100);
    // dy ranges from -40 (top, 255) to +40 (bottom, 0)
    const normThrottle = constrainVal((radius - dy) / (2 * radius), 0, 1);
    throttle = Math.round(normThrottle * 255);
  }
  function reset() {
    knob.style.left = "40px";
    knob.style.top = "80px"; // snap to bottom = 0 throttle
    yaw = 0;
    throttle = 0;
  }
  stick.addEventListener("touchstart", (e) => { active = true; handle(e.touches[0].clientX, e.touches[0].clientY); e.preventDefault(); }, {passive:false});
  stick.addEventListener("touchmove", (e) => { if (!active) return; handle(e.touches[0].clientX, e.touches[0].clientY); e.preventDefault(); }, {passive:false});
  stick.addEventListener("touchend", (e) => { active = false; reset(); e.preventDefault(); }, {passive:false});
})();
</script>
</body>
</html>
)HTML";

// ---------- MPU6050 functions ----------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(PWR_MGMT_1, 0x00); // wake up MPU6050
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAX = Wire.read() << 8 | Wire.read();
  int16_t rawAY = Wire.read() << 8 | Wire.read();
  int16_t rawAZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();

  accelX = rawAX / accelScale;
  accelY = rawAY / accelScale;
  accelZ = rawAZ / accelScale;
  gyroX = rawGX / gyroScale;
  gyroY = rawGY / gyroScale;
  gyroZ = rawGZ / gyroScale;

  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0;
  if (lastIMUTime == 0) dt = 0.004; // first-run fallback
  lastIMUTime = now;

  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  float accelRoll  = atan2(accelY, accelZ) * 180.0 / PI;

  // Complementary filter: mostly gyro (fast, drifts), corrected by accel (slow, noisy)
  pitch = 0.98 * (pitch + gyroY * dt) + 0.02 * accelPitch;
  roll  = 0.98 * (roll + gyroX * dt) + 0.02 * accelRoll;
  yawRate = gyroZ;
}

// ---------- Motor output ----------
void setMotors(int m1, int m2, int m3, int m4) {
  ledcWrite(M1_PIN, constrain(m1, 0, 255));
  ledcWrite(M2_PIN, constrain(m2, 0, 255));
  ledcWrite(M3_PIN, constrain(m3, 0, 255));
  ledcWrite(M4_PIN, constrain(m4, 0, 255));
}

void allMotorsOff() {
  setMotors(0, 0, 0, 0);
}

// ---------- WebSocket handling ----------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    char msg[64];
    size_t copyLen = min(len, sizeof(msg) - 1);
    memcpy(msg, data, copyLen);
    msg[copyLen] = '\0';

    int a, t, y, p, r;
    if (sscanf(msg, "%d,%d,%d,%d,%d", &a, &t, &y, &p, &r) == 5) {
      // Arming safety interlock: Reject arming if throttle is not at zero
      if (a && !inArmed) {
        if (t <= MIN_ARM_THROTTLE) {
          inArmed = 1;
        } else {
          inArmed = 0; // Prevent accidental high-throttle arming
        }
      } else if (!a) {
        inArmed = 0;
      }

      inThrottle = constrain(t, 0, 255);
      inYaw = constrain(y, -100, 100);
      inPitch = constrain(p, -100, 100);
      inRoll = constrain(r, -100, 100);
      lastMsgTime = millis();
    }
  } else if (type == WS_EVT_DISCONNECT) {
    // failsafe loop will catch this via lastMsgTime timeout
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuInit();

  ledcAttach(M1_PIN, pwmFreq, pwmResolution);
  ledcAttach(M2_PIN, pwmFreq, pwmResolution);
  ledcAttach(M3_PIN, pwmFreq, pwmResolution);
  ledcAttach(M4_PIN, pwmFreq, pwmResolution);
  allMotorsOff();

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", controlPage);
  });

  server.begin();
  lastMsgTime = millis(); // avoid instant failsafe trip on boot
}

void loop() {
  ws.cleanupClients();

  // ---- Failsafe check ----
  bool connectionLost = (millis() - lastMsgTime) > FAILSAFE_MS;
  if (connectionLost) {
    inArmed = 0;
    inThrottle = 0;
    inYaw = 0; inPitch = 0; inRoll = 0;
    allMotorsOff();
  }

  // ---- Read sensors ----
  readIMU();

  // ---- Run PID (angle mode pitch/roll, rate mode yaw) ----
  static unsigned long lastLoopTime = 0;
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  if (lastLoopTime == 0 || dt <= 0.0 || dt > 0.05) dt = 0.004;
  lastLoopTime = now;

  float targetPitch = inPitch * 0.3;   // scale joystick -100..100 to a modest angle target
  float targetRoll  = inRoll * 0.3;
  float targetYawRate = inYaw * 1.0;   // deg/sec target

  float pitchOut = pidUpdate(pitchPID, targetPitch, pitch, dt);
  float rollOut  = pidUpdate(rollPID, targetRoll, roll, dt);
  float yawOut   = pidUpdate(yawPID, targetYawRate, yawRate, dt);

  // ---- Motor mixing (X frame), with throttle slew-rate limiting ----
  // currentThrottle chases inThrottle gradually instead of jumping instantly
  static float currentThrottle = 0;
  const float THROTTLE_RAMP_RATE = 0.8; // Gentle ramp to avoid sudden inductive current spikes

  if (inArmed && inThrottle > MIN_ARM_THROTTLE) {
    float targetThrottle = inThrottle;
    if (currentThrottle < targetThrottle) {
      currentThrottle = min(currentThrottle + THROTTLE_RAMP_RATE, targetThrottle);
    } else {
      // Faster throttle reduction for responsive disarm / throttle cuts
      currentThrottle = max(currentThrottle - (THROTTLE_RAMP_RATE * 2.0f), targetThrottle);
    }

    // Attenuate PID authority at low throttle so ground vibration doesn't kick motors violently
    float pidAuthority = constrain((currentThrottle - MIN_ARM_THROTTLE) / 40.0f, 0.0f, 1.0f);
    float pOut = pitchOut * pidAuthority;
    float rOut = rollOut * pidAuthority;
    float yOut = yawOut * pidAuthority;

    int base = (int)currentThrottle;
    int m1 = base - pOut + rOut + YAW_M1 * yOut; // front-left
    int m2 = base - pOut - rOut + YAW_M2 * yOut; // front-right
    int m3 = base + pOut - rOut + YAW_M3 * yOut; // rear-right
    int m4 = base + pOut + rOut + YAW_M4 * yOut; // rear-left
    setMotors(m1, m2, m3, m4);
  } else {
    currentThrottle = 0; // reset ramp so next arm always starts from zero
    allMotorsOff();
    // reset PID integrators and prevError while disarmed so they don't kick on arm
    pitchPID.integral = 0;
    pitchPID.prevError = 0;
    rollPID.integral = 0;
    rollPID.prevError = 0;
    yawPID.integral = 0;
    yawPID.prevError = 0;
  }

  delay(4); // ~250Hz loop, adjust if needed
}
