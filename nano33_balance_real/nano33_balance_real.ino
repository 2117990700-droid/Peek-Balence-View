#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <ctype.h>              // for toupper
#include <WiFiNINA.h>
#include <WebSocketsClient.h>   // Links2004 WebSockets CLIENT
#include <ArduinoJson.h>

/* ===========================================================
   Nano 33 IoT + dual MPU6050 (chest/waist)
   Detects: NORMAL/IMBALANCE + FALL risk
   Axis mapping (per mounting orientation):
     Body frame unified: X=right(+), Y=forward(+), Z=up(+)
     Chest:  z→Y, x→X, y→Z
     Waist:  z→Y, y→-X, x→Z
   NOTE: This version is a WS **client** that connects to your Node relay
         (server.js with ws library) at ws://<PC-LAN-IP>:8081/
   =========================================================== */

// ===== Wi-Fi configuration =====
char ssid[] = "Galaxy A22 5G1918";   // <-- your WiFi SSID
char pass[] = "zojk3182";            // <-- your WiFi password

// Connect to Node relay instead of hosting WS server
const char* SERVER_HOST = "10.190.9.170"; // <-- set to your PC IP from ipconfig
const uint16_t SERVER_PORT = 8081;

// WebSocket client
WebSocketsClient wsClient;
bool wsEnabled = false;

// ===== I2C & IMU =====
// Using ±4g → 1g ≈ 8192 counts
const float ACC_SCALE = 8192.0f;
#define CHEST_ADDR 0x68
#define WAIST_ADDR 0x69
MPU6050 mpuChest(CHEST_ADDR);
MPU6050 mpuWaist(WAIST_ADDR);

// ===== Axis mapping =====
const uint8_t CHEST_MAP[3] = {0, 2, 1};     // X<-rawX, Y<-rawZ, Z<-rawY
const int8_t  CHEST_SIGN[3]= {+1, +1, +1};

const uint8_t WAIST_MAP[3] = {1, 2, 0};     // X<-rawY, Y<-rawZ, Z<-rawX
const int8_t  WAIST_SIGN[3]= {-1, +1, +1};  // y inverted

// ===== Timing & filtering =====
const uint16_t UPDATE_MS = 200;         // Output every 0.2 s
const uint16_t SAMPLE_INTERVAL_MS = 10; // ~100 Hz sampling
unsigned long lastSampleMs = 0;
unsigned long windowStartMs = 0;

const float LPF_A = 0.7f, LPF_B = 0.3f; // First-order low-pass for |a|

// ===== Thresholds (scaled by k_eff) =====
float SWAY_RMS_HIGH      = 0.18f; // RMS(|a|-1g) threshold (g)
float CHEST_WAIST_DELTA  = 0.25f; // Chest vs waist |a| delta threshold (g)
const float STEP_THR_WALK = 1.15f; // Step peak threshold (g, LPF |a|)
const uint16_t REFRACT_MS = 220;   // Step refractory (per sensor)
const float FREE_FALL_G   = 0.55f; // Low-g threshold
const float IMPACT_G      = 2.6f;  // Impact threshold (with ±4g)
const uint16_t FF_TO_IMPACT_MS = 500;  // Max low-g → impact interval
const uint16_t STILL_AFTER_MS  = 2000; // Stillness after impact

// ===== Prior grade & age → k_eff =====
enum { GRADE_GREEN=0, GRADE_AMBER=1, GRADE_RED=2 };
float   BASE_SWAY  = 0.0f;     // Optional sway baseline (from 'B' cmd)
uint8_t BASE_GRADE = GRADE_GREEN;

float k_age   = 1.05f; // Default adult
float k_grade = 1.0f;  // G/A/R -> 1.00/0.80/0.65

// —— Step counting & frequency EMA —— //
unsigned long lastUnifiedStep = 0;
uint16_t unifiedStepCountWindow = 0;
const uint16_t UNIFIED_REFRACT_MS = 250;
float freqEMA = 0.0f; const float FREQ_EMA_A=0.8f, FREQ_EMA_B=0.2f;

// —— RMS accumulators —— //
float rmsAccChest = 0.0f, rmsAccWaist = 0.0f; uint16_t rmsCount = 0;
float lpAmagChest = 1.0f, lpAmagWaist = 1.0f;
float t = 1.0f;
unsigned long lastPeakChest = 0, lastPeakWaist = 0;

// —— Output mode —— //
enum OutputMode { MODE_PLOT, MODE_MONITOR };
OutputMode outMode = MODE_PLOT;
bool verboseAngles = false; // for debug/‘Z’ only

// —— Tilt/orientation (debug) —— //
float gCx=0, gCy=0, gCz=1;
float gWx=0, gWy=0, gWz=1;
const float GEMA_ALPHA = 0.05f;
float chestRollDeg=0, chestPitchDeg=0;
float waistRollDeg=0, waistPitchDeg=0;
float roll0 = 0.0f, pitch0 = 0.0f;

// ===== Imbalance state machine =====
const uint16_t IMB_ENTER_MS = 800;
const uint16_t IMB_EXIT_MS  = 800;
unsigned long imbPersistTimer = 0;
bool imbalance = false;

// ===== Fall detection =====
bool ffSeen = false; unsigned long ffTime = 0;
bool impactSeen = false; unsigned long impactTime = 0;
bool fallAnnounced = false;

// ========= Prototypes =========
void setAgeYears(float ageY);
void onWsEvent(WStype_t type, uint8_t * payload, size_t length);
void wsBroadcastMetrics(float chestRMS, float waistRMS,
                        const char* STATE, const char* FALL_RISK,
                        float k_eff, float freqHz,
                        float thrSway, float thrCW);

// ====== Utilities ======
inline void mapBodyAxes(int16_t rx,int16_t ry,int16_t rz,
                        const uint8_t M[3], const int8_t S[3],
                        float &bx,float &by,float &bz){
  int16_t raw[3] = {rx, ry, rz};
  bx = (S[0] * raw[M[0]]) / ACC_SCALE;  // X=right(+)
  by = (S[1] * raw[M[1]]) / ACC_SCALE;  // Y=forward(+)
  bz = (S[2] * raw[M[2]]) / ACC_SCALE;  // Z=up(+)
}

static inline float riskScaleForU8(uint8_t g) {
  switch (g) {
    case GRADE_GREEN: return 1.00f;
    case GRADE_AMBER: return 0.80f;
    case GRADE_RED:   return 0.65f;
    default:          return 1.00f;
  }
}

static inline bool risingCross(float prev, float now, float thr,
                               unsigned long nowMs, unsigned long &lastPeak){
  bool hit = (prev < thr && now >= thr) && (nowMs - lastPeak) > REFRACT_MS;
  if (hit) lastPeak = nowMs;
  return hit;
}

// ====== Age prior ======
void setAgeYears(float ageY){
  if (ageY >= 16.0f) { k_age = 1.05f; }    // Adult/teen
  else if (ageY < 3.5f) { k_age = 0.85f; } // 2–3 years
  else if (ageY < 5.5f) { k_age = 0.90f; } // 4–5 years
  else { k_age = 1.00f; }                  // 6 years
}

// ====== Serial commands (AGE/B + modes) ======
void handleModeSwitch() {
  while (Serial.available()) {
    // AGE <years>
    if (Serial.peek()=='A' || Serial.peek()=='a') {
      String line = Serial.readStringUntil('\n'); line.trim();
      if (line.startsWith("AGE") || line.startsWith("age") || line.startsWith("Age")) {
        int sp = line.indexOf(' ');
        if (sp>0) {
          float ageY = line.substring(sp+1).toFloat();
          setAgeYears(ageY);
          Serial.print("# AGE set: "); Serial.print(ageY,1);
          Serial.print("y, k_age="); Serial.println(k_age,2);
        }
        continue;
      }
    }
    // B <sway0> <G/A/R>
    if (Serial.peek()=='B' || Serial.peek()=='b') {
      String line = Serial.readStringUntil('\n'); line.trim();
      if (line.length()) {
        // parse: B 0.12 A
        float s0 = BASE_SWAY; char gr='G';
        int sp1 = line.indexOf(' ');
        if (sp1>0) {
          s0 = line.substring(sp1+1).toFloat();
          int sp2 = line.indexOf(' ', sp1+1);
          if (sp2>0 && (sp2+1) < (int)line.length()) gr = toupper(line[sp2+1]);
        }
        BASE_SWAY = s0;
        if (gr=='G') BASE_GRADE=GRADE_GREEN; else if (gr=='A') BASE_GRADE=GRADE_AMBER; else BASE_GRADE=GRADE_RED;
        k_grade = riskScaleForU8(BASE_GRADE);
        Serial.print("# Baseline sway0="); Serial.print(BASE_SWAY,3); Serial.print(" g, grade="); Serial.println(gr);
      }
      continue;
    }
    // Mode switches (P/M) and zero calibration (Z)
    char ch = Serial.read();
    if (ch=='P' || ch=='p') outMode = MODE_PLOT;
    else if (ch=='M' || ch=='m') outMode = MODE_MONITOR;
    else if (ch=='Z' || ch=='z') { roll0 = chestRollDeg; pitch0 = chestPitchDeg; verboseAngles = !verboseAngles; Serial.print("# Zero tilt: roll0="); Serial.print(roll0,1); Serial.print(" pitch0="); Serial.println(pitch0,1); Serial.println(verboseAngles ? "ON" : "OFF"); }
  }
}

// ====== WebSocket event callback (client) ======
void onWsEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to Node relay");
      wsClient.sendTXT("ROLE:nano");
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      break;
    case WStype_TEXT:
      Serial.print("[WS<-] "); Serial.write(payload, length); Serial.println();
      // Optionally parse commands from UI here using ArduinoJson
      break;
    case WStype_PING:
    case WStype_PONG:
    default:
      break;
  }
}

// ====== Broadcast current metrics to Node/UI ======
void wsBroadcastMetrics(float chestRMS, float waistRMS,
                        const char* STATE, const char* FALL_RISK,
                        float k_eff, float freqHz,
                        float thrSway, float thrCW) {
  if (!wsEnabled) return;
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{"
      "\"type\":\"nano\"," 
      "\"chestRMS\":%.3f,"
      "\"waistRMS\":%.3f,"
      "\"state\":\"%s\"," 
      "\"fallRisk\":\"%s\"," 
      "\"k\":%.2f,"
      "\"freqHz\":%.2f,"
      "\"thrSway\":%.3f,"
      "\"thrCW\":%.3f"
    "}",
    chestRMS, waistRMS, STATE, FALL_RISK, k_eff, freqHz, thrSway, thrCW
  );
  if (n > 0 && n < (int)sizeof(buf)) wsClient.sendTXT(buf);
}

// ========================== Setup ==========================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis(); while (!Serial && millis() - t0 < 2000) {}
  Serial.setTimeout(10); // avoid long blocking in readStringUntil

  // I2C
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  // Sensors
  bool chestOK = mpuChest.testConnection();
  bool waistOK = mpuWaist.testConnection();
  if (!chestOK || !waistOK) {
    // If using two MPU6050 on same bus, ensure one has AD0 pulled up to make 0x69
    mpuChest.initialize(); mpuWaist.initialize();
    delay(50);
    chestOK = mpuChest.testConnection();
    waistOK = mpuWaist.testConnection();
  }
  if (!chestOK && !waistOK) {
    Serial.println("# Both MPUs missing, check wiring & 3.3V!");
    while (1) { delay(1000); }
  }

  // Prior defaults
  k_grade = riskScaleForU8(BASE_GRADE);
  setAgeYears(25.0f);

  // Wi-Fi connect with timeout (non-fatal)
  Serial.print(F("# Connecting WiFi ")); Serial.println(ssid);
  WiFi.begin(ssid, pass);
  unsigned long w0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - w0) < 15000UL) {
    delay(500);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\n# WiFi OK, IP=")); Serial.println(WiFi.localIP());
    wsClient.begin(SERVER_HOST, SERVER_PORT, "/");
    wsClient.onEvent(onWsEvent);
    wsClient.setReconnectInterval(2000); // auto reconnect
    // wsClient.enableHeartbeat(15000, 3000, 2); // 15s ping, 3s timeout, 2 tries
    wsEnabled = true;
    Serial.println(F("# WebSocket client -> ws://"));
  } else {
    Serial.println(F("\n# WiFi failed, continuing without WebSocket"));
    wsEnabled = false;
  }

  windowStartMs = millis();
  Serial.println("# MODE=PLOT. Commands: 'M' monitor, 'P' plot, 'Z' zero, 'V' verbose, 'AGE <y>', 'B <sway0> <G/A/R>'.");
}

// ========================= Main Loop =======================
void loop() {
  if (wsEnabled) wsClient.loop();

  unsigned long now = millis();
  handleModeSwitch();

  // Sampling rate control
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  // ===== Read IMUs =====
  int16_t axc, ayc, azc, gxc, gyc, gzc;
  int16_t axw, ayw, azw, gxw, gyw, gzw;
  mpuChest.getMotion6(&axc,&ayc,&azc,&gxc,&gyc,&gzc);
  mpuWaist.getMotion6(&axw,&ayw,&azw,&gxw,&gyw,&gzw);

  float acx, acy, acz, awx, awy, awz;
  mapBodyAxes(axc,ayc,azc, CHEST_MAP, CHEST_SIGN, acx,acy,acz);
  mapBodyAxes(axw,ayw,azw, WAIST_MAP, WAIST_SIGN, awx,awy,awz);

  float amagChest = sqrtf(acx*acx + acy*acy + acz*acz);
  float amagWaist = sqrtf(awx*awx + awy*awy + awz*awz);

  lpAmagChest = LPF_A*lpAmagChest + LPF_B*amagChest;
  lpAmagWaist = LPF_A*lpAmagWaist + LPF_B*amagWaist;

  float dcChest = lpAmagChest - 1.0f;
  float dcWaist = lpAmagWaist - 1.0f;
  rmsAccChest += dcChest*dcChest;
  rmsAccWaist += dcWaist*dcWaist;
  rmsCount++;

  // Gravity direction EMA (debug)
  gCx = (1-GEMA_ALPHA)*gCx + GEMA_ALPHA*acx;
  gCy = (1-GEMA_ALPHA)*gCy + GEMA_ALPHA*acy;
  gCz = (1-GEMA_ALPHA)*gCz + GEMA_ALPHA*acz;
  gWx = (1-GEMA_ALPHA)*gWx + GEMA_ALPHA*awx;
  gWy = (1-GEMA_ALPHA)*gWy + GEMA_ALPHA*awy;
  gWz = (1-GEMA_ALPHA)*gWz + GEMA_ALPHA*awz;

  // Tilt (debug)
  chestRollDeg = atan2f(gCy, gCz) * 180.0f / 3.1415926f;
  chestPitchDeg = atan2f(-gCx, sqrtf(gCy*gCy + gCz*gCz)) * 180.0f / 3.1415926f;
  waistRollDeg  = atan2f(gWy, gWz) * 180.0f / 3.1415926f;
  waistPitchDeg = atan2f(-gWx, sqrtf(gWy*gWy + gWz*gWz)) * 180.0f / 3.1415926f;

  // Step peaks
  bool chestPeak = risingCross(0.0f, lpAmagChest, STEP_THR_WALK, now, lastPeakChest);
  bool waistPeak = risingCross(0.0f, lpAmagWaist, STEP_THR_WALK, now, lastPeakWaist);
  if ((chestPeak || waistPeak) && (now - lastUnifiedStep) > UNIFIED_REFRACT_MS) {
    unifiedStepCountWindow++;
    lastUnifiedStep = now;
  }

  // Low-g / impact events
  if (!ffSeen && (lpAmagChest < FREE_FALL_G && lpAmagWaist < FREE_FALL_G)) { ffSeen = true; ffTime = now; }
  if (!impactSeen && (lpAmagChest > IMPACT_G || lpAmagWaist > IMPACT_G)) { impactSeen = true; impactTime = now; }

  // ==== Output window ====
  if ((now - windowStartMs) >= UPDATE_MS) {
    const float dt = (now - windowStartMs) / 1000.0f;

    // RMS over window
    float chestRMS = sqrtf(rmsAccChest / max(1, (int)rmsCount));
    float waistRMS = sqrtf(rmsAccWaist / max(1, (int)rmsCount));

    // Unified step frequency (per second over window)
    float freqInst = (float)unifiedStepCountWindow / dt; // steps/s ~ Hz
    freqEMA = FREQ_EMA_A*freqEMA + FREQ_EMA_B*freqInst;

    // Effective thresholds
    float k_eff = k_age * k_grade;
    float SWAY_RMS_HIGH_eff = (SWAY_RMS_HIGH + BASE_SWAY) * k_eff;
    float CHEST_WAIST_DELTA_eff = CHEST_WAIST_DELTA * k_eff;

    // Imbalance decision
    bool rmsHigh = (chestRMS > SWAY_RMS_HIGH_eff) || (waistRMS > SWAY_RMS_HIGH_eff);
    bool cwDelta = fabsf(chestRMS - waistRMS) > CHEST_WAIST_DELTA_eff;

    const char* STATE = "STILL";
    if (!imbalance) {
      if (rmsHigh || cwDelta) {
        if (imbPersistTimer==0) imbPersistTimer = now; // enter timer
        if ((now - imbPersistTimer) >= IMB_ENTER_MS) { imbalance = true; STATE = "IMBALANCE"; }
      } else {
        imbPersistTimer = 0; // reset
      }
    } else {
      if (!(rmsHigh || cwDelta)) {
        if (imbPersistTimer==0) imbPersistTimer = now; // exit timer
        if ((now - imbPersistTimer) >= IMB_EXIT_MS) { imbalance = false; STATE = "STILL"; }
      } else {
        imbPersistTimer = 0; // stay
        STATE = "IMBALANCE";
      }
    }

    // Fall risk decision tree
    const char* FALL_RISK = "LOW";
    if (ffSeen && impactSeen && (impactTime > ffTime) && (impactTime - ffTime) <= FF_TO_IMPACT_MS) {
      // Low-g followed by impact
      // If stillness afterwards → FALL
      if ((now - impactTime) >= STILL_AFTER_MS) {
        FALL_RISK = "FALL";
        fallAnnounced = true; ffSeen=false; impactSeen=false; ffTime=impactTime=0;
      } else {
        FALL_RISK = "HIGH";
      }
    } else if (rmsHigh && freqEMA < 0.4f) {
      // High sway without stepping much
      FALL_RISK = "MEDIUM";
    } else {
      FALL_RISK = imbalance ? "MEDIUM" : "LOW";
    }

    // Send to Node/UI
    wsBroadcastMetrics(chestRMS, waistRMS, STATE, FALL_RISK,
                       k_eff, freqEMA, SWAY_RMS_HIGH_eff, CHEST_WAIST_DELTA_eff);

    // ==== Event lifecycle ====
    if (fallAnnounced) {
      // clear after one broadcast to allow next detection
      ffSeen = false; ffTime = 0; impactTime = 0; fallAnnounced = false;
    } else {
      // expire stale partial sequences
      if (ffSeen && (now - ffTime) > FF_TO_IMPACT_MS) { ffSeen = false; ffTime = 0; }
      if (impactTime && (now - impactTime) > (STILL_AFTER_MS + 1500)) { impactTime = 0; }
    }

    // Reset window
    windowStartMs = now;
    rmsAccChest = rmsAccWaist = 0.0f; rmsCount = 0;
    unifiedStepCountWindow = 0;
  }
}
