
#include <Arduino.h>
#include <math.h>

#define USE_RAW_FEAT   0
#define DEBUG_RAW_CH  -1

#define SensorPin0 A0
#define SensorPin1 A1
#define SensorPin2 A2
#define SensorPin3 A3
#define SensorPin4 A4
#define SensorPin5 A5
const uint8_t CH_PINS[6] = {SensorPin0, SensorPin1, SensorPin2, SensorPin3, SensorPin4, SensorPin5};

const float ADC_REF_V   = 5.0f;
const int   ADC_MAX     = 1023;
const float ZERO_SHIFT  = 0.0f;

float OUT_THRESH_N = 0.0f;

const uint8_t  READS_PER_LOOP = 1;
const uint16_t CALI_SAMPLES   = 100;
const uint16_t SAMPLE_US      = 40;

const float    CONTACT_THRESH_N   = 20.0f; 
const uint16_t CONTACT_HOLD_MS    = 1000;
const uint16_t AUTOTEST_MS        = 5000;
const uint16_t RETRIGGER_GAP_MS   = 0;

float BASE_SWAY = 1.2f;

struct Stat { double mean=0, m2=0; unsigned long n=0; };
static inline void updStat(Stat& s, double x){
  s.n++; double d = x - s.mean; s.mean += d / s.n; s.m2 += d * (x - s.mean);
}

static bool inContact=false, testing=false;
static unsigned long tContact=0, tEnd=0, tLastDone=0;
static Stat sLR, sAP;

static float LAST_LR_rms = -1.0f;
static float LAST_AP_rms = -1.0f;
static float LAST_Sway   = -1.0f;
static float LAST_Ratio  = -1.0f;
static const char* LAST_Grade = "NA";

float zero_feat[6]  = {0};
float scale_ch[6];
float offset_ch[6];
bool  has_zero[6]   = {false};
bool  has_gain[6]   = {false};

float   acc_feat[6] = {0};
uint8_t winCount    = 0;
const uint8_t WIN_AVG = 8;

uint16_t read_adc_med3(uint8_t pin){
  uint16_t a = analogRead(pin);
  uint16_t b = analogRead(pin);
  uint16_t c = analogRead(pin);
  // 返回中位数
  return max(min(a,b), min(max(a,b), c));
}

static inline float counts_to_feat(float adc_counts_avg) {
#if USE_RAW_FEAT
  return adc_counts_avg;
#else
  float a = (adc_counts_avg * ADC_REF_V / ADC_MAX) - ZERO_SHIFT; 
  float t = (a * 10.0f - 1.0f) / 100.0f;
  float feat = -647320.0f * t * t + 223207.0f * t + 1038.6f;
  return feat;
#endif
}

static inline float fallback_feat_to_N(float feat) {
#if USE_RAW_FEAT

  const float K = 200.0f / 1023.0f;
  return feat * K;
#else
  const float GF_TO_N = 0.00980665f;
  return feat * GF_TO_N;
#endif
}

float read_channel_feat(uint8_t ch, uint16_t samples) {
  uint8_t pin = CH_PINS[ch];
  (void)analogRead(pin);
  delayMicroseconds(SAMPLE_US);

  float sum = 0.0f;
  for (uint16_t i = 0; i < samples; ++i) {
    uint16_t raw = read_adc_med3(pin);
    sum += counts_to_feat((float)raw);
    delayMicroseconds(SAMPLE_US);
  }
  return sum / (float)samples;
}

float feat_to_output_N(uint8_t ch, float feat) {
  float N;
  if (has_zero[ch] && has_gain[ch]) {
    N = scale_ch[ch] * feat + offset_ch[ch];
  } else {
    N = fallback_feat_to_N(feat);
  }
  if (N < OUT_THRESH_N) N = 0.0f;
  return N;
}

void print_help() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  zero all"));
  Serial.println(F("  zero <ch>            (ch = 0..5)"));
  Serial.println(F("  load <ch> <N>        (apply known load in Newton on that channel)"));
  Serial.println(F("  show"));
  Serial.println(F("  base <sway0>         (set personal baseline sway, e.g. base 0.120)"));
}

void handle_command(String line) {
  line.trim();
  if (line.length() == 0) return;

  String a, b, c;
  int sp1 = line.indexOf(' ');
  if (sp1 < 0) {
    a = line;
  } else {
    a = line.substring(0, sp1);
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp2 < 0) {
      b = line.substring(sp1 + 1);
    } else {
      b = line.substring(sp1 + 1, sp2);
      c = line.substring(sp2 + 1);
    }
  }
  a.toLowerCase(); b.toLowerCase();

  if (a == "help") { print_help(); return; }

  if (a == "base") {
    // 用法： base 0.120
    float b0 = b.toFloat();
    if (b0 > 0.0f) {
      BASE_SWAY = 1.0f + b0;
      Serial.print("THR:");
      Serial.println(BASE_SWAY, 3);
    } else {
      Serial.println(F("ERR: base <sway0> must be > 0"));
    }
    return;
  }

  if (a == "zero") {
    if (b == "all") {
      Serial.println(F("[ZERO] Keep all sensors unloaded..."));
      for (uint8_t ch = 0; ch < 6; ++ch) {
        zero_feat[ch] = read_channel_feat(ch, CALI_SAMPLES);
        has_zero[ch]  = true;
      }
      Serial.println(F("[ZERO] Done for all channels."));
    } else {
      int ch = b.toInt();
      if (ch < 0 || ch > 5) { Serial.println(F("ERR: ch must be 0..5 or 'all'")); return; }
      Serial.print(F("[ZERO] ch=")); Serial.println(ch);
      zero_feat[ch] = read_channel_feat(ch, CALI_SAMPLES);
      has_zero[ch]  = true;
      Serial.print(F("[ZERO] feat=")); Serial.println(zero_feat[ch], 3);
      Serial.println(F("[ZERO] Done."));
    }
    return;
  }

  if (a == "load") {
    int ch = b.toInt();
    if (ch < 0 || ch > 5) { Serial.println(F("ERR: ch must be 0..5")); return; }
    float loadN = c.toFloat();
    if (loadN <= 0) { Serial.println(F("ERR: <N> must be positive")); return; }
    if (!has_zero[ch]) { Serial.println(F("ERR: Do 'zero <ch>' first")); return; }

    Serial.print(F("[LOAD] ch=")); Serial.print(ch);
    Serial.print(F(", target N=")); Serial.println(loadN, 3);
    Serial.println(F("Place the weight now and keep it stable..."));

    float load_feat = read_channel_feat(ch, CALI_SAMPLES);
    float delta_feat = load_feat - zero_feat[ch];
    if (fabs(delta_feat) < 1e-3f) {
      Serial.println(F("ERR: delta feat too small; check wiring/weight"));
      return;
    }
    scale_ch[ch]  = loadN / delta_feat;
    offset_ch[ch] = -scale_ch[ch] * zero_feat[ch];
    has_gain[ch]  = true;

    Serial.print(F("[CAL] ch=")); Serial.print(ch);
    Serial.print(F(", zero_feat=")); Serial.print(zero_feat[ch], 3);
    Serial.print(F(", load_feat=")); Serial.print(load_feat, 3);
    Serial.print(F(", scale=")); Serial.print(scale_ch[ch], 6);
    Serial.print(F(", offset=")); Serial.println(offset_ch[ch], 3);
    Serial.println(F("[CAL] Done."));
    return;
  }

  if (a == "show") {
    for (uint8_t ch = 0; ch < 6; ++ch) {
      Serial.print(F("ch")); Serial.print(ch);
      Serial.print(F(": zero="));  Serial.print(zero_feat[ch], 3);
      Serial.print(F(", scale=")); Serial.print(scale_ch[ch], 6);
      Serial.print(F(", offset="));Serial.print(offset_ch[ch], 3);
      Serial.print(F(", ready=")); Serial.println(has_zero[ch] && has_gain[ch] ? "Y":"N");
    }
    Serial.print(F("OUT_THRESH_N=")); Serial.println(OUT_THRESH_N, 2);
    Serial.print(F("BASE_SWAY="));    Serial.println(BASE_SWAY, 3);
    return;
  }

  Serial.println(F("Unknown cmd. Type 'help'."));
}

void setup() {
  Serial.begin(115200);
  delay(100);
  analogReference(DEFAULT);

  for (uint8_t ch = 0; ch < 6; ++ch) {
    scale_ch[ch]  = (USE_RAW_FEAT ? (200.0f/1023.0f) : 0.00980665f);
    offset_ch[ch] = 0.0f;
  }

  Serial.println(F("6ch pressure reader (UNO) + Auto 5s Sway test"));
  print_help();

  if (BASE_SWAY > 0.0f) {
    Serial.print("THR:");
    Serial.println(BASE_SWAY, 3);
  }
}

String cmdBuf;
void loop() {
  unsigned long now = millis();

  for (uint8_t ch = 0; ch < 6; ++ch) {
    uint16_t raw = read_adc_med3(CH_PINS[ch]);

#if DEBUG_RAW_CH >= 0
    if (ch == DEBUG_RAW_CH) {
      Serial.print(F("RAW")); Serial.print(ch); Serial.print('=');
      Serial.println(raw);
    }
#endif

    float feat = counts_to_feat((float)raw);
    acc_feat[ch] += feat;
  }

  winCount++;
  if (winCount >= WIN_AVG) {
    float featAvg[6];
    for (uint8_t ch = 0; ch < 6; ++ch) {
      featAvg[ch]   = acc_feat[ch] / (float)WIN_AVG;
      acc_feat[ch]  = 0.0f;
    }
    winCount = 0;

    float A1_N = feat_to_output_N(0, featAvg[0]);
    float B1_N = feat_to_output_N(1, featAvg[1]);
    float C1_N = feat_to_output_N(2, featAvg[2]);
    float A2_N = feat_to_output_N(3, featAvg[3]);
    float B2_N = feat_to_output_N(4, featAvg[4]);
    float C2_N = feat_to_output_N(5, featAvg[5]);

    float FL = A1_N + B1_N + C1_N;
    float FR = A2_N + B2_N + C2_N;
    float Ftot = FL + FR;

    float LR = 0.0f, AP = 0.0f;
    if (Ftot >= CONTACT_THRESH_N) {
      LR = (FR - FL) / Ftot;
      AP = ((B1_N + C1_N + B2_N + C2_N) - (A1_N + A2_N)) / Ftot;
    }

    if (Ftot >= CONTACT_THRESH_N) {
      if (!inContact) { inContact = true; tContact = now; }
      if (!testing && (now - tContact >= CONTACT_HOLD_MS) && (now - tLastDone >= RETRIGGER_GAP_MS)) {
        testing = true; tEnd = now + AUTOTEST_MS;
        sLR = Stat(); sAP = Stat(); 
        Serial.println(F("[AUTO] start 5s test"));
      }
    } else {
      inContact = false;
    }

    if (testing) {
      if (Ftot >= CONTACT_THRESH_N) {
        updStat(sLR, LR);
        updStat(sAP, AP);
      }
      if ((long)(now - tEnd) >= 0) {
        testing = false; tLastDone = now;

        double LR_rms = (sLR.n>1)? sqrt(sLR.m2/(sLR.n-1)) : 0.0;
        double AP_rms = (sAP.n>1)? sqrt(sAP.m2/(sAP.n-1)) : 0.0;
        double Sway   = LR_rms + AP_rms;

        const char* grade = "need_baseline";
        float ratio_out = -1.0f;
        if (BASE_SWAY > 0.0f) {
          double ratio = Sway / BASE_SWAY;
          ratio_out = (float)ratio;
          if (ratio <= 1.2)       grade = "Green";
          else if (ratio <= 1.5)  grade = "Amber";
          else                    grade = "Red";

          Serial.print("RESULT_AUTO,ratio="); Serial.print(ratio,3); Serial.print(",");
        }
        Serial.print("LR_rms="); Serial.print(LR_rms,3);
        Serial.print(",AP_rms="); Serial.print(AP_rms,3);
        Serial.print(",Sway=");   Serial.print(Sway,3);
        Serial.print(",LR_mean=");Serial.print(sLR.mean,3);
        Serial.print(",AP_mean=");Serial.print(sAP.mean,3);
        Serial.print(",n=");      Serial.print(sLR.n);
        Serial.print(",grade=");  Serial.println(grade);

        LAST_LR_rms = (float)LR_rms;
        LAST_AP_rms = (float)AP_rms;
        LAST_Sway   = (float)Sway;
        LAST_Grade  = grade;
        LAST_Ratio  = ratio_out; 
      }
    }

    Serial.print("A1: "); Serial.print(A1_N, 1); Serial.println(" N");
    Serial.print("B1: "); Serial.print(B1_N, 1); Serial.println(" N");
    Serial.print("C1: "); Serial.print(C1_N, 1); Serial.println(" N");
    Serial.print("A2: "); Serial.print(A2_N, 1); Serial.println(" N");
    Serial.print("B2: "); Serial.print(B2_N, 1); Serial.println(" N");
    Serial.print("C2: "); Serial.print(C2_N, 1); Serial.println(" N");

    Serial.print("BASE_RATIO_LAST: ");
    if (LAST_Ratio >= 0.0f) Serial.println(LAST_Ratio + 1.0f, 3); 
    else                    Serial.println("NA");

    delay(500); 
  }


  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdBuf.length()) { handle_command(cmdBuf); cmdBuf = ""; }
    } else {
      cmdBuf += ch;
      if (cmdBuf.length() > 100) cmdBuf = ""; 
    }
  }

  delay(5);
}
