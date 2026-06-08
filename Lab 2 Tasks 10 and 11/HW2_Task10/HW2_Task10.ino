#include <Arduino_APDS9960.h>
#include <Arduino_LSM9DS1.h>
#include <PDM.h>

// ── Thresholds ──────────────────────────────────────────────
#define MIC_THRESHOLD       500     // raw RMS above this → noisy
#define LIGHT_THRESHOLD     100     // clear channel below this → dark
#define MOTION_THRESHOLD    0.08f   // accel delta (g) above this → moving
#define PROX_THRESHOLD      50      // proximity above this → near (0–255)

// ── PDM mic globals ─────────────────────────────────────────
static volatile int16_t micBuffer[256];
static volatile int     micSamples = 0;

void onPDMdata() {
  int bytes = PDM.available();
  PDM.read((int16_t*)micBuffer, bytes);
  micSamples = bytes / 2;
}

int computeMicRMS() {
  long sum = 0;
  int  n   = micSamples;
  for (int i = 0; i < n; i++) sum += (long)micBuffer[i] * micBuffer[i];
  return (n > 0) ? (int)sqrt((double)sum / n) : 0;
}

// ── IMU baseline ────────────────────────────────────────────
float baseAx = 0, baseAy = 0, baseAz = 0;

void calibrateIMU() {
  float sx = 0, sy = 0, sz = 0;
  int   count = 0;
  unsigned long t = millis();
  while (millis() - t < 1000) {
    float x, y, z;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      sx += x; sy += y; sz += z; count++;
    }
  }
  if (count > 0) { baseAx = sx/count; baseAy = sy/count; baseAz = sz/count; }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // APDS9960
  if (!APDS.begin()) {
    Serial.println("ERROR: APDS9960 init failed");
    while (1);
  }

  // IMU
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU init failed");
    while (1);
  }

  // PDM microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("ERROR: PDM init failed");
    while (1);
  }

  Serial.println("Calibrating IMU baseline (keep board still)...");
  calibrateIMU();
  Serial.println("Ready.\n");
}

void loop() {
  // ── 1. Microphone ──────────────────────────────────────────
  int micLevel = computeMicRMS();
  bool sound   = (micLevel > MIC_THRESHOLD);

  // ── 2. Light (APDS9960 clear channel) ─────────────────────
  int r, g, b, clear;
  bool dark = true;
  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, clear);
    dark = (clear < LIGHT_THRESHOLD);
  }

  // ── 3. Motion (IMU accelerometer delta) ───────────────────
  float ax, ay, az;
  float motionVal = 0;
  bool  moving    = false;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    float dx = ax - baseAx;
    float dy = ay - baseAy;
    float dz = az - baseAz;
    motionVal = sqrt(dx*dx + dy*dy + dz*dz);
    moving    = (motionVal > MOTION_THRESHOLD);
    // slowly track baseline to handle slow drift
    baseAx += 0.01f * dx;
    baseAy += 0.01f * dy;
    baseAz += 0.01f * dz;
  }

  // ── 4. Proximity (APDS9960) ────────────────────────────────
  int  prox = 0;
  bool near = false;
  if (APDS.proximityAvailable()) {
    prox = APDS.readProximity();   // 0 = far, 255 = very near
    near = (prox > PROX_THRESHOLD);
  }

  // ── 5. Rule-based fusion ───────────────────────────────────
  String label;
  if      (!sound && !dark && !moving && !near) label = "QUIET_BRIGHT_STEADY_FAR";
  else if ( sound && !dark && !moving && !near) label = "NOISY_BRIGHT_STEADY_FAR";
  else if (!sound &&  dark && !moving &&  near) label = "QUIET_DARK_STEADY_NEAR";
  else if ( sound && !dark &&  moving &&  near) label = "NOISY_BRIGHT_MOVING_NEAR";
  else {
    // closest match fallback
    int score[4] = {0, 0, 0, 0};
    // QUIET_BRIGHT_STEADY_FAR  → !sound !dark !moving !near
    score[0] = (!sound?1:0) + (!dark?1:0) + (!moving?1:0) + (!near?1:0);
    // NOISY_BRIGHT_STEADY_FAR  →  sound !dark !moving !near
    score[1] = ( sound?1:0) + (!dark?1:0) + (!moving?1:0) + (!near?1:0);
    // QUIET_DARK_STEADY_NEAR   → !sound  dark !moving  near
    score[2] = (!sound?1:0) + ( dark?1:0) + (!moving?1:0) + ( near?1:0);
    // NOISY_BRIGHT_MOVING_NEAR →  sound !dark  moving  near
    score[3] = ( sound?1:0) + (!dark?1:0) + ( moving?1:0) + ( near?1:0);

    int best = 0;
    for (int i = 1; i < 4; i++) if (score[i] > score[best]) best = i;
    const char* labels[] = {
      "QUIET_BRIGHT_STEADY_FAR", "NOISY_BRIGHT_STEADY_FAR",
      "QUIET_DARK_STEADY_NEAR",  "NOISY_BRIGHT_MOVING_NEAR"
    };
    label = String(labels[best]) + " (approx)";
  }

  // ── 6. Serial output ───────────────────────────────────────
  Serial.print("mic="); Serial.print(micLevel);
  Serial.print("  clear="); Serial.print(clear);
  Serial.print("  motion="); Serial.print(motionVal, 3);
  Serial.print("  prox="); Serial.println(prox);

  Serial.print("sound="); Serial.print(sound);
  Serial.print("  dark="); Serial.print(dark);
  Serial.print("  moving="); Serial.print(moving);
  Serial.print("  near="); Serial.println(near);

  Serial.print("FINAL_LABEL: "); Serial.println(label);
  Serial.println("─────────────────────────────────");

  delay(300);
}