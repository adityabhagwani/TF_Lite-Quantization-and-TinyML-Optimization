#include <Arduino_APDS9960.h>
#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>

// ── Thresholds ──────────────────────────────────────────────
#define HUMID_JUMP_THRESHOLD     3.0f
#define TEMP_RISE_THRESHOLD      0.8f
#define MAG_SHIFT_THRESHOLD      50.0f
#define LIGHT_CHANGE_THRESHOLD   80

float baseRH, baseTemp, baseMag;
int   baseClear;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!APDS.begin())  { Serial.println("ERROR: APDS9960 init failed"); while(1); }
  if (!HS300x.begin()) { Serial.println("ERROR: HS3003 init failed");  while(1); }
  if (!IMU.begin())   { Serial.println("ERROR: IMU init failed");      while(1); }

  // Calibrate baseline over 2 seconds
  Serial.println("Calibrating baseline (keep environment stable)...");
  float sRH=0, sTemp=0, sMag=0; int sClear=0, count=0;
  unsigned long t = millis();
  while (millis() - t < 2000) {
    float mx, my, mz;
    int r, g, b, c;
    if (HS300x.available()) {
      sRH   += HS300x.readHumidity();
      sTemp += HS300x.readTemperature();
    }
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(mx, my, mz);
      sMag += sqrt(mx*mx + my*my + mz*mz);
    }
    if (APDS.colorAvailable()) {
      APDS.readColor(r, g, b, c);
      sClear += c;
    }
    count++;
    delay(50);
  }
  baseRH    = sRH    / count;
  baseTemp  = sTemp  / count;
  baseMag   = sMag   / count;
  baseClear = sClear / count;
  Serial.println("Baseline set. Running...\n");
}

void loop() {
  static unsigned long lastTrigger = 0;

  // ── Read sensors ───────────────────────────────────────────
  float rh   = HS300x.readHumidity();
  float temp = HS300x.readTemperature();

  float mx, my, mz, mag = 0;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    mag = sqrt(mx*mx + my*my + mz*mz);
  }

  int r=0, g=0, b=0, clear=0;
  if (APDS.colorAvailable()) APDS.readColor(r, g, b, clear);

  // ── Binary decisions ───────────────────────────────────────
  bool humid_jump            = (rh   - baseRH)    > HUMID_JUMP_THRESHOLD;
  bool temp_rise             = (temp - baseTemp)   > TEMP_RISE_THRESHOLD;
  bool mag_shift             = (mag  - baseMag)    > MAG_SHIFT_THRESHOLD;
  bool light_or_color_change = abs(clear - baseClear) > LIGHT_CHANGE_THRESHOLD;

  bool cooldownOk = (millis() - lastTrigger > 500);

  // ── Rule-based label ───────────────────────────────────────
  String label;
  if (mag_shift && cooldownOk) {
    label = "MAGNETIC_DISTURBANCE_EVENT"; lastTrigger = millis();
  } else if (light_or_color_change && cooldownOk) {
    label = "LIGHT_OR_COLOR_CHANGE_EVENT"; lastTrigger = millis();
  } else if ((humid_jump || temp_rise) && cooldownOk) {
    label = "BREATH_OR_WARM_AIR_EVENT"; lastTrigger = millis();
  } else {
    label = "BASELINE_NORMAL";
  }

  // ── Serial output ──────────────────────────────────────────
  Serial.print("rh=");    Serial.print(rh, 1);
  Serial.print("  temp="); Serial.print(temp, 1);
  Serial.print("  mag=");  Serial.print(mag, 1);
  Serial.print("  r=");   Serial.print(r);
  Serial.print("  g=");   Serial.print(g);
  Serial.print("  b=");   Serial.print(b);
  Serial.print("  clear="); Serial.println(clear);

  Serial.print("humid_jump=");              Serial.print(humid_jump);
  Serial.print("  temp_rise=");             Serial.print(temp_rise);
  Serial.print("  mag_shift=");             Serial.print(mag_shift);
  Serial.print("  light_or_color_change="); Serial.println(light_or_color_change);

  Serial.print("FINAL_LABEL: "); Serial.println(label);
  Serial.println("─────────────────────────────────");

  delay(350);
}