#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MAX31855.h>
#include <PID_v1.h>

// ─── Display ───────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─── Pin definitions ───────────────────────────────────────
const int SPI_CLK_PIN  = 4;   // MAX31855 SCK
const int SPI_MISO_PIN = 5;   // MAX31855 DO
const int CS_PIN       = 7;   // MAX31855 CS
const int SDA_PIN      = 8;   // OLED SDA
const int SCL_PIN      = 9;   // OLED SCL
const int BOILER_PIN   = 10;  // SSR control
const int TOGGLE_PIN   = 20;  // Mode toggle button, pullup, closes to GND
const int ADC_BUTTONS  = 21;  // Resistor ladder: temp up (10kΩ) + temp down (47kΩ)
const int PUMP_PIN     = 2;   // MOC3021 PWM out
// const int ADC_POT      = 6;  // Pot wiper — not yet wired (pot snapped)
const int BREW_SENSE      = 1;   // Dry contact closure, 10kΩ pullup to 3.3V, active low
// const int ADC_PRESSURE = 2;  // Pressure transducer — not yet wired

// ─── Thermocouple ──────────────────────────────────────────
Adafruit_MAX31855 thermocouple(SPI_CLK_PIN, CS_PIN, SPI_MISO_PIN);

// ─── PID ───────────────────────────────────────────────────
// Kd bumped 180→280 to dampen 5-10F warmup overshoot
// If still overshooting: increase Kd toward 350-400
// If response near setpoint feels sluggish: back Kd toward 220
double BOILER_SETPOINT = 205.0;
double CURRENT_TEMP    = 0.0;
double PID_OUTPUT      = 0.0;
double Kp = 38, Ki = 4, Kd = 250;

unsigned long previousTime = 0;
unsigned long cycleTime    = 2000;
unsigned long offTime      = 0;
bool BOILER_ON             = false;

PID mainPID(&CURRENT_TEMP, &PID_OUTPUT, &BOILER_SETPOINT, Kp, Ki, Kd, DIRECT);

// ─── Mode ──────────────────────────────────────────────────
bool STEAM_ON = false;

const double BREW_SETPOINT_DEFAULT  = 205.0;
const double STEAM_SETPOINT_DEFAULT = 280.0;
const double BREW_SETPOINT_MIN      = 185.0;
const double BREW_SETPOINT_MAX      = 220.0;
const double STEAM_SETPOINT_MIN     = 250.0;
const double STEAM_SETPOINT_MAX     = 300.0;

// ─── Shot timer ────────────────────────────────────────────
bool brewActive                = false;
bool lastBrewRaw               = HIGH;
unsigned long brewStartTime    = 0;
unsigned long shotElapsed      = 0;   // ms, frozen at shot end
bool shotDone                  = false;

// ─── Display timing ────────────────────────────────────────
unsigned long lastDisplayUpdate      = 0;
const unsigned long DISPLAY_INTERVAL = 2000;

// ─── Button ladder ADC thresholds ──────────────────────────
// Temp up (left):     10kΩ to GND → ~1.65V → ADC ~2048
// Temp down (center): 47kΩ to GND → ~2.77V → ADC ~3440
// Nothing pressed:    floating high          → ADC ~4095
const int THRESH_TEMPUP_LO   = 1500;
const int THRESH_TEMPUP_HI   = 2600;
const int THRESH_TEMPDOWN_LO = 2700;
const int THRESH_TEMPDOWN_HI = 3800;

// ─── Button state ──────────────────────────────────────────
bool lastToggleRaw                 = HIGH;
unsigned long toggleDebounce       = 0;
const unsigned long TOGGLE_DEBOUNCE_MS = 50;

bool tempUpPressed                 = false;
bool tempDownPressed               = false;
unsigned long leftButtonHoldStart  = 0;
bool settingsStubFired             = false;
unsigned long lastTempUpDebounce   = 0;
unsigned long lastTempDownDebounce = 0;
const unsigned long DEBOUNCE_MS    = 50;
const unsigned long LONG_PRESS_MS  = 2000;

// ─── Setup ─────────────────────────────────────────────────
void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  pinMode(BOILER_PIN, OUTPUT);
  digitalWrite(BOILER_PIN, LOW);

  pinMode(TOGGLE_PIN, INPUT_PULLUP);
  pinMode(BREW_SENSE, INPUT_PULLUP);

  ledcAttach(PUMP_PIN, 60, 8);

  mainPID.SetMode(AUTOMATIC);
  mainPID.SetOutputLimits(0, cycleTime);
  mainPID.SetSampleTime(1000);

  delay(500);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  startupScreen();
  delay(1500);
  display.clearDisplay();
}

void pumpOn(int duty){
  ledcWrite(PUMP_PIN, duty);
}

void pumpOff(){
  ledcWrite(PUMP_PIN, 0);
}

// ─── Loop ──────────────────────────────────────────────────
void loop() {
  readToggle();
  readButtons();
  readBrewSense();

  CURRENT_TEMP = thermocouple.readFahrenheit() - 10;

  mainPID.Compute();

  unsigned long now = millis();

  if (!BOILER_ON && now - previousTime >= cycleTime) {
    previousTime = now;
    offTime = now + (unsigned long)PID_OUTPUT;
    BOILER_ON = true;
    digitalWrite(BOILER_PIN, HIGH);
  }

  if (BOILER_ON && now >= offTime) {
    digitalWrite(BOILER_PIN, LOW);
    BOILER_ON = false;
  }

  unsigned long displayInterval = brewActive ? 500UL : DISPLAY_INTERVAL;
  if (now - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = now;
    mainDisplay();
  }
}

// ─── Toggle reading (GPIO20, firmware latched) ─────────────
void readToggle() {
  unsigned long now = millis();
  bool raw = digitalRead(TOGGLE_PIN);

  if (lastToggleRaw == HIGH && raw == LOW) {
    if (now - toggleDebounce > TOGGLE_DEBOUNCE_MS) {
      STEAM_ON = !STEAM_ON;
      BOILER_SETPOINT = STEAM_ON ? STEAM_SETPOINT_DEFAULT : BREW_SETPOINT_DEFAULT;
      resetPID();
      toggleDebounce = now;
    }
  }

  lastToggleRaw = raw;
}

// ─── Button ladder reading (GPIO21) ────────────────────────
void readButtons() {
  unsigned long now = millis();
  int raw = analogRead(ADC_BUTTONS);

  // Uncomment to debug ADC values:
  // Serial.println(raw);

  // ── Temp up (left button) ──
  bool tempUpNow = (raw >= THRESH_TEMPUP_LO && raw < THRESH_TEMPUP_HI);

  if (tempUpNow && !tempUpPressed) {
    leftButtonHoldStart = now;
    settingsStubFired = false;
    lastTempUpDebounce = now;
  }

  if (tempUpNow && !settingsStubFired) {
    if (now - leftButtonHoldStart >= LONG_PRESS_MS) {
      settingsStubFired = true;
      enterSettingsStub();
    }
  }

  if (!tempUpNow && tempUpPressed && !settingsStubFired) {
    if (now - lastTempUpDebounce > DEBOUNCE_MS) {
      adjustSetpoint(1.0);
    }
  }

  tempUpPressed = tempUpNow;

  // ── Temp down (center button) ──
  bool tempDownNow = (raw >= THRESH_TEMPDOWN_LO && raw < THRESH_TEMPDOWN_HI);

  if (tempDownNow && !tempDownPressed) {
    lastTempDownDebounce = now;
  }

  if (!tempDownNow && tempDownPressed) {
    if (now - lastTempDownDebounce > DEBOUNCE_MS) {
      adjustSetpoint(-1.0);
    }
  }

  tempDownPressed = tempDownNow;
}

// ─── Setpoint adjustment ───────────────────────────────────
void adjustSetpoint(double delta) {
  BOILER_SETPOINT += delta;
  if (STEAM_ON) {
    BOILER_SETPOINT = constrain(BOILER_SETPOINT, STEAM_SETPOINT_MIN, STEAM_SETPOINT_MAX);
  } else {
    BOILER_SETPOINT = constrain(BOILER_SETPOINT, BREW_SETPOINT_MIN, BREW_SETPOINT_MAX);
  }
}

// ─── PID reset ─────────────────────────────────────────────
void resetPID() {
  mainPID.SetMode(MANUAL);
  PID_OUTPUT = 0;
  mainPID.SetMode(AUTOMATIC);
}

// ─── Settings stub ─────────────────────────────────────────
void enterSettingsStub() {
  // TODO: implement settings menu
  // Suggested: Kp, Ki, Kd tuning, brew/steam setpoint defaults,
  // PID cycle time, temp offset calibration
  Serial.println(F("Settings mode triggered (not yet implemented)"));
}

// ─── Brew sense + shot timer (GPIO1, active low) ───────────
void readBrewSense() {
  bool raw = digitalRead(BREW_SENSE);

  if (lastBrewRaw == HIGH && raw == LOW) {
    brewActive = true;
    brewStartTime = millis();
    shotDone = false;
    pumpOn(180); // start pump immediately on switch close
} else if (lastBrewRaw == LOW && raw == HIGH) {
    brewActive = false;
    shotElapsed = millis() - brewStartTime;
    shotDone = true;
    pumpOff(); // stop pump immediately on switch open
}

  lastBrewRaw = raw;
}

// ─── Startup screen ────────────────────────────────────────
void startupScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.println(F("ESPresso"));
  display.setTextSize(1);
  display.setCursor(28, 38);
  display.println(F("warming up..."));
  display.display();
}

// ─── Main display ──────────────────────────────────────────
void mainDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Mode header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("Mode: %s", STEAM_ON ? "STEAM" : "BREW");

  // Boiler indicator
  display.setCursor(90, 0);
  display.print(BOILER_ON ? "[ON] " : "[OFF]");

  // Divider
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Current temp large
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.printf("%.1fF", CURRENT_TEMP);

  // Setpoint
  display.setTextSize(1);
  display.setCursor(0, 38);
  display.printf("Set:  %.1fF", BOILER_SETPOINT);

  // PID on time
  display.setCursor(0, 48);
  display.printf("OnT:  %lums", (unsigned long)PID_OUTPUT);


  // Shot timer / delta
  display.setCursor(0, 56);
  if (brewActive) {
    unsigned long s = (millis() - brewStartTime) / 1000;
    display.printf("Shot: %lus [>>>]", s);
  } else if (shotDone) {
    display.printf("Shot: %.1fs", shotElapsed / 1000.0);
  } else {
    double delta = BOILER_SETPOINT - CURRENT_TEMP;
    display.printf("Err:  %+.1fF", delta);
  }

  display.display();
}
