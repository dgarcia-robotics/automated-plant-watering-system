/******************************************************************************
  Project : Automated Plant Watering System (Tinkercad/Arduino UNO)
  Author  : David Garcia
  School  : [UAT RBT173 – Final Project]
  Email   : [DAVGARCI@UAT.EDU]
  Date    : [2025-08-24]
  Version : 5.1.0 (Robust IR handling + crisp blue “comet” watering animation)

  Overview
  --------
  1) Monitors a soil moisture sensor (analog) and an ultrasonic sensor (HC-SR04)
    that acts as a tank level check. If moisture < threshold AND tank is OK,
    the system runs a pump (DC motor) for a fixed duration with smooth PWM
    ramp-up/ramp-down.
  2) UI:
      - 16x2 LCD shows mode (AUTO/MAN), moisture %, pump state, threshold,
        raw ADC reading, and tank status (T:OK / T:LO / T:--).
      - NeoPixel ring indicates state:
          Green = moisture bar (more LEDs = wetter soil, AUTO & tank OK)
          Blue  = watering active (rotating “comet”)
          Amber = MANUAL mode (AUTO off)
          Red   = tank LOW (auto watering is inhibited)
      - Piezo buzzer gives short beeps for feedback.
  3) All timing is non-blocking (millis()), so IR/LCD/animation remain responsive.

  IR Remote (NEC) – Buttons & Actions
  -----------------------------------
  POWER : Toggle AUTO -> MANUAL
  PLAY  : Manual pump toggle (on/off immediately)
  UP    : Increase moisture threshold by +5%  (cap at 100)
  DOWN  : Decrease moisture threshold by −5%  (floor at 0)

  How It Works (logic)
  --------------------
  1) Read soil moisture (average of 8 samples) -> convert to % using current
     wet/dry calibration constants in code: calWet (100%) and calDry (0%).
  2) Measure tank distance (HC-SR04). A hysteresis window around 30 cm avoids
     flicker—once LOW, it must go below 29 cm to return to OK, and vice versa.
  3) AUTO mode: if (% moisture < threshold) AND tank OK AND not already pumping,
     start pump for “waterSeconds”. Motor is driven via NPN transistor
     with PWM soft start/stop to reduce current spikes and noise.
  4) NeoPixel is updated at a throttled frame rate so IR decoding stays reliable.

  Quick Wiring Checklist (UNO)
  ----------------------------
  1) Soil moisture AO -> A0   | VCC = 5V, GND = GND
  2) HC-SR04 TRIG -> D13, ECHO -> A1 (used as digital input), VCC = 5V, GND = GND
  3) IR Receiver OUT -> D2, VCC = 5V, GND = GND
  4) NeoPixel DIN -> D6, +5V and GND shared with UNO
  5) Piezo + -> D8, − -> GND
  6) Pump/Motor:
      Motor + -> +5V
      Motor − -> NPN collector
      NPN emitter -> GND
      D9 -> 1 kΩ resistor -> NPN base
      Flyback diode (1N4007) across motor: stripe (cathode) on Motor +, anode on Motor −
  7) LCD 16x2 (HD44780, 4-bit):
      RS=D12, E=D11, D4=D5, D5=D4, D6=D3, D7=D7, RW=GND, VSS=GND, VDD=5V,
      V0 from 10 kΩ pot wiper (ends to 5V/GND), backlight A via 220 Ω to 5V, K to GND
******************************************************************************/

#include <LiquidCrystal.h>

// IMPORTANT (IRremote v4+): define the receive pin BEFORE including IRremote.h
#define DECODE_NEC
#define IR_RECEIVE_PIN 2
#define EXCLUDE_UNIVERSAL_PROTOCOLS
#define EXCLUDE_EXOTIC_PROTOCOLS
#include <IRremote.h>

#include <Adafruit_NeoPixel.h>

// -------- Pins --------
#define PIN_SOIL A0   // Analog input from soil moisture sensor (0..1023)
#define PIN_ECHO A1   // HC-SR04 echo pin (we read it as a digital input)
#define PIN_TRIG 13   // HC-SR04 trigger output
#define PIN_PIEZO 8   // Piezo buzzer (driven by manual toggling)
#define PIN_PUMPPWM 9 // PWM output to NPN base (via ~1k) that drives the motor
#define PIN_NEOPIX 6  // NeoPixel DIN (data in)

// LCD 16x2 (4-bit parallel interface)
#define PIN_LCD_RS 12
#define PIN_LCD_E 11
#define PIN_LCD_D4 5
#define PIN_LCD_D5 4
#define PIN_LCD_D6 3
#define PIN_LCD_D7 7

// -------- NeoPixel setup --------
#define NUM_PIXELS 16
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIX, NEO_GRB + NEO_KHZ800);

// -------- LCD object --------
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// -------- IR code helpers (robust matching across common remotes) --------
// We compare only the lower 24 bits so 0xFFxxxx00 and 0x00FFxxxx both match.
static inline bool match24(uint32_t raw, uint32_t code)
{
    return (raw & 0xFFFFFFUL) == (code & 0xFFFFFFUL);
}

// Known RAW32 (full 32-bit) patterns for POWER and PLAY on remote.
const uint32_t RAW_PWR_32 = 0xFF00BF00UL; // POWER -> toggle AUTO/MAN
const uint32_t RAW_PLY_32 = 0xFF05BF00UL; // PLAY  -> toggle pump immediately

// “Family A/B” 24-bit codes (popular NEC sets).
#define CODE_UP_A 0xFF629DUL
#define CODE_DN_A 0xFFA857UL
#define CODE_UP_B 0xFF986700UL
#define CODE_DN_B 0xFF38C700UL
static inline bool any2(uint32_t raw, uint32_t cA, uint32_t cB) { return match24(raw, cA) || match24(raw, cB); }

// RAW32 byte-order variants
static inline bool isUpRaw32(uint32_t r) { return match24(r, 0xFF629D00UL) || match24(r, 0x00FF629DUL); }
static inline bool isDownRaw32(uint32_t r) { return match24(r, 0xFFA85700UL) || match24(r, 0x00FFA857UL); }

// 8-bit command fallbacks
static inline bool isUp8(uint8_t c) { return (c == 0x0A || c == 0x46 || c == 0x62); }   // includes VOL+ alias
static inline bool isDown8(uint8_t c) { return (c == 0x08 || c == 0x15 || c == 0xA8); } // includes VOL- alias

// -------- Config (user-tunable) --------
// calWet/calDry define how the raw sensor reading maps to % moisture.
// • calWet  = ADC value when soil is fully wet  -> 100%
// • calDry  = ADC value when soil is fully dry  ->   0%
int calWet = 350;
int calDry = 750;
int threshPct = 45;   // If moisture% < threshPct (and tank OK), auto starts watering
int waterSeconds = 6; // Pump run time per watering cycle (seconds)

// Tank status thresholds (hysteresis avoids flicker near boundary)
const int TANK_OK_MAX_CM = 30; // <=30 cm from sensor means water present/OK
const int TANK_HYST_CM = 1;    // 1 cm band to prevent rapid toggling

// -------- Runtime state --------
bool autoEnabled = true;     // AUTO (true) vs MANUAL (false)
bool pumping = false;        // Is the pump currently running?
unsigned long pumpEndMs = 0; // When to stop the current pump cycle (millis)

bool tankLowState = false; // false=OK, true=LOW
long lastTankCm = -1;      // Last measured distance (cm), -1 = timeout/no echo

// For showing the last key pressed on the LCD bottom-right for ~1.2s
char lastKey[4] = "   ";
unsigned long lastKeyStamp = 0;

// NeoPixel frame limiter so IR stays reliable (avoid spamming .show() too fast)
unsigned long tPix = 0;
const unsigned long PIX_PERIOD_MS = 90;

// ---------- Beeper (no tone()) ---------------------------------------------
void beep(unsigned int onMs = 40, byte times = 1, unsigned int gapMs = 60)
{
    pinMode(PIN_PIEZO, OUTPUT);
    for (byte t = 0; t < times; t++)
    {
        unsigned long start = millis();
        while (millis() - start < onMs)
        {
            digitalWrite(PIN_PIEZO, HIGH);
            delayMicroseconds(250);
            digitalWrite(PIN_PIEZO, LOW);
            delayMicroseconds(250);
        }
        delay(gapMs);
    }
}

// ---------- Moisture reading & scaling -------------------------------------
int readMoisturePct()
{
    long sum = 0;
    for (int i = 0; i < 8; i++)
    {
        sum += analogRead(PIN_SOIL);
        delay(2);
    }
    int raw = sum / 8;
    long pct = map(raw, calDry, calWet, 0, 100); // raw=calDry -> 0%, raw=calWet -> 100%
    if (pct < 0)
        pct = 0;
    if (pct > 100)
        pct = 100;
    return (int)pct;
}

// ---------- HC-SR04 distance (tank level) ----------------------------------
long pingCM()
{
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long us = pulseIn(PIN_ECHO, HIGH, 30000UL); // 30 ms timeout
    if (us == 0)
        return -1;  // no echo
    return us / 58; // µs → cm
}

// Update tankLowState with 1 cm hysteresis around 30 cm
void updateTankState()
{
    long cm = pingCM();
    lastTankCm = cm; // store latest (or -1 on timeout)
    if (cm < 0)
        return; // keep previous LOW/OK if sensor timed out

    if (!tankLowState)
    {
        if (cm > TANK_OK_MAX_CM + TANK_HYST_CM)
            tankLowState = true; // go to LOW
    }
    else
    {
        if (cm < TANK_OK_MAX_CM - TANK_HYST_CM)
            tankLowState = false; // back to OK
    }
}

// ---------- Pump control (soft start/stop PWM ramp) ------------------------
void pumpStart(int seconds)
{
    // INTERLOCK: never start if tank is LOW
    if (pumping || tankLowState)
        return;

    for (int p = 0; p <= 255; p += 12)
    {
        analogWrite(PIN_PUMPPWM, p);
        delay(15);
    }
    pumping = true;
    pumpEndMs = millis() + (unsigned long)seconds * 1000UL;
}

void pumpStop()
{
    for (int p = 255; p >= 0; p -= 18)
    {
        analogWrite(PIN_PUMPPWM, p);
        delay(10);
    }
    analogWrite(PIN_PUMPPWM, 0);
    pumping = false;
}

// ---------- NeoPixel (idle bar / warnings / comet during pump) -------------
void showIdleBar(int moisturePct)
{
    pixels.clear();
    int lit = map(moisturePct, 0, 100, 0, NUM_PIXELS);
    for (int i = 0; i < lit; i++)
        pixels.setPixelColor(i, pixels.Color(0, 150, 0)); // green bar
    pixels.show();
}

void showAmber()
{
    pixels.fill(pixels.Color(120, 60, 0));
    pixels.show();
} // Manual mode
void showRed()
{
    pixels.fill(pixels.Color(150, 0, 0));
    pixels.show();
} // Tank LOW

// “Comet” watering animation: bright head with fading tail, rotates around ring
void showPumpAnimComet()
{
    static uint8_t head = 0;
    const uint8_t TRAIL_LEN = 4;
    const uint8_t tail[TRAIL_LEN] = {180, 70, 25, 6}; // brightness levels

    pixels.clear();
    for (uint8_t k = 0; k < TRAIL_LEN; k++)
    {
        int idx = (head + NUM_PIXELS - k) % NUM_PIXELS;
        pixels.setPixelColor(idx, pixels.Color(0, 0, tail[k])); // blue shades
    }
    pixels.show();
    head = (head + 1) % NUM_PIXELS;
}

// Frame-rate limiter so NeoPixel updates don’t starve IR decoding
void showStatusThrottled(int moisturePct)
{
    if (millis() - tPix < PIX_PERIOD_MS)
        return;
    tPix = millis();

    if (pumping)
        showPumpAnimComet();
    else if (tankLowState)
        showRed();
    else if (!autoEnabled)
        showAmber();
    else
        showIdleBar(moisturePct);
}

// ---------- LCD helpers -----------------------------------------------------
void lcdPrint16(uint8_t row, const char *text)
{
    lcd.setCursor(0, row);
    lcd.print(text);
    for (int i = strlen(text); i < 16; i++)
        lcd.print(' ');
}

// Update both LCD lines
void updateLCD(int moisturePct)
{
    // Line 1: mode + % + pump
    char line1[17];
    snprintf(line1, sizeof(line1), "%s %3d%% P:%s",
             autoEnabled ? "AUTO" : "MAN ",
             moisturePct,
             pumping ? "on " : "off");
    lcdPrint16(0, line1);

    // Line 2: left = "THxx Rxxxx", right = "K:XXX" (1.2s) else T:OK/LO/--
    int raw = analogRead(PIN_SOIL);

    lcd.setCursor(0, 1);
    char left[13];
    snprintf(left, sizeof(left), "TH%02d R%4d", threshPct, raw);
    lcd.print(left);
    for (int i = strlen(left); i < 12; i++)
        lcd.print(' ');

    lcd.setCursor(12, 1);
    if (millis() - lastKeyStamp < 1200)
    {
        lcd.print("K:");
        lcd.print(lastKey[0]);
        lcd.print(lastKey[1]);
        lcd.print(lastKey[2]);
    }
    else
    {
        if (lastTankCm < 0)
            lcd.print("T:--"); // no echo / timeout
        else if (tankLowState)
            lcd.print("T:LO"); // tank low
        else
            lcd.print("T:OK"); // tank OK
    }
}

// ---------- “Remember last key” helper for LCD overlay ----------------------
void rememberKey(char a, char b, char c)
{
    lastKey[0] = a;
    lastKey[1] = b;
    lastKey[2] = c;
    lastKey[3] = '\0';
    lastKeyStamp = millis();
}

// ---------- IR Handler (decode once, ignore repeats, then act) --------------
void handleIR()
{
    if (!IrReceiver.decode())
        return;

    uint32_t raw = IrReceiver.decodedIRData.decodedRawData; // full raw code
    bool rep = (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT);
    uint8_t cmd = IrReceiver.decodedIRData.command; // 8-bit fallback

    if (!rep)
    {
        if (match24(raw, RAW_PWR_32))
        {
            autoEnabled = !autoEnabled;
            beep();
            rememberKey('P', 'W', 'R');
        }
        else if (match24(raw, RAW_PLY_32))
        {
            if (pumping)
            {
                pumpStop(); // manual OFF always allowed
            }
            else if (!tankLowState)
            {
                pumpStart(waterSeconds); // manual ON only if tank OK
            }
            // if tankLowState is true, ignore start request
            beep();
            rememberKey('P', 'L', 'Y');
        }
        else if (isUpRaw32(raw) || isUp8(cmd) || any2(raw, CODE_UP_A, CODE_UP_B))
        {
            threshPct += 5;
            if (threshPct > 100)
                threshPct = 100;
            beep();
            rememberKey('U', 'P', ' ');
        }
        else if (isDownRaw32(raw) || isDown8(cmd) || any2(raw, CODE_DN_A, CODE_DN_B))
        {
            threshPct -= 5;
            if (threshPct < 0)
                threshPct = 0;
            beep();
            rememberKey('D', 'N', ' ');
        }
    }
    IrReceiver.resume(); // ready for the next IR frame
}

// ---------- Setup (runs once at power-up) -----------------------------------
void setup()
{
    // Basic pin modes
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_PUMPPWM, OUTPUT);
    analogWrite(PIN_PUMPPWM, 0); // ensure pump off
    pinMode(PIN_PIEZO, OUTPUT);

    // Start LCD and NeoPixel
    lcd.begin(16, 2);
    pixels.begin();
    pixels.clear();
    pixels.show();

    // Start IRreceiver (v4+) with LED feedback disabled to reduce timing jitter
    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

    // Quick audible/visual self-test
    beep(80, 2, 80);
    pixels.fill(pixels.Color(100, 0, 0));
    pixels.show();
    delay(150);
    pixels.fill(pixels.Color(0, 100, 0));
    pixels.show();
    delay(150);
    pixels.fill(pixels.Color(0, 0, 100));
    pixels.show();
    delay(150);
    pixels.clear();
    pixels.show();

    // Boot banner
    lcdPrint16(0, "PlantAuto SIMPLE");
    lcdPrint16(1, "Init... OK");
    delay(400);
}

// ---------- Main loop (non-blocking timing) ---------------------------------
void loop()
{
    static unsigned long tMoist = 0, tLCD = 0, tAuto = 0;
    static int moistPct = 0;

    // Always listen for IR input; returns immediately if no frame arrived
    handleIR();

    // Sample moisture ~4 times a second (averaged inside function)
    if (millis() - tMoist > 250)
    {
        moistPct = readMoisturePct();
        tMoist = millis();
    }

    // Once per second: tank check, auto-watering, and pump timeout
    if (millis() - tAuto > 1000)
    {
        tAuto = millis();

        updateTankState(); // refresh tank OK/LOW

        // INTERLOCK: if tank is LOW, stop the pump immediately
        if (tankLowState && pumping)
        {
            pumpStop();
        }

        if (pumping && millis() >= pumpEndMs)
            pumpStop(); // time-based stop

        // Auto water if below threshold AND tank is OK AND not already pumping
        if (autoEnabled && !pumping)
        {
            if ((moistPct < threshPct) && !tankLowState)
                pumpStart(waterSeconds);
        }
    }

    // NeoPixel state view with frame limiting (helps IR reliability)
    showStatusThrottled(moistPct);

    // Update LCD about twice a second
    if (millis() - tLCD > 500)
    {
        tLCD = millis();
        updateLCD(moistPct);
    }
}
