#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <DHT.h>
#include "DFRobot_PH.h"
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>
#include <ESP_Mail_Client.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// ── 4.0" ST7796S SPI TFT display, display-only (local status panel) ──
// Uses LovyanGFX for the driver. All config (driver, pins) is a
// plain C++ class defined right here in the sketch (see below) — no
// library folder editing needed. Touch was dropped entirely; the
// panel is navigated with two physical FORWARD/BACK push-buttons
// instead (see BTN_FORWARD_PIN / BTN_BACK_PIN further down).
//
// Install "LovyanGFX" (by lovyan03) via Library Manager. That's it.
#include <SPI.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/panel/Panel_ST7796.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796  _panel_instance;
  lgfx::Bus_SPI       _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host   = HSPI_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 27000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 22;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 15;
      cfg.pin_rst          = 17;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 320;
      cfg.panel_height     = 480;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.readable         = true;
      cfg.invert           = false;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

// Standard RGB565 colors (same values TFT_eSPI uses) — defined
// directly so the drawing code below doesn't depend on any optional
// compatibility header.
#define TFT_BLACK      0x0000
#define TFT_WHITE      0xFFFF
#define TFT_RED        0xF800
#define TFT_GREEN      0x07E0
#define TFT_BLUE       0x001F
#define TFT_CYAN       0x07FF
#define TFT_YELLOW     0xFFE0
#define TFT_ORANGE     0xFD20
#define TFT_DARKGREY   0x7BEF
#define TFT_LIGHTGREY  0xC618

/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║       AWG-HEROES IoT DASHBOARD — ESP32 Web Server v9.1    ║
 * ║  Sensors: pH · TDS · DHT22 · 4x Float Switch (2 tanks)   ║
 * ║  Control: 4-Relay Panel · MANUAL or AUTOMATIC mode ·     ║
 * ║           Master STOP ALL · Email + SMS Alerts ·         ║
 * ║           4.0" ST7796S Status Panel (display-only,       ║
 * ║           landscape, 2-button navigation)                ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * FIX IN v9.1 — Manual-mode Fill Pump could never be held ON while
 * the raw (10L) tank's LOW float was triggered. enforceFillPumpCutoff()
 * was force-cutting relay 3 on EVERY loop pass whenever
 * (g_treatedTankFull || g_rawTankLow) was true, in BOTH modes — but
 * the /relay endpoint's manual-block checks never blocked on
 * g_rawTankLow, so a manual ON request would appear to succeed for
 * one loop iteration and then get silently snapped back OFF. The
 * raw-tank-low dry-run cutoff is already handled correctly inside
 * runAutomaticControl() for Automatic mode, so enforceFillPumpCutoff()
 * now only force-corrects the overflow case (treated tank full),
 * leaving Manual mode free to run the pump with the raw tank low
 * (e.g. for priming/testing). See enforceFillPumpCutoff() below.
 *
 * NEW IN v2.0 — dropped the touch panel entirely (resistive touch on
 * this module kept fighting us) in favor of a display-only status
 * panel navigated by two physical push-buttons: FORWARD (GPIO23) and
 * BACK (GPIO16, freed up now that touch's T_CS pin isn't needed).
 * Mode/relay/faucet control stays on the web dashboard — this panel
 * just shows status. 8 screens, cycling forward/backward:
 *   1. SYSTEM STATUS       — mode, stage, water status, WiFi, uptime
 *   2. FLOAT SWITCHES/TANKS— all 4 float switches + tank summaries
 *   3. LIVE READINGS       — pH, TDS, temp, humidity, dew point
 *   4. RELAY STATUS        — all 4 relays, read-only
 *   5. WATER YIELD         — humidity yield tier + condensation gap
 *   6. TRENDS: pH & TDS    — last 24h line charts
 *   7. TRENDS: Temp/Hum/Dew— last 24h line charts
 *   8. ALERT LOGS          — last 8 alerts (new firmware-side ring
 *                            buffer, fed by logAlert() alongside
 *                            every sendAlertEmail() call)
 * Wiring for the buttons: one leg to the GPIO, the other leg to GND
 * — internal pull-ups are enabled in code, so no external resistor
 * needed. Button pressed = LOW = triggers the page change.
 *
 * NEW IN v1.9 — switched to LovyanGFX. TFT_eSPI required editing a
 * separate User_Setup.h file inside the library folder (its touch
 * functions are compiled in a .cpp that can't see sketch #defines),
 * which caused repeated "undefined reference" build errors. Every
 * bit of config — driver, pins — was a plain C++ class defined
 * right in the sketch (see the LGFX class near the top).
 *
 * NEW IN v1.8 — corrected driver: your panel's own listing confirms
 * ST7796S (not ILI9488 — both are 480x320 so the mixup was an easy
 * one, but the two chips have different init sequences, which is
 * why the screen looked wrong before).
 *
 * WHAT CHANGED FROM v3:
 *   - Each tank now has TWO float switches instead of one:
 *       - Raw/Unfiltered tank (10L): HIGH sensor (full) + LOW
 *         sensor (empty)
 *       - Treated/Filtered tank (20L): HIGH sensor (full) + LOW
 *         sensor (empty)
 *     Per your wiring notes: both "up/full" switches are
 *     ACTIVE-HIGH, both "down/empty" switches are ACTIVE-LOW.
 *     If your actual wiring differs, just flip the *_ACTIVE
 *     defines below — nothing else needs to change.
 *   - Added SYSTEM MODE: MANUAL or AUTOMATIC, selectable from
 *     the dashboard.
 *       MANUAL  — you control all 4 relays yourself via the
 *                 toggle switches, exactly like before.
 *       AUTOMATIC — runs a full brew cycle on its own:
 *         1) Compressor/Dehumidifier ON until the raw tank
 *            (10L) reads FULL → sends an SMS notification.
 *            If the treated tank is already full at this point,
 *            skips straight to step 3 instead of starting the pump.
 *         2) Fill/Main Pump + UV Sterilizer ON (compressor off)
 *            until the raw tank reads EMPTY (low sensor) → loops
 *            straight back to step 1 for the next batch.
 *         3) PARKED — triggered if the treated tank (20L) reads
 *            FULL (either from step 1 or mid-transfer in step 2).
 *            Everything OFF. Only resumes the pump once the
 *            treated tank's LOW sensor fires (i.e. someone drew
 *            water from the faucet and made room again).
 *       Safety: if the treated tank fills up mid-transfer
 *       (before the raw tank empties), the pump + UV are cut
 *       immediately either way, in both modes.
 *   - Added a master "STOP ALL" control — instantly kills all
 *     4 relays and drops the system back to Manual mode. Works
 *     regardless of which mode you were in.
 *   - Added a "no water detected" safety guard: if pH AND TDS
 *     both read ~0 for a full 30 seconds straight (sensors
 *     likely dry / not actually in water), the Compressor, UV,
 *     and Fill Pump (relays 1-3) are held off / refuse to turn
 *     on — no point running that hardware blind. Once valid
 *     readings return, it resumes normally. The Faucet Pump
 *     (relay 4) is not affected by this guard.
 *   - Added SMS alerts (via the Semaphore SMS API over HTTPS —
 *     no GSM module needed since the ESP32 already has WiFi).
 *     Fill in your API key below.
 *
 * LIBRARIES REQUIRED (install via Library Manager):
 *   - ESPAsyncWebServer  (ESP32Async fork, matches AsyncTCP below)
 *   - AsyncTCP           (ESP32Async fork)
 *   - DFRobot_PH         (by DFRobot)
 *   - DHT sensor library (by Adafruit)
 *   - ArduinoJson        (by Benoit Blanchon)
 *   - ESP Mail Client    (by Mobizt) — for Gmail SMTP alert emails
 *
 * PIN ASSIGNMENTS:
 *   GPIO 34  → pH sensor analog
 *   GPIO 35  → TDS sensor analog
 *   GPIO 21  → DHT22 data
 *   GPIO 27  → Float switch — Raw tank (10L)      FULL  (active-HIGH)
 *   GPIO 33  → Float switch — Raw tank (10L)      EMPTY (active-LOW)
 *   GPIO 32  → Float switch — Treated tank (20L)  FULL  (active-HIGH)
 *   GPIO  4  → Float switch — Treated tank (20L)  EMPTY (active-LOW)
 *   GPIO 25  → Relay 1: Compressor / Dehumidifier
 *   GPIO 26  → Relay 2: UV Sterilizer
 *   GPIO  5  → Relay 3: Fill/Main Pump (auto-cutoff when 20L tank full)
 *   GPIO 18  → Relay 4: Faucet Pump (dispense — always manual)
 *
 * NOTE: Hourly history is kept in RAM only (resets on reboot).
 *       No SD card yet — add one later for persistent logging
 *       without changing the rest of this structure.
 */

// ── WiFi Credentials ────────────────────────────────────────────
const char* WIFI_SSID     = "BAWAL CONNECT 4G";
const char* WIFI_PASSWORD = "kenhiroseJR2025";

// ── Supabase Cloud ──────────────────────────────────────────────
// IMPORTANT: Use the Supabase PUBLISHABLE key here.
// NEVER put the sb_secret_... key in the ESP32 firmware.
#define SUPABASE_URL "https://pdzshhfbrlemcqfufqps.supabase.co"
#define SUPABASE_PUBLISHABLE_KEY "sb_publishable_cOK-3oZGZKyqoCmyWIjviA_G9LEY3RQ"

// Upload one sensor/status record every 10 seconds.
#define SUPABASE_UPLOAD_INTERVAL 10000UL

// ── Email Alerts (Gmail SMTP) ────────────────────────────────────
// Fill in SMTP_PASS yourself on your own machine — this is a Gmail
// "App Password" (16 chars, NOT your normal Gmail password).
// Generate one at: myaccount.google.com > Security > App Passwords
// (requires 2-Step Verification enabled on the account first)
#define SMTP_HOST        "smtp.gmail.com"
#define SMTP_PORT        465
#define SMTP_SENDER      "andradamj2@gmail.com"   // the Gmail account that SENDS the alert
#define SMTP_PASS        "fjugfxhnjfwqijey"
#define SMTP_RECIPIENT   "mjandrada111@gmail.com"   // who RECEIVES the alert (can be same or different)

SMTPSession smtp;

// Cooldown so the same alert type doesn't spam your inbox every 2s
#define EMAIL_COOLDOWN_MS  300000UL   // 5 minutes between repeat alerts of same type
unsigned long lastEmailPH           = 0;
unsigned long lastEmailTDS          = 0;
unsigned long lastEmailRawFull      = 0;
unsigned long lastEmailTreatedFull  = 0;
unsigned long lastEmailPotability   = 0;

// ── SMS Alerts (Semaphore SMS Gateway — api.semaphore.co) ────────
// Sign up at semaphore.co, load some SMS credits, and paste your
// API key below. This posts over HTTPS, so no SIM800L/GSM module
// is needed — the ESP32's existing WiFi handles it.
#define SMS_API_KEY           "YOUR_SEMAPHORE_API_KEY"
#define SMS_RECIPIENT_NUMBER  "09171234567"   // PH mobile number
#define SMS_SENDER_NAME       "AWGHEROES"     // must match an approved Sender Name, or omit

#define SMS_COOLDOWN_MS  60000UL   // 1 minute between repeat SMS of same type
unsigned long lastSmsRawFull     = 0;
unsigned long lastSmsTreatedFull = 0;

// ── NTP ─────────────────────────────────────────────────────────
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET    = 28800;   // UTC+8 Philippines
const int   DST_OFFSET    = 0;

// ── Pin Definitions ─────────────────────────────────────────────
#define PH_PIN               34
#define TDS_PIN              35
#define DHTPIN               21

#define FLOAT_RAW_HIGH_PIN       27   // Raw tank (10L)     — FULL
#define FLOAT_RAW_LOW_PIN        33   // Raw tank (10L)     — EMPTY
#define FLOAT_TREATED_HIGH_PIN   32   // Treated tank (20L) — FULL
#define FLOAT_TREATED_LOW_PIN     4   // Treated tank (20L) — EMPTY

#define RELAY_1_PIN 25   // Compressor / Dehumidifier
#define RELAY_2_PIN 26   // UV Sterilizer
#define RELAY_3_PIN 5    // Fill/Main Pump (auto-cutoff when treated tank full)
#define RELAY_4_PIN 18   // Faucet Pump (dispense to user)

// ── TFT backlight pin ──────────────────────────────────────────
// Per our chat: wire the module's LED pin straight to 3.3V/5V for
// full brightness (a GPIO can't source enough current for it).
// If you'd rather keep it code-controllable, wire it through a
// transistor switch and uncomment TFT_BL below + the LOAD in setup.
// #define TFT_BL_PIN 21   // example only if you add a transistor switch

// Set to true if your relay module is ACTIVE-LOW (most cheap modules are)
#define RELAY_ACTIVE_LOW true

// Per-sensor active states — both "full" switches read active-HIGH,
// both "empty" switches read active-LOW (per your wiring). Flip
// individually here if your actual hardware differs.
#define FLOAT_RAW_HIGH_ACTIVE       HIGH
#define FLOAT_RAW_LOW_ACTIVE        LOW
#define FLOAT_TREATED_HIGH_ACTIVE   HIGH
#define FLOAT_TREATED_LOW_ACTIVE    LOW
const unsigned long FLOAT_DEBOUNCE_MS = 200;

// "No water detected" safety guard — if pH AND TDS both read
// essentially 0 for this long straight, assume the sensors are dry
// and hold off powering the Compressor/UV/Fill Pump.
const unsigned long SENSOR_CONFIRM_MS = 60000UL;   // 1 minute

// ── Sensor Config ───────────────────────────────────────────────
#define DHTTYPE         DHT22
#define PH_OFFSET       3.64f
#define TDS_OFFSET      24.0f
#define VREF            3.3f
#define SCOUNT          30

// ── History buffer sizes ────────────────────────────────────────
#define MINUTE_POINTS   30   // last 30 readings (~1 min apart in UI sampling)
#define HOUR_POINTS     24   // last 24 hourly readings
#define TREND_POINTS    60   // last 60 one-minute readings (LCD trend screens)

// ── Sensor Objects ──────────────────────────────────────────────
DHT dht(DHTPIN, DHTTYPE);
DFRobot_PH ph;

// ── Sensor Values ───────────────────────────────────────────────
float g_ph          = 0.0f;
float g_tds         = 0.0f;
float g_humidity    = 0.0f;
float g_temperature = 25.0f;
float g_dewPoint    = 0.0f;
float g_condGap     = 0.0f;   // temperature - dew point (smaller = better condensation)
bool  g_rawTankFull      = false;  // Raw tank (10L)     — HIGH sensor (FLOAT 2)
bool  g_rawTankLow       = false;  // Raw tank (10L)     — LOW sensor  (FLOAT 1) (true = empty)
bool  g_treatedTankFull  = false;  // Treated tank (20L) — HIGH sensor (FLOAT 3)
bool  g_treatedTankLow   = false;  // Treated tank (20L) — LOW sensor  (FLOAT 4) (true = empty)

// ── TDS Buffer ──────────────────────────────────────────────────
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

// ── Float Switch Debounce State (one struct per sensor) ──────────
struct FloatSensor {
  uint8_t pin;
  int     activeState;
  bool    debounced;
  bool    lastRaw;
  unsigned long lastChangeTime;
};

FloatSensor fsRawHigh     = { FLOAT_RAW_HIGH_PIN,     FLOAT_RAW_HIGH_ACTIVE,     false, false, 0 };
FloatSensor fsRawLow      = { FLOAT_RAW_LOW_PIN,      FLOAT_RAW_LOW_ACTIVE,      false, false, 0 };
FloatSensor fsTreatedHigh = { FLOAT_TREATED_HIGH_PIN, FLOAT_TREATED_HIGH_ACTIVE, false, false, 0 };
FloatSensor fsTreatedLow  = { FLOAT_TREATED_LOW_PIN,  FLOAT_TREATED_LOW_ACTIVE,  false, false, 0 };

// "No water detected" guard state (pH + TDS both ~0)
bool g_noWaterDetected      = false;  // debounced result — true = hold power off
bool g_noWaterRaw           = false;
unsigned long g_noWaterChangeTime = 0;

// ── Relay State (1=Compressor, 2=UV, 3=Fill Pump, 4=Faucet Pump) ─
bool relayState[4] = { false, false, false, false };
const char* relayNames[4] = { "Compressor", "UV Sterilizer", "Fill Pump", "Faucet Pump" };

// ── System Mode + Automatic Cycle State ──────────────────────────
// NOTE: The automatic cycle is now LEVEL-BASED (continuous conditions
// checked every loop) instead of a step-by-step state machine. See
// runAutomaticControl() for the full explanation. g_pumpArmed is the
// only piece of memory it needs.
enum SystemMode { MODE_MANUAL, MODE_AUTOMATIC };
SystemMode g_systemMode = MODE_MANUAL;

// ── TFT object (local on-device panel, display-only) ──────────────
LGFX tft;

// ── Physical FORWARD / BACK / SELECT push-buttons ──────────────────
// FORWARD/BACK: wired button-to-GND, internal pull-ups (active LOW).
// SELECT: single press = move to next controllable item; a second
// quick press within SELECT_DOUBLE_TAP_MS = toggle that item on/off.
// SELECT is wired the same way as FORWARD/BACK — one leg to GPIO2,
// the other to GND. GPIO2 has internal pull-up support like the
// other two, and since this wiring pulls it LOW when pressed, that's
// actually the safe boot state for GPIO2 — no boot-mode risk at all.
#define BTN_FORWARD_PIN 23
#define BTN_BACK_PIN    16
#define BTN_SELECT_PIN  2
#define SELECT_DOUBLE_TAP_MS 350

#define SELECT_ITEMS_COUNT 5
const char* SELECT_ITEM_NAMES[SELECT_ITEMS_COUNT] = {
  "MODE", "COMPRESSOR", "FILL PUMP", "UV STERILIZER", "FAUCET PUMP"
};
// Maps a SELECT cursor position to a relay id (1-4); -1 = not a relay (it's MODE).
int SELECT_RELAY_ID[SELECT_ITEMS_COUNT] = { -1, 1, 3, 2, 4 };
int g_selectIndex = 0;

// Simple rectangle helper — used for drawing labeled status boxes
// (no longer touch hit-boxes now that the panel is display-only).
struct Rect { int16_t x, y, w, h; };

enum PanelScreen {
  SCR_HOME = 0,       // System status + system mode
  SCR_FLOATS,         // Float switch status + tank status
  SCR_LIVE,           // Live sensor readings
  SCR_RELAYS,         // Relay control panel condition (read-only)
  SCR_YIELD,          // Water yield potential
  SCR_TREND_PHTDS,    // Trends: pH & TDS
  SCR_TREND_TEMPHUM,  // Trends: Temp & Humidity + Dew Point
  SCR_ALERTS,         // Alert logs
  SCR_COUNT
};
PanelScreen g_panelScreen = SCR_HOME;

unsigned long g_lastPanelRefresh = 0;

// ── Firmware-side alert log (for the on-panel Alert Logs screen) ──
// The web dashboard's alert history lives only in the browser's own
// JS — this is a small ring buffer so the LCD panel has something
// to show too. Fed by logAlert() alongside each sendAlertEmail() call.
#define ALERT_LOG_SIZE 8
struct AlertLogEntry { String time; String msg; };
AlertLogEntry g_alertLog[ALERT_LOG_SIZE];
int g_alertLogCount = 0;
int g_alertLogIndex = 0;



// Becomes true the first time the raw tank (10L) reads FULL (FLOAT 2)
// after entering Automatic mode, and then stays true — this "arms"
// the Fill Pump so it's allowed to run. It intentionally does NOT
// reset just because the raw tank drains again (FLOAT 1), since the
// Compressor and Fill Pump are allowed to run at the same time.
bool g_pumpArmed = false;

// ── Alerts ──────────────────────────────────────────────────────
bool alertPH  = false;
bool alertTDS = false;

// ── Forward Declarations ──────────────────────────────────────────
// The Arduino IDE auto-generates prototypes for functions defined
// below their first use, but it can miss ones with reference
// parameters (like trySendSMS below) — declaring them explicitly
// here avoids "was not declared in this scope" build errors.
bool   sendSMS(const String& message);
String urlEncode(const String& s);
void   trySendSMS(unsigned long &lastTime, const String& message);
void   logAlert(const String& msg);
void   drawStatusDots();

// ── Server & WebSocket ──────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── Uptime ──────────────────────────────────────────────────────
unsigned long bootMillis = 0;

// ── Rolling history (RAM only) ───────────────────────────────────
struct HistoryPoint {
  float ph, tds, temp, hum, dewPoint;
  char  timeLabel[6];   // "HH:MM", filled from NTP-synced time when recorded
};

HistoryPoint minuteHistory[MINUTE_POINTS];
int minuteHistCount = 0;
int minuteHistIndex = 0;

HistoryPoint hourHistory[HOUR_POINTS];
int hourHistCount = 0;
int hourHistIndex = 0;

// True 1-point-per-minute buffer, last 60 minutes — this is what the
// LCD's trend screens read from (the hourly buffer above only fills
// in once an hour, so it looked empty on the LCD for a long time).
HistoryPoint trendHistory[TREND_POINTS];
int trendHistCount = 0;
int trendHistIndex = 0;

unsigned long lastMinuteSample = 0;
unsigned long lastHourSample   = 0;

// ════════════════════════════════════════════════════════════════
//  HTML PAGE (stored in PROGMEM to save RAM)
// ════════════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AWG-HEROES Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg:       #0b0f14;
    --surface:  #131922;
    --surface2: #1b232f;
    --border:   #232d3a;
    --text:     #e8edf3;
    --muted:    #7e8ba0;
    --blue:     #4d8af0;
    --teal:     #18c3ad;
    --green:    #2ecf71;
    --amber:    #f5a623;
    --red:      #ef4f5f;
    --purple:   #b07bf0;
    --cyan:     #29c5e8;
    --radius:   14px;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: radial-gradient(circle at top right, #101723 0%, var(--bg) 60%);
    color: var(--text);
    font-family: 'Inter', 'Segoe UI', system-ui, sans-serif;
    font-size: 14px;
    min-height: 100vh;
  }

  header {
    background: linear-gradient(180deg, var(--surface), #10151d);
    border-bottom: 1px solid var(--border);
    padding: 16px 28px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    position: sticky;
    top: 0;
    z-index: 100;
    backdrop-filter: blur(6px);
  }
  .logo { display: flex; align-items: center; gap: 12px; }
  .logo-icon {
    width: 40px; height: 40px; border-radius: 10px;
    background: linear-gradient(135deg, var(--teal), var(--blue));
    display: flex; align-items: center; justify-content: center;
    font-size: 19px;
    box-shadow: 0 4px 14px -4px #18c3ad80;
  }
  .logo-text { font-size: 18px; font-weight: 700; letter-spacing: -0.3px; }
  .logo-sub  { font-size: 11px; color: var(--muted); margin-top: 1px; }
  .header-right { display: flex; align-items: center; gap: 16px; }
  .wifi-badge {
    background: #18c3ad1a;
    border: 1px solid #18c3ad40;
    color: var(--teal);
    padding: 5px 12px;
    border-radius: 20px;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.3px;
  }
  #clock { font-size: 13px; color: var(--muted); font-variant-numeric: tabular-nums; }

  #alertBar {
    display: none;
    background: #ef4f5f14;
    border-bottom: 1px solid #ef4f5f40;
    padding: 11px 28px;
    color: var(--red);
    font-size: 13px;
    font-weight: 600;
    gap: 14px;
    flex-wrap: wrap;
  }
  #alertBar.show { display: flex; align-items: center; }

  main { padding: 22px 28px; max-width: 1320px; margin: 0 auto; }

  .section-label {
    font-size: 11px;
    color: var(--muted);
    letter-spacing: 1.5px;
    text-transform: uppercase;
    font-weight: 700;
    margin: 28px 0 12px;
  }
  .section-label:first-child { margin-top: 0; }

  .status-row {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    gap: 12px;
    margin-bottom: 8px;
  }
  .stat-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 16px 18px;
  }
  .stat-label { font-size: 10px; color: var(--muted); letter-spacing: 1px; text-transform: uppercase; font-weight: 600; }
  .stat-val   { font-size: 22px; font-weight: 700; margin-top: 5px; font-variant-numeric: tabular-nums; }

  .sensors-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
    gap: 14px;
  }
  .sensor-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
    transition: border-color 0.2s, transform 0.15s;
  }
  .sensor-card:hover { transform: translateY(-2px); }
  .sensor-card.alert { border-color: var(--red) !important; }
  .sensor-top { display: flex; align-items: center; justify-content: space-between; margin-bottom: 14px; }
  .sensor-icon {
    width: 40px; height: 40px; border-radius: 11px;
    display: flex; align-items: center; justify-content: center;
    font-size: 19px;
  }
  .sensor-name  { font-size: 11px; color: var(--muted); letter-spacing: 1px; text-transform: uppercase; margin-bottom: 3px; font-weight: 600; }
  .sensor-value { font-size: 30px; font-weight: 800; font-variant-numeric: tabular-nums; line-height: 1; }
  .sensor-unit  { font-size: 13px; font-weight: 500; color: var(--muted); margin-left: 3px; }
  .sensor-status {
    margin-top: 11px;
    font-size: 11px;
    font-weight: 700;
    padding: 4px 10px;
    border-radius: 20px;
    display: inline-block;
  }
  .s-ok       { background: #2ecf7120; color: var(--green); }
  .s-warn     { background: #f5a62320; color: var(--amber); }
  .s-danger   { background: #ef4f5f20; color: var(--red);   }
  .s-info     { background: #4d8af020; color: var(--blue);  }
  .sensor-hint { font-size: 10px; color: var(--muted); margin-top: 8px; }

  /* ── System status dots ── */
  .status-dots-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 16px 20px;
    display: flex;
    flex-wrap: wrap;
    gap: 20px;
    margin-bottom: 20px;
  }
  .status-dot-item { display: flex; align-items: center; gap: 8px; }
  .status-dot {
    width: 14px; height: 14px; border-radius: 50%;
    background: #3a3f4b;
    box-shadow: none;
    transition: background .2s, box-shadow .2s;
    display: inline-block;
  }
  .status-dot.on-green  { background: var(--teal, #18c3ad); box-shadow: 0 0 8px var(--teal, #18c3ad); }
  .status-dot.on-yellow { background: #f5c518; box-shadow: 0 0 8px #f5c518; }
  .status-dot.on-blue   { background: var(--blue, #4d8af0); box-shadow: 0 0 8px var(--blue, #4d8af0); }
  .status-dot.on-red    { background: var(--red, #ef4f5f); box-shadow: 0 0 8px var(--red, #ef4f5f); }
  .status-dot-label { font-size: 12px; color: var(--muted); }

  /* ── System Mode panel ── */
  .mode-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 16px;
  }
  .mode-buttons { display: flex; gap: 8px; }
  .mode-btn {
    background: var(--surface2);
    border: 1px solid var(--border);
    color: var(--muted);
    font-size: 13px;
    font-weight: 700;
    padding: 10px 18px;
    border-radius: 10px;
    cursor: pointer;
    transition: all 0.2s;
  }
  .mode-btn.active { background: var(--blue); color: #fff; border-color: var(--blue); }
  .mode-btn#modeBtnAuto.active { background: var(--teal); border-color: var(--teal); }
  .mode-status { display: flex; align-items: center; gap: 10px; flex: 1; justify-content: center; }
  .mode-badge {
    font-size: 12px; font-weight: 800; letter-spacing: 0.5px;
    padding: 6px 14px; border-radius: 20px;
  }
  .mode-badge.manual { background: #4d8af020; color: var(--blue); }
  .mode-badge.auto   { background: #18c3ad25; color: var(--teal); }
  .mode-stage { font-size: 12px; color: var(--muted); }
  .stopall-btn {
    background: #ef4f5f20;
    border: 1px solid #ef4f5f60;
    color: var(--red);
    font-size: 13px;
    font-weight: 800;
    padding: 10px 18px;
    border-radius: 10px;
    cursor: pointer;
    transition: all 0.2s;
  }
  .stopall-btn:hover { background: var(--red); color: #fff; }

  /* ── Tank Full status ── */
  .tank-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
    gap: 14px;
  }
  .tank-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
    display: flex;
    gap: 18px;
    align-items: center;
    transition: border-color 0.3s, background 0.3s;
  }
  .tank-card.full    { border-color: #2ecf7160; background: #2ecf710c; }
  .tank-card.partial { border-color: #4d8af060; background: #4d8af00c; }
  .tank-card.empty   { border-color: #f5a62360; background: #f5a6230c; }
  .tank-icon { font-size: 36px; flex-shrink: 0; }
  .tank-info { flex: 1; }
  .tank-label { font-size: 21px; font-weight: 800; margin-top: 2px; }
  .tank-card.full    .tank-label { color: var(--green); }
  .tank-card.partial .tank-label { color: var(--blue); }
  .tank-card.empty   .tank-label { color: var(--amber); }
  .tank-reason { font-size: 12px; color: var(--muted); margin-top: 4px; }

  /* ── Float switch raw status ── */
  .float-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    gap: 12px;
  }
  .float-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 15px 16px;
    display: flex;
    align-items: center;
    gap: 12px;
    transition: border-color 0.2s, background 0.2s;
  }
  .float-card.triggered { border-color: #2ecf7160; background: #2ecf710c; }
  .float-dot {
    width: 12px; height: 12px; border-radius: 50%;
    background: var(--surface2); flex-shrink: 0;
    transition: background 0.2s, box-shadow 0.2s;
  }
  .float-card.triggered .float-dot { background: var(--green); box-shadow: 0 0 8px #2ecf71a0; }
  .float-info { flex: 1; }
  .float-name { font-size: 11px; color: var(--muted); font-weight: 600; }
  .float-desc { font-size: 10px; color: var(--muted); margin-top: 1px; }
  .float-state { font-size: 13px; font-weight: 800; margin-top: 3px; }
  .float-card.triggered .float-state { color: var(--green); }
  .float-card:not(.triggered) .float-state { color: var(--muted); }

  /* ── Potability ── */
  .potability-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
    display: flex;
    gap: 16px;
    align-items: center;
    transition: border-color 0.3s, background 0.3s;
  }
  .potability-card.drinkable { border-color: #2ecf7160; background: #2ecf710c; }
  .potability-card.caution   { border-color: #f5a62360; background: #f5a6230c; }
  .potability-card.unsafe    { border-color: #ef4f5f60; background: #ef4f5f0c; }
  .pot-icon { font-size: 34px; flex-shrink: 0; }
  .pot-info { flex: 1; }
  .pot-label  { font-size: 21px; font-weight: 800; margin-top: 2px; }
  .pot-reason { font-size: 12px; color: var(--muted); margin-top: 4px; }
  .drinkable .pot-label { color: var(--green); }
  .caution   .pot-label { color: var(--amber); }
  .unsafe    .pot-label { color: var(--red); }

  /* ── Yield card (humidity + dew point + condensation) ── */
  .yield-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
    gap: 14px;
  }
  .yield-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
  }
  .yield-title { font-size: 11px; color: var(--muted); letter-spacing: 1px; text-transform: uppercase; font-weight: 700; margin-bottom: 12px; }
  .yield-big { font-size: 26px; font-weight: 800; }
  .yield-tier {
    margin-top: 10px;
    font-size: 12px;
    font-weight: 700;
    padding: 5px 12px;
    border-radius: 20px;
    display: inline-block;
  }
  .tier-poor      { background: #ef4f5f20; color: var(--red); }
  .tier-fair      { background: #f5a62320; color: var(--amber); }
  .tier-good      { background: #2ecf7120; color: var(--green); }
  .tier-excellent { background: #18c3ad25; color: var(--teal); }
  .yield-desc { font-size: 11px; color: var(--muted); margin-top: 10px; line-height: 1.5; }
  .gap-bar-wrap { margin-top: 14px; }
  .gap-bar-track { height: 8px; border-radius: 6px; background: var(--surface2); overflow: hidden; }
  .gap-bar-fill { height: 100%; border-radius: 6px; transition: width 0.5s ease, background 0.5s ease; }
  .gap-bar-label { display: flex; justify-content: space-between; font-size: 10px; color: var(--muted); margin-top: 6px; }

  /* ── Relay control panel ── */
  .relay-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
    gap: 14px;
  }
  .relay-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 18px 20px;
    display: flex;
    flex-direction: column;
    gap: 10px;
    transition: border-color 0.2s;
  }
  .relay-card.on { border-color: #18c3ad60; }
  .relay-top { display: flex; align-items: center; justify-content: space-between; }
  .relay-name-wrap { display: flex; align-items: center; gap: 10px; }
  .relay-icon {
    width: 36px; height: 36px; border-radius: 10px;
    display: flex; align-items: center; justify-content: center;
    font-size: 17px; background: var(--surface2);
  }
  .relay-name { font-weight: 700; font-size: 13px; }
  .relay-sub  { font-size: 10px; color: var(--muted); margin-top: 1px; }

  .toggle-switch { position: relative; width: 50px; height: 27px; flex-shrink: 0; }
  .toggle-switch input { opacity: 0; width: 0; height: 0; }
  .toggle-slider {
    position: absolute; cursor: pointer; inset: 0;
    background: var(--surface2); border: 1px solid var(--border);
    border-radius: 34px; transition: background 0.25s;
  }
  .toggle-slider:before {
    content: ""; position: absolute; height: 19px; width: 19px;
    left: 3px; bottom: 3px; background: #fff; border-radius: 50%;
    transition: transform 0.25s; box-shadow: 0 1px 3px rgba(0,0,0,0.4);
  }
  .toggle-switch input:checked + .toggle-slider { background: var(--teal); border-color: var(--teal); }
  .toggle-switch input:checked + .toggle-slider:before { transform: translateX(22px); }
  .toggle-switch input:disabled + .toggle-slider { opacity: 0.5; cursor: not-allowed; }

  .relay-status-row { display: flex; align-items: center; justify-content: space-between; }
  .relay-status { font-size: 11px; font-weight: 700; padding: 3px 10px; border-radius: 20px; }
  .r-on  { background: #2ecf7120; color: var(--green); }
  .r-off { background: var(--surface2); color: var(--muted); }
  .relay-hint { font-size: 10px; color: var(--muted); }

  /* ── Charts ── */
  .charts-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
    gap: 14px;
  }
  .chart-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
  }
  .chart-top { display: flex; align-items: center; justify-content: space-between; margin-bottom: 14px; }
  .chart-title { font-size: 12px; color: var(--muted); letter-spacing: 1px; text-transform: uppercase; font-weight: 700; }
  .chart-toggle {
    display: flex; background: var(--surface2); border-radius: 8px; padding: 3px; gap: 2px;
  }
  .chart-toggle button {
    background: transparent; border: none; color: var(--muted);
    font-size: 10px; font-weight: 700; letter-spacing: 0.5px; text-transform: uppercase;
    padding: 5px 10px; border-radius: 6px; cursor: pointer; transition: all 0.15s;
  }
  .chart-toggle button.active { background: var(--blue); color: #fff; }
  .chart-wrap  { position: relative; height: 180px; }

  /* ── Alerts log ── */
  .log-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
  }
  .log-title  { font-size: 12px; color: var(--muted); letter-spacing: 1px; text-transform: uppercase; margin-bottom: 14px; font-weight: 700; }
  .log-list   { max-height: 170px; overflow-y: auto; display: flex; flex-direction: column; gap: 6px; }
  .log-entry  { display: flex; align-items: flex-start; gap: 10px; font-size: 12px; padding: 9px 11px; border-radius: 9px; background: var(--surface2); }
  .log-time   { color: var(--muted); flex-shrink: 0; font-variant-numeric: tabular-nums; }
  .log-msg    { color: var(--text); }
  .log-empty  { color: var(--muted); font-size: 12px; text-align: center; padding: 22px 0; }

  footer { text-align: center; padding: 18px; color: var(--muted); font-size: 11px; border-top: 1px solid var(--border); margin-top: 10px; }

  @media (max-width: 600px) {
    main   { padding: 16px; }
    header { padding: 13px 16px; }
    .logo-text { font-size: 15px; }
  }
</style>
</head>
<body>

<header>
  <div class="logo">
    <div class="logo-icon">💧</div>
    <div>
      <div class="logo-text">AWG-HEROES Dashboard</div>
      <div class="logo-sub">Atmospheric Water Generator · IoT Monitor</div>
    </div>
  </div>
  <div class="header-right">
    <div class="wifi-badge">● WiFi</div>
    <div id="clock">--:--:--</div>
  </div>
</header>

<div id="alertBar">
  <span>⚠️</span>
  <span id="alertMsg">—</span>
</div>

<main>

  <div class="section-label">System Status</div>
  <div class="status-dots-card">
    <div class="status-dot-item">
      <span class="status-dot" id="dotPower"></span>
      <span class="status-dot-label">System Powered</span>
    </div>
    <div class="status-dot-item">
      <span class="status-dot" id="dotHarvest"></span>
      <span class="status-dot-label">Harvesting Water</span>
    </div>
    <div class="status-dot-item">
      <span class="status-dot" id="dotTank10"></span>
      <span class="status-dot-label">10L Tank Full</span>
    </div>
    <div class="status-dot-item">
      <span class="status-dot" id="dotTank20"></span>
      <span class="status-dot-label">20L Tank Full</span>
    </div>
    <div class="status-dot-item">
      <span class="status-dot" id="dotAlerts"></span>
      <span class="status-dot-label">Alerts</span>
    </div>
  </div>

  <div class="section-label">System Mode</div>
  <div class="mode-card">
    <div class="mode-buttons">
      <button class="mode-btn active" id="modeBtnManual" onclick="setMode('manual')">🎛️ Manual</button>
      <button class="mode-btn" id="modeBtnAuto" onclick="setMode('auto')">🤖 Automatic</button>
    </div>
    <div class="mode-status">
      <span class="mode-badge manual" id="modeBadge">MANUAL</span>
      <span class="mode-stage" id="modeStage"></span>
    </div>
    <button class="stopall-btn" onclick="stopAll()">⛔ STOP ALL</button>
  </div>

  <div class="status-row">
    <div class="stat-card">
      <div class="stat-label">Uptime</div>
      <div class="stat-val" id="uptime" style="font-size:16px;margin-top:7px">—</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Free Heap</div>
      <div class="stat-val" id="heap" style="color:var(--teal)">—</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">WiFi Signal</div>
      <div class="stat-val" id="rssi" style="color:var(--blue)">—</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Active Alerts</div>
      <div class="stat-val" id="alertCount" style="color:var(--red)">0</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Hourly Points Logged</div>
      <div class="stat-val" id="hourCount" style="color:var(--cyan)">0 / 24</div>
    </div>
  </div>

  <div class="section-label">Float Switch Status (Raw Sensors)</div>
  <div class="float-grid">
    <div class="float-card" id="floatCard1">
      <div class="float-dot" id="floatDot1"></div>
      <div class="float-info">
        <div class="float-name">FLOAT 1 · 10L LOW</div>
        <div class="float-desc">Raw tank empty sensor</div>
        <div class="float-state" id="floatState1">OFF</div>
      </div>
    </div>
    <div class="float-card" id="floatCard2">
      <div class="float-dot" id="floatDot2"></div>
      <div class="float-info">
        <div class="float-name">FLOAT 2 · 10L HIGH</div>
        <div class="float-desc">Raw tank full sensor</div>
        <div class="float-state" id="floatState2">OFF</div>
      </div>
    </div>
    <div class="float-card" id="floatCard3">
      <div class="float-dot" id="floatDot3"></div>
      <div class="float-info">
        <div class="float-name">FLOAT 3 · 20L HIGH</div>
        <div class="float-desc">Treated tank full sensor</div>
        <div class="float-state" id="floatState3">OFF</div>
      </div>
    </div>
    <div class="float-card" id="floatCard4">
      <div class="float-dot" id="floatDot4"></div>
      <div class="float-info">
        <div class="float-name">FLOAT 4 · 20L LOW</div>
        <div class="float-desc">Treated tank empty sensor</div>
        <div class="float-state" id="floatState4">OFF</div>
      </div>
    </div>
  </div>

  <div class="section-label">Tank Status (Derived)</div>
  <div class="tank-grid">
    <div class="tank-card partial" id="tankCardRaw">
      <div class="tank-icon" id="tankIconRaw">💧</div>
      <div class="tank-info">
        <div class="sensor-name">Raw Tank · 10L (Unfiltered)</div>
        <div class="tank-label" id="tankLabelRaw">—</div>
        <div class="tank-reason" id="tankReasonRaw">Waiting for sensor data…</div>
      </div>
    </div>
    <div class="tank-card partial" id="tankCardTreated">
      <div class="tank-icon" id="tankIconTreated">💧</div>
      <div class="tank-info">
        <div class="sensor-name">Treated Tank · 20L (Filtered)</div>
        <div class="tank-label" id="tankLabelTreated">—</div>
        <div class="tank-reason" id="tankReasonTreated">Waiting for sensor data…</div>
      </div>
    </div>
  </div>

  <div class="section-label">Drinkability Verdict</div>
  <div class="potability-card" id="potabilityCard">
    <div class="pot-icon" id="potIcon">💧</div>
    <div class="pot-info">
      <div class="sensor-name">Water quality verdict</div>
      <div class="pot-label" id="potLabel">—</div>
      <div class="pot-reason" id="potReason">Waiting for sensor data…</div>
    </div>
  </div>

  <div class="section-label">Live Sensor Readings</div>
  <div class="sensors-grid">

    <div class="sensor-card" id="card-ph">
      <div class="sensor-top"><div class="sensor-icon" style="background:#b07bf020">🧪</div></div>
      <div class="sensor-name">pH Level</div>
      <div><span class="sensor-value" id="val-ph" style="color:var(--purple)">—</span><span class="sensor-unit">pH</span></div>
      <div style="margin-top:8px"><span class="sensor-status" id="st-ph">—</span></div>
      <div class="sensor-hint">Safe: 6.5 – 8.5</div>
    </div>

    <div class="sensor-card" id="card-tds">
      <div class="sensor-top"><div class="sensor-icon" style="background:#4d8af020">💎</div></div>
      <div class="sensor-name">TDS</div>
      <div><span class="sensor-value" id="val-tds" style="color:var(--blue)">—</span><span class="sensor-unit">ppm</span></div>
      <div style="margin-top:8px"><span class="sensor-status" id="st-tds">—</span></div>
      <div class="sensor-hint">Drinking: &lt; 500 ppm</div>
    </div>

    <div class="sensor-card" id="card-temp">
      <div class="sensor-top"><div class="sensor-icon" style="background:#ef4f5f20">🌡️</div></div>
      <div class="sensor-name">Temperature</div>
      <div><span class="sensor-value" id="val-temp" style="color:var(--red)">—</span><span class="sensor-unit">°C</span></div>
      <div style="margin-top:8px"><span class="sensor-status" id="st-temp">—</span></div>
      <div class="sensor-hint">Normal: 15 – 35 °C</div>
    </div>

    <div class="sensor-card" id="card-hum">
      <div class="sensor-top"><div class="sensor-icon" style="background:#18c3ad20">💧</div></div>
      <div class="sensor-name">Humidity</div>
      <div><span class="sensor-value" id="val-hum" style="color:var(--teal)">—</span><span class="sensor-unit">%</span></div>
      <div style="margin-top:8px"><span class="sensor-status" id="st-hum">—</span></div>
      <div class="sensor-hint">Higher = more yield potential</div>
    </div>

    <div class="sensor-card" id="card-dew">
      <div class="sensor-top"><div class="sensor-icon" style="background:#29c5e820">❄️</div></div>
      <div class="sensor-name">Dew Point</div>
      <div><span class="sensor-value" id="val-dew" style="color:var(--cyan)">—</span><span class="sensor-unit">°C</span></div>
      <div style="margin-top:8px"><span class="sensor-status" id="st-dew">—</span></div>
      <div class="sensor-hint">Temp at which condensation forms</div>
    </div>

  </div>

  <div class="section-label">Relay Control Panel</div>
  <div class="relay-grid">

    <div class="relay-card" id="relay-card-1">
      <div class="relay-top">
        <div class="relay-name-wrap">
          <div class="relay-icon">🧊</div>
          <div>
            <div class="relay-name">Compressor</div>
            <div class="relay-sub">Dehumidifier</div>
          </div>
        </div>
        <label class="toggle-switch">
          <input type="checkbox" id="relayToggle1" onchange="setRelay(1, this.checked)">
          <span class="toggle-slider"></span>
        </label>
      </div>
      <div class="relay-status-row">
        <span class="relay-status r-off" id="relayStatus1">OFF</span>
        <span class="relay-hint">Manual</span>
      </div>
    </div>

    <div class="relay-card" id="relay-card-2">
      <div class="relay-top">
        <div class="relay-name-wrap">
          <div class="relay-icon">🔆</div>
          <div>
            <div class="relay-name">UV Sterilizer</div>
            <div class="relay-sub">Disinfection</div>
          </div>
        </div>
        <label class="toggle-switch">
          <input type="checkbox" id="relayToggle2" onchange="setRelay(2, this.checked)">
          <span class="toggle-slider"></span>
        </label>
      </div>
      <div class="relay-status-row">
        <span class="relay-status r-off" id="relayStatus2">OFF</span>
        <span class="relay-hint">Manual</span>
      </div>
    </div>

    <div class="relay-card" id="relay-card-3">
      <div class="relay-top">
        <div class="relay-name-wrap">
          <div class="relay-icon">🚰</div>
          <div>
            <div class="relay-name">Fill Pump</div>
            <div class="relay-sub">Tank filling</div>
          </div>
        </div>
        <label class="toggle-switch">
          <input type="checkbox" id="relayToggle3" onchange="setRelay(3, this.checked)">
          <span class="toggle-slider"></span>
        </label>
      </div>
      <div class="relay-status-row">
        <span class="relay-status r-off" id="relayStatus3">OFF</span>
        <span class="relay-hint">Cuts off on 20L full</span>
      </div>
    </div>

    <div class="relay-card" id="relay-card-4">
      <div class="relay-top">
        <div class="relay-name-wrap">
          <div class="relay-icon">🚿</div>
          <div>
            <div class="relay-name">Faucet Pump</div>
            <div class="relay-sub">Dispense water</div>
          </div>
        </div>
        <label class="toggle-switch">
          <input type="checkbox" id="relayToggle4" onchange="setRelay(4, this.checked)">
          <span class="toggle-slider"></span>
        </label>
      </div>
      <div class="relay-status-row">
        <span class="relay-status r-off" id="relayStatus4">OFF</span>
        <span class="relay-hint">Manual</span>
      </div>
    </div>

  </div>

  <div class="section-label">Water Yield Potential</div>
  <div class="yield-grid">

    <div class="yield-card">
      <div class="yield-title">Humidity Yield Rating</div>
      <div class="yield-big" id="humYieldVal">—%</div>
      <div class="yield-tier" id="humYieldTier">—</div>
      <div class="yield-desc" id="humYieldDesc">Waiting for sensor data…</div>
    </div>

    <div class="yield-card">
      <div class="yield-title">Condensation Potential</div>
      <div class="yield-big"><span id="condGapVal">—</span><span style="font-size:15px;color:var(--muted)"> °C gap</span></div>
      <div class="yield-tier" id="condGapTier">—</div>
      <div class="yield-desc">Smaller gap between air temp and dew point means condensation happens more easily — better yield.</div>
      <div class="gap-bar-wrap">
        <div class="gap-bar-track"><div class="gap-bar-fill" id="gapBarFill" style="width:0%;background:var(--green)"></div></div>
        <div class="gap-bar-label"><span>0°C (ideal)</span><span>15°C+ (poor)</span></div>
      </div>
    </div>

  </div>

  <div class="section-label">Trends</div>
  <div class="charts-grid">

    <div class="chart-card">
      <div class="chart-top">
        <div class="chart-title">pH</div>
        <div class="chart-toggle" data-chart="PH"><button class="active" data-range="min">Minute</button><button data-range="hour">Hourly</button></div>
      </div>
      <div class="chart-wrap"><canvas id="chartPH"></canvas></div>
    </div>

    <div class="chart-card">
      <div class="chart-top">
        <div class="chart-title">TDS</div>
        <div class="chart-toggle" data-chart="TDS"><button class="active" data-range="min">Minute</button><button data-range="hour">Hourly</button></div>
      </div>
      <div class="chart-wrap"><canvas id="chartTDS"></canvas></div>
    </div>

    <div class="chart-card">
      <div class="chart-top">
        <div class="chart-title">Temperature & Humidity</div>
        <div class="chart-toggle" data-chart="TH"><button class="active" data-range="min">Minute</button><button data-range="hour">Hourly</button></div>
      </div>
      <div class="chart-wrap"><canvas id="chartTH"></canvas></div>
    </div>

    <div class="chart-card">
      <div class="chart-top">
        <div class="chart-title">Dew Point</div>
        <div class="chart-toggle" data-chart="Dew"><button class="active" data-range="min">Minute</button><button data-range="hour">Hourly</button></div>
      </div>
      <div class="chart-wrap"><canvas id="chartDew"></canvas></div>
    </div>

  </div>

  <div class="section-label">Alert Log</div>
  <div class="log-card">
    <div class="log-list" id="logList">
      <div class="log-empty">No alerts yet — system nominal.</div>
    </div>
  </div>

</main>

<footer>AWG-HEROES IoT Dashboard · ESP32 · Built with ESPAsyncWebServer</footer>

<script>
const MAX_POINTS = 30;

// ── Local per-sensor history buffers (mirror server, kept for live minute view) ──
const minLabels = [];
const hourLabels = [];

const data = {
  PH:    { min: [], hour: [] },
  TDS:   { min: [], hour: [] },
  Temp:  { min: [], hour: [] },
  Hum:   { min: [], hour: [] },
  Dew:   { min: [], hour: [] }
};

function makeChart(id, datasets, yMin, yMax) {
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: { labels: minLabels, datasets },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      plugins: { legend: { labels: { color: '#7e8ba0', font: { size: 11 }, boxWidth: 10 } } },
      scales: {
        x: { ticks: { color: '#7e8ba0', font: { size: 10 }, maxTicksLimit: 6 }, grid: { color: '#232d3a' } },
        y: { min: yMin, max: yMax, ticks: { color: '#7e8ba0', font: { size: 10 } }, grid: { color: '#232d3a' } }
      }
    }
  });
}

const ds = (label, color, arr) => ({
  label, data: arr,
  borderColor: color, backgroundColor: color + '20',
  borderWidth: 2, pointRadius: 0, fill: true, tension: 0.4
});

const charts = {
  PH:    makeChart('chartPH',    [ds('pH',      '#b07bf0', data.PH.min)],                                   0, 14),
  TDS:   makeChart('chartTDS',   [ds('ppm',     '#4d8af0', data.TDS.min)],                                  0, 1000),
  TH:    makeChart('chartTH',    [ds('Temp °C', '#ef4f5f', data.Temp.min), ds('Humid %', '#18c3ad', data.Hum.min)], 0, 100),
  Dew:   makeChart('chartDew',   [ds('Dew °C',  '#29c5e8', data.Dew.min)],                                   0, 40)
};

// Track which range each chart card is currently showing
const chartRange = { PH: 'min', TDS: 'min', TH: 'min', Dew: 'min' };

document.querySelectorAll('.chart-toggle').forEach(toggle => {
  const key = toggle.dataset.chart;
  toggle.querySelectorAll('button').forEach(btn => {
    btn.addEventListener('click', () => {
      toggle.querySelectorAll('button').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      chartRange[key] = btn.dataset.range;
      applyRange(key);
    });
  });
});

function applyRange(key) {
  const range = chartRange[key];
  const labels = range === 'min' ? minLabels : hourLabels;
  if (key === 'TH') {
    charts.TH.data.labels = labels;
    charts.TH.data.datasets[0].data = range === 'min' ? data.Temp.min : data.Temp.hour;
    charts.TH.data.datasets[1].data = range === 'min' ? data.Hum.min  : data.Hum.hour;
    charts.TH.update();
    return;
  }
  const map = { PH: 'PH', TDS: 'TDS', Dew: 'Dew' };
  const dataKey = map[key];
  charts[key].data.labels = labels;
  charts[key].data.datasets[0].data = range === 'min' ? data[dataKey].min : data[dataKey].hour;
  charts[key].update();
}

function pushCapped(arr, val) {
  arr.push(val);
  if (arr.length > MAX_POINTS) arr.shift();
}

let alertLog = [];
let alertCounts = 0;

function addAlert(msg) {
  const now = new Date();
  const ts  = now.toTimeString().slice(0,8);
  alertLog.unshift({ ts, msg });
  if (alertLog.length > 50) alertLog.pop();
  alertCounts++;
  renderLog();
}

function renderLog() {
  const el = document.getElementById('logList');
  if (alertLog.length === 0) {
    el.innerHTML = '<div class="log-empty">No alerts yet — system nominal.</div>';
    return;
  }
  el.innerHTML = alertLog.map(a =>
    `<div class="log-entry"><span class="log-time">${a.ts}</span><span class="log-msg">${a.msg}</span></div>`
  ).join('');
}

let prevAlerts = { ph: false, tds: false, potability: undefined, rawFull: undefined, treatedFull: undefined, rawLow: undefined, treatedLow: undefined };

// Humidity yield tiers for AWG condensation suitability
function humidityYield(hum) {
  if (hum < 40)  return { tier: 'poor',      label: 'Poor — Low Yield',      cls: 'tier-poor',      desc: 'Humidity below 40% rarely condenses enough water to be worthwhile. Expect minimal output.' };
  if (hum < 60)  return { tier: 'fair',      label: 'Fair — Moderate Yield', cls: 'tier-fair',       desc: 'Workable but inefficient. Yield will be noticeably lower than humid conditions.' };
  if (hum < 80)  return { tier: 'good',      label: 'Good — Solid Yield',    cls: 'tier-good',       desc: 'Good conditions for an AWG unit. Expect steady, reliable water output.' };
  return                { tier: 'excellent', label: 'Excellent — Peak Yield', cls: 'tier-excellent', desc: 'Ideal humidity for atmospheric water generation. Expect maximum output.' };
}

function condensationTier(gap) {
  if (gap <= 3)  return { label: 'Excellent', cls: 'tier-excellent', color: '#18c3ad' };
  if (gap <= 7)  return { label: 'Good',      cls: 'tier-good',      color: '#2ecf71' };
  if (gap <= 12) return { label: 'Fair',      cls: 'tier-fair',      color: '#f5a623' };
  return              { label: 'Poor',      cls: 'tier-poor',      color: '#ef4f5f' };
}

function setRelay(id, on) {
  fetch(`/relay?id=${id}&state=${on ? 'on' : 'off'}`)
    .then(res => res.json())
    .then(d => updateRelayUI(d.id, d.state === 'on'))
    .catch(() => {});
}

function updateRelayUI(id, on) {
  document.getElementById('relayToggle' + id).checked = on;
  const st = document.getElementById('relayStatus' + id);
  st.textContent = on ? 'ON' : 'OFF';
  st.className = 'relay-status ' + (on ? 'r-on' : 'r-off');
  document.getElementById('relay-card-' + id).classList.toggle('on', on);
}

let currentMode = 'manual';

function setMode(m) {
  fetch(`/mode?mode=${m}`)
    .then(r => r.json())
    .then(d => applyMode(d.mode))
    .catch(() => {});
}

function stopAll() {
  if (!confirm('Stop ALL relays and switch back to Manual mode?')) return;
  fetch('/stopall').then(() => applyMode('manual')).catch(() => {});
}

function applyMode(m) {
  currentMode = m;
  document.getElementById('modeBtnManual').classList.toggle('active', m === 'manual');
  document.getElementById('modeBtnAuto').classList.toggle('active', m === 'auto');
  const badge = document.getElementById('modeBadge');
  badge.textContent = m === 'auto' ? 'AUTOMATIC' : 'MANUAL';
  badge.className = 'mode-badge ' + (m === 'auto' ? 'auto' : 'manual');
  // Relays 1-3 are driven by the automatic cycle while it's active;
  // Relay 4 (Faucet Pump) always stays manually controllable.
  [1, 2, 3].forEach(id => {
    document.getElementById('relayToggle' + id).disabled = (m === 'auto');
  });
}

// Tank state from the two float switches: FULL > EMPTY > PARTIAL(default)
function tankState(full, low) {
  if (full) return { label: 'Full',    cls: 'full',    icon: '🪣' };
  if (low)  return { label: 'Empty',   cls: 'empty',   icon: '🚱' };
  return           { label: 'Partial', cls: 'partial', icon: '💧' };
}

function updateFloatCard(id, triggered) {
  document.getElementById('floatCard' + id).classList.toggle('triggered', triggered);
  document.getElementById('floatState' + id).textContent = triggered ? 'TRIGGERED' : 'OFF';
}

function update(d) {
  const t = new Date().toTimeString().slice(0,8);
  if (minLabels.length >= MAX_POINTS) minLabels.shift();
  minLabels.push(t);

  // pH
  document.getElementById('val-ph').textContent = d.ph.toFixed(2);
  const phOk = d.ph >= 6.5 && d.ph <= 8.5;
  document.getElementById('st-ph').outerHTML =
    `<span class="sensor-status ${phOk?'s-ok':'s-danger'}" id="st-ph">${phOk?'✓ Safe':'⚠ Out of range'}</span>`;
  document.getElementById('card-ph').classList.toggle('alert', !phOk);
  pushCapped(data.PH.min, d.ph);
  if (chartRange.PH === 'min') { charts.PH.data.labels = minLabels; charts.PH.update(); }
  if (!phOk && !prevAlerts.ph) addAlert(`pH ${d.ph.toFixed(2)} is out of safe range (6.5–8.5)`);
  prevAlerts.ph = !phOk;

  // TDS
  document.getElementById('val-tds').textContent = Math.round(d.tds);
  const tdsOk = d.tds < 500;
  document.getElementById('st-tds').outerHTML =
    `<span class="sensor-status ${tdsOk?'s-ok':'s-warn'}" id="st-tds">${tdsOk?'✓ Drinkable':'⚠ High TDS'}</span>`;
  document.getElementById('card-tds').classList.toggle('alert', !tdsOk);
  pushCapped(data.TDS.min, d.tds);
  if (chartRange.TDS === 'min') { charts.TDS.data.labels = minLabels; charts.TDS.update(); }
  if (!tdsOk && !prevAlerts.tds) addAlert(`TDS ${Math.round(d.tds)} ppm exceeds 500 ppm limit`);
  prevAlerts.tds = !tdsOk;

  // Temp
  document.getElementById('val-temp').textContent = d.temp.toFixed(1);
  const tempOk = d.temp >= 15 && d.temp <= 35;
  document.getElementById('st-temp').outerHTML =
    `<span class="sensor-status ${tempOk?'s-ok':'s-warn'}" id="st-temp">${tempOk?'✓ Normal':'⚠ Check temp'}</span>`;
  pushCapped(data.Temp.min, d.temp);

  // Humidity
  document.getElementById('val-hum').textContent = d.hum.toFixed(1);
  const humOk = d.hum >= 60;
  document.getElementById('st-hum').outerHTML =
    `<span class="sensor-status ${humOk?'s-ok':'s-warn'}" id="st-hum">${humOk?'✓ Good':'⚠ Low — less water yield'}</span>`;
  pushCapped(data.Hum.min, d.hum);
  if (chartRange.TH === 'min') { charts.TH.data.labels = minLabels; charts.TH.update(); }

  // Dew point
  document.getElementById('val-dew').textContent = d.dewPoint.toFixed(1);
  pushCapped(data.Dew.min, d.dewPoint);
  if (chartRange.Dew === 'min') { charts.Dew.data.labels = minLabels; charts.Dew.update(); }
  const condGap = d.temp - d.dewPoint;
  const dewOk = condGap <= 7;
  document.getElementById('st-dew').outerHTML =
    `<span class="sensor-status ${dewOk?'s-ok':'s-info'}" id="st-dew">${dewOk?'✓ Close to air temp':'Wider gap'}</span>`;

  // Float switch raw status (FLOAT1=10L low, FLOAT2=10L high, FLOAT3=20L high, FLOAT4=20L low)
  updateFloatCard(1, d.rawTankLow);
  updateFloatCard(2, d.rawTankFull);
  updateFloatCard(3, d.treatedTankFull);
  updateFloatCard(4, d.treatedTankLow);

  // Tank status (HIGH + LOW float switches per tank)
  const rawCard = document.getElementById('tankCardRaw');
  const rs = tankState(d.rawTankFull, d.rawTankLow);
  rawCard.classList.remove('full', 'partial', 'empty');
  rawCard.classList.add(rs.cls);
  document.getElementById('tankIconRaw').textContent  = rs.icon;
  document.getElementById('tankLabelRaw').textContent = rs.label;
  document.getElementById('tankReasonRaw').textContent =
    rs.cls === 'full'  ? 'Raw water ready — automatic transfer/filtration can begin.' :
    rs.cls === 'empty' ? 'Raw tank is empty — collecting condensate from the AWG unit.' :
                          'Collecting condensate from the AWG unit.';
  if (d.rawTankFull !== prevAlerts.rawFull && prevAlerts.rawFull !== undefined && d.rawTankFull) {
    addAlert('Raw tank (10L) is full — ready to transfer through the filter');
  }
  if (d.rawTankLow !== prevAlerts.rawLow && prevAlerts.rawLow !== undefined && d.rawTankLow) {
    addAlert('Raw tank (10L) is now empty');
  }
  prevAlerts.rawFull = d.rawTankFull;
  prevAlerts.rawLow  = d.rawTankLow;

  const treatedCard = document.getElementById('tankCardTreated');
  const ts = tankState(d.treatedTankFull, d.treatedTankLow);
  treatedCard.classList.remove('full', 'partial', 'empty');
  treatedCard.classList.add(ts.cls);
  document.getElementById('tankIconTreated').textContent  = ts.icon;
  document.getElementById('tankLabelTreated').textContent = ts.label;
  document.getElementById('tankReasonTreated').textContent =
    ts.cls === 'full'  ? 'Treated tank full — fill pump auto-cutoff active.' :
    ts.cls === 'empty' ? 'Treated tank is empty — safe to run the fill pump.' :
                          'Safe to run the fill pump into this tank.';
  if (d.treatedTankFull !== prevAlerts.treatedFull && prevAlerts.treatedFull !== undefined && d.treatedTankFull) {
    addAlert('Treated tank (20L) is full — fill pump auto-cutoff engaged');
  }
  if (d.treatedTankLow !== prevAlerts.treatedLow && prevAlerts.treatedLow !== undefined && d.treatedTankLow) {
    addAlert('Treated tank (20L) is now empty');
  }
  prevAlerts.treatedFull = d.treatedTankFull;
  prevAlerts.treatedLow  = d.treatedTankLow;

  // System mode + automatic stage (synced across all connected clients)
  if (d.mode) applyMode(d.mode);
  document.getElementById('modeStage').textContent = d.mode === 'auto' ? ('Stage: ' + d.autoStage) : '';

  // System status dots: power / harvesting / tank10 / tank20 / alerts
  document.getElementById('dotPower').classList.add('on-green');   // page is receiving live data = ESP32 is powered
  const harvesting = d.relays && d.relays[0];   // relay 1 = Compressor
  document.getElementById('dotHarvest').classList.toggle('on-yellow', !!harvesting);
  document.getElementById('dotTank10').classList.toggle('on-blue', !!d.rawTankFull);
  document.getElementById('dotTank20').classList.toggle('on-blue', !!d.treatedTankFull);
  document.getElementById('dotAlerts').classList.toggle('on-red', alertLog.length > 0);

  // Relay states (synced across all connected clients)
  if (d.relays) {
    d.relays.forEach((on, i) => updateRelayUI(i + 1, on));
  }

  // Humidity yield rating
  const hy = humidityYield(d.hum);
  document.getElementById('humYieldVal').textContent = d.hum.toFixed(1) + '%';
  document.getElementById('humYieldTier').textContent = hy.label;
  document.getElementById('humYieldTier').className = 'yield-tier ' + hy.cls;
  document.getElementById('humYieldDesc').textContent = hy.desc;

  // Condensation potential (temp - dew point gap)
  const ct = condensationTier(condGap);
  document.getElementById('condGapVal').textContent = condGap.toFixed(1);
  document.getElementById('condGapTier').textContent = ct.label;
  document.getElementById('condGapTier').className = 'yield-tier ' + ct.cls;
  const gapPct = Math.max(0, Math.min(100, 100 - (condGap / 15) * 100));
  document.getElementById('gapBarFill').style.width = gapPct + '%';
  document.getElementById('gapBarFill').style.background = ct.color;

  // Drinkability verdict
  const potCard = document.getElementById('potabilityCard');
  potCard.classList.remove('drinkable', 'caution', 'unsafe');
  let potClass, potIcon;
  if (d.potability === 2)      { potClass = 'drinkable'; potIcon = '✅'; }
  else if (d.potability === 1) { potClass = 'caution';   potIcon = '⚠️'; }
  else                         { potClass = 'unsafe';    potIcon = '⛔'; }
  potCard.classList.add(potClass);
  document.getElementById('potIcon').textContent   = potIcon;
  document.getElementById('potLabel').textContent  = d.potabilityLabel;
  document.getElementById('potReason').textContent = d.potabilityReason;

  if (d.potability !== prevAlerts.potability && prevAlerts.potability !== undefined) {
    if (d.potability === 0) addAlert(`Water NOT drinkable — ${d.potabilityReason}`);
    else if (d.potability === 1) addAlert(`Water quality borderline — ${d.potabilityReason}`);
  }
  prevAlerts.potability = d.potability;

  // Status row
  document.getElementById('heap').textContent   = (d.heap / 1024).toFixed(0) + ' KB';
  document.getElementById('rssi').textContent   = d.rssi + ' dBm';
  document.getElementById('uptime').textContent = d.uptime;
  document.getElementById('hourCount').textContent = d.hourCount + ' / 24';

  // Alert bar
  const allAlerts = [];
  if (!phOk)    allAlerts.push('🧪 pH out of range');
  if (!tdsOk)   allAlerts.push('💎 High TDS');
  if (d.potability === 0) allAlerts.push('⛔ Water not drinkable');
  if (d.noWater) allAlerts.push('🚫 No water detected — pumps held off');
  alertCounts = allAlerts.length;
  document.getElementById('alertCount').textContent = alertCounts;
  if (allAlerts.length > 0) {
    document.getElementById('alertBar').classList.add('show');
    document.getElementById('alertMsg').textContent = allAlerts.join('  ·  ');
  } else {
    document.getElementById('alertBar').classList.remove('show');
  }

  // Hourly history pushed from server snapshot (if provided)
  if (d.hourly) {
    hourLabels.length = 0;
    data.PH.hour.length = 0; data.TDS.hour.length = 0; data.Temp.hour.length = 0;
    data.Hum.hour.length = 0; data.Dew.hour.length = 0;
    d.hourly.forEach(h => {
      hourLabels.push(h.t);
      data.PH.hour.push(h.ph); data.TDS.hour.push(h.tds); data.Temp.hour.push(h.temp);
      data.Hum.hour.push(h.hum); data.Dew.hour.push(h.dew);
    });
    ['PH','TDS','TH','Dew'].forEach(k => { if (chartRange[k] === 'hour') applyRange(k); });
  }
}

setInterval(() => {
  document.getElementById('clock').textContent = new Date().toTimeString().slice(0,8);
}, 1000);

let ws;
function connectWS() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onmessage = e => {
    try { update(JSON.parse(e.data)); } catch(err) {}
  };
  ws.onclose = () => setTimeout(connectWS, 2000);
}
connectWS();
</script>
</body>
</html>
)rawhtml";


// ════════════════════════════════════════════════════════════════
//  RELAY HELPERS
// ════════════════════════════════════════════════════════════════

void relayWrite(uint8_t pin, bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

uint8_t relayPinFromIndex(int i) {
  switch (i) {
    case 1: return RELAY_1_PIN;
    case 2: return RELAY_2_PIN;
    case 3: return RELAY_3_PIN;
    case 4: return RELAY_4_PIN;
    default: return 255;
  }
}

void setRelayState(int id, bool on) {
  uint8_t pin = relayPinFromIndex(id);
  if (pin == 255) return;
  relayWrite(pin, on);
  relayState[id - 1] = on;
}

void setupRelays() {
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
  pinMode(RELAY_4_PIN, OUTPUT);
  for (int i = 1; i <= 4; i++) setRelayState(i, false);
}

// ── FIXED in v9.1 ──────────────────────────────────────────────────
// Called every loop to make sure the fill pump can never keep running
// once the TREATED (20L) tank is full — that's a genuine overflow
// risk in both Manual and Automatic mode, so it stays enforced
// unconditionally here.
//
// The previous version ALSO force-cut the pump whenever the RAW
// (10L) tank read LOW/empty, in both modes. That's correct for
// Automatic mode (and is still handled there, inside
// runAutomaticControl()'s pumpOn condition) — but in Manual mode it
// meant flipping the Fill Pump toggle ON while the 10L tank was
// empty would appear to work for one loop iteration and then get
// silently snapped back OFF, since the /relay endpoint's manual
// block checks never actually blocked on g_rawTankLow to begin with.
// That mismatch is what caused the toggle to refuse to "stick."
//
// Dropping the g_rawTankLow condition here restores your ability to
// manually run the Fill Pump (e.g. for priming/testing) even while
// the raw tank reads low — the overflow protection against the
// treated tank is unaffected.
void enforceFillPumpCutoff() {
  if (g_treatedTankFull && relayState[2]) {  // relay 3 = index 2 = Fill Pump
    setRelayState(3, false);
  }
}

// Debounces the "pH AND TDS both read ~0" condition over a full 30s
// window before trusting it — a single noisy reading shouldn't trip
// this. When it's confirmed true, g_noWaterDetected holds Compressor/
// UV/Fill Pump off (see below) since those sensors being ~0 usually
// means they're dry / not actually sitting in water.
void updateWaterDetectionGuard() {
  bool currentlyNoWater = (g_ph <= 0.05f && g_tds <= 1.0f);
  if (currentlyNoWater != g_noWaterRaw) {
    g_noWaterChangeTime = millis();
  }
  if ((millis() - g_noWaterChangeTime) > SENSOR_CONFIRM_MS) {
    g_noWaterDetected = currentlyNoWater;
  }
  g_noWaterRaw = currentlyNoWater;
}

// ════════════════════════════════════════════════════════════════
//  AUTOMATIC CONTROL — level-based (continuous condition) logic
// ════════════════════════════════════════════════════════════════
// This replaces the old step-by-step state machine with simple,
// continuously-evaluated rules per your latest wiring/behavior spec:
//
//   FLOAT 1 = Raw tank (10L)     LOW  sensor  → g_rawTankLow
//   FLOAT 2 = Raw tank (10L)     HIGH sensor  → g_rawTankFull
//   FLOAT 3 = Treated tank (20L) HIGH sensor  → g_treatedTankFull
//   FLOAT 4 = Treated tank (20L) LOW  sensor  → g_treatedTankLow
//
//   COMPRESSOR (Relay 1):
//     ON whenever the raw tank (10L) is NOT full (FLOAT 2 = false).
//     OFF the instant FLOAT 2 triggers (10L full) — no need to wait
//     for anything else. This is independent of the pump, so the
//     compressor can refill the 10L tank *while* the pump is also
//     transferring it out — confirmed you want them to run together.
//
//   FILL PUMP + UV (Relays 3 & 2):
//     "Armed" the first time FLOAT 2 (10L full) is ever seen after
//     entering Automatic mode — g_pumpArmed latches true and stays
//     true (it does NOT un-arm itself), since the pump is allowed to
//     keep running concurrently with the compressor refilling the
//     10L tank behind it.
//     Once armed: Pump/UV = ON whenever the treated tank (20L) is
//     NOT full (FLOAT 3 = false) AND the raw tank (10L) is NOT empty
//     (FLOAT 1 = false). Two independent cutoffs:
//       - FLOAT 3 triggers (20L full)  → pump OFF (overflow protection)
//       - FLOAT 1 triggers (10L empty) → pump OFF (dry-run protection —
//         nothing left in the 10L tank to actually pump)
//     Either one dropping low again brings the pump back ON automatically
//     — no separate "resume" step needed, it just falls out of the
//     same condition being re-checked every loop.
//
//   PARKED / BOTH FULL:
//     If FLOAT 2 (10L full) AND FLOAT 3 (20L full) are true at the
//     same time, both rules above naturally turn everything off
//     (Compressor off because FLOAT2, Pump/UV off because FLOAT3) —
//     no special-case code needed.
//
//   Safety: the "no water detected" guard (dry pH/TDS sensors) still
//   force-holds Compressor/UV/Fill Pump off regardless of float state.
//   The Faucet Pump (relay 4) is never touched here — always manual.
// ════════════════════════════════════════════════════════════════
void runAutomaticControl() {
  if (g_systemMode != MODE_AUTOMATIC) return;

  if (g_noWaterDetected) {
    setRelayState(1, false);
    setRelayState(2, false);
    setRelayState(3, false);
    return;   // held off until valid readings return
  }

  // Arm the pump the first time the raw tank reaches full. Stays
  // armed afterward — does not reset when FLOAT 2 later goes low,
  // since compressor + pump are allowed to run concurrently.
  if (g_rawTankFull) {
    if (!g_pumpArmed) {
      trySendSMS(lastSmsRawFull, "AWG-HEROES: Raw tank (10L) FULL. Filtration/transfer enabled.");
    }
    g_pumpArmed = true;
  }

  bool compressorOn = !g_rawTankFull;                 // fill the 10L tank whenever it's not full
  bool pumpOn        = g_pumpArmed && !g_treatedTankFull && !g_rawTankLow;
  // ^ transfer whenever 20L has room AND the 10L tank actually has
  //   water to pump (FLOAT 1 not triggered) — prevents dry-running

  setRelayState(1, compressorOn);
  setRelayState(2, pumpOn);   // UV Sterilizer follows the pump
  setRelayState(3, pumpOn);   // Fill/Main Pump

  if (g_treatedTankLow) {
    // Room freed up in the treated tank — SMS only fires once per cooldown,
    // pumpOn above already resumes automatically on its own.
    trySendSMS(lastSmsTreatedFull, "AWG-HEROES: Treated tank drawn down — transfer active again.");
  }
}

// Human-readable label for the dashboard's "Stage" text, derived
// live from the same conditions runAutomaticControl() uses — purely
// for display, doesn't drive any relay logic itself.
String autoStageLabel() {
  if (g_noWaterDetected) return "Held — no water detected";
  bool compressorOn = !g_rawTankFull;
  bool pumpOn        = g_pumpArmed && !g_treatedTankFull && !g_rawTankLow;
  if (compressorOn && pumpOn)  return "Filling 10L + Transferring to 20L";
  if (compressorOn && !pumpOn) {
    if (!g_pumpArmed)          return "Filling 10L tank";
    if (g_treatedTankFull)     return "Filling 10L (20L tank full — pump paused)";
    if (g_rawTankLow)          return "Filling 10L (tank ran dry — pump paused)";
    return "Filling 10L tank";
  }
  if (!compressorOn && pumpOn) return "Transferring to 20L tank";
  return "Both tanks full — idle";
}


// ════════════════════════════════════════════════════════════════
//  HELPERS
// ════════════════════════════════════════════════════════════════

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
    for (i = 0; i < iFilterLen - j - 1; i++)
      if (bTab[i] > bTab[i+1]) { bTemp=bTab[i]; bTab[i]=bTab[i+1]; bTab[i+1]=bTemp; }
  return (iFilterLen & 1) ? bTab[(iFilterLen-1)/2] : (bTab[iFilterLen/2]+bTab[iFilterLen/2-1])/2;
}

String getUptime() {
  unsigned long s = (millis() - bootMillis) / 1000;
  int h = s / 3600; s %= 3600;
  int m = s / 60;   s %= 60;
  char buf[12];
  sprintf(buf, "%02dh %02dm %02ds", h, m, (int)s);
  return String(buf);
}

// Magnus-Tetens approximation for dew point (°C), given temp (°C) and relative humidity (%)
float calcDewPoint(float tempC, float humidityPct) {
  const float a = 17.27f;
  const float b = 237.7f;
  float rh = humidityPct;
  if (rh < 1.0f) rh = 1.0f;       // avoid log(0)
  if (rh > 100.0f) rh = 100.0f;
  float alpha = ((a * tempC) / (b + tempC)) + log(rh / 100.0f);
  return (b * alpha) / (a - alpha);
}

// Returns 2 = Drinkable, 1 = Borderline (caution), 0 = Not drinkable
int getPotability() {
  bool phOk  = (g_ph >= 6.5f && g_ph <= 8.5f);
  bool tdsOk = (g_tds < 500.0f);

  if (phOk && tdsOk) return 2;
  if (g_tds > 1000.0f || g_ph < 5.5f || g_ph > 9.5f) return 0;
  return 1;
}

const char* potabilityLabel(int level) {
  if (level == 2) return "Drinkable";
  if (level == 1) return "Caution — borderline";
  return "Not drinkable";
}

const char* potabilityReason(int level) {
  bool phOk  = (g_ph >= 6.5f && g_ph <= 8.5f);
  bool tdsOk = (g_tds < 500.0f);
  if (level == 2) return "pH and TDS are both within safe drinking limits.";
  if (!phOk && !tdsOk) return "pH and TDS are both outside safe drinking limits.";
  if (!phOk) return "pH is outside the safe 6.5-8.5 range.";
  if (!tdsOk) return "TDS exceeds the 500 ppm safe drinking limit.";
  return "Readings are borderline — recommend re-testing.";
}

// Returns current local time as "HH:MM" using NTP-synced clock.
// If NTP hasn't synced yet, falls back to uptime-based "Hh MMm".
void getTimeLabel(char* out, size_t outLen) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 100)) {
    strftime(out, outLen, "%H:%M", &timeinfo);
  } else {
    unsigned long s = (millis() - bootMillis) / 1000;
    int h = s / 3600; s %= 3600;
    int m = s / 60;
    snprintf(out, outLen, "%02dh%02dm", h, m);
  }
}

void recordMinuteHistory() {
  HistoryPoint p;
  p.ph = g_ph; p.tds = g_tds; p.temp = g_temperature;
  p.hum = g_humidity; p.dewPoint = g_dewPoint;
  getTimeLabel(p.timeLabel, sizeof(p.timeLabel));
  minuteHistory[minuteHistIndex] = p;
  minuteHistIndex = (minuteHistIndex + 1) % MINUTE_POINTS;
  if (minuteHistCount < MINUTE_POINTS) minuteHistCount++;
}

void recordHourHistory() {
  HistoryPoint p;
  p.ph = g_ph; p.tds = g_tds; p.temp = g_temperature;
  p.hum = g_humidity; p.dewPoint = g_dewPoint;
  getTimeLabel(p.timeLabel, sizeof(p.timeLabel));
  hourHistory[hourHistIndex] = p;
  hourHistIndex = (hourHistIndex + 1) % HOUR_POINTS;
  if (hourHistCount < HOUR_POINTS) hourHistCount++;
}

void recordTrendHistory() {
  HistoryPoint p;
  p.ph = g_ph; p.tds = g_tds; p.temp = g_temperature;
  p.hum = g_humidity; p.dewPoint = g_dewPoint;
  getTimeLabel(p.timeLabel, sizeof(p.timeLabel));
  trendHistory[trendHistIndex] = p;
  trendHistIndex = (trendHistIndex + 1) % TREND_POINTS;
  if (trendHistCount < TREND_POINTS) trendHistCount++;
}

void smtpCallback(SMTP_Status status) {
  Serial.println(status.info());
}

bool sendAlertEmail(const String& subject, const String& bodyText) {
  smtp.debug(0);
  smtp.callback(smtpCallback);

  Session_Config config;
  config.server.host_name = SMTP_HOST;
  config.server.port      = SMTP_PORT;
  config.login.email       = SMTP_SENDER;
  config.login.password    = SMTP_PASS;
  config.login.user_domain = "";

  SMTP_Message message;
  message.sender.name    = "AWG-HEROES Dashboard";
  message.sender.email    = SMTP_SENDER;
  message.subject         = subject;
  message.addRecipient("AWG Owner", SMTP_RECIPIENT);
  message.text.content    = bodyText.c_str();
  message.text.charSet    = "utf-8";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

  if (!smtp.connect(&config)) {
    Serial.println("SMTP connect failed");
    return false;
  }
  if (!MailClient.sendMail(&smtp, &message)) {
    Serial.println("Email send failed: " + smtp.errorReason());
    return false;
  }
  smtp.closeSession();
  return true;
}

// Minimal application/x-www-form-urlencoded encoder for the SMS body text
String urlEncode(const String& s) {
  String out;
  char buf[4];
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else if (c == ' ') {
      out += '+';
    } else {
      sprintf(buf, "%%%02X", (unsigned char)c);
      out += buf;
    }
  }
  return out;
}

// Sends a text message via the Semaphore SMS API (HTTPS POST). No GSM
// module required — this rides on the ESP32's existing WiFi connection.
bool sendSMS(const String& message) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();   // skip TLS cert validation — fine for a hobby/capstone project
  HTTPClient https;

  if (!https.begin(client, "https://api.semaphore.co/api/v4/messages")) {
    Serial.println("SMS: unable to open HTTPS connection");
    return false;
  }
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "apikey=" + String(SMS_API_KEY) +
                "&number=" + String(SMS_RECIPIENT_NUMBER) +
                "&sendername=" + String(SMS_SENDER_NAME) +
                "&message=" + urlEncode(message);

  int code = https.POST(body);
  Serial.printf("SMS send response code: %d\n", code);
  https.end();
  return code > 0 && code < 300;
}

// Cooldown-gated SMS send — pass in the specific lastTime tracker for
// that alert type so different alerts don't share one cooldown clock.
void trySendSMS(unsigned long &lastTime, const String& message) {
  unsigned long now = millis();
  if (now - lastTime < SMS_COOLDOWN_MS) return;
  lastTime = now;
  sendSMS(message);
}

// Checks each alert condition against its own cooldown timer so the
// same alert type doesn't spam the inbox every 2s while it stays true.
void checkAndSendAlerts() {
  unsigned long now = millis();
  bool phOk  = (g_ph >= 6.5f && g_ph <= 8.5f);
  bool tdsOk = (g_tds < 500.0f);
  int  potability = getPotability();

  if (!phOk && (now - lastEmailPH >= EMAIL_COOLDOWN_MS)) {
    lastEmailPH = now;
    String body = "pH reading is out of the safe range (6.5-8.5).\n\nCurrent pH: " + String(g_ph, 2) +
                  "\nTemperature: " + String(g_temperature, 1) + " C\nTime since boot: " + getUptime();
    sendAlertEmail("AWG Alert: pH out of range", body);
    logAlert("pH out of range (" + String(g_ph, 2) + ")");
  }

  if (!tdsOk && (now - lastEmailTDS >= EMAIL_COOLDOWN_MS)) {
    lastEmailTDS = now;
    String body = "TDS reading exceeds the 500 ppm safe drinking limit.\n\nCurrent TDS: " + String(g_tds, 0) +
                  " ppm\nTime since boot: " + getUptime();
    sendAlertEmail("AWG Alert: High TDS", body);
    logAlert("High TDS (" + String(g_tds, 0) + " ppm)");
  }

  if (g_rawTankFull && (now - lastEmailRawFull >= EMAIL_COOLDOWN_MS)) {
    lastEmailRawFull = now;
    String body = "The Raw/Unfiltered tank (10L) float switch has detected that it is full.\nReady to be transferred/filtered into the treated tank.\n\nTime since boot: " + getUptime();
    sendAlertEmail("AWG Notice: Raw Tank Full (10L)", body);
    logAlert("Raw tank (10L) FULL");
  }

  if (g_treatedTankFull && (now - lastEmailTreatedFull >= EMAIL_COOLDOWN_MS)) {
    lastEmailTreatedFull = now;
    String body = "The Treated/Filtered tank (20L) float switch has detected that it is full.\nThe fill pump has been automatically cut off.\n\nTime since boot: " + getUptime();
    sendAlertEmail("AWG Notice: Treated Tank Full (20L)", body);
    logAlert("Treated tank (20L) FULL");
  }

  if (potability == 0 && (now - lastEmailPotability >= EMAIL_COOLDOWN_MS)) {
    lastEmailPotability = now;
    String body = String("Water is NOT drinkable.\n\nReason: ") + potabilityReason(potability) +
                  "\npH: " + String(g_ph, 2) + "\nTDS: " + String(g_tds, 0) +
                  " ppm\nTime since boot: " + getUptime();
    sendAlertEmail("AWG Alert: Water Not Drinkable", body);
    logAlert(String("Water NOT drinkable - ") + potabilityReason(potability));
  }
}


void broadcastSensorData() {
  int potability = getPotability();

  // Hourly history needs more room; size doc generously
  StaticJsonDocument<2048> doc;
  doc["ph"]              = g_ph;
  doc["tds"]             = g_tds;
  doc["temp"]            = g_temperature;
  doc["hum"]             = g_humidity;
  doc["dewPoint"]        = g_dewPoint;
  doc["rawTankFull"]     = g_rawTankFull;      // FLOAT 2
  doc["rawTankLow"]      = g_rawTankLow;       // FLOAT 1
  doc["treatedTankFull"] = g_treatedTankFull;  // FLOAT 3
  doc["treatedTankLow"]  = g_treatedTankLow;   // FLOAT 4
  doc["noWater"]         = g_noWaterDetected;
  doc["mode"]            = (g_systemMode == MODE_AUTOMATIC) ? "auto" : "manual";
  doc["autoStage"]       = (g_systemMode == MODE_AUTOMATIC) ? autoStageLabel() : "";
  doc["heap"]            = ESP.getFreeHeap();
  doc["rssi"]            = WiFi.RSSI();
  doc["uptime"]          = getUptime();
  doc["hourCount"]       = hourHistCount;
  doc["potability"]      = potability;
  doc["potabilityLabel"]  = potabilityLabel(potability);
  doc["potabilityReason"] = potabilityReason(potability);

  JsonArray relays = doc.createNestedArray("relays");
  for (int i = 0; i < 4; i++) relays.add(relayState[i]);

  JsonArray hourly = doc.createNestedArray("hourly");
  int count = hourHistCount;
  int startIdx = (hourHistIndex - count + HOUR_POINTS) % HOUR_POINTS;
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % HOUR_POINTS;
    JsonObject o = hourly.createNestedObject();
    o["t"]    = hourHistory[idx].timeLabel;
    o["ph"]   = hourHistory[idx].ph;
    o["tds"]  = hourHistory[idx].tds;
    o["temp"] = hourHistory[idx].temp;
    o["hum"]  = hourHistory[idx].hum;
    o["dew"]  = hourHistory[idx].dewPoint;
  }

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void*, uint8_t*, size_t) {
  // nothing needed — server pushes data
}

// ════════════════════════════════════════════════════════════════
//  READ SENSORS
// ════════════════════════════════════════════════════════════════

void readPH() {
  float voltage = analogRead(PH_PIN) / 4096.0f * 3300.0f;
  float raw     = ph.readPH(voltage, g_temperature);
  g_ph          = raw - PH_OFFSET;
  if (g_ph < 0) g_ph = 0;
}

void readTDS() {
  analogBuffer[analogBufferIndex++] = analogRead(TDS_PIN);
  if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;

  for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];
  float avgV = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 4095.0f;
  float comp = 1.0f + 0.02f * (g_temperature - 25.0f);
  float cv   = avgV / comp;
  g_tds      = (133.42f*cv*cv*cv - 255.86f*cv*cv + 857.39f*cv) * 0.5f + TDS_OFFSET;
  if (g_tds < 0) g_tds = 0;
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) g_humidity    = h;
  if (!isnan(t)) g_temperature = t;
  g_dewPoint = calcDewPoint(g_temperature, g_humidity);
  g_condGap  = g_temperature - g_dewPoint;
}

// Debounces one float switch reading against its own timer.
void updateFloatSensor(FloatSensor &fs) {
  int raw = digitalRead(fs.pin);
  bool current = (raw == fs.activeState);
  if (current != fs.lastRaw) {
    fs.lastChangeTime = millis();
  }
  if ((millis() - fs.lastChangeTime) > FLOAT_DEBOUNCE_MS) {
    fs.debounced = current;
  }
  fs.lastRaw = current;
}

// Reads and debounces all 4 float switches (2 per tank), then syncs
// the readable g_* globals used everywhere else in the sketch.
void readFloatSensors() {
  bool prevRawFull     = g_rawTankFull;
  bool prevRawLow       = g_rawTankLow;
  bool prevTreatedFull = g_treatedTankFull;
  bool prevTreatedLow   = g_treatedTankLow;

  updateFloatSensor(fsRawHigh);
  updateFloatSensor(fsRawLow);
  updateFloatSensor(fsTreatedHigh);
  updateFloatSensor(fsTreatedLow);

  g_rawTankFull     = fsRawHigh.debounced;
  g_rawTankLow      = fsRawLow.debounced;
  g_treatedTankFull = fsTreatedHigh.debounced;
  g_treatedTankLow  = fsTreatedLow.debounced;

  if (g_rawTankFull != prevRawFull)         Serial.println(g_rawTankFull ? "FLOAT 2 (10L HIGH): FULL" : "FLOAT 2 (10L HIGH): not full");
  if (g_rawTankLow != prevRawLow)           Serial.println(g_rawTankLow ? "FLOAT 1 (10L LOW): EMPTY" : "FLOAT 1 (10L LOW): has water");
  if (g_treatedTankFull != prevTreatedFull) Serial.println(g_treatedTankFull ? "FLOAT 3 (20L HIGH): FULL" : "FLOAT 3 (20L HIGH): not full");
  if (g_treatedTankLow != prevTreatedLow)   Serial.println(g_treatedTankLow ? "FLOAT 4 (20L LOW): EMPTY" : "FLOAT 4 (20L LOW): has water");
}

// ════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ════════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════════
//  DISPLAY PANEL — local on-device status screens (read-only)
// ════════════════════════════════════════════════════════════════
// 8 screens, cycled with two physical push-buttons (FORWARD/BACK).
// Display-only — mode/relay/faucet control still happens from the
// web dashboard. Order: HOME -> FLOATS -> LIVE -> RELAYS -> YIELD ->
// TREND(pH/TDS) -> TREND(Temp/Hum/Dew) -> ALERTS -> (wraps to HOME)

const char* modeLabel() { return (g_systemMode == MODE_AUTOMATIC) ? "AUTO" : "MANUAL"; }

const char* SCREEN_TITLES[SCR_COUNT] = {
  "SYSTEM STATUS", "FLOAT SWITCHES & TANKS", "LIVE READINGS",
  "RELAY STATUS", "WATER YIELD POTENTIAL", "TRENDS: pH & TDS",
  "TRENDS: TEMP / HUMIDITY / DEW", "ALERT LOGS"
};

// Draws the top title bar + "n/8" page indicator — only clears the
// whole screen when actually switching pages (fullRedraw), so the
// once-a-second refresh doesn't flash/flicker the whole panel.
void drawScreenHeader(bool fullRedraw) {
  if (fullRedraw) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 20);
    tft.print(SCREEN_TITLES[g_panelScreen]);

    char pageStr[8];
    snprintf(pageStr, sizeof(pageStr), "%d/%d", g_panelScreen + 1, (int)SCR_COUNT);
    int16_t pw = tft.textWidth(pageStr);
    tft.setCursor(480 - pw - 10, 20);
    tft.print(pageStr);

    tft.drawFastHLine(0, 34, 480, TFT_DARKGREY);
  }
  drawStatusDots();   // redrawn every call, even on periodic (non-full) refresh
}

// 5-dot system status strip, top of every screen: system power, water
// harvesting (compressor), 10L/20L tank full, and active alerts.
void drawStatusDots() {
  struct DotDef { const char* label; uint16_t color; };
  bool harvesting = relayState[0];   // relay 1 = Compressor
  DotDef dots[5] = {
    { "PWR",  TFT_GREEN },
    { "HARV", harvesting ? TFT_YELLOW : TFT_DARKGREY },
    { "T10",  g_rawTankFull ? TFT_CYAN : TFT_DARKGREY },
    { "T20",  g_treatedTankFull ? TFT_CYAN : TFT_DARKGREY },
    { "ALRT", g_alertLogCount > 0 ? TFT_RED : TFT_DARKGREY },
  };
  int xs[5] = { 8, 100, 192, 284, 376 };
  for (int i = 0; i < 5; i++) {
    tft.fillCircle(xs[i] + 5, 8, 5, dots[i].color);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(xs[i] + 16, 4);
    tft.print(dots[i].label);
  }
}

// Bottom status bar showing the SELECT button's current cursor item
// and its live state. Redrawn every call, even on periodic refresh.
void drawSelectStatusBar() {
  tft.fillRect(0, 306, 480, 14, TFT_BLACK);
  tft.drawFastHLine(0, 304, 480, TFT_DARKGREY);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(8, 308);
  tft.print("SELECT: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print(SELECT_ITEM_NAMES[g_selectIndex]);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("  =  ");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  if (g_selectIndex == 0) {
    tft.print(modeLabel());
  } else {
    int id = SELECT_RELAY_ID[g_selectIndex];
    tft.print(relayState[id - 1] ? "ON" : "OFF");
  }
}

// Generic labeled status chip — used across several screens.
void drawStatusChip(Rect r, const char* label, const char* value, uint16_t fillColor) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, fillColor);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, fillColor);
  tft.setTextSize(1);
  tft.setCursor(r.x + 8, r.y + 6);
  tft.print(label);
  tft.setTextSize(2);
  int16_t vw = tft.textWidth(value);
  tft.setCursor(r.x + (r.w - vw) / 2, r.y + r.h - 26);
  tft.print(value);
}

// ── SCR_HOME — system status + system mode ────────────────────────
void drawHomeScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  bool isAuto = (g_systemMode == MODE_AUTOMATIC);
  drawStatusChip({ 20, 45, 220, 70 }, "SYSTEM MODE", modeLabel(), isAuto ? TFT_ORANGE : TFT_GREEN);

  int potability = getPotability();
  uint16_t potColor = (potability == 2) ? TFT_GREEN : (potability == 1) ? TFT_YELLOW : TFT_RED;
  drawStatusChip({ 250, 45, 210, 70 }, "WATER STATUS", potabilityLabel(potability), potColor);

  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(20, 130);
  tft.print(isAuto ? autoStageLabel() : "Manual control active");

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(20, 175);
  tft.printf("Uptime: %s", getUptime().c_str());

  tft.setCursor(20, 195);
  if (WiFi.status() == WL_CONNECTED) {
    tft.printf("WiFi: connected  IP %s  RSSI %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    tft.print("WiFi: not connected");
  }

  tft.setCursor(20, 215);
  tft.printf("Free heap: %u bytes", ESP.getFreeHeap());

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(20, 245);
  tft.print(g_noWaterDetected ? "No-water guard: ACTIVE (sensors reading dry)" : "No-water guard: off");
}

// ── SCR_FLOATS — float switches + tank status ──────────────────────
void drawFloatsScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  drawStatusChip({ 20,  45, 210, 70 }, "FLOAT 1 - Raw Low",     g_rawTankLow     ? "EMPTY" : "OK",   g_rawTankLow     ? TFT_RED : TFT_GREEN);
  drawStatusChip({ 250, 45, 210, 70 }, "FLOAT 2 - Raw Full",    g_rawTankFull    ? "FULL"  : "OK",   g_rawTankFull    ? TFT_YELLOW : TFT_GREEN);
  drawStatusChip({ 20, 125, 210, 70 }, "FLOAT 3 - Treated Full", g_treatedTankFull ? "FULL"  : "OK", g_treatedTankFull ? TFT_YELLOW : TFT_GREEN);
  drawStatusChip({ 250, 125, 210, 70 }, "FLOAT 4 - Treated Low", g_treatedTankLow ? "EMPTY" : "OK",  g_treatedTankLow ? TFT_RED : TFT_GREEN);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(20, 215);
  tft.printf("Raw tank (10L):     %s",
             g_rawTankFull ? "FULL" : (g_rawTankLow ? "EMPTY" : "OK"));
  tft.setCursor(20, 250);
  tft.printf("Treated tank (20L): %s",
             g_treatedTankFull ? "FULL" : (g_treatedTankLow ? "EMPTY" : "OK"));
}

// ── SCR_LIVE — live sensor readings ────────────────────────────────
void drawLiveScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(20, 50);  tft.printf("pH:           %.2f", g_ph);
  tft.setCursor(20, 80);  tft.printf("TDS:          %.0f ppm", g_tds);
  tft.setCursor(20, 110); tft.printf("Temperature:  %.1f C", g_temperature);
  tft.setCursor(20, 140); tft.printf("Humidity:     %.0f %%", g_humidity);
  tft.setCursor(20, 170); tft.printf("Dew Point:    %.1f C", g_dewPoint);

  int potability = getPotability();
  uint16_t potColor = (potability == 2) ? TFT_GREEN : (potability == 1) ? TFT_YELLOW : TFT_RED;
  tft.setTextColor(potColor, TFT_BLACK);
  tft.setCursor(20, 215);
  tft.printf("Potability: %s", potabilityLabel(potability));
  tft.setTextSize(1);
  tft.setCursor(20, 245);
  tft.print(potabilityReason(potability));
}

// ── SCR_RELAYS — relay status (read-only) ──────────────────────────
void drawRelaysScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  Rect boxes[4] = { { 20, 45, 210, 100 }, { 250, 45, 210, 100 }, { 20, 165, 210, 100 }, { 250, 165, 210, 100 } };
  bool autoMode = (g_systemMode == MODE_AUTOMATIC);

  for (int i = 0; i < 4; i++) {
    bool on = relayState[i];
    bool blocked = (autoMode && i != 3);   // relay 4 (index 3) is always manual
    uint16_t fill = blocked ? TFT_DARKGREY : (on ? TFT_GREEN : TFT_RED);
    drawStatusChip(boxes[i], relayNames[i], on ? "ON" : "OFF", fill);
  }

  tft.setTextSize(1);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(20, 280);
  if (autoMode) tft.print("AUTO mode is driving relays 1-3. Faucet Pump always stays manual.");
}

// ── SCR_YIELD — water yield potential ──────────────────────────────
struct YieldTier { const char* label; uint16_t color; const char* desc; };

YieldTier humidityYieldTier(float hum) {
  if (hum < 40) return { "POOR", TFT_RED,    "Below 40% humidity rarely condenses enough water to be worthwhile." };
  if (hum < 60) return { "FAIR", TFT_ORANGE, "Workable but inefficient. Expect lower output than humid conditions." };
  if (hum < 80) return { "GOOD", TFT_GREEN,  "Good conditions for an AWG unit. Expect steady, reliable output." };
  return          { "EXCELLENT", TFT_CYAN,   "Ideal humidity for atmospheric water generation. Expect max output." };
}

void drawYieldScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  YieldTier hy = humidityYieldTier(g_humidity);
  char humVal[16];
  snprintf(humVal, sizeof(humVal), "%.0f%% - %s", g_humidity, hy.label);
  drawStatusChip({ 20, 45, 440, 70 }, "HUMIDITY YIELD RATING", humVal, hy.color);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(20, 122);
  tft.print(hy.desc);

  float condGap = g_temperature - g_dewPoint;
  uint16_t gapColor = (condGap < 3) ? TFT_CYAN : (condGap < 7) ? TFT_GREEN : (condGap < 12) ? TFT_ORANGE : TFT_RED;
  char gapVal[24];
  snprintf(gapVal, sizeof(gapVal), "%.1f C gap", condGap);
  drawStatusChip({ 20, 150, 440, 70 }, "CONDENSATION POTENTIAL", gapVal, gapColor);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(20, 227);
  tft.print("Smaller gap between air temp and dew point = easier condensation = better yield.");
}

// ── Shared trend-chart drawing helper ──────────────────────────────
void drawLineSeries(int x0, int y0, int w, int h, float* vals, int count, uint16_t color) {
  if (count < 2) return;
  float vmin = vals[0], vmax = vals[0];
  for (int i = 1; i < count; i++) {
    if (vals[i] < vmin) vmin = vals[i];
    if (vals[i] > vmax) vmax = vals[i];
  }
  if (vmax - vmin < 0.5f) { vmax += 1; vmin -= 1; }

  int prevX = 0, prevY = 0;
  for (int i = 0; i < count; i++) {
    int px = x0 + (int)((float)i / (count - 1) * w);
    int py = y0 + h - (int)((vals[i] - vmin) / (vmax - vmin) * h);
    if (i > 0) tft.drawLine(prevX, prevY, px, py, color);
    prevX = px; prevY = py;
  }
}

// Pulls trendHistory[] (1 point/minute, last 60 min) into chronological
// float arrays for the LCD's trend charts.
int loadTrendField(float* out, int field) {
  // field: 0=ph, 1=tds, 2=temp, 3=hum, 4=dew
  int count = trendHistCount;
  int startIdx = (trendHistIndex - count + TREND_POINTS) % TREND_POINTS;
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % TREND_POINTS;
    switch (field) {
      case 0: out[i] = trendHistory[idx].ph; break;
      case 1: out[i] = trendHistory[idx].tds; break;
      case 2: out[i] = trendHistory[idx].temp; break;
      case 3: out[i] = trendHistory[idx].hum; break;
      default: out[i] = trendHistory[idx].dewPoint; break;
    }
  }
  return count;
}

// ── SCR_TREND_PHTDS — trends: pH & TDS ─────────────────────────────
void drawTrendPhTdsScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  float vals[TREND_POINTS];
  int count = loadTrendField(vals, 0);

  if (count < 2) {
    if (!fullRedraw) return;
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(20, 60);
    tft.print("Not enough minute history yet -- check back in a minute or two.");
    return;
  }

  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(20, 45);
  tft.print("pH (last 60 min)      ");
  tft.fillRect(20, 60, 440, 90, TFT_BLACK);
  drawLineSeries(20, 60, 440, 90, vals, count, TFT_CYAN);
  tft.drawRect(20, 60, 440, 90, TFT_DARKGREY);

  count = loadTrendField(vals, 1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(20, 165);
  tft.print("TDS ppm (last 60 min) ");
  tft.fillRect(20, 180, 440, 90, TFT_BLACK);
  drawLineSeries(20, 180, 440, 90, vals, count, TFT_YELLOW);
  tft.drawRect(20, 180, 440, 90, TFT_DARKGREY);
}

// ── SCR_TREND_TEMPHUM — trends: temp, humidity, dew point ──────────
void drawTrendTempHumScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);

  float vals[TREND_POINTS];
  int count = loadTrendField(vals, 2);

  if (count < 2) {
    if (!fullRedraw) return;
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(20, 60);
    tft.print("Not enough minute history yet -- check back in a minute or two.");
    return;
  }

  tft.setTextSize(1);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(20, 45);
  tft.print("Temp (orange) + Dew Point (cyan) -- C, last 60 min");
  tft.fillRect(20, 60, 440, 90, TFT_BLACK);
  drawLineSeries(20, 60, 440, 90, vals, count, TFT_ORANGE);
  count = loadTrendField(vals, 4);
  drawLineSeries(20, 60, 440, 90, vals, count, TFT_CYAN);
  tft.drawRect(20, 60, 440, 90, TFT_DARKGREY);

  count = loadTrendField(vals, 3);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(20, 165);
  tft.print("Humidity % (last 60 min)   ");
  tft.fillRect(20, 180, 440, 90, TFT_BLACK);
  drawLineSeries(20, 180, 440, 90, vals, count, TFT_GREEN);
  tft.drawRect(20, 180, 440, 90, TFT_DARKGREY);
}

// ── SCR_ALERTS — alert logs ─────────────────────────────────────────
void logAlert(const String& msg) {
  g_alertLog[g_alertLogIndex].time = getUptime();
  g_alertLog[g_alertLogIndex].msg  = msg;
  g_alertLogIndex = (g_alertLogIndex + 1) % ALERT_LOG_SIZE;
  if (g_alertLogCount < ALERT_LOG_SIZE) g_alertLogCount++;
}

void drawAlertsScreen(bool fullRedraw) {
  drawScreenHeader(fullRedraw);
  if (!fullRedraw) return;   // only changes when a new alert fires

  if (g_alertLogCount == 0) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(20, 60);
    tft.print("No alerts logged yet.");
    return;
  }

  tft.setTextSize(1);
  int y = 45;
  int startIdx = (g_alertLogIndex - g_alertLogCount + ALERT_LOG_SIZE) % ALERT_LOG_SIZE;
  for (int i = g_alertLogCount - 1; i >= 0; i--) {
    int idx = (startIdx + i) % ALERT_LOG_SIZE;
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(20, y);
    tft.print(g_alertLog[idx].time);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(140, y);
    tft.print(g_alertLog[idx].msg);
    y += 33;
    if (y > 290) break;
  }
}

void drawPanel(bool fullRedraw) {
  switch (g_panelScreen) {
    case SCR_HOME:          drawHomeScreen(fullRedraw);         break;
    case SCR_FLOATS:        drawFloatsScreen(fullRedraw);       break;
    case SCR_LIVE:           drawLiveScreen(fullRedraw);         break;
    case SCR_RELAYS:        drawRelaysScreen(fullRedraw);       break;
    case SCR_YIELD:         drawYieldScreen(fullRedraw);        break;
    case SCR_TREND_PHTDS:   drawTrendPhTdsScreen(fullRedraw);   break;
    case SCR_TREND_TEMPHUM: drawTrendTempHumScreen(fullRedraw); break;
    case SCR_ALERTS:        drawAlertsScreen(fullRedraw);       break;
    default: break;
  }
  drawSelectStatusBar();   // always shown, every screen, every refresh
}

void setupDisplayPanel() {
  pinMode(BTN_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BTN_BACK_PIN, INPUT_PULLUP);
  pinMode(BTN_SELECT_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(1);   // landscape, 480x320
  tft.fillScreen(TFT_BLACK);

  drawPanel(true);
}

// Reads the two nav buttons (active LOW, internal pull-ups) and
// switches screens on a clean press, with simple debounce.
void handlePanelButtons() {
  static bool lastFwd = HIGH, lastBack = HIGH;
  static unsigned long lastPressMs = 0;
  unsigned long now = millis();

  bool fwd  = digitalRead(BTN_FORWARD_PIN);
  bool back = digitalRead(BTN_BACK_PIN);

  if (fwd == LOW && lastFwd == HIGH && now - lastPressMs > 200) {
    g_panelScreen = (PanelScreen)((g_panelScreen + 1) % SCR_COUNT);
    drawPanel(true);
    lastPressMs = now;
  } else if (back == LOW && lastBack == HIGH && now - lastPressMs > 200) {
    g_panelScreen = (PanelScreen)((g_panelScreen + SCR_COUNT - 1) % SCR_COUNT);
    drawPanel(true);
    lastPressMs = now;
  }

  lastFwd = fwd;
  lastBack = back;
}

// Toggles whichever item the SELECT cursor is currently on. For MODE,
// flips MANUAL<->AUTOMATIC. For a relay, mirrors the web dashboard's
// /relay route rules exactly: blocked in Auto (except the Faucet
// Pump), blocked with no water confirmed, blocked turning the Fill
// Pump on while the treated tank's already full.
void selectToggleCurrent() {
  if (g_selectIndex == 0) {
    g_systemMode = (g_systemMode == MODE_AUTOMATIC) ? MODE_MANUAL : MODE_AUTOMATIC;
    if (g_systemMode == MODE_AUTOMATIC) g_pumpArmed = false;
    return;
  }
  int id = SELECT_RELAY_ID[g_selectIndex];
  if (g_systemMode == MODE_AUTOMATIC && id != 4) return;   // Auto is driving relays 1-3
  bool turningOn = !relayState[id - 1];
  if (id != 4 && turningOn && g_noWaterDetected) return;
  if (id == 3 && turningOn && g_treatedTankFull) return;
  setRelayState(id, turningOn);
}

// Single press = move the cursor to the next item. A second press
// arriving within SELECT_DOUBLE_TAP_MS of the first = toggle whatever
// item the cursor lands on. Either way, the bottom status bar updates
// immediately so you always see the effect of a press right away.
void handleSelectButton() {
  static bool lastState = HIGH;
  static unsigned long lastPressMs = 0;
  unsigned long now = millis();

  bool state = digitalRead(BTN_SELECT_PIN);
  if (state == LOW && lastState == HIGH) {
    if (now - lastPressMs < SELECT_DOUBLE_TAP_MS) {
      selectToggleCurrent();
    } else {
      g_selectIndex = (g_selectIndex + 1) % SELECT_ITEMS_COUNT;
    }
    lastPressMs = now;
    drawPanel(false);
  }
  lastState = state;
}



// ═══════════════════════════════════════════════════════════════
// SUPABASE CLOUD UPLOAD
// ═══════════════════════════════════════════════════════════════

void uploadToSupabase() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[SUPABASE] WiFi disconnected - upload skipped");
    return;
  }

  WiFiClientSecure client;

  // Initial prototype/testing connection.
  // For a production deployment, use certificate validation.
  client.setInsecure();

  HTTPClient http;

  String url = String(SUPABASE_URL) + "/rest/v1/awg_sensor_data";

  Serial.println();
  Serial.println("[SUPABASE] Uploading AWG-HEROES data...");

  if (!http.begin(client, url)) {
    Serial.println("[SUPABASE] HTTP begin failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_PUBLISHABLE_KEY);
  http.addHeader("Prefer", "return=minimal");

  // Prevent invalid JSON if a floating-point sensor ever becomes NaN/Inf.
  float phValue   = isfinite(g_ph)          ? g_ph          : 0.0f;
  float tdsValue  = isfinite(g_tds)         ? g_tds         : 0.0f;
  float tempValue = isfinite(g_temperature) ? g_temperature : 0.0f;
  float humValue  = isfinite(g_humidity)    ? g_humidity    : 0.0f;
  float dewValue  = isfinite(g_dewPoint)    ? g_dewPoint    : 0.0f;

  String status = autoStageLabel();
  status.replace("\\", "\\\\");
  status.replace("\"", "\\\"");

  String json = "{";

  json += "\"ph\":" + String(phValue, 2);
  json += ",\"tds\":" + String(tdsValue, 2);
  json += ",\"temperature\":" + String(tempValue, 2);
  json += ",\"humidity\":" + String(humValue, 2);
  json += ",\"dew_point\":" + String(dewValue, 2);

  json += ",\"raw_tank_low\":";
  json += g_rawTankLow ? "true" : "false";

  json += ",\"raw_tank_full\":";
  json += g_rawTankFull ? "true" : "false";

  json += ",\"treated_tank_low\":";
  json += g_treatedTankLow ? "true" : "false";

  json += ",\"treated_tank_full\":";
  json += g_treatedTankFull ? "true" : "false";

  json += ",\"relay_1\":";
  json += relayState[0] ? "true" : "false";

  json += ",\"relay_2\":";
  json += relayState[1] ? "true" : "false";

  json += ",\"relay_3\":";
  json += relayState[2] ? "true" : "false";

  json += ",\"relay_4\":";
  json += relayState[3] ? "true" : "false";

  json += ",\"system_mode\":\"";
  json += (g_systemMode == MODE_AUTOMATIC) ? "AUTO" : "MANUAL";
  json += "\"";

  json += ",\"water_status\":\"";
  json += status;
  json += "\"";

  json += ",\"no_water_detected\":";
  json += g_noWaterDetected ? "true" : "false";

  json += "}";

  Serial.println("[SUPABASE] JSON:");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("[SUPABASE] HTTP response: ");
  Serial.println(httpCode);

  if (httpCode >= 200 && httpCode < 300) {
    Serial.println("[SUPABASE] SUCCESS - data uploaded!");
  } else if (httpCode > 0) {
    Serial.println("[SUPABASE] Server response:");
    Serial.println(http.getString());
  } else {
    Serial.print("[SUPABASE] Connection error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

void setup() {
  Serial.begin(115200);

  EEPROM.begin(32);
  dht.begin();
  ph.begin();

  pinMode(FLOAT_RAW_HIGH_PIN, INPUT_PULLUP);
  pinMode(FLOAT_RAW_LOW_PIN, INPUT_PULLUP);
  pinMode(FLOAT_TREATED_HIGH_PIN, INPUT_PULLUP);
  pinMode(FLOAT_TREATED_LOW_PIN, INPUT_PULLUP);
  setupRelays();
  setupDisplayPanel();

  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  configTime(GMT_OFFSET, DST_OFFSET, NTP_SERVER);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });

  // Manual relay control — GET /relay?id=1..4&state=on|off
  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("id") || !request->hasParam("state")) {
      request->send(400, "application/json", "{\"error\":\"missing id or state\"}");
      return;
    }

    int id = request->getParam("id")->value().toInt();
    String state = request->getParam("state")->value();

    if (id < 1 || id > 4) {
      request->send(400, "application/json", "{\"error\":\"invalid relay id\"}");
      return;
    }

    bool on = (state == "on");

    // Automatic mode drives relays 1-3 itself — refuse manual changes to
    // those while it's active. Relay 4 (Faucet Pump) always stays manual.
    if (g_systemMode == MODE_AUTOMATIC && id != 4) {
      StaticJsonDocument<128> doc;
      doc["id"] = id;
      doc["state"] = relayState[id - 1] ? "on" : "off";
      doc["blocked"] = true;
      String out;
      serializeJson(doc, out);
      request->send(200, "application/json", out);
      return;
    }

    // Safety: refuse to turn Compressor/UV/Fill Pump ON with no water confirmed present
    if (id != 4 && on && g_noWaterDetected) {
      StaticJsonDocument<128> doc;
      doc["id"] = id;
      doc["state"] = "off";
      doc["blocked"] = true;
      String out;
      serializeJson(doc, out);
      request->send(200, "application/json", out);
      return;
    }

    // Safety: refuse to turn the Fill Pump ON while the treated tank is already full
    if (id == 3 && on && g_treatedTankFull) {
      StaticJsonDocument<128> doc;
      doc["id"] = id;
      doc["state"] = "off";
      doc["blocked"] = true;
      String out;
      serializeJson(doc, out);
      request->send(200, "application/json", out);
      return;
    }

    setRelayState(id, on);

    StaticJsonDocument<128> doc;
    doc["id"] = id;
    doc["state"] = on ? "on" : "off";
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/relay/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<128> doc;
    JsonArray arr = doc.createNestedArray("relays");
    for (int i = 0; i < 4; i++) arr.add(relayState[i]);
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // Switch between MANUAL and AUTOMATIC — GET /mode?mode=manual|auto
  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("mode")) {
      request->send(400, "application/json", "{\"error\":\"missing mode\"}");
      return;
    }
    String m = request->getParam("mode")->value();
    if (m == "auto") {
      g_systemMode = MODE_AUTOMATIC;
      g_pumpArmed  = false;   // always (re)start the cycle from the top
    } else {
      g_systemMode = MODE_MANUAL;       // relays keep whatever state they were in
    }
    StaticJsonDocument<64> doc;
    doc["mode"] = (g_systemMode == MODE_AUTOMATIC) ? "auto" : "manual";
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // Master stop — kills all 4 relays and forces Manual mode. GET /stopall
  server.on("/stopall", HTTP_GET, [](AsyncWebServerRequest* request) {
    for (int i = 1; i <= 4; i++) setRelayState(i, false);
    g_systemMode = MODE_MANUAL;
    g_pumpArmed  = false;
    request->send(200, "application/json", "{\"stopped\":true}");
  });

  server.begin();
  bootMillis = millis();
  Serial.println("AWG-HEROES Dashboard ready!");
}

void loop() {
  static unsigned long lastSensor = 0;
  static unsigned long lastTDS    = 0;
  static unsigned long lastBcast  = 0;
  static unsigned long lastSupabase = 0;

  unsigned long now = millis();

  // Both float switches are debounced on every loop pass (cheap digitalRead)
  readFloatSensors();
  updateWaterDetectionGuard();
  enforceFillPumpCutoff();
  runAutomaticControl();

  handlePanelButtons();
  handleSelectButton();
  if (now - g_lastPanelRefresh >= 1000UL) {
    g_lastPanelRefresh = now;
    drawPanel(false);
  }

  // DHT + pH every 2s
  if (now - lastSensor >= 2000UL) {
    lastSensor = now;
    readDHT();
    readPH();
    checkAndSendAlerts();
  }

  // TDS every 40ms (builds median buffer)
  if (now - lastTDS >= 40UL) {
    lastTDS = now;
    readTDS();
  }

  // Minute-resolution sample for charts (every 2s matches broadcast rate)
  if (now - lastMinuteSample >= 2000UL) {
    lastMinuteSample = now;
    recordMinuteHistory();
  }

  // Hourly snapshot (every 1 hour = 3600000 ms)
  if (now - lastHourSample >= 3600000UL) {
    lastHourSample = now;
    recordHourHistory();
  }

  // Minute-level snapshot for the LCD trend screens (every 60000 ms)
  static unsigned long lastTrendSample = 0;
  if (now - lastTrendSample >= 60000UL) {
    lastTrendSample = now;
    recordTrendHistory();
  }

  // Broadcast to WebSocket every 2s
  if (now - lastBcast >= 2000UL) {
    lastBcast = now;
    broadcastSensorData();
    ws.cleanupClients();
  }

  // Upload sensor/status data to Supabase every 10 seconds.
  // This is intentionally slower than the local dashboard updates.
  if (now - lastSupabase >= SUPABASE_UPLOAD_INTERVAL) {
    lastSupabase = now;
    uploadToSupabase();
  }
}
