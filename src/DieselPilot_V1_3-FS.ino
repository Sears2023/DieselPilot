/*
 * ═══════════════════════════════════════════════════════════════════════════
 *                        DIESEL PILOT WEB + ERROR CODES
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * Full-featured ESP32 D1 Mini - Actual Board Pinout controller for Chinese diesel heaters
 * 
 * Features:
 * - WiFi STA mode with AP recovery fallback
 * - Web GUI (dark theme)
 * - OLED SH1106 display (IP + status)
 * - Auto/Manual pairing
 * - Real-time heater control
 * - MQTT integration (Home Assistant ready)
 * - ERROR CODE DECODING (BYTE[7])
 * 
 * Hardware:
 * - ESP32
 * - CC1101 @ 433.937 MHz
 * - SH1106 OLED (I2C)
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 *                      Version: V2.2 - CLEANED
 * ═══════════════════════════════════════════════════════════════════════════
 * Changes:
 * - Added error code decoding from BYTE[7]
 * - Error display in web interface
 * - Error code published to MQTT
 * - Error history tracking (last 10 errors)
 * - Pre-Heat weekly scheduling with NTP time synchronisation
 */


#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ArduinoOTA.h>
#include <time.h>
#include "esp_sntp.h"

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE CONFIG
// ═══════════════════════════════════════════════════════════════════════════

// CC1101 Pins - ESP32 D1 Mini / WROOM
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SS    5
#define PIN_GDO2  26

// I2C Pins (OLED) - ESP32 D1 Mini
#define PIN_SDA   21
#define PIN_SCL   22

// Physical control buttons
// Wire each button between the GPIO pin and GND.
// Internal pull-ups are used, so no external resistors are required.
#define BUTTON_POWER  16
#define BUTTON_UP     17
#define BUTTON_DOWN   4
#define BUTTON_MODE   25

// Button debounce time
#define BUTTON_DEBOUNCE_MS 50

// OLED Configuration
#define USE_OLED  true

// Commands
#define CMD_WAKEUP 0x23
#define CMD_MODE   0x24
#define CMD_POWER  0x2B
#define CMD_UP     0x3C
#define CMD_DOWN   0x3E

// Heater States
#define STATE_OFF            0
#define STATE_STARTUP        1
#define STATE_WARMING        2
#define STATE_WARMING_WAIT   3
#define STATE_PRE_RUN        4
#define STATE_RUNNING        5
#define STATE_SHUTDOWN       6
#define STATE_SHUTTING_DOWN  7
#define STATE_COOLING        8

// Error Codes (BYTE[7])
#define ERR_NONE           0x00  // ✅ NO ERROR
#define ERR_ON             0x01  // ⚡ ON - Starting
#define ERR_UNDERVOLTAGE   0x02  // 🔋 UNDERVOLTAGE
#define ERR_OVERVOLTAGE    0x03  // ⚡ OVERVOLTAGE
#define ERR_SPARK_PLUG     0x04  // 🔌 SPARK PLUG ERROR
#define ERR_OIL_PUMP       0x05  // 🛢️ OIL PUMP ERROR
#define ERR_OVERHEAT       0x06  // 🌡️ OVERHEAT ERROR
#define ERR_MOTOR          0x07  // ⚙️ MOTOR ERROR
#define ERR_DISCONNECT     0x08  // 🔌 DISCONNECT ERROR
#define ERR_EXTINGUISHED   0x09  // 🔥 EXTINGUISHED
#define ERR_SENSOR         0x0A  // 🌡️ SENSOR ERROR
#define ERR_IGNITION       0x0B  // 🔥 IGNITION ERROR
#define ERR_STANDBY        0x0C  // ⏸️ STANDBY

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

WebServer server(80);
Preferences prefs;
WiFiClient espClient;
PubSubClient mqtt(espClient);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

//Versions
String version = "1.4";
// WiFi
String apSSID = "Diesel-Pilot";
String apPassword = "12345678";
String staSSID = "SEARS";
String staPassword = "Summer08Mylee";
bool useAP = true;
bool apStarted = false;
bool staConnected = false;
bool timeFromNTP = false;
String timeZone = "Europe/London";
bool daylightSavingEnabled = true;
unsigned long lastNtpAttempt = 0;
unsigned long lastTimeSave = 0;

// MQTT
String deviceName = "DieselPilot";
String heaterName = "Diesel Pilot";
String mqttServer = "";
int mqttPort = 1883;
String mqttTopic = "diesel";
String mqttUser = "";
String mqttPassword = "";
bool mqttAuthEnabled = false;
bool mqttEnabled = false;

// OTA
String otaPassword = "dieselpilot";  // Default OTA password
bool otaEnabled = true;              // OTA enabled by default

// Heater
uint32_t heaterAddress = 0x00000000;
uint8_t packetSeq = 0;
bool heaterPaired = false;

// Frost Mode
// Configurable automatic start/stop temperatures.
bool frostMode = false;
bool frostHeaterStarted = false;
bool frostManualOverride = false;
int frostStartTemp = 3;   // Start heater below this temperature
int frostStopTemp = 10;   // Stop heater at/above this temperature
uint16_t frostRestartDelayMinutes = 30; // Minimum time after Frost shutdown before automatic restart
uint32_t frostLastShutdownMillis = 0;   // When Frost Mode last commanded the heater OFF

// Pre-Heat Schedule
// Weekly schedule. Each selected day has one ON and one OFF time.
bool preheatEnabled = false;
uint8_t preheatDays = 0;              // Bit 0=Mon ... Bit 6=Sun
uint16_t preheatOnMinute = 420;       // Default 07:00
uint16_t preheatOffMinute = 480;      // Default 08:00
bool preheatHeaterStarted = false;
bool preheatOffCancelledToday = false;
bool preheatManualOverride = false;
int preheatSessionDay = -1;
int preheatStartSetpoint = -1000;
int preheatLastMinute = -1;
bool timeSynced = false;

void notePreheatSetpointChange();

// Status
struct {
    uint8_t state = 0;
    uint8_t power = 0;
    uint16_t voltage = 0;
    int8_t ambientTemp = 0;
    uint8_t caseTemp = 0;
    int8_t setpoint = 0;
    uint8_t pumpFreq = 0;
    bool autoMode = false;
    int16_t rssi = 0;
    uint8_t errorCode = 0;  // NEW: Error code from BYTE[7]
    unsigned long lastUpdate = 0;
} heaterStatus;

// Error History (last 10 errors)
struct ErrorHistoryEntry {
    uint8_t errorCode;
    unsigned long timestamp;
};
ErrorHistoryEntry errorHistory[10];
int errorHistoryIndex = 0;

// Display
String displayLine1 = "Diesel Pilot";
String displayLine2 = "Initializing...";
String displayLine3 = "";
String displayLine4 = "";

// Timing
unsigned long lastUpdate = 0;
unsigned long lastDisplay = 0;
unsigned long lastMQTTRetry = 0;
const unsigned long mqttRetryInterval = 30000;

// ═══════════════════════════════════════════════════════════════════════════
// ERROR CODE DECODER
// ═══════════════════════════════════════════════════════════════════════════

const char* getErrorName(uint8_t code) {
    switch(code) {
        case ERR_NONE:         return "NORMAL";      // Normal for OFF
        case ERR_ON:           return "NORMAL";      // Normal for ON
        case ERR_UNDERVOLTAGE: return "UNDERVOLTAGE";
        case ERR_OVERVOLTAGE:  return "OVERVOLTAGE";
        case ERR_SPARK_PLUG:   return "SPARK PLUG";
        case ERR_OIL_PUMP:     return "OIL PUMP";
        case ERR_OVERHEAT:     return "OVERHEAT";
        case ERR_MOTOR:        return "MOTOR";
        case ERR_DISCONNECT:   return "DISCONNECT";
        case ERR_EXTINGUISHED: return "EXTINGUISHED";
        case ERR_SENSOR:       return "SENSOR";
        case ERR_IGNITION:     return "IGNITION";
        case ERR_STANDBY:      return "STANDBY";
        default:               return "UNKNOWN";
    }
}


void addErrorToHistory(uint8_t errorCode) {
    // Only add if it's not ERR_NONE and different from last error
    if(errorCode == ERR_NONE) return;
    if(errorHistoryIndex > 0 && errorHistory[(errorHistoryIndex - 1) % 10].errorCode == errorCode) return;
    
    errorHistory[errorHistoryIndex % 10].errorCode = errorCode;
    errorHistory[errorHistoryIndex % 10].timestamp = millis();
    errorHistoryIndex++;
}

// ═══════════════════════════════════════════════════════════════════════════
// CC1101 LOW-LEVEL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void cc1101_writeReg(uint8_t addr, uint8_t val) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    SPI.transfer(val);
    digitalWrite(PIN_SS, HIGH);
}

void cc1101_writeBurst(uint8_t addr, uint8_t len, uint8_t* bytes) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    for(int i = 0; i < len; i++) {
        SPI.transfer(bytes[i]);
    }
    digitalWrite(PIN_SS, HIGH);
}

void cc1101_strobe(uint8_t addr) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    digitalWrite(PIN_SS, HIGH);
}

uint8_t cc1101_readReg(uint8_t addr) {
    digitalWrite(PIN_SS, LOW);
    while(digitalRead(PIN_MISO));
    SPI.transfer(addr);
    uint8_t val = SPI.transfer(0xFF);
    digitalWrite(PIN_SS, HIGH);
    return val;
}

// ═══════════════════════════════════════════════════════════════════════════
// CRC-16/MODBUS
// ═══════════════════════════════════════════════════════════════════════════

uint16_t crc16_modbus(uint8_t* buf, int len) {
    uint16_t crc = 0xFFFF;
    for(int pos = 0; pos < len; pos++) {
        crc ^= (uint8_t)buf[pos];
        for(int i = 8; i != 0; i--) {
            if((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════
// CC1101 INIT
// ═══════════════════════════════════════════════════════════════════════════

void cc1101_init() {
    cc1101_strobe(0x30); // SRES
    delay(100);
    
    cc1101_writeReg(0x00, 0x07); // IOCFG2
    cc1101_writeReg(0x02, 0x06); // IOCFG0
    cc1101_writeReg(0x03, 0x47); // FIFOTHR
    cc1101_writeReg(0x07, 0x04); // PKTCTRL1
    cc1101_writeReg(0x08, 0x05); // PKTCTRL0
    cc1101_writeReg(0x0A, 0x00); // CHANNR
    cc1101_writeReg(0x0B, 0x06); // FSCTRL1
    cc1101_writeReg(0x0C, 0x00); // FSCTRL0
    
    // 433.960 MHz
    cc1101_writeReg(0x0D, 0x10); // FREQ2
    cc1101_writeReg(0x0E, 0xB0); // FREQ1
    cc1101_writeReg(0x0F, 0xD6); // FREQ0
    
    cc1101_writeReg(0x10, 0xF8); // MDMCFG4
    cc1101_writeReg(0x11, 0x93); // MDMCFG3
    cc1101_writeReg(0x12, 0x13); // MDMCFG2
    cc1101_writeReg(0x13, 0x22); // MDMCFG1
    cc1101_writeReg(0x14, 0xF8); // MDMCFG0
    cc1101_writeReg(0x15, 0x26); // DEVIATN
    cc1101_writeReg(0x17, 0x30); // MCSM1
    cc1101_writeReg(0x18, 0x18); // MCSM0
    cc1101_writeReg(0x19, 0x16); // FOCCFG
    cc1101_writeReg(0x1A, 0x6C); // BSCFG
    cc1101_writeReg(0x1B, 0x03); // AGCTRL2
    cc1101_writeReg(0x1C, 0x40); // AGCTRL1
    cc1101_writeReg(0x1D, 0x91); // AGCTRL0
    cc1101_writeReg(0x20, 0xFB); // WORCTRL
    cc1101_writeReg(0x21, 0x56); // FREND1
    cc1101_writeReg(0x22, 0x17); // FREND0
    cc1101_writeReg(0x23, 0xE9); // FSCAL3
    cc1101_writeReg(0x24, 0x2A); // FSCAL2
    cc1101_writeReg(0x25, 0x00); // FSCAL1
    cc1101_writeReg(0x26, 0x1F); // FSCAL0
    cc1101_writeReg(0x2C, 0x81); // TEST2
    cc1101_writeReg(0x2D, 0x35); // TEST1
    cc1101_writeReg(0x2E, 0x09); // TEST0
    cc1101_writeReg(0x09, 0x00); // ADDR
    cc1101_writeReg(0x04, 0x7E); // SYNC1
    cc1101_writeReg(0x05, 0x3C); // SYNC0
    
    uint8_t paTable[8] = {0x00, 0x12, 0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0};
    cc1101_writeBurst(0x7E, 8, paTable);
    
    cc1101_strobe(0x31); // SFSTXON
    cc1101_strobe(0x36); // SIDLE
    cc1101_strobe(0x3B); // SFTX
    cc1101_strobe(0x36); // SIDLE
    cc1101_strobe(0x3A); // SFRX
    delay(136);
    
    Serial.println("✅ CC1101 initialized");
}

// ═══════════════════════════════════════════════════════════════════════════
// TX/RX FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void txFlush() {
    cc1101_strobe(0x36); // SIDLE
    cc1101_strobe(0x3B); // SFTX
    delay(16);
}

void txBurst(uint8_t len, uint8_t* bytes) {
    txFlush();
    cc1101_writeBurst(0x7F, len, bytes);
    cc1101_strobe(0x35); // STX
}

void sendCommand(uint8_t cmd) {
    if(!heaterPaired) return;
    
    uint8_t buf[10];
    buf[0] = 9;
    buf[1] = cmd;
    buf[2] = (heaterAddress >> 24) & 0xFF;
    buf[3] = (heaterAddress >> 16) & 0xFF;
    buf[4] = (heaterAddress >> 8) & 0xFF;
    buf[5] = heaterAddress & 0xFF;
    buf[6] = packetSeq++;
    buf[9] = 0;
    
    uint16_t crc = crc16_modbus(buf, 7);
    buf[7] = (crc >> 8) & 0xFF;
    buf[8] = crc & 0xFF;
    
    for(int i = 0; i < 10; i++) {
        txBurst(10, buf);
        unsigned long t = millis();
        while(cc1101_readReg(0xF5) != 0x01) {
            delay(1);
            if(millis() - t > 100) return;
        }
    }
}

void rxFlush() {
    cc1101_strobe(0x36); // SIDLE
    cc1101_readReg(0xBF); // Dummy read
    cc1101_strobe(0x3A); // SFRX
    delay(16);
}

void rxEnable() {
    cc1101_strobe(0x34); // SRX
}

bool receivePacket(uint8_t* bytes, uint16_t timeout) {
    unsigned long t = millis();
    uint8_t rxLen;
    
    rxFlush();
    rxEnable();
    
    while(1) {
        yield();
        if(millis() - t > timeout) return false;
        
        while(!digitalRead(PIN_GDO2)) {
            yield();
            if(millis() - t > timeout) return false;
        }
        
        delay(5);
        rxLen = cc1101_readReg(0xFB);
        
        if(rxLen >= 23 && rxLen <= 26) break;
        
        rxFlush();
        rxEnable();
    }
    
    for(int i = 0; i < rxLen; i++) {
        bytes[i] = cc1101_readReg(0xBF);
    }
    
    rxFlush();
    
    uint16_t crc = crc16_modbus(bytes, 21);
    uint16_t rxCrc = (bytes[21] << 8) | bytes[22];
    
    return (crc == rxCrc);
}

// ═══════════════════════════════════════════════════════════════════════════
// HEATER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// PRE-HEAT SCHEDULER
// ═══════════════════════════════════════════════════════════════════════════

bool getLocalDateTime(struct tm &timeinfo) {
    time_t now = time(nullptr);
    if(now < 1704067200) { // 2024-01-01 - no usable clock yet
        return false;
    }
    localtime_r(&now, &timeinfo);
    timeSynced = true;
    return true;
}

void restoreLastKnownTime() {
    time_t saved = (time_t)prefs.getULong64("lastEpoch", 0);
    if(saved < 1704067200) return;

    struct timeval tv;
    tv.tv_sec = saved;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    timeSynced = true;
    timeFromNTP = false;
    Serial.println("⚠️ Restored last known clock time: " + String((unsigned long)saved));
}

void saveLastKnownTime() {
    time_t now = time(nullptr);
    if(now < 1704067200) return;
    prefs.putULong64("lastEpoch", (uint64_t)now);
    lastTimeSave = millis();
}

void applyTimezone() {
    // ESP32 uses POSIX TZ strings. These common zones cover the most useful
    // choices for DieselPilot while keeping the firmware self-contained.
    String tz = "UTC0";

    if(timeZone == "Europe/London") {
        tz = daylightSavingEnabled ? "GMT0BST,M3.5.0/1,M10.5.0/2" : "GMT0";
    } else if(timeZone == "Europe/Dublin") {
        tz = daylightSavingEnabled ? "GMT0IST,M3.5.0/1,M10.5.0/2" : "GMT0";
    } else if(timeZone == "Europe/Paris") {
        tz = daylightSavingEnabled ? "CET-1CEST,M3.5.0/2,M10.5.0/3" : "CET-1";
    } else if(timeZone == "Europe/Berlin") {
        tz = daylightSavingEnabled ? "CET-1CEST,M3.5.0/2,M10.5.0/3" : "CET-1";
    } else if(timeZone == "America/New_York") {
        tz = daylightSavingEnabled ? "EST5EDT,M3.2.0/2,M11.1.0/2" : "EST5";
    } else if(timeZone == "America/Chicago") {
        tz = daylightSavingEnabled ? "CST6CDT,M3.2.0/2,M11.1.0/2" : "CST6";
    } else if(timeZone == "America/Denver") {
        tz = daylightSavingEnabled ? "MST7MDT,M3.2.0/2,M11.1.0/2" : "MST7";
    } else if(timeZone == "America/Los_Angeles") {
        tz = daylightSavingEnabled ? "PST8PDT,M3.2.0/2,M11.1.0/2" : "PST8";
    } else if(timeZone == "America/Toronto") {
        tz = daylightSavingEnabled ? "EST5EDT,M3.2.0/2,M11.1.0/2" : "EST5";
    } else if(timeZone == "Australia/Sydney") {
        tz = daylightSavingEnabled ? "AEST-10AEDT,M10.1.0/2,M4.1.0/3" : "AEST-10";
    } else if(timeZone == "Pacific/Auckland") {
        tz = daylightSavingEnabled ? "NZST-12NZDT,M9.5.0/2,M4.1.0/3" : "NZST-12";
    } else if(timeZone == "Asia/Kolkata") {
        tz = "IST-5:30";
    } else if(timeZone == "Asia/Tokyo") {
        tz = "JST-9";
    } else if(timeZone == "Asia/Singapore") {
        tz = "SGT-8";
    }

    setenv("TZ", tz.c_str(), 1);
    tzset();
    Serial.println("Timezone applied: " + timeZone + " | DST: " + String(daylightSavingEnabled ? "ON" : "OFF"));
}

String getCurrentTimeString() {
    struct tm t;
    if(!getLocalDateTime(t)) return "--:--";
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    return String(buf);
}

void handleTimeSync() {
    // NTP needs the STA connection. Keep retrying in case the router/internet
    // was unavailable during boot.
    if(staConnected && !timeFromNTP && millis() - lastNtpAttempt >= 10000) {
        lastNtpAttempt = millis();
        configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
        applyTimezone();
    }

    // Only call the time source NTP after the SNTP client confirms a sync.
    if(staConnected && sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
        struct tm t;
        if(getLocalDateTime(t)) {
            if(!timeFromNTP) {
                timeFromNTP = true;
                timeSynced = true;
                Serial.println("✅ NTP time synchronised: " + formatScheduleTime(t.tm_hour * 60 + t.tm_min));
                saveLastKnownTime();
            }
        }
    }

    // Once synchronised, periodically save the clock so a later reboot has a
    // useful fallback even if the router/internet is temporarily unavailable.
    if(timeFromNTP && millis() - lastTimeSave >= 60000) {
        saveLastKnownTime();
    }
}


String formatScheduleTime(uint16_t minuteOfDay) {
    char buf[6];
    snprintf(buf, sizeof(buf), "%02u:%02u", minuteOfDay / 60, minuteOfDay % 60);
    return String(buf);
}

String getCurrentUKTimeString() {
    struct tm t;
    if(!getLocalDateTime(t)) return "--:--";
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    return String(buf);
}

bool isPreheatDaySelected(int dayOfWeek) {
    // tm_wday: Sunday=0, Monday=1 ... Saturday=6
    int dayIndex = (dayOfWeek == 0) ? 6 : dayOfWeek - 1;
    return (preheatDays & (1 << dayIndex)) != 0;
}

bool isPreheatWindowActive(const struct tm &t) {
    if(!preheatEnabled || !isPreheatDaySelected(t.tm_wday)) return false;

    int minuteNow = t.tm_hour * 60 + t.tm_min;

    // Require a normal same-day ON/OFF window.
    if(preheatOnMinute >= preheatOffMinute) return false;

    return minuteNow >= preheatOnMinute && minuteNow < preheatOffMinute;
}

void notePreheatSetpointChange() {
    if(heaterStatus.lastUpdate == 0) return;

    struct tm t;
    if(getLocalDateTime(t) && isPreheatWindowActive(t)) {
        preheatOffCancelledToday = true;
        preheatManualOverride = true;
        preheatHeaterStarted = false;
        frostManualOverride = true;
        frostHeaterStarted = false;
        Serial.println("⏰ Manual setpoint change detected - Pre-Heat/Frost automatic control overridden");
    }
}

void handlePreheatSchedule() {
    if(!preheatEnabled || !heaterPaired) return;

    struct tm t;
    if(!getLocalDateTime(t)) return;

    int minuteNow = t.tm_hour * 60 + t.tm_min;
    int todayIndex = (t.tm_wday == 0) ? 6 : t.tm_wday - 1;
    bool selectedToday = (preheatDays & (1 << todayIndex)) != 0;
    // Calendar-day key avoids DST changes affecting the session identity.
    int dayKey = (t.tm_year + 1900) * 10000 + (t.tm_mon + 1) * 100 + t.tm_mday;

    // A new calendar day starts a fresh scheduled session.
    if(preheatSessionDay != dayKey) {
        preheatSessionDay = dayKey;
        preheatHeaterStarted = false;
        preheatOffCancelledToday = false;
        preheatManualOverride = false;
        preheatStartSetpoint = -1000;
        preheatLastMinute = -1;
    }

    if(!selectedToday || preheatOnMinute >= preheatOffMinute) return;

    bool inWindow = minuteNow >= preheatOnMinute && minuteNow < preheatOffMinute;

    // At the start of the scheduled window, remember the target temperature.
    // This lets us detect a deliberate setpoint change during today's window.
    if(inWindow && preheatStartSetpoint == -1000 && heaterStatus.lastUpdate > 0) {
        preheatStartSetpoint = heaterStatus.setpoint;
    }

    // If the target temperature has been changed during this scheduled window,
    // cancel today's automatic OFF command. The schedule will resume tomorrow.
    if(inWindow && preheatStartSetpoint != -1000 && heaterStatus.lastUpdate > 0 &&
       heaterStatus.setpoint != preheatStartSetpoint) {
        if(!preheatOffCancelledToday) {
            Serial.println("⏰ Pre-Heat: Target temperature changed - cancelling today's OFF");
        }
        preheatOffCancelledToday = true;
    }

    // ON: only toggle the heater if it is actually OFF. This prevents the
    // schedule from accidentally turning off a heater already running manually.
    if(inWindow && !preheatManualOverride && !preheatHeaterStarted && heaterStatus.state == STATE_OFF) {
        if(minuteNow >= preheatOnMinute) {
            Serial.println("⏰ Pre-Heat: Scheduled ON - starting heater");
            sendCommand(CMD_POWER);
            preheatHeaterStarted = true;
        }
    }

    // OFF: only turn off a heater that this scheduled session started,
    // and only if the user has not changed the target temperature.
    if(minuteNow >= preheatOffMinute && !preheatOffCancelledToday && preheatHeaterStarted) {
        if(preheatLastMinute != minuteNow) {
            if(heaterStatus.state != STATE_OFF &&
               heaterStatus.state != STATE_SHUTDOWN &&
               heaterStatus.state != STATE_SHUTTING_DOWN &&
               heaterStatus.state != STATE_COOLING) {
                Serial.println("⏰ Pre-Heat: Scheduled OFF - stopping heater");
                sendCommand(CMD_POWER);
            }
            preheatLastMinute = minuteNow;
        }
        preheatHeaterStarted = false;
    }
}

void handleFrostMode() {
    if(!frostMode || !heaterPaired || heaterStatus.lastUpdate == 0) return;

    // A manual intervention overrides Frost Mode, including the automatic
    // restart lockout. A manual POWER command is therefore always allowed.
    if(frostManualOverride) {
        if(heaterStatus.ambientTemp >= frostStopTemp) {
            frostManualOverride = false;
            Serial.println("❄️ Frost Mode: Manual override cleared - temperature recovered");
        } else {
            return;
        }
    }

    // Do not act on stale temperature data.
    if(millis() - heaterStatus.lastUpdate > 10000) return;

    int temp = heaterStatus.ambientTemp;

    // Frost protection: only consider an automatic restart once the low
    // temperature threshold has actually been reached. The restart delay is
    // measured from the moment Frost Mode commanded the previous shutdown.
    if(temp < frostStartTemp && heaterStatus.state == STATE_OFF && !frostHeaterStarted) {
        uint32_t lockoutMs = (uint32_t)frostRestartDelayMinutes * 60000UL;
        bool restartLocked = frostLastShutdownMillis != 0 &&
                             (uint32_t)(millis() - frostLastShutdownMillis) < lockoutMs;

        if(restartLocked) {
            uint32_t elapsed = millis() - frostLastShutdownMillis;
            uint32_t remainingMs = lockoutMs - elapsed;
            uint32_t remainingMinutes = (remainingMs + 59999UL) / 60000UL;
            Serial.printf("❄️ Frost Mode: Low temperature reached - restart locked for %lu more minute(s)\n", remainingMinutes);
            return;
        }

        Serial.printf("❄️ Frost Mode: Ambient below %dC - starting heater\n", frostStartTemp);
        sendCommand(CMD_POWER);
        frostHeaterStarted = true;
        return;
    }

    // Stop the heater once the ambient temperature reaches the configured
    // frost-stop temperature. Only shut down a heater that Frost Mode started.
    if(temp >= frostStopTemp && frostHeaterStarted) {
        if(heaterStatus.state != STATE_OFF &&
           heaterStatus.state != STATE_SHUTDOWN &&
           heaterStatus.state != STATE_SHUTTING_DOWN &&
           heaterStatus.state != STATE_COOLING) {
            Serial.printf("❄️ Frost Mode: Ambient reached %dC - stopping heater\n", frostStopTemp);
            sendCommand(CMD_POWER);
            // Start the restart lockout at the moment Frost Mode requests shutdown.
            frostLastShutdownMillis = millis();
        }
        frostHeaterStarted = false;
    }
}

void updateHeaterStatus() {
    if(!heaterPaired) return;
    
    sendCommand(CMD_WAKEUP);
    
    uint8_t buf[32];
    if(receivePacket(buf, 2000)) {
        uint32_t addr = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | 
                        ((uint32_t)buf[4] << 8) | buf[5];
        
        if(addr == heaterAddress) {
            heaterStatus.state = buf[6];
            heaterStatus.power = buf[7];           // BYTE[7] = ERROR CODE!
            heaterStatus.errorCode = buf[7];       // Store error code
            heaterStatus.voltage = buf[9];
            heaterStatus.ambientTemp = (int8_t)buf[10];
            heaterStatus.caseTemp = buf[12];
            int previousSetpoint = heaterStatus.setpoint;
            heaterStatus.setpoint = (int8_t)buf[13];
            heaterStatus.autoMode = (buf[14] == 0x32);

            // A setpoint change reported by the heater itself is treated as
            // manual intervention. This catches changes made on the heater's
            // own controller, not just changes made through DieselPilot.
            if(heaterStatus.lastUpdate > 0 && heaterStatus.setpoint != previousSetpoint) {
                struct tm setpointTime;
                if(getLocalDateTime(setpointTime) && isPreheatWindowActive(setpointTime)) {
                    preheatOffCancelledToday = true;
                    preheatManualOverride = true;
                    preheatHeaterStarted = false;
                }
                frostManualOverride = true;
                frostHeaterStarted = false;
                Serial.printf("Manual heater setpoint change detected: %d -> %d - automatic control overridden\n", previousSetpoint, heaterStatus.setpoint);
            }
            heaterStatus.pumpFreq = buf[15];
            heaterStatus.rssi = (buf[23] - (buf[23] >= 128 ? 256 : 0)) / 2 - 74;
            heaterStatus.lastUpdate = millis();
            
            // Add error to history if not ERR_NONE
            addErrorToHistory(heaterStatus.errorCode);
            
            // Publish to MQTT
            if(mqttEnabled && mqtt.connected()) {
                publishMQTT();
            }

            // Evaluate Frost Mode using the newly received ambient temperature.
            handleFrostMode();
        }
    }
}

uint32_t findHeater(uint16_t timeout) {
    Serial.println("Searching for heater...");
    displayLine2 = "Pairing...";
    updateDisplay();
    
    uint8_t buf[32];
    if(receivePacket(buf, timeout)) {
        uint32_t addr = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | 
                        ((uint32_t)buf[4] << 8) | buf[5];
        return addr;
    }
    return 0;
}

const char* getStateName(uint8_t state) {
    switch(state) {
        case STATE_OFF: return "OFF";
        case STATE_STARTUP: return "STARTUP";
        case STATE_WARMING: return "WARMING";
        case STATE_WARMING_WAIT: return "WARM WAIT";
        case STATE_PRE_RUN: return "PRE-RUN";
        case STATE_RUNNING: return "RUNNING";
        case STATE_SHUTDOWN: return "SHUTDOWN";
        case STATE_SHUTTING_DOWN: return "SHUTTING";
        case STATE_COOLING: return "COOLING";
        default: return "UNKNOWN";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// OTA FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void setupOTA() {
    if(!otaEnabled) return;
    
    ArduinoOTA.setHostname(deviceName.c_str());
    ArduinoOTA.setPassword(otaPassword.c_str());
    
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("OTA: Start updating " + type);
        displayLine1 = "OTA UPDATE";
        displayLine2 = "Updating...";
        displayLine3 = "DO NOT";
        displayLine4 = "POWER OFF!";
        updateDisplay();
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA: Update complete");
        displayLine2 = "Complete!";
        displayLine3 = "Rebooting...";
        displayLine4 = "";
        updateDisplay();
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        unsigned int percent = (progress / (total / 100));
        Serial.printf("OTA Progress: %u%%\r", percent);
        displayLine2 = "Progress: " + String(percent) + "%";
        updateDisplay();
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        
        displayLine1 = "OTA ERROR!";
        displayLine2 = "Error: " + String(error);
        displayLine3 = "Rebooting...";
        displayLine4 = "";
        updateDisplay();
        delay(3000);
        ESP.restart();
    });
    
    ArduinoOTA.begin();
    Serial.println("✅ OTA enabled on port 3232");
    Serial.println("   Hostname: " + deviceName);
}

// ═══════════════════════════════════════════════════════════════════════════
// MQTT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Manual control always takes priority over Frost Mode and Pre-Heat.
// Clear the automatic-session ownership flags so neither mode can undo
// the user's command during the current session. The modes remain enabled
// and can take control again when their normal conditions are met later.
void cancelAutomaticModesForManualInput() {
    if(preheatHeaterStarted || frostHeaterStarted) {
        Serial.println("Manual input: overriding Frost Mode / Pre-Heat");
    }
    preheatHeaterStarted = false;
    frostHeaterStarted = false;
    preheatOffCancelledToday = true;
    preheatManualOverride = true;
    frostManualOverride = true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for(int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    String topicStr = String(topic);
    
    if(topicStr == mqttTopic + "/cmd/power") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_POWER);
    } else if(topicStr == mqttTopic + "/cmd/up") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_UP);
        notePreheatSetpointChange();
    } else if(topicStr == mqttTopic + "/cmd/down") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_DOWN);
        notePreheatSetpointChange();
    } else if(topicStr == mqttTopic + "/cmd/mode") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_MODE);
    }
}

void connectMQTT() {
    if(!mqttEnabled || mqttServer.length() == 0) return;
    
    if(mqtt.connected()) {
        mqtt.disconnect();
        delay(100);
    }
    
    mqtt.setServer(mqttServer.c_str(), mqttPort);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    
    const int maxRetries = 3;
    const int retryDelay = 2000;
    
    for(int attempt = 1; attempt <= maxRetries; attempt++) {
        Serial.print("MQTT connection attempt " + String(attempt) + "/" + String(maxRetries) + "... ");
        
        bool connected = false;
        if(mqttAuthEnabled) {
            connected = mqtt.connect(deviceName.c_str(), mqttUser.c_str(), mqttPassword.c_str());
        } else {
            connected = mqtt.connect(deviceName.c_str());
        }
        
        if(connected) {
            Serial.println("✅ Connected!");
            mqtt.subscribe((mqttTopic + "/cmd/#").c_str());
            return;
        } else {
            Serial.println("❌ Failed (State: " + String(mqtt.state()) + ")");
            if(attempt < maxRetries) {
                Serial.println("Retrying in " + String(retryDelay/1000) + " seconds...");
                delay(retryDelay);
            }
        }
    }
    
    Serial.println("⚠️ MQTT connection failed after " + String(maxRetries) + " attempts.");
}

void publishMQTT() {
    if(!mqtt.connected()) return;
    
    mqtt.publish((mqttTopic + "/state").c_str(), getStateName(heaterStatus.state));
    mqtt.publish((mqttTopic + "/voltage").c_str(), String(heaterStatus.voltage / 10.0, 1).c_str());
    mqtt.publish((mqttTopic + "/ambient").c_str(), String(heaterStatus.ambientTemp).c_str());
    mqtt.publish((mqttTopic + "/case").c_str(), String(heaterStatus.caseTemp).c_str());
    mqtt.publish((mqttTopic + "/setpoint").c_str(), String(heaterStatus.setpoint).c_str());
    mqtt.publish((mqttTopic + "/pump").c_str(), String(heaterStatus.pumpFreq / 10.0, 1).c_str());
    mqtt.publish((mqttTopic + "/mode").c_str(), heaterStatus.autoMode ? "AUTO" : "MANUAL");
    mqtt.publish((mqttTopic + "/rssi").c_str(), String(heaterStatus.rssi).c_str());
    
    // ERROR: Only short name (perfect for OLED and HA history)
    mqtt.publish((mqttTopic + "/error").c_str(), getErrorName(heaterStatus.errorCode));
}

// ═══════════════════════════════════════════════════════════════════════════
// DISPLAY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

int centreTextX(const String &text) {
    int width = display.getStrWidth(text.c_str());
    int x = (128 - width) / 2;
    return max(0, x);
}

// OLED header
// Displays the user-configured heater name as a static title.

// Frost Mode status marquee
const char *frostStatusText = "FROST MODE - RUNNING";
int frostStatusScrollX = 128;
unsigned long lastFrostStatusScroll = 0;
unsigned long frostStatusScrollInterval = 120; // milliseconds
// Pre-Heat status marquee
const char *preheatStatusText = "PRE HEAT - RUNNING";
int preheatStatusScrollX = 128;
unsigned long lastPreheatStatusScroll = 0;
unsigned long preheatStatusScrollInterval = 120; // milliseconds

// Physical button OLED indicator.
// Only one indicator can be active at a time and each press replaces the previous one.
char physicalButtonIndicator = 0;
unsigned long physicalButtonIndicatorUntil = 0;
const unsigned long PHYSICAL_BUTTON_INDICATOR_MS = 1000;

void showPhysicalButtonIndicator(char indicator) {
    physicalButtonIndicator = indicator;
    physicalButtonIndicatorUntil = millis() + PHYSICAL_BUTTON_INDICATOR_MS;
}

void drawPhysicalButtonIndicator() {
    if (physicalButtonIndicator == 0) return;

    if ((long)(millis() - physicalButtonIndicatorUntil) >= 0) {
        physicalButtonIndicator = 0;
        return;
    }

    // Centre-right of the 128x64 OLED.
    const int cx = 116;
    const int cy = 51;

    if (physicalButtonIndicator == 'U') {
        // Up arrow
        display.drawLine(cx, cy + 4, cx, cy - 4);
        display.drawLine(cx, cy - 4, cx - 3, cy - 1);
        display.drawLine(cx, cy - 4, cx + 3, cy - 1);
    } else if (physicalButtonIndicator == 'D') {
        // Down arrow
        display.drawLine(cx, cy - 4, cx, cy + 4);
        display.drawLine(cx, cy + 4, cx - 3, cy + 1);
        display.drawLine(cx, cy + 4, cx + 3, cy + 1);
    } else {
        display.setFont(u8g2_font_7x13_tf);
        String indicatorText(physicalButtonIndicator);
        display.drawStr(cx - 3, cy + 5, indicatorText.c_str());
    }
}

// Draw a compact WiFi signal-strength indicator in the OLED header.
// 4 bars = excellent, 3 = good, 2 = fair, 1 = weak, 0 = disconnected/AP only.
void drawWifiSignalBars() {
    int bars = 0;

    if (staConnected && WiFi.status() == WL_CONNECTED) {
        int rssi = WiFi.RSSI();
        if (rssi >= -55)      bars = 4;
        else if (rssi >= -67) bars = 3;
        else if (rssi >= -75) bars = 2;
        else if (rssi >= -85) bars = 1;
    }

    const int baseX = 113;
    const int baseY = 10;

    // Clear the small indicator area first.
    display.setDrawColor(0);
    display.drawBox(111, 1, 17, 10);
    display.setDrawColor(1);

    // Four ascending bars.
    for (int i = 0; i < 4; i++) {
        int h = 2 + (i * 2);
        int x = baseX + (i * 4);
        if (i < bars) {
            display.drawBox(x, baseY - h + 1, 2, h);
        } else {
            display.drawFrame(x, baseY - h + 1, 2, h);
        }
    }
}

void updateDisplay() {
#if USE_OLED
    display.clearBuffer();
    
    // === HEATER NAME HEADER ===
    // Static user-configured heater name on the top line.
    display.drawFrame(0, 0, 128, 12);
    display.setFont(u8g2_font_6x10_tf);

    String oledHeaterName = heaterName;
    int maxHeaderWidth = 107;  // Leave room for the WiFi indicator.
    if(display.getStrWidth(oledHeaterName.c_str()) > maxHeaderWidth) {
        while(oledHeaterName.length() > 1 &&
              display.getStrWidth(oledHeaterName.c_str()) > maxHeaderWidth) {
            oledHeaterName.remove(oledHeaterName.length() - 1);
        }
    }
    int headerWidth = display.getStrWidth(oledHeaterName.c_str());
    int headerX = max(1, (109 - headerWidth) / 2);
    display.setClipWindow(0, 0, 109, 11);
    display.drawStr(headerX, 9, oledHeaterName.c_str());
    display.setMaxClipWindow();

    // WiFi signal strength (STA connection).
    drawWifiSignalBars();
    
    // === MAIN CONTENT AREA ===
    display.setFont(u8g2_font_7x13_tf);
    
    // Line 2 - Big and bold
    display.drawStr(centreTextX(displayLine2), 26, displayLine2.c_str());
    if(preheatEnabled) {
        display.setFont(u8g2_font_6x10_tf);
        display.drawStr(111, 26, "PH");
    }
    
    // Line 3 - Medium
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(centreTextX(displayLine3), 40, displayLine3.c_str());
    if(frostMode) {
        display.drawStr(111, 40, "FM");
    }
    
    // Line 4 - Pre-Heat has priority over Frost; manual input clears both session flags.
    display.setFont(u8g2_font_5x8_tf);

    if(preheatEnabled && preheatHeaterStarted) {
        int preheatWidth = display.getStrWidth(preheatStatusText);

        if(millis() - lastPreheatStatusScroll >= preheatStatusScrollInterval) {
            lastPreheatStatusScroll = millis();
            preheatStatusScrollX--;

            if(preheatStatusScrollX < -preheatWidth - 8) {
                preheatStatusScrollX = 128;
            }
        }

        display.drawStr(preheatStatusScrollX, 52, preheatStatusText);

    } else if(frostMode && frostHeaterStarted) {
        int frostWidth = display.getStrWidth(frostStatusText);

        if(millis() - lastFrostStatusScroll >= frostStatusScrollInterval) {
            lastFrostStatusScroll = millis();
            frostStatusScrollX--;

            if(frostStatusScrollX < -frostWidth - 8) {
                frostStatusScrollX = 128;
            }
        }

        display.drawStr(frostStatusScrollX, 52, frostStatusText);

    } else {
        display.drawStr(centreTextX(displayLine4), 52, displayLine4.c_str());
    }

        drawPhysicalButtonIndicator();

        display.sendBuffer();
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER HANDLERS  
// ═══════════════════════════════════════════════════════════════════════════

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Diesel Pilot</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #0a0a0a;
            color: #e0e0e0;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            padding: 20px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { 
            color: #ff6b00; 
            margin-bottom: 30px;
            text-align: center;
            text-shadow: 0 0 10px rgba(255, 107, 0, 0.5);
        }
        
        /* TABS */
        .tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            border-bottom: 2px solid #333;
        }
        .tab {
            background: #1a1a1a;
            border: 1px solid #333;
            border-bottom: none;
            padding: 12px 24px;
            cursor: pointer;
            border-radius: 8px 8px 0 0;
            transition: all 0.3s;
        }
        .tab:hover { background: #252525; }
        .tab.active {
            background: #ff6b00;
            color: white;
            border-color: #ff6b00;
        }
        .tab-content { display: none; }
        .tab-content.active { display: block; }
        
        .card {
            background: #1a1a1a;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            border: 1px solid #333;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        .card h2 {
            color: #ff6b00;
            margin-bottom: 15px;
            font-size: 1.3em;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            background: #0f0f0f;
            padding: 15px;
            border-radius: 8px;
            border: 1px solid #2a2a2a;
        }
        .status-label {
            color: #888;
            font-size: 0.85em;
            margin-bottom: 5px;
        }
        .status-value {
            color: #fff;
            font-size: 1.5em;
            font-weight: bold;
        }
        .auto-mode-status {
            color: #fff;
            font-size: 0.95em;
            line-height: 1.5;
            font-weight: 600;
        }
        .auto-mode-status span {
            font-weight: bold;
        }
        .state-running { color: #4CAF50; }
        .state-off { color: #f44336; }
        .state-other { color: #ff9800; }
        .mode-auto { color: #2196F3; }
        .mode-manual { color: #9C27B0; }
        .error-ok { color: #4CAF50; }
        .error-warning { color: #ff9800; }
        .error-critical { color: #f44336; }
        
        /* ERROR ALERT BOX */
        .error-alert {
            background: #1a0000;
            border: 2px solid #f44336;
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 20px;
            display: none;
        }
        .error-alert.active { display: block; }
        .error-alert-title {
            color: #f44336;
            font-size: 1.2em;
            font-weight: bold;
        }
        
        .btn {
            background: #ff6b00;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 1em;
            margin: 5px;
            transition: all 0.3s;
        }
        .btn:hover {
            background: #ff8533;
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(255, 107, 0, 0.3);
        }
        .btn:active { transform: translateY(0); }
        .btn-group {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 15px;
        }
        input, select {
            background: #0f0f0f;
            border: 1px solid #333;
            color: #e0e0e0;
            padding: 10px;
            border-radius: 6px;
            width: 100%;
            margin: 5px 0;
        }
        input:focus, select:focus {
            outline: none;
            border-color: #ff6b00;
        }
        .form-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin: 10px 0;
        }
        .info-box {
            margin-top:15px; 
            padding:10px; 
            background:#0f0f0f; 
            border-radius:6px; 
            border:1px solid #2a2a2a; 
            font-size:0.9em; 
            color:#888;
        }
        .info-box strong { color:#ff6b00; }
        
        .toggle-container {
            display: flex;
            align-items: center;
            margin: 15px 0;
            gap: 15px;
        }
        .toggle-switch {
            position: relative;
            width: 60px;
            height: 30px;
        }
        .toggle-switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }
        .toggle-slider {
            position: absolute;
            cursor: pointer;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: #333;
            transition: .3s;
            border-radius: 30px;
        }
        .toggle-slider:before {
            position: absolute;
            content: "";
            height: 22px;
            width: 22px;
            left: 4px;
            bottom: 4px;
            background-color: #888;
            transition: .3s;
            border-radius: 50%;
        }
        input:checked + .toggle-slider {
            background-color: #ff6b00;
        }
        input:checked + .toggle-slider:before {
            transform: translateX(30px);
            background-color: white;
        }
        .toggle-label {
            font-size: 1em;
            color: #e0e0e0;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔥 DIESEL PILOT</h1>
        
        <!-- TABS -->
        <div class="tabs">
            <div class="tab active" onclick="switchTab('dashboard', this)">📊 Dashboard</div>
            <div class="tab" onclick="switchTab('preheat', this)">⏰ Pre-Heat</div>
            <div class="tab" onclick="switchTab('frost', this)">❄️ Frost Mode</div>
            <div class="tab" onclick="switchTab('config', this)">⚙️ Config</div>
        </div>
        
        <!-- DASHBOARD TAB -->
        <div id="dashboard" class="tab-content active">
            
            <!-- ERROR ALERT -->
            <div id="errorAlert" class="error-alert">
                <div class="error-alert-title" id="errorTitle">⚠️ ERROR DETECTED</div>
            </div>
            
            <!-- STATUS -->
            <div class="card">
                <h2>📊 Status</h2>
                <div class="status-grid">
                    <div class="status-item">
                        <div class="status-label">State</div>
                        <div class="status-value" id="state">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Error Code</div>
                        <div class="status-value" id="errorCode">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Mode</div>
                        <div class="status-value" id="mode">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Voltage</div>
                        <div class="status-value" id="voltage">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Ambient</div>
                        <div class="status-value" id="ambient">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Case</div>
                        <div class="status-value" id="case">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label" id="setpointLabel">Setpoint</div>
                        <div class="status-value" id="setpoint">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label" id="pumpLabel">Pump</div>
                        <div class="status-value" id="pump">-</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Automatic Modes</div>
                        <div class="auto-mode-status" id="autoModesStatus">
                            <div>Pre-Heat: <span id="autoPreheatStatus">OFF</span></div>
                            <div>Frost Mode: <span id="autoFrostStatus">OFF</span></div>
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- CONTROLS -->
            <div class="card">
                <h2>🎮 Controls</h2>
                <div class="btn-group">
                    <button class="btn" onclick="sendCmd('power')">⚡ POWER</button>
                    <button class="btn" onclick="sendCmd('up')">⬆️ UP</button>
                    <button class="btn" onclick="sendCmd('down')">⬇️ DOWN</button>
                    <button class="btn" onclick="sendCmd('mode')">🔄 MODE</button>
                </div>
                <div class="info-box">
                    <strong>💡 Info:</strong><br>
                    • <strong>AUTO mode</strong>: UP/DOWN adjust <strong>temperature</strong> setpoint<br>
                    • <strong>MANUAL mode</strong>: UP/DOWN adjust <strong>pump frequency</strong><br>
                    • Press MODE to switch between AUTO ↔ MANUAL
                </div>
            </div>
            
        </div>
        
        <!-- PRE-HEAT TAB -->
        <div id="preheat" class="tab-content">
            <div class="card">
                <h2>⏰ Pre-Heat Schedule</h2>
                <div class="toggle-container">
                    <label class="toggle-switch">
                        <input type="checkbox" id="preheatEnabled">
                        <span class="toggle-slider"></span>
                    </label>
                    <span class="toggle-label">Enable Pre-Heat</span>
                </div>

                <div class="info-box">
                    <strong>How it works:</strong><br>
                    On selected days the heater will be started at the ON time and, if Pre-Heat started it, stopped at the OFF time.<br>
                    If the target temperature is changed during the scheduled period, the automatic OFF command is cancelled for that day.
                </div>

                <h3 style="color:#ff6b00; margin:20px 0 10px;">Days</h3>
                <div class="status-grid" style="grid-template-columns:repeat(auto-fit,minmax(90px,1fr));">
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="dayMon" value="0" style="width:auto; margin-right:6px;"> Monday</label>
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="dayTue" value="1" style="width:auto; margin-right:6px;"> Tuesday</label>
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="dayWed" value="2" style="width:auto; margin-right:6px;"> Wednesday</label>
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="dayThu" value="3" style="width:auto; margin-right:6px;"> Thursday</label>
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="dayFri" value="4" style="width:auto; margin-right:6px;"> Friday</label>
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="daySat" value="5" style="width:auto; margin-right:6px;"> Saturday</label>
                    <label class="status-item" style="cursor:pointer; text-align:center;"><input type="checkbox" class="preheat-day" id="daySun" value="6" style="width:auto; margin-right:6px;"> Sunday</label>
                </div>

                <div class="form-row" style="margin-top:20px;">
                    <div>
                        <label style="color:#888;">ON time</label>
                        <input type="time" id="preheatOnTime" value="07:00">
                    </div>
                    <div>
                        <label style="color:#888;">OFF time</label>
                        <input type="time" id="preheatOffTime" value="08:00">
                    </div>
                </div>

                <button class="btn" onclick="savePreheat()">💾 SAVE PRE-HEAT</button>
                <div id="preheatTimeStatus" class="info-box">Time synchronisation: checking...</div>
            </div>
        </div>

        <!-- FROST MODE TAB -->
        <div id="frost" class="tab-content">
            <div class="card">
                <h2>❄️ Frost Mode</h2>
                <div class="toggle-container">
                    <label class="toggle-switch">
                        <input type="checkbox" id="frostEnabled" onchange="toggleFrostMode()">
                        <span class="toggle-slider"></span>
                    </label>
                    <span class="toggle-label">Enable Frost Mode</span>
                </div>

                <div class="info-box">
                    <strong>How it works:</strong><br>
                    Frost Mode automatically starts the heater when the ambient temperature
                    drops below the configured start temperature and shuts it down when the
                    configured stop temperature is reached.<br>
                    Manual heater commands always override automatic Frost Mode control.
                </div>

                <div id="frostRestartStatus" class="info-box" style="display:none;">
                    <strong>❄️ Frost restart lockout:</strong>
                    <span id="frostRestartRemaining">-</span>
                </div>

                <h3 style="color:#ff6b00; margin:20px 0 10px;">Settings</h3>

                <label>Start heater if temperature drops below:</label><br>
                <input type="number" id="frostStartTemp" min="-30" max="30" step="1" value="3" style="width:90px; margin:6px 0;">
                <strong>°C</strong><br><br>

                <label>Shut heater down at:</label><br>
                <input type="number" id="frostStopTemp" min="-30" max="40" step="1" value="10" style="width:90px; margin:6px 0;">
                <strong>°C</strong><br><br>

                <label>Minimum time before automatic restart:</label><br>
                <input type="number" id="frostRestartDelay" min="30" max="240" step="5" value="30" style="width:90px; margin:6px 0;">
                <strong>minutes</strong><br><br>

                <div class="info-box">
                    Minimum 30 minutes. The countdown is only shown after the low-temperature
                    start threshold has been reached. Manual POWER starts always override this
                    restart lockout.
                </div>

                <button class="btn" onclick="saveFrostSettings()">💾 SAVE FROST SETTINGS</button>
            </div>
        </div>

        <!-- CONFIG TAB -->
        <div id="config" class="tab-content">
            
            <!-- PAIRING -->
            <div class="card">
                <h2>🔗 Pairing</h2>
                <button class="btn" onclick="autoPair()">🔍 AUTO PAIR</button>
                <div class="form-row">
                    <input type="text" id="manualAddr" placeholder="0xCA00445B">
                    <button class="btn" onclick="manualPair()">✏️ MANUAL PAIR</button>
                </div>
                <div style="margin-top:10px; color:#888;">
                    Current: <span id="currentAddr">Not paired</span>
                </div>
                <div class="info-box">
                    <strong>📡 Auto Pairing:</strong> Hold pairing button on heater panel for 60s<br>
                    <strong>✏️ Manual Pairing:</strong> Enter heater address (find with RTL-SDR)
                </div>
            </div>
            
            <!-- HEATER NAME -->
            <div class="card">
                <h2>🔥 Heater Name</h2>
                <input type="text" id="heaterName" maxlength="32" placeholder="e.g., Garage Heater">
                <button class="btn" onclick="saveHeaterName()">💾 SAVE HEATER NAME</button>
                <div class="info-box">
                    <strong>🖥️ OLED:</strong> This name is displayed on the top line of the OLED display.<br>
                    The name is separate from the WiFi hostname.
                </div>
            </div>

            <!-- WIFI CONFIG -->
            <div class="card">
                <h2>📡 WiFi Configuration</h2>
                <input type="text" id="deviceName" placeholder="Hostname (e.g., DieselPilot-Garage)">
                <input type="text" id="wifiSSID" placeholder="Your WiFi SSID">
                <input type="password" id="wifiPass" placeholder="WiFi Password">
                <button class="btn" onclick="saveWiFi()">💾 SAVE & REBOOT</button>
                <div class="info-box">
                    <strong>📛 Hostname:</strong> Device name used for WiFi hostname and MQTT Client ID.<br>
                    <strong>ℹ️ Note:</strong> After saving, ESP32 will reboot and connect to your WiFi.
                </div>
            </div>
            
            <!-- MQTT CONFIG -->
            <div class="card">
                <h2>📨 MQTT Configuration</h2>
                <input type="text" id="mqttServer" placeholder="MQTT Broker IP (e.g., 192.168.1.100)">
                <div class="form-row">
                    <input type="number" id="mqttPort" placeholder="Port" value="1883">
                    <input type="text" id="mqttTopic" placeholder="Topic (e.g., diesel)">
                </div>
                <div class="toggle-container">
                    <label class="toggle-switch">
                        <input type="checkbox" id="mqttAuthEnabled" onchange="toggleMQTTAuth()">
                        <span class="toggle-slider"></span>
                    </label>
                    <span class="toggle-label">Authentication Required</span>
                </div>
                <div id="mqttAuthFields" style="display: none;">
                    <input type="text" id="mqttUser" placeholder="Username">
                    <input type="password" id="mqttPass" placeholder="Password">
                </div>
                <button class="btn" onclick="saveMQTT()">💾 SAVE MQTT</button>
                <div class="info-box">
                    <strong>🏠 Home Assistant:</strong> Configure MQTT broker IP and topic.<br>
                    Topic: <code>diesel/error</code> - Short error name (perfect for HA history)
                </div>
            </div>
            
            <!-- OTA CONFIG -->
            <div class="card">
                <h2>🔄 OTA Update Configuration</h2>
                <div class="toggle-container">
                    <label class="toggle-switch">
                        <input type="checkbox" id="otaEnabled" checked>
                        <span class="toggle-slider"></span>
                    </label>
                    <span class="toggle-label">Enable OTA Updates</span>
                </div>
                <input type="password" id="otaPassword" placeholder="OTA Password (default: dieselpilot)">
                <button class="btn" onclick="saveOTA()">💾 SAVE OTA CONFIG</button>
                <div class="info-box">
                    <strong>📡 OTA Updates:</strong> Update firmware wirelessly using Arduino IDE or PlatformIO.<br>
                    <strong>Port:</strong> 3232 | <strong>Hostname:</strong> <span id="otaHostname">-</span><br>
                    <strong>⚠️ Security:</strong> Change default password for better security!<br>
                    <br>
                    <strong>Arduino IDE:</strong> Tools → Port → Network Ports → [hostname]<br>
                    <strong>Command Line:</strong> <code>platformio run -t upload --upload-port [IP]</code>
                </div>
            </div>
            
            <!-- TIME CONFIG -->
            <div class="card">
                <h2>🕐 Time & Time Zone</h2>
                <div class="form-row">
                    <div>
                        <label style="color:#888;">Time Zone</label>
                        <select id="timeZone" style="max-width:280px;">
                            <option value="Europe/London">London (GMT/BST)</option>
                            <option value="Europe/Dublin">Dublin (GMT/IST)</option>
                            <option value="Europe/Paris">Paris (CET/CEST)</option>
                            <option value="Europe/Berlin">Berlin (CET/CEST)</option>
                            <option value="America/New_York">New York (EST/EDT)</option>
                            <option value="America/Chicago">Chicago (CST/CDT)</option>
                            <option value="America/Denver">Denver (MST/MDT)</option>
                            <option value="America/Los_Angeles">Los Angeles (PST/PDT)</option>
                            <option value="America/Toronto">Toronto (EST/EDT)</option>
                            <option value="Australia/Sydney">Sydney (AEST/AEDT)</option>
                            <option value="Pacific/Auckland">Auckland (NZST/NZDT)</option>
                            <option value="Asia/Kolkata">India (IST)</option>
                            <option value="Asia/Tokyo">Tokyo (JST)</option>
                            <option value="Asia/Singapore">Singapore (SGT)</option>
                            <option value="UTC">UTC</option>
                        </select>
                    </div>
                </div>
                <div class="toggle-container" style="margin-top:12px;">
                    <label class="toggle-switch">
                        <input type="checkbox" id="daylightSavingEnabled">
                        <span class="toggle-slider"></span>
                    </label>
                    <span class="toggle-label">Enable daylight saving time</span>
                </div>
                <div class="info-box">
                    <strong>ℹ️ Daylight saving:</strong> When enabled, supported time zones automatically change between their standard and daylight-saving offsets. Turn it off to keep standard time year-round.
                </div>
                <button class="btn" onclick="saveTimeSettings()">💾 SAVE TIME SETTINGS</button>
            </div>
            
            <!-- OLED DISPLAY CONFIG -->
            <div class="card">
                <h2>🖥️ OLED Status Scroll Speed</h2>
                <div class="info-box">
                    Choose the scrolling speed for the Frost Mode and Pre-Heat status messages.<br>
                    <strong>120 ms = Slow</strong> &nbsp; | &nbsp; <strong>90 ms = Medium</strong> &nbsp; | &nbsp; <strong>60 ms = Fast</strong>
                </div>
                <div class="form-row">
                    <div>
                        <label style="color:#888;">Frost Mode</label>
                        <select id="frostScrollSpeed" style="max-width:200px;">
                            <option value="120">Slow (120 ms)</option>
                            <option value="90">Medium (90 ms)</option>
                            <option value="60">Fast (60 ms)</option>
                        </select>
                    </div>
                </div>
                <div>
                    <label style="color:#888;">Pre-Heat</label>
                    <select id="preheatScrollSpeed" style="max-width:200px;">
                        <option value="120">Slow (120 ms)</option>
                        <option value="90">Medium (90 ms)</option>
                        <option value="60">Fast (60 ms)</option>
                    </select>
                </div>
                <button class="btn" onclick="saveDisplaySettings()">💾 SAVE DISPLAY SETTINGS</button>
            </div>

            <!-- SYSTEM INFO -->
            <div class="card">
                <h2>ℹ️ System Info</h2>
                <div style="font-family: monospace; font-size: 0.9em; line-height: 1.6;">
                    <div>Hostname: <span id="hostname">-</span></div>
                    <div>WiFi Mode: <span id="wifiMode">-</span></div>
                    <div>IP Address: <span id="ipAddr">-</span></div>
                    <div>MQTT: <span id="mqttStatus">-</span></div>
                    <div>OTA: <span id="otaStatus">-</span></div>
                    <div>Uptime: <span id="uptime">-</span></div>
                </div>
                <div style="margin-top: 20px;">
                    <button class="btn" onclick="rebootDevice()" style="background: #ff9800;">🔄 REBOOT DEVICE</button>
                    <button class="btn" onclick="factoryReset()" style="background: #d32f2f;">⚠️ FACTORY RESET</button>
                </div>
                <div class="info-box">
                    <strong>🔄 Reboot:</strong> Restarts device without losing settings.<br>
                    <strong>⚠️ Factory Reset:</strong> Erases ALL settings (WiFi, MQTT, OTA, pairing). Use to reconfigure from scratch.
                </div>
            </div>
            
        </div>
    </div>
    
    <script>
        // TAB SWITCHING
        function switchTab(tabName, tabElement) {
            // Explicitly switch the visible content panel.
            document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));

            let panel = document.getElementById(tabName);
            if(panel) {
                panel.classList.add('active');
            }

            if(tabElement) {
                tabElement.classList.add('active');
            }
        }
        
        // STATUS UPDATE
        let preheatFormDirty = false;

        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(d => {
                    // State
                    document.getElementById('state').innerText = d.state;
                    let stateEl = document.getElementById('state');
                    stateEl.className = 'status-value ' + 
                        (d.state === 'RUNNING' ? 'state-running' : 
                         d.state === 'OFF' ? 'state-off' : 'state-other');
                    
                    // Error Code
                    document.getElementById('errorCode').innerText = d.errorName;
                    let errorEl = document.getElementById('errorCode');
                    errorEl.className = 'status-value ' + 
                        (d.errorCode === 0 ? 'error-ok' : 
                         d.errorCode <= 1 ? 'error-warning' : 'error-critical');
                    
                    // Error Alert
                    let errorAlert = document.getElementById('errorAlert');
                    if(d.errorCode > 0 && d.errorCode !== 1 && d.errorCode !== 12) {
                        errorAlert.classList.add('active');
                        document.getElementById('errorTitle').innerText = '⚠️ ' + d.errorName;
                    } else {
                        errorAlert.classList.remove('active');
                    }
                    
                    // Mode
                    document.getElementById('mode').innerText = d.mode;
                    let modeEl = document.getElementById('mode');
                    modeEl.className = 'status-value ' + 
                        (d.mode === 'AUTO' ? 'mode-auto' : 'mode-manual');
                    
                    // Basic values
                    document.getElementById('voltage').innerText = d.voltage + 'V';
                    document.getElementById('ambient').innerText = d.ambient + '°C';
                    document.getElementById('case').innerText = d.case + '°C';
                    document.getElementById('currentAddr').innerText = d.addr;

                    // Automatic mode status shown on the Dashboard.
                    let autoPreheatStatus = document.getElementById('autoPreheatStatus');
                    let autoFrostStatus = document.getElementById('autoFrostStatus');
                    if(autoPreheatStatus) {
                        autoPreheatStatus.innerText = d.preheatEnabled ? 'ON' : 'OFF';
                    }
                    if(autoFrostStatus) {
                        autoFrostStatus.innerText = d.frostMode ? 'ON' : 'OFF';
                    }

                    // Frost Mode settings
                    if(document.getElementById('frostEnabled')) {
                        document.getElementById('frostEnabled').checked = d.frostMode;
                        document.getElementById('frostStartTemp').value = d.frostStartTemp;
                        document.getElementById('frostStopTemp').value = d.frostStopTemp;
                        document.getElementById('frostRestartDelay').value = d.frostRestartDelay;
                    }

                    // Frost restart lockout is only displayed after the low-temperature
                    // threshold has been reached and an automatic restart is currently blocked.
                    let frostLockoutBox = document.getElementById('frostRestartStatus');
                    let frostRemaining = d.frostRestartRemainingSeconds || 0;
                    if(frostLockoutBox) {
                        if(d.frostMode && d.ambient < d.frostStartTemp && frostRemaining > 0) {
                            let mins = Math.floor(frostRemaining / 60);
                            let secs = frostRemaining % 60;
                            document.getElementById('frostRestartRemaining').innerText =
                                mins + ' min ' + String(secs).padStart(2, '0') + ' sec remaining before automatic restart';
                            frostLockoutBox.style.display = 'block';
                        } else {
                            frostLockoutBox.style.display = 'none';
                        }
                    }

                    // Pre-Heat settings/status
                    if(document.getElementById('preheatEnabled') && !preheatFormDirty) {
                        document.getElementById('preheatEnabled').checked = d.preheatEnabled;
                        let mask = d.preheatDays || 0;
                        document.querySelectorAll('.preheat-day').forEach(cb => {
                            cb.checked = (mask & (1 << parseInt(cb.value))) !== 0;
                        });
                        document.getElementById('preheatOnTime').value = d.preheatOn || '07:00';
                        document.getElementById('preheatOffTime').value = d.preheatOff || '08:00';
                        document.getElementById('preheatTimeStatus').innerText = d.timeSource === 'NTP' ?
                            'Time synchronisation: OK (UK time) - Current time: ' + (d.currentUKTime || '--:--') :
                            (d.timeSource === 'LAST KNOWN' ?
                                'Time synchronisation: LAST KNOWN TIME (NTP unavailable) - Current time: ' + (d.currentUKTime || '--:--') :
                                'Time synchronisation: WAITING FOR NTP - Current time: ' + (d.currentUKTime || '--:--'));
                    }
                    
                    // Heater name
                    let heaterNameEl = document.getElementById('heaterName');
                    if(heaterNameEl && document.activeElement !== heaterNameEl) {
                        heaterNameEl.value = d.heaterName || 'Diesel Pilot';
                    }

                    // OLED scroll speed settings
                    if(document.activeElement !== document.getElementById('frostScrollSpeed')) {
                        document.getElementById('frostScrollSpeed').value = d.frostScrollSpeed;
                    }
                    if(document.activeElement !== document.getElementById('preheatScrollSpeed')) {
                        document.getElementById('preheatScrollSpeed').value = d.preheatScrollSpeed;
                    }
                    if(document.activeElement !== document.getElementById('timeZone')) {
                        document.getElementById('timeZone').value = d.timeZone || 'Europe/London';
                    }
                    document.getElementById('daylightSavingEnabled').checked = d.daylightSavingEnabled !== false;

                    // Smart labels
                    if(d.mode === 'AUTO') {
                        document.getElementById('setpointLabel').innerText = 'Setpoint (°C)';
                        document.getElementById('pumpLabel').innerText = 'Pump (actual)';
                        document.getElementById('setpoint').innerText = d.setpoint + '°C';
                        document.getElementById('pump').innerText = d.pump + 'Hz';
                    } else {
                        document.getElementById('setpointLabel').innerText = 'Last Setpoint';
                        document.getElementById('pumpLabel').innerText = 'Pump (set)';
                        document.getElementById('setpoint').innerText = d.setpoint + '°C';
                        document.getElementById('pump').innerText = d.pump + 'Hz';
                    }
                });
        }
        
        // SYSTEM INFO UPDATE
        function updateSystemInfo() {
            fetch('/api/info')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('hostname').innerText = d.hostname;
                    document.getElementById('otaHostname').innerText = d.hostname;
                    document.getElementById('wifiMode').innerText = d.wifiMode;
                    document.getElementById('ipAddr').innerText = d.ip;
                    document.getElementById('mqttStatus').innerText = d.mqtt;
                    document.getElementById('otaStatus').innerText = d.ota;
                    document.getElementById('uptime').innerText = d.uptime;
                });
        }
        
        function sendCmd(cmd) {
            fetch('/api/cmd?c=' + cmd).then(() => setTimeout(updateStatus, 500));
        }

        function toggleFrostMode() {
            let enabled = document.getElementById('frostEnabled').checked ? '1' : '0';
            fetch('/api/frost?enabled=' + enabled)
                .then(r => r.text())
                .then(() => updateStatus());
        }

        function saveFrostSettings() {
            let start = parseInt(document.getElementById('frostStartTemp').value);
            let stop = parseInt(document.getElementById('frostStopTemp').value);
            let restartDelay = parseInt(document.getElementById('frostRestartDelay').value);

            if(isNaN(start) || isNaN(stop) || isNaN(restartDelay)) {
                alert('Please enter valid temperatures.');
                return;
            }

            if(stop < start + 5) {
                alert('The shutdown temperature must be at least 5°C higher than the start temperature.');
                return;
            }

            if(restartDelay < 30) {
                alert('The minimum restart delay is 30 minutes.');
                return;
            }

            fetch('/api/frost?enabled=' + (document.getElementById('frostEnabled').checked ? '1' : '0')
                + '&start=' + start + '&stop=' + stop + '&restart=' + restartDelay)
                .then(r => r.text())
                .then(msg => {
                    alert(msg);
                    updateStatus();
                });
        }
        
        function savePreheat() {
            let enabled = document.getElementById('preheatEnabled').checked ? '1' : '0';
            let days = 0;
            document.querySelectorAll('.preheat-day').forEach(cb => {
                if(cb.checked) days |= (1 << parseInt(cb.value));
            });

            let on = document.getElementById('preheatOnTime').value || '07:00';
            let off = document.getElementById('preheatOffTime').value || '08:00';
            let onParts = on.split(':');
            let offParts = off.split(':');
            let onHour = parseInt(onParts[0]);
            let onMinute = parseInt(onParts[1]);
            let offHour = parseInt(offParts[0]);
            let offMinute = parseInt(offParts[1]);

            if(onHour * 60 + onMinute >= offHour * 60 + offMinute) {
                alert('Please set an ON time earlier than the OFF time.');
                return;
            }
            if(enabled === '1' && days === 0) {
                alert('Please select at least one day.');
                return;
            }

            fetch('/api/preheat?enabled=' + enabled +
                  '&days=' + days +
                  '&onHour=' + onHour +
                  '&onMinute=' + onMinute +
                  '&offHour=' + offHour +
                  '&offMinute=' + offMinute)
                .then(r => r.text())
                .then(msg => {
                    preheatFormDirty = false;
                    alert(msg);
                    updateStatus();
                });
        }

        function autoPair() {
            if(confirm('Start auto pairing? Hold pairing button on heater!')) {
                fetch('/api/pair/auto').then(r => r.text()).then(alert);
            }
        }
        
        function manualPair() {
            let addr = document.getElementById('manualAddr').value;
            fetch('/api/pair/manual?addr=' + addr).then(r => r.text()).then(alert);
        }
        
        function saveHeaterName() {
            let name = document.getElementById('heaterName').value.trim();
            if(name.length === 0) {
                alert('Please enter a heater name.');
                return;
            }

            fetch('/api/heatername?name=' + encodeURIComponent(name))
                .then(r => r.text()).then(msg => {
                    alert(msg);
                    updateStatus();
                });
        }

        function saveWiFi() {
            let deviceName = document.getElementById('deviceName').value;
            let ssid = document.getElementById('wifiSSID').value;
            let pass = document.getElementById('wifiPass').value;
            if(confirm('Save WiFi config and reboot?')) {
                fetch('/api/wifi?deviceName=' + encodeURIComponent(deviceName) + 
                      '&ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass))
                    .then(r => r.text()).then(msg => {
                        alert(msg);
                        setTimeout(() => location.reload(), 2000);
                    });
            }
        }
        
        function toggleMQTTAuth() {
            let authFields = document.getElementById('mqttAuthFields');
            let checkbox = document.getElementById('mqttAuthEnabled');
            authFields.style.display = checkbox.checked ? 'block' : 'none';
        }
        
        function saveMQTT() {
            let server = document.getElementById('mqttServer').value;
            let port = document.getElementById('mqttPort').value;
            let topic = document.getElementById('mqttTopic').value;
            let authEnabled = document.getElementById('mqttAuthEnabled').checked ? '1' : '0';
            let user = document.getElementById('mqttUser').value;
            let pass = document.getElementById('mqttPass').value;
            
            fetch('/api/mqtt?server=' + server + '&port=' + port + '&topic=' + topic + 
                  '&authEnabled=' + authEnabled + '&user=' + encodeURIComponent(user) + 
                  '&pass=' + encodeURIComponent(pass))
                .then(r => r.text()).then(alert);
        }
        
        function saveTimeSettings() {
            let zone = document.getElementById('timeZone').value;
            let dst = document.getElementById('daylightSavingEnabled').checked ? '1' : '0';
            fetch('/api/time?zone=' + encodeURIComponent(zone) + '&dst=' + dst)
                .then(r => r.text())
                .then(msg => {
                    alert(msg);
                    updateStatus();
                });
        }

        function saveDisplaySettings() {
            let frost = parseInt(document.getElementById('frostScrollSpeed').value);
            let preheat = parseInt(document.getElementById('preheatScrollSpeed').value);

            if(isNaN(frost) || isNaN(preheat)) {
                alert('Please enter valid scroll speeds.');
                return;
            }

            frost = Math.max(50, Math.min(500, frost));
            preheat = Math.max(50, Math.min(500, preheat));

            fetch('/api/display?frost=' + frost + '&preheat=' + preheat)
                .then(r => r.text())
                .then(msg => {
                    alert(msg);
                    updateStatus();
                });
        }

        function saveOTA() {
            let enabled = document.getElementById('otaEnabled').checked ? '1' : '0';
            let password = document.getElementById('otaPassword').value;
            
            fetch('/api/ota?enabled=' + enabled + '&pass=' + encodeURIComponent(password))
                .then(r => r.text()).then(msg => {
                    alert(msg);
                    if(enabled === '1') {
                        setTimeout(() => location.reload(), 2000);
                    }
                });
        }
        
        function rebootDevice() {
            if(confirm('Reboot device now?')) {
                fetch('/api/reboot').then(() => {
                    alert('Device rebooting... Wait 10 seconds and refresh page.');
                });
            }
        }
        
        function factoryReset() {
            if(confirm('⚠️ FACTORY RESET - This will erase ALL settings!\n\nAre you absolutely sure?')) {
                if(confirm('⚠️ FINAL WARNING!\n\nAll WiFi, MQTT, OTA configs and heater pairing will be lost!\n\nContinue?')) {
                    fetch('/api/factory').then(() => {
                        alert('Factory reset complete! Device will reboot in AP mode.\n\nSSID: Diesel-Pilot\nPassword: 12345678');
                        setTimeout(() => location.reload(), 3000);
                    });
                }
            }
        }
        
        // Mark Pre-Heat form as changed so auto-refresh does not overwrite
        // settings while the user is editing them.
        document.querySelectorAll('#preheat input').forEach(el => {
            el.addEventListener('change', () => { preheatFormDirty = true; });
            el.addEventListener('input', () => { preheatFormDirty = true; });
        });

        // Auto-refresh
        setInterval(updateStatus, 2000);
        setInterval(updateSystemInfo, 5000);
        updateStatus();
        updateSystemInfo();
    </script>
</body>
</html>
)rawliteral";
    
    server.send(200, "text/html", html);
}

void handleAPI_Status() {
    String json = "{";
    json += "\"state\":\"" + String(getStateName(heaterStatus.state)) + "\",";
    json += "\"voltage\":" + String(heaterStatus.voltage / 10.0, 1) + ",";
    json += "\"ambient\":" + String(heaterStatus.ambientTemp) + ",";
    json += "\"case\":" + String(heaterStatus.caseTemp) + ",";
    json += "\"setpoint\":" + String(heaterStatus.setpoint) + ",";
    json += "\"pump\":" + String(heaterStatus.pumpFreq / 10.0, 1) + ",";
    json += "\"mode\":\"" + String(heaterStatus.autoMode ? "AUTO" : "MANUAL") + "\",";
    json += "\"rssi\":" + String(heaterStatus.rssi) + ",";
    json += "\"addr\":\"" + (heaterPaired ? String(heaterAddress, HEX) : "Not paired") + "\",";
    
    // Error: only short name
    json += "\"errorCode\":" + String(heaterStatus.errorCode) + ",";
    json += "\"errorName\":\"" + String(getErrorName(heaterStatus.errorCode)) + "\",";
    json += "\"heaterName\":\"" + heaterName + "\",";
    json += "\"frostMode\":" + String(frostMode ? "true" : "false") + ",";
    json += "\"frostStartTemp\":" + String(frostStartTemp) + ",";
    json += "\"frostStopTemp\":" + String(frostStopTemp) + ",";
    json += "\"frostRestartDelay\":" + String(frostRestartDelayMinutes) + ",";
    uint32_t frostRemainingSeconds = 0;
    if(frostMode && heaterStatus.lastUpdate > 0 && heaterStatus.ambientTemp < frostStartTemp && frostLastShutdownMillis != 0) {
        uint32_t lockoutMs = (uint32_t)frostRestartDelayMinutes * 60000UL;
        uint32_t elapsed = millis() - frostLastShutdownMillis;
        if(elapsed < lockoutMs) frostRemainingSeconds = (lockoutMs - elapsed + 999UL) / 1000UL;
    }
    json += "\"frostRestartRemainingSeconds\":" + String(frostRemainingSeconds) + ",";
    json += "\"preheatEnabled\":" + String(preheatEnabled ? "true" : "false") + ",";
    json += "\"preheatDays\":" + String(preheatDays) + ",";
    json += "\"preheatOn\":\"" + formatScheduleTime(preheatOnMinute) + "\",";
    json += "\"preheatOff\":\"" + formatScheduleTime(preheatOffMinute) + "\",";
    json += "\"timeSynced\":" + String(timeSynced ? "true" : "false") + ",";
    json += "\"timeSource\":\"" + String(timeFromNTP ? "NTP" : (timeSynced ? "LAST KNOWN" : "WAITING")) + "\",";
    json += "\"currentUKTime\":\"" + getCurrentTimeString() + "\",";
    json += "\"currentTime\":\"" + getCurrentTimeString() + "\",";
    json += "\"timeZone\":\"" + timeZone + "\",";
    json += "\"daylightSavingEnabled\":" + String(daylightSavingEnabled ? "true" : "false") + ",";
    json += "\"frostScrollSpeed\":" + String(frostStatusScrollInterval) + ",";
    json += "\"preheatScrollSpeed\":" + String(preheatStatusScrollInterval);
    
    json += "}";
    server.send(200, "application/json", json);
}

void handleAPI_Command() {
    String cmd = server.arg("c");
    if(cmd == "power") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_POWER);
    } else if(cmd == "up") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_UP);
        notePreheatSetpointChange();
    }
    else if(cmd == "down") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_DOWN);
        notePreheatSetpointChange();
    }
    else if(cmd == "mode") {
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_MODE);
    }
    server.send(200, "text/plain", "OK");
}

void handleAPI_HeaterName() {
    String requestedName = server.arg("name");
    requestedName.trim();

    if(requestedName.length() == 0) {
        server.send(400, "text/plain", "Heater name cannot be empty");
        return;
    }

    if(requestedName.length() > 32) {
        requestedName = requestedName.substring(0, 32);
    }

    heaterName = requestedName;
    prefs.putString("heaterName", heaterName);

    server.send(200, "text/plain", "Heater name saved!");
}

void handleAPI_Frost() {
    frostMode = (server.arg("enabled") == "1");

    if(server.hasArg("start")) {
        int requestedStart = server.arg("start").toInt();
        int requestedStop = server.hasArg("stop") ? server.arg("stop").toInt() : frostStopTemp;
        int requestedRestart = server.hasArg("restart") ? server.arg("restart").toInt() : frostRestartDelayMinutes;

        // Keep the settings within sensible limits. Frost hysteresis must be
        // at least 5°C and the automatic restart delay must be at least 30 minutes.
        requestedStart = constrain(requestedStart, -30, 30);
        requestedStop = constrain(requestedStop, -30, 40);
        requestedRestart = constrain(requestedRestart, 30, 240);

        if(requestedStop < requestedStart + 5) {
            server.send(400, "text/plain", "Shutdown temperature must be at least 5C higher than start temperature");
            return;
        }

        frostStartTemp = requestedStart;
        frostStopTemp = requestedStop;
        frostRestartDelayMinutes = (uint16_t)requestedRestart;
        prefs.putInt("frostStart", frostStartTemp);
        prefs.putInt("frostStop", frostStopTemp);
        prefs.putUInt("frostRestart", frostRestartDelayMinutes);

        Serial.printf("❄️ Frost settings saved: start below %dC, stop at %dC, restart delay %u minutes\n",
                      frostStartTemp, frostStopTemp, frostRestartDelayMinutes);
    }

    if(frostMode) {
        Serial.println("❄️ Frost Mode ENABLED");
        // Start a fresh automatic frost cycle.
        frostHeaterStarted = false;
    } else {
        Serial.println("❄️ Frost Mode DISABLED");
        frostHeaterStarted = false;
    }

    prefs.putBool("frostMode", frostMode);

    server.send(200, "text/plain", frostMode ? "Frost Mode enabled" : "Frost Mode disabled");
}

void handleAPI_Preheat() {
    preheatEnabled = (server.arg("enabled") == "1");

    uint8_t days = (uint8_t)server.arg("days").toInt();
    preheatDays = days & 0x7F;

    int onHour = server.arg("onHour").toInt();
    int onMinute = server.arg("onMinute").toInt();
    int offHour = server.arg("offHour").toInt();
    int offMinute = server.arg("offMinute").toInt();

    if(onHour < 0) onHour = 0;
    if(onHour > 23) onHour = 23;
    if(onMinute < 0) onMinute = 0;
    if(onMinute > 59) onMinute = 59;
    if(offHour < 0) offHour = 0;
    if(offHour > 23) offHour = 23;
    if(offMinute < 0) offMinute = 0;
    if(offMinute > 59) offMinute = 59;

    preheatOnMinute = onHour * 60 + onMinute;
    preheatOffMinute = offHour * 60 + offMinute;

    prefs.putBool("preheatEn", preheatEnabled);
    prefs.putUChar("preheatDays", preheatDays);
    prefs.putUShort("preheatOn", preheatOnMinute);
    prefs.putUShort("preheatOff", preheatOffMinute);

    // Changing the schedule starts a fresh scheduler state.
    preheatSessionDay = -1;
    preheatHeaterStarted = false;
    preheatOffCancelledToday = false;
    preheatStartSetpoint = -1000;
    preheatLastMinute = -1;

    Serial.println(String("⏰ Pre-Heat ") + (preheatEnabled ? "ENABLED" : "DISABLED") +
                   " | Days mask: " + String(preheatDays) +
                   " | ON: " + formatScheduleTime(preheatOnMinute) +
                   " | OFF: " + formatScheduleTime(preheatOffMinute));

    if(preheatOnMinute >= preheatOffMinute) {
        server.send(200, "text/plain", "Pre-Heat saved. ON time must be earlier than OFF time.");
    } else {
        server.send(200, "text/plain", "Pre-Heat schedule saved");
    }
}

void handleAPI_Time() {
    String requestedZone = server.arg("zone");
    bool requestedDST = (server.arg("dst") == "1");

    // Only accept zones implemented by applyTimezone().
    const char* validZones[] = {
        "Europe/London", "Europe/Dublin", "Europe/Paris", "Europe/Berlin",
        "America/New_York", "America/Chicago", "America/Denver", "America/Los_Angeles",
        "America/Toronto", "Australia/Sydney", "Pacific/Auckland", "Asia/Kolkata",
        "Asia/Tokyo", "Asia/Singapore", "UTC"
    };
    bool valid = false;
    for(size_t i = 0; i < sizeof(validZones) / sizeof(validZones[0]); i++) {
        if(requestedZone == validZones[i]) { valid = true; break; }
    }

    if(!valid) {
        server.send(400, "text/plain", "Invalid time zone");
        return;
    }

    timeZone = requestedZone;
    daylightSavingEnabled = requestedDST;
    prefs.putString("timeZone", timeZone);
    prefs.putBool("dstEnabled", daylightSavingEnabled);
    applyTimezone();

    server.send(200, "text/plain", "Time zone settings saved");
}

void handleAPI_Display() {
    if(server.hasArg("frost")) frostStatusScrollInterval = constrain(server.arg("frost").toInt(), 50, 500);
    if(server.hasArg("preheat")) preheatStatusScrollInterval = constrain(server.arg("preheat").toInt(), 50, 500);

    prefs.putUInt("frostScroll", (uint32_t)frostStatusScrollInterval);
    prefs.putUInt("preheatScroll", (uint32_t)preheatStatusScrollInterval);

    lastFrostStatusScroll = millis();
    lastPreheatStatusScroll = millis();

    Serial.printf("OLED scroll speeds saved: Frost %lums | Pre-Heat %lums\n",
                  frostStatusScrollInterval, preheatStatusScrollInterval);
    server.send(200, "text/plain", "OLED scroll speeds saved");
}

void handleAPI_PairAuto() {
    uint32_t addr = findHeater(60000);
    if(addr != 0) {
        heaterAddress = addr;
        heaterPaired = true;
        prefs.putUInt("heaterAddr", heaterAddress);
        server.send(200, "text/plain", "Paired: 0x" + String(heaterAddress, HEX));
    } else {
        server.send(200, "text/plain", "Pairing failed!");
    }
}

void handleAPI_PairManual() {
    String addrStr = server.arg("addr");
    heaterAddress = strtoul(addrStr.c_str(), NULL, 0);
    heaterPaired = true;
    prefs.putUInt("heaterAddr", heaterAddress);
    server.send(200, "text/plain", "Paired: 0x" + String(heaterAddress, HEX));
}

void handleAPI_WiFi() {
    deviceName = server.arg("deviceName");
    if(deviceName.length() == 0) deviceName = "DieselPilot";
    staSSID = server.arg("ssid");
    staPassword = server.arg("pass");
    
    prefs.putString("deviceName", deviceName);
    prefs.putString("staSSID", staSSID);
    prefs.putString("staPass", staPassword);
    
    server.send(200, "text/plain", "WiFi saved! Rebooting...");
    delay(1000);
    ESP.restart();
}

void handleAPI_MQTT() {
    mqttServer = server.arg("server");
    mqttPort = server.arg("port").toInt();
    mqttTopic = server.arg("topic");
    mqttAuthEnabled = (server.arg("authEnabled") == "1");
    mqttUser = server.arg("user");
    mqttPassword = server.arg("pass");
    mqttEnabled = (mqttServer.length() > 0);
    
    prefs.putString("mqttServer", mqttServer);
    prefs.putInt("mqttPort", mqttPort);
    prefs.putString("mqttTopic", mqttTopic);
    prefs.putBool("mqttAuthEn", mqttAuthEnabled);
    prefs.putString("mqttUser", mqttUser);
    prefs.putString("mqttPass", mqttPassword);
    prefs.putBool("mqttEnabled", mqttEnabled);
    WiFi.setHostname(deviceName.c_str());
    
    server.send(200, "text/plain", "MQTT saved!");
    
    if(mqttEnabled) {
        connectMQTT();
    }
}

void handleAPI_Info() {
    String json = "{";
    json += "\"hostname\":\"" + deviceName + "\",";
    json += "\"wifiMode\":\"" + String(staConnected ? "AP + STA" : "AP") + "\",";
    json += "\"ip\":\"" + (staConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\",";
    json += "\"apIP\":\"" + (apStarted ? WiFi.softAPIP().toString() : "") + "\",";
    json += "\"staIP\":\"" + (staConnected ? WiFi.localIP().toString() : "") + "\",";
    json += "\"mqtt\":\"" + String(mqttEnabled && mqtt.connected() ? "Connected" : "Disconnected") + "\",";
    json += "\"ota\":\"" + String(otaEnabled ? "Enabled" : "Disabled") + "\",";
    json += "\"uptime\":\"" + String(millis() / 1000 / 60) + " min\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleAPI_OTA() {
    otaEnabled = (server.arg("enabled") == "1");
    String newPass = server.arg("pass");
    if(newPass.length() > 0) {
        otaPassword = newPass;
    }
    
    prefs.putBool("otaEnabled", otaEnabled);
    prefs.putString("otaPass", otaPassword);
    
    server.send(200, "text/plain", "OTA config saved! Reboot to apply.");
}

void handleAPI_Reboot() {
    server.send(200, "text/plain", "Rebooting...");
    delay(500);
    ESP.restart();
}

void handleAPI_Factory() {
    server.send(200, "text/plain", "Factory reset in progress...");
    
    Serial.println("\n⚠️ FACTORY RESET - Clearing all preferences...");
    
    // Clear all stored preferences
    prefs.clear();
    
    // Show on OLED
    displayLine1 = "FACTORY RESET";
    displayLine2 = "Clearing...";
    displayLine3 = "All settings";
    displayLine4 = "erased!";
    updateDisplay();
    
    delay(2000);
    
    Serial.println("✅ Factory reset complete! Rebooting...");
    ESP.restart();
}

// ═══════════════════════════════════════════════════════════════════════════
// PHYSICAL BUTTONS
// ═══════════════════════════════════════════════════════════════════════════

void handlePhysicalButtons() {
    // Each button is active LOW because it is wired to GND and uses
    // the ESP32 internal pull-up resistor.
    static bool lastPower = HIGH;
    static bool lastUp = HIGH;
    static bool lastDown = HIGH;
    static bool lastMode = HIGH;

    static unsigned long lastPowerTime = 0;
    static unsigned long lastUpTime = 0;
    static unsigned long lastDownTime = 0;
    static unsigned long lastModeTime = 0;

    unsigned long now = millis();

    bool power = digitalRead(BUTTON_POWER);
    bool up = digitalRead(BUTTON_UP);
    bool down = digitalRead(BUTTON_DOWN);
    bool mode = digitalRead(BUTTON_MODE);

    // Trigger only when the button changes from HIGH to LOW.
    // This prevents a held button from repeatedly sending commands.
    if (power == LOW && lastPower == HIGH && now - lastPowerTime > BUTTON_DEBOUNCE_MS) {
        lastPowerTime = now;
        Serial.println("Physical button: POWER");
        showPhysicalButtonIndicator('P');
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_POWER);
    }

    if (up == LOW && lastUp == HIGH && now - lastUpTime > BUTTON_DEBOUNCE_MS) {
        lastUpTime = now;
        Serial.println("Physical button: UP");
        showPhysicalButtonIndicator('U');
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_UP);
        notePreheatSetpointChange();
    }

    if (down == LOW && lastDown == HIGH && now - lastDownTime > BUTTON_DEBOUNCE_MS) {
        lastDownTime = now;
        Serial.println("Physical button: DOWN");
        showPhysicalButtonIndicator('D');
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_DOWN);
        notePreheatSetpointChange();
    }

    if (mode == LOW && lastMode == HIGH && now - lastModeTime > BUTTON_DEBOUNCE_MS) {
        lastModeTime = now;
        Serial.println("Physical button: MODE");
        showPhysicalButtonIndicator('M');
        cancelAutomaticModesForManualInput();
        sendCommand(CMD_MODE);
    }

    lastPower = power;
    lastUp = up;
    lastDown = down;
    lastMode = mode;
}

// ═══════════════════════════════════════════════════════════════════════════
// Load all saved user settings from Preferences.
void loadSettings() {
    heaterAddress = prefs.getUInt("heaterAddr", 0);

    heaterPaired = (heaterAddress != 0);
    deviceName = prefs.getString("deviceName", "DieselPilot");

    heaterName = prefs.getString("heaterName", "Diesel Pilot");

    staSSID = prefs.getString("staSSID", "");

    staPassword = prefs.getString("staPass", "");

    timeZone = prefs.getString("timeZone", "Europe/London");
    daylightSavingEnabled = prefs.getBool("dstEnabled", true);

    // If no WiFi credentials have previously been saved in Preferences,
    // use the credentials defined at the top of the sketch.
    if (staSSID.length() == 0) {
        staSSID = "SEARS";
        staPassword = "Summer08Mylee";
        Serial.println("WiFi credentials not saved - using sketch defaults");
    }

    mqttServer = prefs.getString("mqttServer", "");

    mqttPort = prefs.getInt("mqttPort", 1883);

    mqttTopic = prefs.getString("mqttTopic", "diesel");

    mqttAuthEnabled = prefs.getBool("mqttAuthEn", false);

    mqttUser = prefs.getString("mqttUser", "");

    mqttPassword = prefs.getString("mqttPass", "");

    mqttEnabled = prefs.getBool("mqttEnabled", false);

    otaEnabled = prefs.getBool("otaEnabled", true);

    otaPassword = prefs.getString("otaPass", "dieselpilot");

    frostMode = prefs.getBool("frostMode", false);

    frostStartTemp = prefs.getInt("frostStart", 3);

    frostStopTemp = prefs.getInt("frostStop", 10);

    frostRestartDelayMinutes = (uint16_t)constrain(prefs.getUInt("frostRestart", 30), 30UL, 240UL);

    if(frostStopTemp < frostStartTemp + 5) frostStopTemp = frostStartTemp + 5;
    frostHeaterStarted = false;

    preheatEnabled = prefs.getBool("preheatEn", false);

    preheatDays = prefs.getUChar("preheatDays", 0);

    preheatOnMinute = prefs.getUShort("preheatOn", 420);

    preheatOffMinute = prefs.getUShort("preheatOff", 480);

    preheatHeaterStarted = false;
    preheatOffCancelledToday = false;
    preheatSessionDay = -1;
    preheatStartSetpoint = -1000;
    preheatLastMinute = -1;

    // OLED scroll speeds (milliseconds per pixel step)
    frostStatusScrollInterval = constrain((unsigned long)prefs.getUInt("frostScroll", 120), 50UL, 500UL);
    preheatStatusScrollInterval = constrain((unsigned long)prefs.getUInt("preheatScroll", 120), 50UL, 500UL);
    
}

// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n═══════════════════════════════════════");
    Serial.println("    DIESEL PILOT WEB + ERROR CODES");
    Serial.println("═══════════════════════════════════════\n");
    
#if USE_OLED
    Wire.begin(PIN_SDA, PIN_SCL);
    display.begin();
    display.setContrast(155);
    displayLine1 = "Diesel Pilot";
    displayLine2 = "Starting...";
    displayLine3 = "Mixed up by Sears";
    displayLine4 = "Happy Heating :)";
    updateDisplay();
    Serial.println("✅ OLED initialized");
    delay(2000);
#else
    Serial.println("ℹ️  OLED disabled");
#endif
    prefs.begin("diesel", false);

    loadSettings();

    // Physical control buttons
    pinMode(BUTTON_POWER, INPUT_PULLUP);
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_MODE, INPUT_PULLUP);

    pinMode(PIN_SCK, OUTPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_GDO2, INPUT);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
    cc1101_init();
    WiFi.setHostname(deviceName.c_str());
    WiFi.mode(WIFI_AP_STA);

    // Start the DieselPilot access point initially as a recovery network.
    // If normal WiFi connects successfully, the AP is shut down below.
    // If normal WiFi fails, the AP remains available for recovery/configuration.
    apStarted = WiFi.softAP(apSSID.c_str(), apPassword.c_str());
    useAP = apStarted;
    if(apStarted) {
        Serial.println("✅ DieselPilot AP started");
        Serial.println("SSID: " + apSSID);
        Serial.println("AP IP: " + WiFi.softAPIP().toString());
        displayLine2 = "AP: " + apSSID;
        displayLine3 = WiFi.softAPIP().toString();
    }

    if(staSSID.length() > 0) {
        Serial.println("Connecting to WiFi: " + staSSID);
        WiFi.begin(staSSID.c_str(), staPassword.c_str());
        int attempts = 0;
        while(WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }

        if(WiFi.status() == WL_CONNECTED) {
            staConnected = true;
            Serial.println("\n✅ WiFi connected!");
            Serial.println("STA IP: " + WiFi.localIP().toString());

            // Normal WiFi is now available, so shut down the DieselPilot
            // recovery access point. This leaves the ESP32 in STA-only mode.
            if(apStarted) {
                Serial.println("🔌 Stopping DieselPilot AP - normal WiFi is connected");
                WiFi.softAPdisconnect(true);
                apStarted = false;
            }
            WiFi.mode(WIFI_STA);
            useAP = false;

            displayLine2 = "WiFi: " + staSSID;
            displayLine3 = WiFi.localIP().toString();
        } else {
            staConnected = false;
            Serial.println("\n⚠️ WiFi not connected - AP remains available for recovery");
        }
    }

    // Restore a previously saved time immediately. If NTP is available it will
    // replace this fallback shortly afterwards.
    restoreLastKnownTime();

    if(staConnected) {
        configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
        applyTimezone();
        lastNtpAttempt = millis();
        struct tm initialTime;
        if(getLocalDateTime(initialTime)) {
            Serial.println("⏳ NTP requested - using last known time until NTP confirms synchronisation");
        } else {
            Serial.println("⏳ NTP requested - waiting for synchronisation");
        }
    } else if(timeSynced) {
        Serial.println("⚠️ No WiFi connection - using last known time until NTP is available");
    } else {
        Serial.println("⚠️ No WiFi connection and no saved time - Pre-Heat waiting for NTP");
    }
    displayLine4 = heaterPaired ? "Paired!" : "Not paired";
    updateDisplay();
    // Init OTA (only if connected to WiFi, not in AP mode)
    if(staConnected && otaEnabled) {
        setupOTA();
    }
    
    if(mqttEnabled) {
        connectMQTT();
    }
    
    server.on("/", handleRoot);
    server.on("/api/status", handleAPI_Status);
    server.on("/api/info", handleAPI_Info);
    server.on("/api/cmd", handleAPI_Command);
    server.on("/api/heatername", handleAPI_HeaterName);
    server.on("/api/frost", handleAPI_Frost);
    server.on("/api/preheat", handleAPI_Preheat);
    server.on("/api/time", handleAPI_Time);
    server.on("/api/display", handleAPI_Display);
    server.on("/api/pair/auto", handleAPI_PairAuto);
    server.on("/api/pair/manual", handleAPI_PairManual);
    server.on("/api/wifi", handleAPI_WiFi);
    server.on("/api/mqtt", handleAPI_MQTT);
    server.on("/api/ota", handleAPI_OTA);
    server.on("/api/factory", handleAPI_Factory);
    server.on("/api/reboot", handleAPI_Reboot);
    server.begin();
    
    Serial.println("\n✅ Web server started");
    Serial.println("Ready!");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastDisplay = 0;
    
    yield();

    // Check physical heater control buttons
    handlePhysicalButtons();

    server.handleClient();

    // Keep NTP synchronisation alive and maintain a saved fallback clock.
    handleTimeSync();
    
    // Handle OTA updates
    if(otaEnabled && staConnected) {
        ArduinoOTA.handle();
    }
    
    if(mqttEnabled && !mqtt.connected()) {
        if(millis() - lastMQTTRetry > mqttRetryInterval) {
            lastMQTTRetry = millis();
            connectMQTT();
        }
    }
    if(mqttEnabled && mqtt.connected()) {
        mqtt.loop();
    }
    
    if(millis() - lastUpdate > 3000 && heaterPaired) {
        lastUpdate = millis();
        updateHeaterStatus();
    }

    // Check the Pre-Heat schedule every loop. The scheduler itself only acts
    // at the configured ON/OFF times and when a valid NTP time is available.
    handlePreheatSchedule();
    
    if(millis() - lastDisplay > 50) {
        lastDisplay = millis();
        
        if(heaterPaired && heaterStatus.lastUpdate > 0) {
            // Line 1: Always title
            displayLine1 = "DIESEL PILOT " + version;
            
            // Line 2: Big state name
            displayLine2 = String(getStateName(heaterStatus.state));
            
            // Line 3: Temperature info or error
            if(heaterStatus.errorCode > 1 && heaterStatus.errorCode != 12) {
                // Critical error - show it!
                displayLine3 = "! " + String(getErrorName(heaterStatus.errorCode)) + " !";
            } else {
                // Normal operation - show temps
                if(heaterStatus.autoMode) {
                    // AUTO mode - show ambient -> setpoint
                    displayLine3 = String(heaterStatus.ambientTemp) + "C -> " + String(heaterStatus.setpoint) + "C";
                } else {
                    // MANUAL mode - show ambient + pump
                    displayLine3 = String(heaterStatus.ambientTemp) + "C  P:" + String(heaterStatus.pumpFreq / 10.0, 1) + "Hz";
                }
            }
            
            // Line 4: Voltage + Mode
            String modeIcon = heaterStatus.autoMode ? "A" : "M";
            displayLine4 = String(heaterStatus.voltage / 10.0, 1) + "V  [" + modeIcon + "]  " + String(heaterStatus.caseTemp) + "C";
            
        } else if(heaterPaired) {
            // Paired but no data yet
            displayLine1 = "DIESEL PILOT";
            displayLine2 = "Waiting...";
            displayLine3 = "No data";
            displayLine4 = "";
        }
        
        updateDisplay();
    }
}
