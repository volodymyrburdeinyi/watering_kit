#include <Wire.h>
#include "src/U8glib/U8glib.h"
#include "src/RTClib/src/RTClib.h"
#include <EEPROM.h>
// EEPROM configuration
#define EEPROM_MAGIC 0xABCD  // Magic number to verify valid data
#define EEPROM_START_ADDR 0
#define EEPROM_MAGIC_ADDR EEPROM_START_ADDR
#define EEPROM_PROGRAMS_ADDR (EEPROM_START_ADDR + 2)

// Display setup
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);    // I2C

// RTC setup
RTC_DS1307 RTC;

// Pin definitions
#define NUM_CHANNELS 4
#define PUMP_PIN 4

// Encoder pins and interrupt
#define ENC_INT PE6  // INT6 for all inputs detection
#define ENC_A MISO   // Encoder A signal
#define ENC_B MOSI   // Encoder B signal
#define ENC_BTN SCK  // Encoder button

// Direct port reading bits
#define ENC_A_BIT  3  // MISO is PB3
#define ENC_B_BIT  2  // MOSI is PB2
#define ENC_BTN_BIT 1 // SCK is PB1
#define readEncoderA() (!!(PINB & (1 << ENC_A_BIT)))
#define readEncoderB() (!!(PINB & (1 << ENC_B_BIT)))
#define readEncoderButton() (!!(PINB & (1 << ENC_BTN_BIT)))

// Timing constants
const unsigned long LONG_PRESS_TIME = 1000;     // 1 second for long press
const unsigned long DEBOUNCE_TIME = 50;         // Button debounce
const unsigned long INTERRUPT_DEBOUNCE = 5000;  // Encoder interrupt debounce

// Channel configuration
const uint8_t CHANNEL_PINS[NUM_CHANNELS] = {6, 8, 9, 10}; // relay1-4 pins
bool channelStates[NUM_CHANNELS] = {false};
bool pumpState = false;

// Days of week
const char* daysOfTheWeek[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

// Programm day patterns
uint8_t currentPattern = 0;

// Day patterns definition
struct DayPattern {
  const bool days[7];
  const char* name;
};
const DayPattern PATTERNS[] PROGMEM = {
  {{1, 1, 1, 1, 1, 1, 1}, "ALL"}, // Every day
  {{1, 0, 0, 0, 0, 0, 0}, "MON"}, // Monday only
  {{0, 1, 0, 0, 0, 0, 0}, "TUE"}, // Tuesday only
  {{0, 0, 1, 0, 0, 0, 0}, "WED"}, // Wednesday only
  {{0, 0, 0, 1, 0, 0, 0}, "THU"}, // Thursday only
  {{0, 0, 0, 0, 1, 0, 0}, "FRI"}, // Friday only
  {{0, 0, 0, 0, 0, 1, 0}, "SAT"}, // Saturday only
  {{0, 0, 0, 0, 0, 0, 1}, "SUN"}, // Sunday only
  {{1, 1, 1, 1, 1, 0, 0}, "WORK"}, // Weekdays
  {{0, 0, 0, 0, 0, 1, 1}, "WEEK"}, // Weekend
  {{1, 0, 1, 0, 1, 0, 0}, "ODD"}, // Odd days
  {{0, 1, 0, 1, 0, 1, 0}, "EVEN"}, // Even days
};
const uint8_t NUM_PATTERNS = sizeof(PATTERNS) / sizeof(PATTERNS[0]);
const char* dayPatternName = "ALL";  // Default pattern name

// Volatile state variables
volatile int encoderValue = 0;
volatile int lastEncoderValue = 0;
volatile uint8_t lastA = HIGH;
volatile uint8_t lastB = HIGH;
volatile int8_t stepCount = 0;
volatile bool stepReady = false;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long lastButtonTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

// Screen field enums
enum HomeFields { HOME_NONE = 0 };
enum ConfigFields {
  PROG_NUMBER = 0,
  PROG_ACTIVE,
  PROG_START,
  PROG_DAYS,
  PROG_DURATION,
  PROG_CHANNELS,
  PROG_FIELDS_COUNT
};
enum ClockFields {
  CLOCK_YEAR = 0,
  CLOCK_MONTH,
  CLOCK_DAY,
  CLOCK_HOUR,
  CLOCK_MIN,
  CLOCK_SEC
};

// Program settings structure
struct Program {
  uint8_t number;     // 1-9
  uint8_t hour;       // 0-23
  uint8_t minute;     // 0-59
  uint8_t duration;   // seconds
  bool days[7];       // M T W T F S S
  bool channels[4];   // 1 2 3 4
  bool active;        // Whether this program is enabled
};
// Array of programs
#define MAX_PROGRAMS 9
Program programs[MAX_PROGRAMS];

// Screen states structure
struct ScreenState {
  uint8_t screen;    // 0:home, 1:config, 2:clock
  uint8_t field;     // Currently selected field
  uint8_t currentProgram;  // Index of program being edited
};

// Global state
ScreenState state = {
  0,    // Start at home screen
  0,    // No field selected
  0     // Start with first program
};
uint8_t daySelector = 0;  // Current day being selected
uint8_t channelSelector = 0;  // Current channel being selected
uint8_t channelBits = 0;  // Binary representation of channels (0000-1111)


struct ScreenManager {
  unsigned long lastUserInteraction = 0;
  const unsigned long SCREEN_TIMEOUT = 30000;  // 30 seconds
  const unsigned long DIM_TIMEOUT = 15000;     // 15 seconds to dim
  bool screenOn = true;
  bool screenDimmed = false;
  uint8_t normalContrast = 255;
  uint8_t dimContrast = 32;

  void setContrast(uint8_t contrast) {
    Wire.beginTransmission(0x3C);
    Wire.write(0x00);
    Wire.write(0x81);
    Wire.write(contrast);
    Wire.endTransmission();
  }

  void turnOff() {
    Wire.beginTransmission(0x3C);
    Wire.write(0x00);
    Wire.write(0xAE);  // Display OFF
    Wire.endTransmission();
  }

  void turnOn() {
    Wire.beginTransmission(0x3C);
    Wire.write(0x00);
    Wire.write(0xAF);  // Display ON
    Wire.endTransmission();
  }

  void checkPower() {
    unsigned long timeSinceInteraction = millis() - lastUserInteraction;

    if (screenOn && timeSinceInteraction > SCREEN_TIMEOUT) {
      screenOn = false;
      screenDimmed = false;
      turnOff();
      Serial.println("Screen OFF");
    } else if (screenOn && !screenDimmed && timeSinceInteraction > DIM_TIMEOUT) {
      screenDimmed = true;
      setContrast(dimContrast);
      Serial.println("Screen DIMMED");
    }
  }

  void wake() {
    if (!screenOn) {
      screenOn = true;
      screenDimmed = false;
      turnOn();
      setContrast(normalContrast);
      Serial.println("Screen ON");
    } else if (screenDimmed) {
      screenDimmed = false;
      setContrast(normalContrast);
      Serial.println("Screen BRIGHT");
    }
    lastUserInteraction = millis();
  }
};
ScreenManager screen;

// Function declarations
void handleButtonPress(bool isPressed);
void handleEncoderChange();
void updateConfigValue(int change);
void updateClockValue(int change);
void drawHomeScreen();
void drawConfigScreen();
void drawClockScreen();
void drawFieldHighlight();
void drawClockFieldHighlight();
void updateWateringSystem();
void testEncoder();
void testScreenTransitions();

void setup() {
  // Initialize display first
  u8g.begin();  // Add this line
  u8g.setFont(u8g_font_6x10);

  // Clear display and show initial message
  u8g.firstPage();
  do {
    u8g.drawStr(0, 10, "Initializing...");
  } while (u8g.nextPage());

  // Initialize communication
  Wire.begin();
  RTC.begin();
  Serial.begin(9600);

  // Initialize encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(ENC_INT, INPUT_PULLUP);

  // Initialize outputs
  pinMode(PUMP_PIN, OUTPUT);
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    pinMode(CHANNEL_PINS[i], OUTPUT);
    digitalWrite(CHANNEL_PINS[i], LOW);
  }
  digitalWrite(PUMP_PIN, LOW);

  // Setup INT6 interrupt
  EICRB &= ~(1 << ISC60);
  EICRB |= (1 << ISC61);
  EIMSK |= (1 << INT6);
  sei();

  // Initialize RTC if needed
  if (!RTC.isrunning()) {
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Try to load programs from EEPROM
  if (!loadPrograms()) {
    // If no valid data, initialize defaults and save
    initializeDefaultPrograms();
    savePrograms();
  }

  delay(1000);  // Show initialization message

  setDisplayContrast(0);


  // Draw home screen
  u8g.firstPage();
  do {
    drawHomeScreen();
  } while (u8g.nextPage());
}

void setDisplayContrast(uint8_t contrast) {
  u8g.firstPage();
  do {
    // Empty page to send commands
  } while (u8g.nextPage());

  // Try direct I2C command
  Wire.beginTransmission(0x3C);  // OLED I2C address
  Wire.write(0x00);              // Command mode
  Wire.write(0x81);              // Contrast command
  Wire.write(contrast);          // Contrast value
  Wire.endTransmission();
}

void loop() {
  // Handle encoder and button
  handleEncoderChange();
  checkButton();

  // Process encoder changes
  if (encoderValue != lastEncoderValue) {
    int change = encoderValue - lastEncoderValue;
    lastEncoderValue = encoderValue;

    switch (state.screen) {
      case 1: // Config screen
        updateConfigValue(change);
        break;
      case 2: // Clock screen
        updateClockValue(change);
        break;
    }
  }

  screen.checkPower();
  if (screen.screenOn) {
    // Update display
    u8g.firstPage();
    do {
      switch (state.screen) {
        case 0:
          drawHomeScreen();
          break;
        case 1:
          drawConfigScreen();
          break;
        case 2:
          drawClockScreen();
          break;
      }
    } while (u8g.nextPage());
  }

  updateWateringSystem();
  // Debug output
  //testEncoder();
  //testScreenTransitions();
}

// Interrupt handler for encoder
ISR(INT6_vect) {
  unsigned long interruptTime = micros();
  if (interruptTime - lastInterruptTime > INTERRUPT_DEBOUNCE) {
    uint8_t A = readEncoderA();
    uint8_t B = readEncoderB();

    if (A != lastA) {
      stepCount += (B ? -1 : 1);
      lastA = A;
      stepReady = true;
    }
    if (B != lastB) {
      stepCount += (A ? 1 : -1);
      lastB = B;
      stepReady = true;
    }

    lastInterruptTime = interruptTime;
  }
}

void handleEncoderChange() {
  if (stepReady) {
    if (stepCount >= 2) {
      encoderValue++;
      stepCount = 0;
    } else if (stepCount <= -2) {
      encoderValue--;
      stepCount = 0;
    }
    stepReady = false;
  }
}

void checkButton() {
  bool reading = readEncoderButton();
  static bool pressHandled = false;

  if (reading != lastButtonState) {
    lastButtonTime = millis();
  }

  if ((millis() - lastButtonTime) > DEBOUNCE_TIME) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == 0) { // Press detected
        handleButtonPress(true);
        pressHandled = false;
      } else if (!pressHandled) { // Release detected
        handleButtonPress(false);
        pressHandled = true;
      }
    }
  }

  lastButtonState = reading;
}

void handleButtonPress(bool isPressed) {
  static unsigned long pressStartTime = 0;

  screen.wake();
  if (!screen.screenOn) return;

  if (isPressed) {
    pressStartTime = millis();
    Serial.println("Button pressed");
  } else {
    unsigned long pressDuration = millis() - pressStartTime;
    Serial.print("Button released, duration: ");
    Serial.println(pressDuration);

    if (pressDuration > LONG_PRESS_TIME) {
      Serial.println("Long press");
      switch (state.screen) {
        case 0: // From Home -> Clock
          state.screen = 2;
          state.field = CLOCK_HOUR;
          break;
        case 1: // From Config -> Save and Home
          savePrograms();  // Save programs to EEPROM
          state.screen = 0;
          state.field = HOME_NONE;
          break;
        case 2: // From Clock -> Save and Home
          state.screen = 0;
          state.field = HOME_NONE;
          break;
      }
    } else {
      Serial.println("Short press");
      switch (state.screen) {
        case 0: // From Home -> Config
          state.screen = 1;
          state.field = PROG_NUMBER;
          break;

        case 1: // In Config screen
          state.field = (state.field + 1) % PROG_FIELDS_COUNT;
          break;

        case 2: // In Clock - cycle fields
          state.field = (state.field + 1) % 6;
          break;
      }
    }
  }
}

void updateConfigValue(int change) {
  Program& prog = programs[state.currentProgram];

  switch (state.field) {
    case PROG_NUMBER:
      state.currentProgram = (state.currentProgram + change + MAX_PROGRAMS) % MAX_PROGRAMS;
      break;

    case PROG_ACTIVE:
      if (change != 0) { // Toggle with encoder
        prog.active = !prog.active;
      }
      break;
    case PROG_START:
      prog.hour = (prog.hour + change + 24) % 24;
      prog.minute = 0;  // Always keep minutes at 0
      break;

    case PROG_DAYS:
      currentPattern = (currentPattern + change + NUM_PATTERNS) % NUM_PATTERNS;
      for (uint8_t i = 0; i < 7; i++) {
        prog.days[i] = pgm_read_byte(&(PATTERNS[currentPattern].days[i]));
      }
      dayPatternName = (const char*)pgm_read_ptr(&(PATTERNS[currentPattern].name));
      break;

    case PROG_DURATION:
      prog.duration = constrain(prog.duration + change, 1, 600);
      break;

    case PROG_CHANNELS:
      channelBits = (channelBits + change + 16) % 16;
      for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        prog.channels[i] = (channelBits & (1 << i)) != 0;
      }
      break;
  }
}

void updateClockValue(int change) {
  DateTime now = RTC.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();

  switch (state.field) {
    case CLOCK_YEAR:
      year = constrain(year + change, 2000, 2099);
      break;
    case CLOCK_MONTH:
      month = constrain(month + change, 1, 12);
      break;
    case CLOCK_DAY:
      day = constrain(day + change, 1, 31);
      break;
    case CLOCK_HOUR:
      hour = (hour + change + 24) % 24;
      break;
    case CLOCK_MIN:
      minute = (minute + change + 60) % 60;
      break;
    case CLOCK_SEC:
      second = (second + change + 60) % 60;
      break;
  }

  RTC.adjust(DateTime(year, month, day, hour, minute, second));
}

// Date and time formatting patterns and helpers
struct TimeFormat {
  // Pattern declarations
  static const char DATE_PATTERN[];
  static const char TIME_PATTERN[];
  static const char TIME_SEC_PATTERN[];
  static const char DATETIME_PATTERN[];
  static const char PROGRAM_TIME_PATTERN[];
  static const char DURATION_PATTERN[];
  static const char LAST_WATERING_PATTERN[];
  static const char NEXT_PATTERN[];
  static const char SET_TIME_PATTERN[];
  static const char YEAR_PATTERN[];
  static const char MONTH_DAY_PATTERN[];

  // Helper functions
  static void formatDateTime(char* buffer, const DateTime& dt) {
    sprintf_P(buffer, DATETIME_PATTERN,
              dt.day(), dt.month(), dt.year() % 100,
              daysOfTheWeek[dt.dayOfTheWeek()],
              dt.hour(), dt.minute());
  }

  static void formatTime(char* buffer, uint8_t hour, uint8_t minute) {
    sprintf_P(buffer, TIME_PATTERN, hour, minute);
  }

  static void formatProgramTime(char* buffer, uint8_t number, uint8_t hour, uint8_t minute) {
    sprintf_P(buffer, PROGRAM_TIME_PATTERN, number, hour, minute);
  }
};

// Pattern definitions
const char TimeFormat::DATE_PATTERN[] PROGMEM = "%02d.%02d.%02d";    // DD.MM.YY
const char TimeFormat::TIME_PATTERN[] PROGMEM = "%02d:%02d";         // HH:MM
const char TimeFormat::TIME_SEC_PATTERN[] PROGMEM = "%02d:%02d:%02d"; // HH:MM:SS
const char TimeFormat::DATETIME_PATTERN[] PROGMEM = "%02d.%02d.%02d %s %02d:%02d"; // DD.MM.YY DDD HH:MM
const char TimeFormat::PROGRAM_TIME_PATTERN[] PROGMEM = "P%d %02d:%02d";
const char TimeFormat::DURATION_PATTERN[] PROGMEM = "%ds   [";
const char TimeFormat::LAST_WATERING_PATTERN[] PROGMEM = "LAST: %02d:%02d %ds";
const char TimeFormat::NEXT_PATTERN[] PROGMEM = "NEXT: ";
const char TimeFormat::SET_TIME_PATTERN[] PROGMEM = "SET TIME:";
const char TimeFormat::YEAR_PATTERN[] PROGMEM = "%04d";
const char TimeFormat::MONTH_DAY_PATTERN[] PROGMEM = "%02d";

void drawHomeScreen() {
  DateTime now = RTC.now();
  uint8_t nextProg;
  DateTime next = getNextWateringTime(&nextProg);
  char buffer[20];

  // Top line: Current date and time
  TimeFormat::formatDateTime(buffer, now);
  u8g.drawStr(0, 8, buffer);
  u8g.drawHLine(0, 11, 128);

  // Second line: Current watering status or next program
  if (pumpState) {
    // Show currently running program
    Program& runningProg = programs[state.currentProgram];
    sprintf(buffer, "RUN: P%d %02d:00 [", state.currentProgram + 1, runningProg.hour);
    u8g.drawStr(0, 22, buffer);

    // Show active channels
    uint8_t xPos = 96;
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      if (runningProg.channels[i]) {
        sprintf(buffer, "%d", i + 1);
        u8g.drawStr(xPos, 22, buffer);
        xPos += 7;
      }
    }
    u8g.drawStr(xPos, 22, "]");
  } else if (next.unixtime() > now.unixtime()) {
    // Show next scheduled program
    Program& nextProgram = programs[nextProg];
    strcpy_P(buffer, TimeFormat::NEXT_PATTERN);
    u8g.drawStr(0, 22, buffer);

    sprintf(buffer, "P%d %02d:00", nextProg + 1, nextProgram.hour);
    u8g.drawStr(30, 22, buffer);

    sprintf(buffer, "[");
    u8g.drawStr(90, 22, buffer);

    uint8_t xPos = 96;
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      if (nextProgram.channels[i]) {
        sprintf(buffer, "%d", i + 1);
        u8g.drawStr(xPos, 22, buffer);
        xPos += 7;
      }
    }
    u8g.drawStr(xPos, 22, "]");
  } else {
    u8g.drawStr(0, 22, "No programs active");
  }

  // Bottom line: Active programs count
  uint8_t activeCount = 0;
  for (uint8_t i = 0; i < MAX_PROGRAMS; i++) {
    if (programs[i].active) activeCount++;
  }
  sprintf(buffer, "Active: %d/%d", activeCount, MAX_PROGRAMS);
  u8g.drawStr(0, 36, buffer);
}

void drawConfigScreen() {
  char buffer[20];
  Program& prog = programs[state.currentProgram];
  u8g.setFont(u8g_font_6x10);

  // Top line: Program number and active state
  sprintf(buffer, "P%d", state.currentProgram + 1);
  u8g.drawStr(0, 10, buffer);
  sprintf(buffer, "%s", prog.active ? "ON" : "OFF");
  u8g.drawStr(20, 10, buffer);

  // Start time on same line
  sprintf(buffer, "Start: %02d:%02d", prog.hour, prog.minute);
  u8g.drawStr(50, 10, buffer);

  // Middle line: Days of week
  const char* days = "M T W T F S S";
  u8g.drawStr(0, 25, days);

  if (state.field == PROG_DAYS) {
    // Show pattern name
    u8g.setColorIndex(1);
    u8g.drawBox(90, 17, 35, 12);
    u8g.setColorIndex(0);
    u8g.drawStr(92, 26, dayPatternName);
    u8g.setColorIndex(1);

    // Show current pattern selection
    for (uint8_t i = 0; i < 7; i++) {
      if (pgm_read_byte(&(PATTERNS[currentPattern].days[i]))) {
        u8g.drawBox(i * 12, 26, 6, 1);
      }
    }
  } else {
    // Show saved pattern
    for (uint8_t i = 0; i < 7; i++) {
      if (prog.days[i]) {
        u8g.drawBox(i * 12, 27, 6, 1);
      }
    }
  }

  // Bottom line: Duration and channels
  sprintf_P(buffer, TimeFormat::DURATION_PATTERN, prog.duration);
  u8g.drawStr(0, 40, buffer);

  uint8_t xPos = 45;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    sprintf(buffer, "%d", i + 1);
    u8g.drawStr(xPos, 40, buffer);
    if (prog.channels[i]) {
      if (state.field == PROG_CHANNELS) {
        u8g.drawBox(xPos, 41, 6, 1);
      } else {
        u8g.drawBox(xPos, 42, 6, 1);
      }
    }
    xPos += 8;
  }
  u8g.drawStr(xPos, 40, "]");

  //drawFieldHighlight();
  const char* fieldNames[] = {"PROGRAM", "ON/OFF", "START", "DAYS", "DURATION", "CHANNELS"};
  u8g.drawStr(0, 63, fieldNames[state.field]);
}


void drawClockScreen() {
  DateTime now = RTC.now();
  char buffer[20];
  u8g.setFont(u8g_font_6x10);

  strcpy_P(buffer, TimeFormat::SET_TIME_PATTERN);
  u8g.drawStr(0, 10, buffer);

  // Year (4 digits = 24px + 6px spacing)
  if (state.field == CLOCK_YEAR) {
    u8g.setColorIndex(1);
    u8g.drawBox(0, 15, 26, 12);
    u8g.setColorIndex(0);
    sprintf_P(buffer, TimeFormat::YEAR_PATTERN, now.year());
    u8g.drawStr(2, 25, buffer);
    u8g.setColorIndex(1);
  } else {
    sprintf_P(buffer, TimeFormat::YEAR_PATTERN, now.year());
    u8g.drawStr(0, 25, buffer);
  }

  u8g.drawStr(30, 25, ".");

  // Month (2 digits = 12px + 6px spacing)
  if (state.field == CLOCK_MONTH) {
    u8g.setColorIndex(1);
    u8g.drawBox(38, 15, 14, 12);
    u8g.setColorIndex(0);
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.month());
    u8g.drawStr(40, 25, buffer);
    u8g.setColorIndex(1);
  } else {
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.month());
    u8g.drawStr(38, 25, buffer);
  }

  u8g.drawStr(56, 25, ".");

  // Day (2 digits = 12px)
  if (state.field == CLOCK_DAY) {
    u8g.setColorIndex(1);
    u8g.drawBox(64, 15, 14, 12);
    u8g.setColorIndex(0);
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.day());
    u8g.drawStr(66, 25, buffer);
    u8g.setColorIndex(1);
  } else {
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.day());
    u8g.drawStr(64, 25, buffer);
  }

  drawTimeFields(now);
}

void drawTimeFields(const DateTime& now) {
  char buffer[10];

  // Hour (2 digits = 12px + 6px spacing)
  if (state.field == CLOCK_HOUR) {
    u8g.setColorIndex(1);
    u8g.drawBox(0, 30, 14, 12);
    u8g.setColorIndex(0);
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.hour());
    u8g.drawStr(2, 40, buffer);
    u8g.setColorIndex(1);
  } else {
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.hour());
    u8g.drawStr(0, 40, buffer);
  }

  u8g.drawStr(18, 40, ":");

  // Minutes (2 digits = 12px + 6px spacing)
  if (state.field == CLOCK_MIN) {
    u8g.setColorIndex(1);
    u8g.drawBox(26, 30, 14, 12);
    u8g.setColorIndex(0);
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.minute());
    u8g.drawStr(28, 40, buffer);
    u8g.setColorIndex(1);
  } else {
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.minute());
    u8g.drawStr(26, 40, buffer);
  }

  u8g.drawStr(44, 40, ":");  // More space before delimiter

  // Seconds (2 digits = 12px)
  if (state.field == CLOCK_SEC) {
    u8g.setColorIndex(1);
    u8g.drawBox(52, 30, 14, 12);  // Increased box width
    u8g.setColorIndex(0);
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.second());
    u8g.drawStr(54, 40, buffer);
    u8g.setColorIndex(1);
  } else {
    sprintf_P(buffer, TimeFormat::MONTH_DAY_PATTERN, now.second());
    u8g.drawStr(52, 40, buffer);
  }
}


void drawFieldHighlight() {
  if (state.screen == 1) { // Config screen
    Program& prog = programs[state.currentProgram];

    switch (state.field) {
      case PROG_NUMBER:
        // Just draw two dots under the program number
        u8g.setColorIndex(1);
        u8g.drawBox(0, 1, 14, 10);
        u8g.setColorIndex(0);
        char buffer[3];
        sprintf(buffer, "P%d", state.currentProgram + 1);
        u8g.drawStr(1, 10, buffer);
        u8g.setColorIndex(1);
        break;

      case PROG_START:
        // Two dots under the time
        u8g.drawPixel(20, 12);
        u8g.drawPixel(22, 12);
        break;

      case PROG_DAYS:
        // No additional highlight needed here as we show pattern name
        // and dots above days in drawConfigScreen
        break;

      case PROG_DURATION:
        // Two dots under duration
        u8g.drawPixel(10, 42);
        u8g.drawPixel(12, 42);
        break;

      case PROG_CHANNELS:
        // Selection handled in drawConfigScreen
        break;
    }
  }
}


DateTime getNextWateringTime(uint8_t* nextProgram) {
  DateTime now = RTC.now();
  DateTime next = now;
  bool foundNext = false;
  *nextProgram = 0;

  // Check each program
  for (uint8_t p = 0; p < MAX_PROGRAMS; p++) {
    if (!programs[p].active) continue;

    // Look ahead up to 7 days
    for (uint8_t daysAhead = 0; daysAhead < 7; daysAhead++) {
      uint8_t checkDay = (now.dayOfTheWeek() + daysAhead) % 7;

      if (programs[p].days[checkDay]) {
        DateTime checkTime = now + TimeSpan(daysAhead, 0, 0, 0);
        DateTime programTime = DateTime(
                                 checkTime.year(),
                                 checkTime.month(),
                                 checkTime.day(),
                                 programs[p].hour,
                                 programs[p].minute,
                                 0
                               );

        if (programTime.unixtime() > now.unixtime() &&
            (!foundNext || programTime.unixtime() < next.unixtime())) {
          next = programTime;
          *nextProgram = p;
          foundNext = true;
        }
      }
    }
  }

  return foundNext ? next : now;
}

void updateWateringSystem() {
  DateTime now = RTC.now();
  static DateTime lastCheck = now;
  static unsigned long wateringStartTime = 0;

  // Check for new watering starts every minute
  if ((now.unixtime() - lastCheck.unixtime()) >= 60) {
    lastCheck = now;

    // Only check for new starts if not currently watering
    if (!pumpState) {
      for (uint8_t i = 0; i < MAX_PROGRAMS; i++) {
        Program& prog = programs[i];
        if (!prog.active) continue;

        // Check if this program should start
        if (prog.days[now.dayOfTheWeek()] &&
            now.hour() == prog.hour && now.minute() == prog.minute) {
          state.currentProgram = i;  // Use existing state variable
          startWatering();
          wateringStartTime = millis();
          break;
        }
      }
    }
  }

  // Check for watering stop every loop cycle (for precise timing)
  if (pumpState) {
    unsigned long elapsedSeconds = (millis() - wateringStartTime) / 1000;
    if (elapsedSeconds >= programs[state.currentProgram].duration) {
      stopWatering();
    }
  }
}

void startWatering() {
  // First activate selected channels from current program
  Program& prog = programs[state.currentProgram];

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    if (prog.channels[i]) {
      digitalWrite(CHANNEL_PINS[i], HIGH);
      channelStates[i] = true;
    }
  }

  digitalWrite(PUMP_PIN, HIGH);
  pumpState = true;
  Serial.println("Watering started");
}

void stopWatering() {
  // First stop pump
  digitalWrite(PUMP_PIN, LOW);
  pumpState = false;

  // Then close all channels
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(CHANNEL_PINS[i], LOW);
    channelStates[i] = false;
  }

  Serial.println("Watering stopped");
}

// Debug functions
void testEncoder() {
  static int lastValue = encoderValue;
  static bool lastButton = buttonState;

  if (encoderValue != lastValue || buttonState != lastButton) {
    Serial.print("Encoder: ");
    Serial.print(encoderValue);
    Serial.print(" Button: ");
    Serial.println(buttonState ? "UP" : "DOWN");
    lastValue = encoderValue;
    lastButton = buttonState;
  }
}

void testScreenTransitions() {
  static uint8_t lastScreen = state.screen;
  static uint8_t lastField = state.field;

  if (lastScreen != state.screen || lastField != state.field) {
    Serial.print("Screen: ");
    Serial.print(state.screen);
    Serial.print(" Field: ");
    Serial.println(state.field);
    lastScreen = state.screen;
    lastField = state.field;
  }
}


struct EEPROMData {
  uint16_t magic;
  Program programs[MAX_PROGRAMS];
};

void savePrograms() {
  EEPROMData data;
  data.magic = EEPROM_MAGIC;

  // Copy current programs
  for (uint8_t i = 0; i < MAX_PROGRAMS; i++) {
    data.programs[i] = programs[i];
  }

  // Write to EEPROM
  EEPROM.put(EEPROM_START_ADDR, data);

  Serial.println("Programs saved to EEPROM");
}

bool loadPrograms() {
  EEPROMData data;

  // Read from EEPROM
  EEPROM.get(EEPROM_START_ADDR, data);

  // Check magic number
  if (data.magic == EEPROM_MAGIC) {
    // Copy programs from EEPROM
    for (uint8_t i = 0; i < MAX_PROGRAMS; i++) {
      programs[i] = data.programs[i];
    }
    Serial.println("Programs loaded from EEPROM");
    return true;
  } else {
    Serial.println("No valid EEPROM data found");
    return false;
  }
}

void initializeDefaultPrograms() {
  for (uint8_t i = 0; i < MAX_PROGRAMS; i++) {
    programs[i] = {
      .number = i + 1,
      .hour = 12,
      .minute = 0,
      .duration = 30,
      .days = {0},
      .channels = {0},
      .active = false
    };
  }
}
