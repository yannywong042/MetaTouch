#include <Wire.h>
#include <SPI.h>
#include <BleKeyboard.h>

#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>

#define BUF_LEN 3
BleKeyboard bleKeyboard("Babe Yani KBD", "Wonny Yang Inc.", 100);

// ——— RFID ———
static const uint8_t SS_PIN  = 5;
static const uint8_t RST_PIN = 22;
bool keyStateRight = false;
bool keyStateLeft = false;
bool keyStateUp = false;
bool keyStateDown = false;

int pusherBias;
int squeezeBias;
int lampToggle = 0;
int activeMode = 0;

MFRC522DriverPinSimple pinDriver(SS_PIN);
MFRC522DriverSPI       spiDriver(pinDriver);
MFRC522                rfid(spiDriver);

// ——— Pump controls ———
const int ENA         = 13;
const int IN1         = 14;
const int IN2         = 12;
const int TOUCHY_PIN_BUTTON = 38;

// ——— Pressure ———
const int YANI_FAV_PRESSURE_SQUEEZE_PIN = 33; 
const int YANI_FAV_PRESSURE_PUSHER_PIN = 34;

const int SQUEEZE_THRESH = 100;
const int PUSHER_THRESH = 100;

// ——— LIS2DW12 accelerometer for twist ———
TwoWire        customWireTwist = TwoWire(0);
int32_t        twistBiasX, twistBiasY, twistBiasZ;

int TWIST_SDA = 21;
int TWIST_SCL = 16;

#define TWIST_BUF_LEN 5  // Increased buffer for more stability
int16_t twistBufZ[TWIST_BUF_LEN] = {0}; // Use int16_t for more precision
uint8_t twistBufIdx = 0;
int8_t twistLastX = 0, twistLastY = 0;

// twist output pins (motor driver to dim LED)
const int ENB         = 27;
const int IN3         = 25; 
const int IN4         = 26; 

// ——— LIS2DW12 accelerometer for rod ———
TwoWire        customWireRod = TwoWire(1);
int32_t        rodBiasX, rodBiasY, rodBiasZ;

int ROD_SDA = 17; 
int ROD_SCL = 4;

#define ROD_BUF_LEN 5  // Slightly larger buffer for stability
int8_t rodBufX[ROD_BUF_LEN] = {0}, rodBufY[ROD_BUF_LEN] = {0}, rodBufZ[ROD_BUF_LEN] = {0};
uint8_t rodBufIdx = 0;

bool matchUid(byte *a, byte *b, byte len) {
  for (byte i = 0; i < len; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

String getUidString(MFRC522::Uid uid) {
  String uidStr = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) uidStr += "0";
    uidStr += String(uid.uidByte[i], HEX);
    if (i < uid.size - 1) uidStr += " ";
  }
  uidStr.toUpperCase();
  return uidStr;
}

void scanBus(TwoWire& bus, const char* name) {
  Serial.printf("Scanning %s...\n", name);
  byte count = 0;
  for (byte addr = 1; addr < 127; ++addr) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("  No I2C devices found.");
  } else {
    Serial.printf("  %d device(s) found on %s.\n", count, name);
  }
}

// ——— Manual init for twist sensor registers ———
void initTwistSensorManually() {
  // CTRL1 (0x20): ODR = 50Hz (0100), Mode = High-Performance, XYZ enable
  customWireTwist.beginTransmission(0x18);
  customWireTwist.write(0x20);
  customWireTwist.write(0b01000111); // ODR = 50Hz, LP mode off, XYZ enabled
  customWireTwist.endTransmission();

  // CTRL6 (0x25): FS = ±2g (default)
  customWireTwist.beginTransmission(0x18);
  customWireTwist.write(0x25);
  customWireTwist.write(0b00000000); // FS = ±2g
  customWireTwist.endTransmission();
}

// ——— Manual init for rod sensor registers ———
void initRodSensorManually() {
  // CTRL1 (0x20): ODR = 50Hz (0100), Mode = High-Performance, XYZ enable
  customWireRod.beginTransmission(0x19);
  customWireRod.write(0x20);
  customWireRod.write(0b01000111); // ODR = 50Hz, LP mode off, XYZ enabled
  customWireRod.endTransmission();

  // CTRL6 (0x25): FS = ±2g (default)
  customWireRod.beginTransmission(0x19);
  customWireRod.write(0x25);
  customWireRod.write(0b00000000); // FS = ±2g
  customWireRod.endTransmission();
}

// ——— Direct register reading function for twist ———
bool readTwistRaw(int16_t& x, int16_t& y, int16_t& z) {
  customWireTwist.beginTransmission(0x18);
  customWireTwist.write(0x28 | 0x80); // OUT_X_L with auto-increment
  customWireTwist.endTransmission(false);
  customWireTwist.requestFrom(0x18, 6);
  
  if (customWireTwist.available() == 6) {
    x = customWireTwist.read() | (customWireTwist.read() << 8);
    y = customWireTwist.read() | (customWireTwist.read() << 8);
    z = customWireTwist.read() | (customWireTwist.read() << 8);
    return true;
  }
  return false;
}

// ——— Direct register reading function for rod ———
bool readRodRaw(int16_t& x, int16_t& y, int16_t& z) {
  customWireRod.beginTransmission(0x19);
  customWireRod.write(0x28 | 0x80); // OUT_X_L with auto-increment
  customWireRod.endTransmission(false);
  customWireRod.requestFrom(0x19, 6);
  
  if (customWireRod.available() == 6) {
    x = customWireRod.read() | (customWireRod.read() << 8);
    y = customWireRod.read() | (customWireRod.read() << 8);
    z = customWireRod.read() | (customWireRod.read() << 8);
    return true;
  }
  return false;
}

/// calibrate the LIS2DW12 so "still" reads zero on Z axis only
void calibrateTwistAccelerometer() {
  constexpr int N = 200; // More samples for better calibration
  int64_t sz = 0;
  delay(1000); // Longer delay to ensure stillness
  
  Serial.println("Calibrating twist sensor - keep device perfectly still...");
  
  for (int i = 0; i < N; i++) {
    int16_t x, y, z;
    if (readTwistRaw(x, y, z)) {
      sz += z; // Only calibrate Z axis
    }
    delay(20); // Slower sampling during calibration
    
    // Progress indicator
    if (i % 20 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  
  twistBiasZ = sz / N; 
  
  // Clear the buffer after calibration
  for (int i = 0; i < TWIST_BUF_LEN; i++) {
    twistBufZ[i] = 0;
  }
  twistBufIdx = 0;
  
  Serial.printf("Twist calibration complete. Z bias: %ld\n", twistBiasZ);
}


void calibrateRodAccelerometer() {
  constexpr int N = 100;
  int64_t sx = 0, sy = 0, sz = 0;
  delay(500);
  
  for (int i = 0; i < N; i++) {
    int16_t x, y, z;
    if (readRodRaw(x, y, z)) {
      sx += x; 
      sy += y; 
      sz += z;
    }
    delay(10);
  }
  
  rodBiasX = sx / N;
  rodBiasY = sy / N;
  rodBiasZ = sz / N;
}

void calibratePusherPressureSensor() {
  constexpr int N = 100;
  delay(200);
  int total = 0;
  for (int i = 0; i < N; i++) {
    total += analogRead(YANI_FAV_PRESSURE_PUSHER_PIN);
    delay(10);
  }

  pusherBias = total / N;

}

void calibrateSqueezePressureSensor() {
  constexpr int N = 100;
  delay(200);
  int total = 0;
  for (int i = 0; i < N; i++) {
    total += analogRead(YANI_FAV_PRESSURE_SQUEEZE_PIN);
    delay(10);
  }

  squeezeBias = total / N;
}

void openGameInChrome() {
  if (bleKeyboard.isConnected()) {
    Serial.println("Opening Chrome and navigating to game...");
    
    // Open Chrome using Windows + R (Run dialog)
    bleKeyboard.press(KEY_LEFT_GUI); // Windows key
    bleKeyboard.press('r');
    delay(100);
    bleKeyboard.releaseAll();
    
    delay(500); // Wait for Run dialog to open
    
    // Type "chrome" to open Chrome
    bleKeyboard.print("chrome");
    delay(100);
    bleKeyboard.press(KEY_RETURN);
    bleKeyboard.release(KEY_RETURN);
    
    delay(1000); // Wait for Chrome to open (adjust if needed)
    
    // Type the URL in the address bar (Chrome focuses on address bar by default)
    bleKeyboard.print("https://zh.y8.com/games/turbo_moto_racer");
    delay(2000);
    
    // Press Enter to navigate to the URL
    bleKeyboard.press(KEY_RETURN);
    bleKeyboard.release(KEY_RETURN);
    
    Serial.println("URL navigation complete!");
  } else {
    Serial.println("BLE keyboard not connected!");
  }
}

// Volume control functions
void toggleMute() {
  if (bleKeyboard.isConnected()) {
    Serial.println("Toggling mute...");
    bleKeyboard.write(KEY_MEDIA_MUTE);
    delay(100);
  } else {
    Serial.println("BLE keyboard not connected!");
  }
}


// ——— Rod reading → BLE arrows ———
void readRodAccelerometer() {
  int16_t rawX, rawY, rawZ;
  
  if (!readRodRaw(rawX, rawY, rawZ)) {
    Serial.println("Failed to read rod accelerometer!");
    return;
  }

  // Apply bias compensation and scale down - try different scaling
  int8_t correctedX = constrain((rawX - rodBiasX) / 350, -127, 127); // Less aggressive scaling
  int8_t correctedY = constrain((rawY - rodBiasY) / 350, -127, 127); // Less aggressive scaling
  int8_t correctedZ = constrain((rawZ - rodBiasZ) / 350, -127, 127); // Less aggressive scaling

  rodBufX[rodBufIdx] = correctedX;
  rodBufY[rodBufIdx] = correctedY;
  rodBufZ[rodBufIdx] = correctedZ;
  rodBufIdx = (rodBufIdx + 1) % ROD_BUF_LEN;

  // Calculate moving average
  int sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < ROD_BUF_LEN; i++) {
    sumX += rodBufX[i];
    sumY += rodBufY[i];
    sumZ += rodBufZ[i];
  }
  int8_t avgX = sumX / ROD_BUF_LEN;
  int8_t avgY = sumY / ROD_BUF_LEN;
  int8_t avgZ = sumZ / ROD_BUF_LEN;

  // Smaller dead zone
  if (abs(avgX) < 4) avgX = 0;
  if (abs(avgY) < 4) avgY = 0;
  if (abs(avgZ) < 4) avgZ = 0;

  const int8_t THRESH = 16; // Slightly lower threshold

  // Release all keys first
  bleKeyboard.release(KEY_RIGHT_ARROW);
  bleKeyboard.release(KEY_LEFT_ARROW);
  bleKeyboard.release(KEY_UP_ARROW);
  bleKeyboard.release(KEY_DOWN_ARROW);



  if (avgZ < -THRESH) {
    bleKeyboard.press(KEY_RIGHT_ARROW);
    Serial.println("RIGHT pressed (avgZ < -THRESH)");
  }
  if (avgZ > THRESH) {
    bleKeyboard.press(KEY_LEFT_ARROW);
    Serial.println("LEFT pressed (avgZ > THRESH)");
  }
  if (avgY > THRESH) {
    bleKeyboard.press(KEY_UP_ARROW);
    Serial.println("UP pressed (avgY > THRESH)");
  }
  if (avgY < -THRESH) {
    bleKeyboard.press(KEY_DOWN_ARROW);
    Serial.println("DOWN pressed (avgY < -THRESH)");
  }
  

}

// ——— Twist reading → motor PWM ———
void readTwistAccelerometer() {
  int16_t rawX, rawY, rawZ;
  
  if (!readTwistRaw(rawX, rawY, rawZ)) {
    Serial.println("Failed to read twist accelerometer!");
    return;
  }

  // Apply bias compensation - ONLY use Z axis for twist motion
  int16_t correctedZ = rawZ - twistBiasZ;
  
  // Store raw corrected value in buffer (no scaling down yet)
  twistBufZ[twistBufIdx] = correctedZ;
  twistBufIdx = (twistBufIdx + 1) % TWIST_BUF_LEN;

  // Calculate moving average for stability
  int32_t sum = 0;
  for (int i = 0; i < TWIST_BUF_LEN; i++) {
    sum += twistBufZ[i];
  }
  int16_t avgZ = sum / TWIST_BUF_LEN;

  // Scale down after averaging for final processing
  int16_t processedZ = avgZ / 64; // Reduced scaling for less sensitivity

  // Larger dead zone for stability
  const int16_t DEAD_ZONE = 25; // Increased dead zone
  if (abs(processedZ) < DEAD_ZONE) {
    // Stop motor in dead zone
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Motor stopped (dead zone)");
    return;
  }

  // Determine direction based on Z-axis twist
  if (processedZ > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Motor CW");
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Motor CCW");
  }

  // More conservative PWM mapping
  int pwm = map(abs(processedZ), DEAD_ZONE, 400, 80, 180); // Reduced max PWM
  pwm = constrain(pwm, 80, 180);

  analogWrite(ENB, pwm);
  Serial.printf("Motor PWM: %d (processedZ: %d, avgZ: %d)\n", pwm, processedZ, avgZ);
}

void setup() {
  Serial.begin(115200);
  bleKeyboard.begin();

  // — Pump, pressure, dim pins —
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TOUCHY_PIN_BUTTON, INPUT);
  pinMode(YANI_FAV_PRESSURE_SQUEEZE_PIN, INPUT);
  pinMode(YANI_FAV_PRESSURE_PUSHER_PIN, INPUT);

  // — RFID init & reset pulse —
  pinMode(SS_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);  delay(50);
  digitalWrite(RST_PIN, HIGH); delay(50);
  SPI.begin();  // SCK=18, MISO=19, MOSI=23
  rfid.PCD_Init();

  // — rod Accelerometer init & calibrate —
  customWireRod.begin(ROD_SDA, ROD_SCL);
  
  // WHO_AM_I check for rod
  customWireRod.beginTransmission(0x19);
  customWireRod.write(0x0F);
  customWireRod.endTransmission(false);
  customWireRod.requestFrom(0x19, 1);
  if (customWireRod.available()) {
    uint8_t id = customWireRod.read();
    Serial.printf("Rod WHO_AM_I: 0x%02X\n", id);
  }
  
  // Manual initialization for rod sensor
  initRodSensorManually();
  


  // — twist Accelerometer init & calibrate —
  customWireTwist.begin(TWIST_SDA, TWIST_SCL);
  
  // WHO_AM_I check for twist - try both addresses
  bool twistFound = false;
  uint8_t twistAddr = 0x18;
  
  // Try address 0x18 first
  customWireTwist.beginTransmission(0x18);
  customWireTwist.write(0x0F);
  if (customWireTwist.endTransmission(false) == 0) {
    customWireTwist.requestFrom(0x18, 1);
    if (customWireTwist.available()) {
      uint8_t id = customWireTwist.read();
      Serial.printf("Twist WHO_AM_I at 0x18: 0x%02X\n", id);
      if (id == 0x44) { // LIS2DW12 WHO_AM_I value
        twistFound = true;
        twistAddr = 0x18;
      }
    }
  }
  
  // If not found at 0x18, try 0x19
  if (!twistFound) {
    customWireTwist.beginTransmission(0x19);
    customWireTwist.write(0x0F);
    if (customWireTwist.endTransmission(false) == 0) {
      customWireTwist.requestFrom(0x19, 1);
      if (customWireTwist.available()) {
        uint8_t id = customWireTwist.read();
        Serial.printf("Twist WHO_AM_I at 0x19: 0x%02X\n", id);
        if (id == 0x44) { // LIS2DW12 WHO_AM_I value
          twistFound = true;
          twistAddr = 0x19;
        }
      }
    }
  }
  
  if (!twistFound) {
    Serial.println("WARNING: Twist accelerometer not found!");
  } else {
    Serial.printf("Twist accelerometer found at address 0x%02X\n", twistAddr);
  }
  
  // Manual initialization for twist sensor
  initTwistSensorManually();

  scanBus(customWireRod, "customWireRod");
  scanBus(customWireTwist, "customWireTwist");


  calibratePusherPressureSensor();
  calibrateSqueezePressureSensor();

  Serial.println("Setup complete. Device controls are now active.");

}

void loop() {

  int pusherReading = analogRead(YANI_FAV_PRESSURE_PUSHER_PIN);
  int squeezeReading = analogRead(YANI_FAV_PRESSURE_SQUEEZE_PIN);

    //Serial.printf("Squeeze change %d\n", squeezeReading - squeezeBias);
    //Serial.printf("Pusher change %d\n", pusherReading - pusherBias);
  // -----------------------GAME mode
  if (activeMode == 1) {
    if (bleKeyboard.isConnected()) {
      readRodAccelerometer();
    }
    
    // Squeeze game mode
    if (squeezeReading - squeezeBias >= SQUEEZE_THRESH) {
    Serial.println("Open game in chrome");
      openGameInChrome();
      delay(300);
    }
    // Pusher game mode
    if (pusherReading - pusherBias >= PUSHER_THRESH) {
      Serial.println("Open game in chrome push");
      openGameInChrome();
      delay(300);
    }
  } 





  // -----------------------LAMP mode
  else if (activeMode == 2) {

    

    // Squeeze
    if (squeezeReading - squeezeBias >= SQUEEZE_THRESH) {
      Serial.println("Lamp mode toggle mute");
      toggleMute();
      delay(300);
    }

    // Pusher
    if (squeezeReading - squeezeBias >= PUSHER_THRESH) {
      Serial.println("Lamp mode pusher");
      delay(300);
    }


    // — Touch → led toggle
    if (digitalRead(TOUCHY_PIN_BUTTON)) {
      if (lampToggle == 1) {
        lampToggle = 0;
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0);
      } else {
        lampToggle = 1;
      } 
      delay(200);
    }

    // Power lamp
    if (lampToggle == 1) {
      // digitalWrite(IN3, HIGH);
      // digitalWrite(IN4, LOW);
      // analogWrite(ENB, 200);
      readTwistAccelerometer();
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
    }
  }


  // Manual pump toggle
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim(); // Remove any whitespace/newlines
    
    if (command == "ON" || command == "on") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 255);
      Serial.println("Pump on");
    }
    else if (command == "OFF" || command == "off") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      Serial.println("Pump turned OFF");
    }
    else if (command == "PUMP" || command == "pump" || command == "pulp") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 255);
      delay(3000);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      Serial.println("Pump turned OFF, wait for calibration");
      calibratePusherPressureSensor();
      calibrateSqueezePressureSensor();
      Serial.println("Done");
    }
    else {
      Serial.println("Unknown command. Use ON or OFF or PUMP");
    }
  }

  

  
  
  

  

  
  

  // — RFID → Serial —
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Card UID: ");
    String currentUid = getUidString(rfid.uid);

    byte controllerUid[] = { 0x04, 0x34, 0x46, 0x3C, 0xBF, 0x2A, 0x81 };
    byte lampUid[]       = { 0x04, 0x4B, 0x3F, 0x3C, 0xBF, 0x2A, 0x81 };

    if (matchUid(rfid.uid.uidByte, controllerUid, rfid.uid.size)) {
      // enter GAME mode
      activeMode = 1;

      Serial.println("Calibrating rod accel — keep still");
      calibrateRodAccelerometer();
      Serial.printf("Rod Bias: X=%ld Y=%ld Z=%ld\n", rodBiasX, rodBiasY, rodBiasZ);
      Serial.println("Game mode activated");
    } else if (matchUid(rfid.uid.uidByte, lampUid, rfid.uid.size)) {
      // enter LAMP mode

      activeMode = 2;
      Serial.println("Calibrating twist accel — keep still");
      calibrateTwistAccelerometer();
      Serial.printf("Twist Bias: X=%ld Y=%ld Z=%ld\n", twistBiasX, twistBiasY, twistBiasZ);
      Serial.println("Lamp mode activated");
      lampToggle = 0;
    }

    MFRC522Debug::PrintUID(Serial, rfid.uid);
    Serial.println();
    rfid.PICC_HaltA();
  }

  // DEBUGGG
  static int debugCounter = 0;
  bool doDebug = (debugCounter % 100 == 0);
  debugCounter++;

  if (doDebug) {
    // Debug output for twist accelerometer using direct reading
    int16_t rawX, rawY, rawZ;
    if (readTwistRaw(rawX, rawY, rawZ)) {
      int16_t correctedZ = rawZ - twistBiasZ;
    
      // Show the processed value that's actually used for motor control
      int32_t sum = 0;
      for (int i = 0; i < TWIST_BUF_LEN; i++) {
        sum += twistBufZ[i];
      }
      int16_t avgZ = sum / TWIST_BUF_LEN;
      int16_t processedZ = avgZ / 64;
      
      Serial.printf("Twist Raw Z=%d  Corrected Z=%d  Avg Z=%d  Processed Z=%d\n", 
                    rawZ, correctedZ, avgZ, processedZ);
    } else {
      Serial.println("TWIST ACCEL READ FAILED");
    }

    // Debug output for rod accelerometer using direct reading
    if (readRodRaw(rawX, rawY, rawZ)) {
      // Apply bias compensation for debug output
      int16_t correctedX = rawX - rodBiasX;
      int16_t correctedY = rawY - rodBiasY;
      int16_t correctedZ = rawZ - rodBiasZ;
      
      int8_t x = int8_t(correctedX >> 8);
      int8_t y = int8_t(correctedY >> 8);
      int8_t z = int8_t(correctedZ >> 8);

      Serial.printf("Rod   ΔX=%d  ΔY=%d  ΔZ=%d\n", x, y, z);
    } else {
      Serial.println("ROD ACCEL READ FAILED");
    }
    
    Serial.printf("Active mode: %d\n", activeMode);
    Serial.printf("Squeeze change %d\n", squeezeReading - squeezeBias);
    Serial.printf("Pusher change %d\n", pusherReading - pusherBias);

  }

  delay(20); // Reduced delay for more responsive control
}