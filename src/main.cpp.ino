#include <Arduino.h>
#include "DHT.h"

// ========== BLE INCLUDES ==========
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

// ========== DIGITAL PINS ==========
#define PIR_PIN      5
#define TRIG_PIN     18
#define ECHO_PIN     19
#define BUTTON_PIN   23   // Toggle button

// ========== DHT11 ==========
#define DHT_PIN      14
#define DHTTYPE      DHT11
DHT dht(DHT_PIN, DHTTYPE);

// ========== ANALOG PINS ==========
#define LDR_PIN      34
#define POT_PIN      15

// ========== RGB LED PINS ==========
#define LED_R        25
#define LED_G        26
#define LED_B        27

// 4095 = darkest; tweak based on room light
const int LDR_DARK_THRESHOLD = 3000;

// Toggle & state
bool systemEnabled      = true;
bool lastButtonPressed  = false;
bool lastAlarmState     = false;   // to detect NEW alarms (edge)

// DHT state
unsigned long lastDhtTime = 0;
float lastTemp = NAN;
float lastHum  = NAN;

// ========== BLE GLOBALS ==========
const char *bleName = "ESP32_Alarm";

// You can replace these with your own random UUIDs
#define SERVICE_UUID           "d0afc074-0001-4b0e-bb34-15d2b1d00001"
#define CHARACTERISTIC_UUID_RX "d0afc074-0002-4b0e-bb34-15d2b1d00002"
#define CHARACTERISTIC_UUID_TX "d0afc074-0003-4b0e-bb34-15d2b1d00003"

BLECharacteristic *pCharacteristic = nullptr;
String receivedText = "";
unsigned long lastMessageTime = 0;

// ========== BLE CALLBACK CLASSES ==========

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    Serial.println("BLE: Client Connected");
  }
  void onDisconnect(BLEServer *pServer) override {
    Serial.println("BLE: Client Disconnected");
    pServer->startAdvertising();
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    receivedText = String(value.c_str());
    lastMessageTime = millis();
    Serial.print("BLE Received: ");
    Serial.println(receivedText);
    // You could parse commands here (e.g., to toggle systemEnabled)
  }
};

// ========== BLE SETUP FUNCTION ==========

void setupBLE() {
  BLEDevice::init(bleName);

  BLEServer *pServer = BLEDevice::createServer();
  if (pServer == nullptr) {
    Serial.println("BLE: Error creating server");
    return;
  }
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  if (pService == nullptr) {
    Serial.println("BLE: Error creating service");
    return;
  }

  // TX characteristic (notify to phone)
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  // RX characteristic (write from phone)
  BLECharacteristic *pCharacteristicRX = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristicRX->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE: Advertising, waiting for client...");
}

// ========== HELPER FUNCTIONS ==========

float computeDistanceThreshold(int potRaw) {
  float minDist = 20.0;
  float maxDist = 150.0;
  float fraction = potRaw / 4095.0;
  return minDist + fraction * (maxDist - minDist);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms
  if (duration == 0) return -1.0;
  return duration / 58.0;
}

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}

// ========== SETUP ==========

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Pins
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  dht.begin();

  setupBLE();

  Serial.println("=== ESP32 Alarm + BLE ===");
}

// ========== LOOP ==========

void loop() {
  unsigned long now = millis();

  // ----- Toggle button -----
  bool buttonNowPressed = (digitalRead(BUTTON_PIN) == LOW); // active LOW
  if (buttonNowPressed && !lastButtonPressed) {
    systemEnabled = !systemEnabled;
    Serial.print("System Toggled: ");
    Serial.println(systemEnabled ? "ON" : "OFF");
  }
  lastButtonPressed = buttonNowPressed;

  // ----- Sensor readings -----
  int pirVal   = digitalRead(PIR_PIN);
  float dist   = readDistanceCm();
  int ldrRaw   = analogRead(LDR_PIN);
  int potRaw   = analogRead(POT_PIN);
  float distThresh = computeDistanceThreshold(potRaw);

  bool isDark  = (ldrRaw > LDR_DARK_THRESHOLD);
  bool motionDetected = (pirVal == 1);
  bool withinDistance = (dist > 0 && dist < distThresh);

  // DHT every 2s (optional)
  if (now - lastDhtTime > 2000) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      lastHum  = h;
      lastTemp = t;
    }
    lastDhtTime = now;
  }

  // ===== ALARM LOGIC =====
  bool alarmTriggered = false;

  if (!systemEnabled) {
    alarmTriggered = false;
  } else if (isDark) {
    // DARK = auto alarm
    alarmTriggered = true;
  } else {
    // LIGHT = need motion + distance
    alarmTriggered = motionDetected && withinDistance;
  }

  // LED: red if alarm, green otherwise
  if (alarmTriggered) {
    setLED(true, false, false);
  } else {
    setLED(false, true, false);
  }

  // ===== ONLY SEND SERIAL & BLE WHEN A NEW DETECTION HAPPENS =====
  if (alarmTriggered && !lastAlarmState) {
    // This block runs only on the rising edge of alarm
    String msg = "DETECTED! PIR=" + String(pirVal) +
                 " Dist=" + String(dist) + "cm" +
                 " LDR=" + String(ldrRaw) +
                 " Dark=" + String(isDark ? "YES" : "NO") +
                 " Pot=" + String(potRaw) +
                 " Thresh=" + String(distThresh, 1);

    Serial.println("-------------------------------------------------");
    Serial.println("ALARM TRIGGERED (new event)");
    Serial.println(msg);
    Serial.print("Temp(C)=");
    Serial.print(lastTemp);
    Serial.print(" Hum(%)=");
    Serial.println(lastHum);

    // Send over BLE (to LightBlue, subscribed to TX characteristic)
    if (pCharacteristic != nullptr) {
      pCharacteristic->setValue(msg.c_str());
      pCharacteristic->notify();
    }
  }

  lastAlarmState = alarmTriggered;

  // Small delay to calm things down
  delay(100);
}
