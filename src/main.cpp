#include <Arduino.h>
#include <NimBLEDevice.h>

#define RS485_DE 21
#define RS485_RE 22
#define RXD2 18
#define TXD2 19

const byte humi[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x02, 0x64, 0x0e};
const byte temp[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x01, 0x24, 0x0f};
const byte cond[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xce};
const byte phph[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0b};
const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
const byte sali[] = {0x01, 0x03, 0x00, 0x07, 0x00, 0x01, 0xe3, 0x5b};
const byte tds[]  = {0x01, 0x03, 0x00, 0x08, 0x00, 0x01, 0xe0, 0x58};
byte values[11];


// BLE UUIDs for Nordic UART Service
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTxCharacteristic;
NimBLECharacteristic* pRxCharacteristic;

// Global sums for sensor averages
float humiditySum = 0, temperatureSum = 0;
float conductivitySum = 0, tdsSum = 0, salinitySum = 0;
float phSum = 0, nitroSum = 0, phosSum = 0, potaSum = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.println("iPhone connected!");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.println("iPhone disconnected!");
        NimBLEDevice::getAdvertising()->start(); // Restart advertising
        Serial.println("Advertising restarted, waiting for connection...");
    }
};

// Add this function to send BLE data
void sendBleData(float humiditySum, float temperatureSum, float conductivitySum, float tdsSum, float salinitySum, float phSum, float nitroSum, float phosSum, float potaSum) {
    String bleMsg;
    bleMsg += "Humidity (avg) = " + String(humiditySum / 10.0, 1) + " %\n";
    bleMsg += "Temperature (avg) = " + String(temperatureSum / 10.0, 1) + " deg.C\n";
    bleMsg += "Conductivity (avg) = " + String(conductivitySum / 10.0, 1) + " uS/cm\n";
    bleMsg += "TDS (avg) = " + String(tdsSum / 10.0, 1) + " mg/L\n";
    bleMsg += "Salinity (avg) = " + String(salinitySum / 10.0, 1) + " ppm\n";
    bleMsg += "pH (avg) = " + String(phSum / 10.0, 1) + "\n";
    bleMsg += "Nitrogen (avg) = " + String(nitroSum / 10.0, 1) + " mg/L\n";
    bleMsg += "Phosphorus (avg) = " + String(phosSum / 10.0, 1) + " mg/L\n";
    bleMsg += "Potassium (avg) = " + String(potaSum / 10.0, 1) + " mg/L\n\n\n\n\n\n\n\n";

    pTxCharacteristic->setValue(bleMsg.c_str());
    pTxCharacteristic->notify();
    Serial.println("Sent to iPhone:\n" + bleMsg);
}

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        std::string value = pCharacteristic->getValue();
        Serial.print("Received from iPhone: ");
        Serial.println(value.c_str());
        // Call sendBleData with the latest averages
        extern float humiditySum, temperatureSum, conductivitySum, tdsSum, salinitySum, phSum, nitroSum, phosSum, potaSum;
        sendBleData(humiditySum, temperatureSum, conductivitySum, tdsSum, salinitySum, phSum, nitroSum, phosSum, potaSum);
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Soil Sensor starting...");
    Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    pinMode(RS485_DE, OUTPUT);
    pinMode(RS485_RE, OUTPUT);
    digitalWrite(RS485_DE, LOW); // Receive mode
    digitalWrite(RS485_RE, LOW); // Receive mode
    delay(1000);

    NimBLEDevice::init("ESP32-SoilSensor");
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );

    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    Serial.println("Waiting for iPhone to connect...");
}
int readModbusValue(const byte* cmd, int scale = 1, int offset = 3) {
  while (Serial1.available()) Serial1.read();
  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
  delay(10);
  for (uint8_t i = 0; i < 8; i++) Serial1.write(cmd[i]);
  Serial1.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);

  unsigned long start = millis();
  int bytesRead = 0;
  while ((bytesRead < 7) && (millis() - start < 500)) {
    if (Serial1.available()) {
      values[bytesRead++] = Serial1.read();
    }
  }
  if (bytesRead == 7) {
    return int(values[offset] << 8 | values[offset + 1]);
  }
  return -1; // error
}

void readSensor(const byte* cmd, const char* label, const char* unit, float scale = 1.0) {
  Serial.print(label);

  // Clear Serial1 buffer
  while (Serial1.available()) Serial1.read();

  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
  delay(10);
  for (uint8_t i = 0; i < 8; i++) Serial1.write(cmd[i]);
  Serial1.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);

  // Wait for response with timeout
  unsigned long start = millis();
  int bytesRead = 0;
  while ((bytesRead < 7) && (millis() - start < 500)) {
    if (Serial1.available()) {
      values[bytesRead++] = Serial1.read();
    }
  }

  if (bytesRead == 7) {
    int val = int(values[3] << 8 | values[4]);
    Serial.print("= ");
    Serial.print(val / scale, 1);
    if (unit) {
      Serial.print(" ");
      Serial.print(unit);
    }
  } else {
    Serial.print("= Error: No/Incomplete response");
  }
  Serial.println();
  delay(200);
}

void readHumidityAndTemperature() {
  // Clear Serial1 buffer
  while (Serial1.available()) Serial1.read();

  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
  delay(10);
  for (uint8_t i = 0; i < 8; i++) Serial1.write(humi[i]);
  Serial1.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);

  // Wait for response with timeout
  unsigned long start = millis();
  int bytesRead = 0;
  while ((bytesRead < 7) && (millis() - start < 500)) {
    if (Serial1.available()) {
      values[bytesRead++] = Serial1.read();
    }
  }

  if (bytesRead >= 7) {
    int humiVal = int(values[3] << 8 | values[4]);
    int tempVal = int(values[5] << 8 | values[6]);
    Serial.print("Humidity = ");
    Serial.print(humiVal / 10.0, 1);
    Serial.println(" %");
    Serial.print("Temperature = ");
    Serial.print(tempVal / 10.0, 1);
    Serial.println(" deg.C");
  } else {
    Serial.println("= Error: No/Incomplete response");
  }
  delay(200);
}

void loop() {
  // Reset the global variables:
  humiditySum = temperatureSum = 0;
  conductivitySum = tdsSum = salinitySum = 0;
  phSum = nitroSum = phosSum = potaSum = 0;

  // Arrays to hold 10 readings for each parameter */
  float humidityArr[10], temperatureArr[10];
  float conductivityArr[10], tdsArr[10], salinityArr[10];
  float phArr[10], nitroArr[10], phosArr[10], potaArr[10];

  // Sums for averaging
  // float humiditySum = 0, temperatureSum = 0;
  // float conductivitySum = 0, tdsSum = 0, salinitySum = 0;
  // float phSum = 0, nitroSum = 0, phosSum = 0, potaSum = 0;
  
  // Take 10 readings
  for (int i = 0; i < 10; i++) {
    // --- Humidity & Temperature ---
    float humidity = 0;
    float temperature = 0;
    float conductivity = 0;
    {
      // Clear Serial1 buffer
      while (Serial1.available()) Serial1.read();

      digitalWrite(RS485_DE, HIGH);
      digitalWrite(RS485_RE, HIGH);
      delay(10);
      for (uint8_t j = 0; j < 8; j++) Serial1.write(humi[j]);
      Serial1.flush();
      digitalWrite(RS485_DE, LOW);
      digitalWrite(RS485_RE, LOW);

      unsigned long start = millis();
      int bytesRead = 0;
      while ((bytesRead < 7) && (millis() - start < 500)) {
        if (Serial1.available()) {
          values[bytesRead++] = Serial1.read();
        }
      }
      if (bytesRead >= 7) {
        int humiVal = int(values[3] << 8 | values[4]);
        int tempVal = int(values[5] << 8 | values[6]);
        humidity = humiVal / 10.0;
        temperature = tempVal / 10.0;
      }
    }
    humidityArr[i] = humidity;
    temperatureArr[i] = temperature;
  
    

    // --- Conductivity ---
    {
      while (Serial1.available()) Serial1.read();
      digitalWrite(RS485_DE, HIGH);
      digitalWrite(RS485_RE, HIGH);
      delay(10);
      for (uint8_t j = 0; j < 8; j++) Serial1.write(cond[j]);
      Serial1.flush();
      digitalWrite(RS485_DE, LOW);
      digitalWrite(RS485_RE, LOW);

      unsigned long start = millis();
      int bytesRead = 0;
      while ((bytesRead < 7) && (millis() - start < 500)) {
        if (Serial1.available()) {
          values[bytesRead++] = Serial1.read();
        }
      }
      if (bytesRead == 7) {
        int val = int(values[3] << 8 | values[4]);
        conductivity = val;
      }
    }
    conductivityArr[i] = conductivity;
    tdsArr[i] = conductivity * 0.64;
    salinityArr[i] = conductivity * 0.55;

    // --- pH ---
    phArr[i] = 0;
    {
      while (Serial1.available()) Serial1.read();
      digitalWrite(RS485_DE, HIGH);
      digitalWrite(RS485_RE, HIGH);
      delay(10);
      for (uint8_t j = 0; j < 8; j++) Serial1.write(phph[j]);
      Serial1.flush();
      digitalWrite(RS485_DE, LOW);
      digitalWrite(RS485_RE, LOW);

      unsigned long start = millis();
      int bytesRead = 0;
      while ((bytesRead < 7) && (millis() - start < 500)) {
        if (Serial1.available()) {
          values[bytesRead++] = Serial1.read();
        }
      }
      if (bytesRead == 7) {
        int val = int(values[3] << 8 | values[4]);
        phArr[i] = val / 10.0;
      }
    }

    // --- Nitrogen ---
    nitroArr[i] = 0;
    {
      while (Serial1.available()) Serial1.read();
      digitalWrite(RS485_DE, HIGH);
      digitalWrite(RS485_RE, HIGH);
      delay(10);
      for (uint8_t j = 0; j < 8; j++) Serial1.write(nitro[j]);
      Serial1.flush();
      digitalWrite(RS485_DE, LOW);
      digitalWrite(RS485_RE, LOW);

      unsigned long start = millis();
      int bytesRead = 0;
      while ((bytesRead < 7) && (millis() - start < 500)) {
        if (Serial1.available()) {
          values[bytesRead++] = Serial1.read();
        }
      }
      if (bytesRead == 7) {
        int val = int(values[3] << 8 | values[4]);
        nitroArr[i] = val;
      }
    }

    // --- Phosphorus ---
    phosArr[i] = 0;
    {
      while (Serial1.available()) Serial1.read();
      digitalWrite(RS485_DE, HIGH);
      digitalWrite(RS485_RE, HIGH);
      delay(10);
      for (uint8_t j = 0; j < 8; j++) Serial1.write(phos[j]);
      Serial1.flush();
      digitalWrite(RS485_DE, LOW);
      digitalWrite(RS485_RE, LOW);

      unsigned long start = millis();
      int bytesRead = 0;
      while ((bytesRead < 7) && (millis() - start < 500)) {
        if (Serial1.available()) {
          values[bytesRead++] = Serial1.read();
        }
      }
      if (bytesRead == 7) {
        int val = int(values[3] << 8 | values[4]);
        phosArr[i] = val;
      }
    }

    // --- Potassium ---
    potaArr[i] = 0;
    {
      while (Serial1.available()) Serial1.read();
      digitalWrite(RS485_DE, HIGH);
      digitalWrite(RS485_RE, HIGH);
      delay(10);
      for (uint8_t j = 0; j < 8; j++) Serial1.write(pota[j]);
      Serial1.flush();
      digitalWrite(RS485_DE, LOW);
      digitalWrite(RS485_RE, LOW);

      unsigned long start = millis();
      int bytesRead = 0;
      while ((bytesRead < 7) && (millis() - start < 500)) {
        if (Serial1.available()) {
          values[bytesRead++] = Serial1.read();
        }
      }
      if (bytesRead == 7) {
        int val = int(values[3] << 8 | values[4]);
        potaArr[i] = val;
      }
    }

    delay(100); // Small delay between readings
  }
  
  // Calculate averages
  for (int i = 0; i < 10; i++) {
    humiditySum += humidityArr[i];
    temperatureSum += temperatureArr[i];
    conductivitySum += conductivityArr[i];
    tdsSum += tdsArr[i];
    salinitySum += salinityArr[i];
    phSum += phArr[i];
    nitroSum += nitroArr[i];
    phosSum += phosArr[i];
    potaSum += potaArr[i];
  }

  Serial.print("Humidity (avg) = ");
  Serial.print(humiditySum / 10.0, 1);
  Serial.println(" %");

  Serial.print("Temperature (avg) = ");
  Serial.print(temperatureSum / 10.0, 1);
  Serial.println(" deg.C");

  Serial.print("Conductivity (avg) = ");
  Serial.print(conductivitySum / 10.0, 1);
  Serial.println(" uS/cm");

  Serial.print("TDS (avg) = ");
  Serial.print(tdsSum / 10.0, 1);
  Serial.println(" mg/L");

  Serial.print("Salinity (avg) = ");
  Serial.print(salinitySum / 10.0, 1);
  Serial.println(" ppm");

  Serial.print("pH (avg) = ");
  Serial.println(phSum / 10.0, 1);

  Serial.print("Nitrogen (avg) = ");
  Serial.print(nitroSum / 10.0, 1);
  Serial.println(" mg/L");

  Serial.print("Phosphorus (avg) = ");
  Serial.print(phosSum / 10.0, 1);
  Serial.println(" mg/L");

  Serial.print("Potassium (avg) = ");
  Serial.print(potaSum / 10.0, 1);
  Serial.println(" mg/L");

  Serial.println();
  delay(10000); // Wait before next set of averages

  // --- BLE Notify to iPhone ---
  if (NimBLEDevice::getServer()->getConnectedCount() > 0) {
      String bleMsg = "Humidity (avg) = " + String(humiditySum / 10.0, 1) + " %\n";
      bleMsg += "Temperature (avg) = " + String(temperatureSum / 10.0, 1) + " deg.C\n";
      bleMsg += "Conductivity (avg) = " + String(conductivitySum / 10.0, 1) + " uS/cm\n";
      bleMsg += "TDS (avg) = " + String(tdsSum / 10.0, 1) + " mg/L\n";
      bleMsg += "Salinity (avg) = " + String(salinitySum / 10.0, 1) + " ppm\n";
      bleMsg += "pH (avg) = " + String(phSum / 10.0, 1) + "\n";
      bleMsg += "Nitrogen (avg) = " + String(nitroSum / 10.0, 1) + " mg/L\n";
      bleMsg += "Phosphorus (avg) = " + String(phosSum / 10.0, 1) + " mg/L\n";
      bleMsg += "Potassium (avg) = " + String(potaSum / 10.0, 1) + " mg/L\n\n\n\n\n\n\n\n\n";

      pTxCharacteristic->setValue(bleMsg.c_str());
      pTxCharacteristic->notify();
      Serial.println("Sent to iPhone:\n" + bleMsg);
  }
}
