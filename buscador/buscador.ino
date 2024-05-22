#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);
double latitud_destino[] = { 6.158494, 0.0, 0.0, 0.0 };
double longitud_destino[] = { -75.516716, 0.0, 0.0, 0.0 };

double lat = 0;
double lon = 0;

double latDistance = 0;
double lonDistance = 0;
double totalDistance = 0;

const double EarthRadiusKm = 6371.0;
const double DegToRad = 0.017453292519943295;

// Definir los protocolos
#define PROTOCOL_I2C 1
#define PROTOCOL_BLUETOOTH 3
#define PROTOCOL_TCP 2
// Pines
#define BUTTON_PIN 15
#define BUTTON_PIN2 18

// Variables compartidas
float latitud = 0.0;
float longitud = 0.0;
float altitud = 0.0;
float latitudObjetivo = 0.0;
int satCuenta = 0;

// LCD
#define I2C_ADDR 0x27
#define BACKLIGHT 0x08
#define En 0x04
#define Rw 0x02
#define Rs 0x01

int protocoloSeleccionado = 0;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
bool buttonPressed2 = false;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay = 150;

// Variables Bluetooth
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID_1("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID charUUID_2("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e");
static BLEUUID charUUID_3("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87f");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

static BLEAdvertisedDevice* myDevice;
BLERemoteCharacteristic* pRemoteChar_1;
BLERemoteCharacteristic* pRemoteChar_2;
BLERemoteCharacteristic* pRemoteChar_3;

// Variables TCP
WiFiClient client;
bool wifiConnected = false;
bool bluetoothConnected = false;
const char* ssid = "ESP32-AccessPoint";
const char* password = "12345678";
const char* serverIP = "192.168.4.1";  // Dirección IP del ESP32 AP

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                    uint8_t* pData,
                    size_t length,
                    bool isNotify) {
  if (pBLERemoteCharacteristic->getUUID().toString() == charUUID_1.toString()) {
    uint32_t counter = pData[0];
    for (int i = 1; i < length; i++) {
      counter = counter | (pData[i] << i * 8);
    }
    Serial.print("Characteristic 1 (Notify) from server: ");
    Serial.println(counter);
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println(" - Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  connected = true;
  pRemoteChar_1 = pRemoteService->getCharacteristic(charUUID_1);
  pRemoteChar_2 = pRemoteService->getCharacteristic(charUUID_2);
  pRemoteChar_3 = pRemoteService->getCharacteristic(charUUID_3);

  if (!connectCharacteristic(pRemoteService, pRemoteChar_1) || !connectCharacteristic(pRemoteService, pRemoteChar_2) || !connectCharacteristic(pRemoteService, pRemoteChar_3)) {
    pClient->disconnect();
    Serial.println("At least one characteristic UUID not found");
    return false;
  }
  return true;
}

bool connectCharacteristic(BLERemoteService* pRemoteService, BLERemoteCharacteristic* l_BLERemoteChar) {
  if (l_BLERemoteChar == nullptr) {
    Serial.print("Failed to find one of the characteristics: ");
    Serial.println(l_BLERemoteChar->getUUID().toString().c_str());
    return false;
  }
  Serial.println(" - Found characteristic: " + String(l_BLERemoteChar->getUUID().toString().c_str()));

  if (l_BLERemoteChar->canNotify())
    l_BLERemoteChar->registerForNotify(notifyCallback);

  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

void escribirMenu(double lantitud_act, double longuitud_act,
                  double Latitud_destino, double longitud_destino,
                  double latDistance, double lonDistance,
                  double totalDistance, int modo, int Sat, int modo2);

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  lcdInit();

  Serial.println("Starting Arduino Multi-Protocol application...");
  Wire.begin();

  xTaskCreatePinnedToCore(
    i2cTask,        // Function to implement the task
    "I2CTask",      // Name of the task
    10000,          // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    NULL,           // Task handle
    0);             // Core where the task should run

  xTaskCreatePinnedToCore(
    bluetoothTask,  // Function to implement the task
    "BluetoothTask",// Name of the task
    10000,          // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    NULL,           // Task handle
    1);             // Core where the task should run

  xTaskCreatePinnedToCore(
    tcpTask,        // Function to implement the task
    "TCPTask",      // Name of the task
    10000,          // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    NULL,           // Task handle
    1);             // Core where the task should run
}

int ObjetivoSeleccionado = 0;

void loop() {
  handleButtonPress();

  if (neogps.available()) {
    GPS();
    Serial.print(String(lat)+" "+String(lon)+" "+String(satCuenta)+" ");
    escribirMenu(lat, lon, latitud_destino[ObjetivoSeleccionado], longitud_destino[ObjetivoSeleccionado], latDistance, lonDistance, totalDistance, protocoloSeleccionado, satCuenta, ObjetivoSeleccionado);
    calculateDistances(lat, lon, latitud_destino[ObjetivoSeleccionado], longitud_destino[ObjetivoSeleccionado], latDistance, lonDistance, totalDistance);
  }
}

void handleButtonPress() {
  int reading = digitalRead(BUTTON_PIN);
  int reading2 = digitalRead(BUTTON_PIN2);

  if (reading == LOW && !buttonPressed && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    protocoloSeleccionado = (protocoloSeleccionado + 1) % 4;
    Serial.print("Protocolo seleccionado: ");
    Serial.println(protocoloSeleccionado);
  } 

  if (reading2 == LOW && !buttonPressed2 && (millis() - lastDebounceTime2) > debounceDelay) {
    ObjetivoSeleccionado = (ObjetivoSeleccionado + 1) % 4;
    Serial.print("Objetivo seleccionado: ");
    Serial.println(ObjetivoSeleccionado);
    lastDebounceTime2 = millis();
  } else if (reading2 == LOW) {
    buttonPressed2 = false;
  }
}

void i2cTask(void * parameter) {
  for (;;) {
    if (protocoloSeleccionado == PROTOCOL_I2C) {
      latitud = i2cRead(0x01);
      longitud = i2cRead(0x02);
      latitud = i2cRead(0x03);
      latitud_destino[1] = latitud;
      longitud_destino[1] = longitud;
      Serial.println("I2C - lat: " + String(latitud, 6) + " lon: " + String(longitud, 6));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void bluetoothTask(void * parameter) {
  for (;;) {
    if (protocoloSeleccionado == PROTOCOL_BLUETOOTH) {
      if (doConnect) {
        if (connectToServer()) {
          Serial.println("We are now connected to the BLE Server.");
        } else {
          Serial.println("We have failed to connect to the server.");
        }
        doConnect = false;
      }

      if (connected) {
        std::string rxValue = pRemoteChar_1->readValue();
        Serial.print("Characteristic 1 (readValue): ");
        Serial.println(rxValue.c_str());

        std::string ryValue = pRemoteChar_2->readValue();
        Serial.print("Characteristic 2 (readValue): ");
        Serial.println(ryValue.c_str());

        std::string rzValue = pRemoteChar_3->readValue();
        Serial.print("Characteristic 3 (readValue): ");
        Serial.println(rzValue.c_str());

        latitud = atof(rxValue.c_str());
        altitud = atof(ryValue.c_str());
        longitud = atof(rzValue.c_str());

        latitud_destino[3] = latitud;
        longitud_destino[3] = longitud;
      } else if (doScan) {
        BLEDevice::getScan()->start(0);
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void tcpTask(void * parameter) {
  for (;;) {
    if (protocoloSeleccionado == PROTOCOL_TCP) {
      if (!wifiConnected) {
        WiFi.begin(ssid, password);
        Serial.print("Conectando a ");
        Serial.print(ssid);

        while (WiFi.status() != WL_CONNECTED) {
          delay(150);
          Serial.print(".");
          if (digitalRead(BUTTON_PIN) == HIGH || digitalRead(BUTTON_PIN2) == HIGH) {
            handleButtonPress();
            Serial.println("Interruption");
            break;
          }
        }
        if (WiFi.status() == WL_CONNECTED) {
          wifiConnected = true;
          Serial.println("");
          Serial.println("WiFi conectado");
        }
      }

      if (wifiConnected && client.connect(serverIP, 80)) {
        Serial.println("Conectado al servidor");

        client.println("GET / HTTP/1.1");
        client.println("Host: " + String(serverIP));
        client.println("Connection: close");
        client.println();

        bool headersEnded = false;
        String response = "";
        while (client.connected() || client.available()) {
          if (client.available()) {
            String line = client.readStringUntil('\n');
            if (!headersEnded) {
              if (line == "\r") {
                headersEnded = true;
              }
            } else {
              response += line;
            }
          }
        }

        parseTCPResponse(response);
        client.stop();
      } else if (!wifiConnected) {
        Serial.println("Fallo al conectar al servidor: WiFi no está conectado");
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void parseTCPResponse(String response) {
  int index1 = response.indexOf(',');
  int index2 = response.indexOf(',', index1 + 1);
  int index3 = response.indexOf(',', index2 + 1);

  String strLongitud = response.substring(0, index1);
  String strLatitud = response.substring(index1 + 1, index2);
  String strAltitud = response.substring(index2 + 1, index3);
  String strLatitudFinal = response.substring(index3 + 1);

  longitud = strLongitud.toFloat();
  latitud = strLatitud.toFloat();
  altitud = strAltitud.toFloat();
  latitudObjetivo = strLatitudFinal.toFloat();
  latitud_destino[2] = longitud;
  longitud_destino[2] = latitud;
}

float i2cRead(uint8_t address) {
  Wire.beginTransmission(0x80);
  Wire.write(address);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(0x80, 4);

  union {
    float floatValue;
    uint8_t bytes[4];
  } data;

  for (int i = 0; i < 4; i++) {
    data.bytes[i] = Wire.read();
  }

  return data.floatValue;
}

// PANTALLA
void escribirMenu(double lantitud_act, double longuitud_act,
                  double Latitud_destino, double longitud_destino,
                  double latDistance, double lonDistance,
                  double totalDistance, int modo, int Sat, int modo2) {
  lcdSetCursor(0, 0);
  lcdPrint(String(lantitud_act, 6));
  lcdSetCursor(9, 0);
  lcdPrint(String(longuitud_act, 6));

  // Lat Long DESTINo
  lcdSetCursor(0, 1);
  lcdPrint(String(Latitud_destino, 6));
  lcdSetCursor(9, 1);
  lcdPrint(String(longitud_destino, 6));
  // NORTE SUR ESTE OESTE
  lcdSetCursor(2, 2);
  lcdPrint("      ");
  lcdSetCursor(2, 3);
  lcdPrint("        ");
  lcdSetCursor(0, 2);

  lcdPrint("N:" + String(latDistance, 0));
  lcdSetCursor(0, 3);
  lcdPrint("O:" + String(lonDistance, 0));
  lcdSetCursor(16, 3);
  lcdPrint("P:" + String(modo));

  lcdSetCursor(11, 3);
  lcdPrint("F:" + String(modo2));

  // DISTANCIA TOTAL, SATELITES
  lcdSetCursor(8, 2);
  lcdPrint("MG:" + String(totalDistance, 0));
  lcdSetCursor(16, 2);
  lcdPrint("S:" + String(Sat));
}

void lcdSendNibble(byte data) {
  data |= BACKLIGHT;
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(data | En);
  Wire.write(data & ~En);
  Wire.endTransmission();
  delayMicroseconds(50);
}

void lcdSendByte(byte data, byte mode) {
  byte highNibble = data & 0xF0;
  byte lowNibble = (data << 4) & 0xF0;
  lcdSendNibble(highNibble | mode);
  lcdSendNibble(lowNibble | mode);
}

void lcdWriteChar(char c) {
  lcdSendByte(c, Rs);
}

void lcdSetCursor(byte col, byte row) {
  byte rowOffsets[] = { 0x00, 0x40, 0x14, 0x54 };
  lcdSendByte(0x80 | (col + rowOffsets[row]), 0);
}

void lcdInit() {
  Wire.begin();
  lcdSendNibble(0x03 << 4);
  lcdSendNibble(0x03 << 4);
  lcdSendNibble(0x03 << 4);
  lcdSendNibble(0x02 << 4);
  lcdSendByte(0x28, 0);
  lcdSendByte(0x0C, 0);
  lcdSendByte(0x06, 0);
  lcdSendByte(0x01, 0);
  delayMicroseconds(2000);
}

void lcdPrint(String str) {
  for (int i = 0; i < str.length(); i++) {
    lcdWriteChar(str[i]);
  }
}

void processGPRMC(String nmea) {
  String fields[12];
  parseNMEA(nmea, fields, 12);

  char status = fields[2].charAt(0);

  if (status == 'A') {
    lat = convertNMEAToDegrees(fields[3]);
    lon = -1 * convertNMEAToDegrees(fields[5]);
    float speed = fields[7].toFloat();
    String date = fields[9];
  } else {
    Serial.println("Data is invalid");
  }
}

void processGPGGA(String nmea) {
  String fields[15];
  parseNMEA(nmea, fields, 15);

  int satCount = fields[7].toInt();
  satCuenta = satCount;
  float altitude = fields[9].toFloat();
}

void parseNMEA(String nmea, String* fields, int fieldCount) {
  int startIndex = 0;
  int fieldIndex = 0;

  while (fieldIndex < fieldCount && startIndex < nmea.length()) {
    int endIndex = nmea.indexOf(',', startIndex);
    if (endIndex == -1) {
      fields[fieldIndex] = nmea.substring(startIndex);
      break;
    } else {
      fields[fieldIndex] = nmea.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
      fieldIndex++;
    }
  }
}

double convertNMEAToDegrees(String nmeaCoord) {
  if (nmeaCoord.length() < 6) {
    return 0.0;
  }
  int degreeIndex = nmeaCoord.indexOf('.') - 2;
  double degrees = nmeaCoord.substring(0, degreeIndex).toDouble();
  double minutes = nmeaCoord.substring(degreeIndex).toDouble();
  return degrees + minutes / 60.0;
}

void GPS() {
  String nmeaSentence = neogps.readStringUntil('\n');
  nmeaSentence.trim();

  if (nmeaSentence.startsWith("$GPRMC")) {
    processGPRMC(nmeaSentence);
  }
  if (nmeaSentence.startsWith("$GPGGA")) {
    processGPGGA(nmeaSentence);
  }
}

void calculateDistances(double lat1, double lon1, double lat2, double lon2,
                        double& latDistance, double& lonDistance, double& totalDistance) {
  double lat1Rad = lat1 * DegToRad;
  double lon1Rad = lon1 * DegToRad;
  double lat2Rad = lat2 * DegToRad;
  double lon2Rad = lon2 * DegToRad;

  double deltaLat = lat2Rad - lat1Rad;
  double deltaLon = lon2Rad - lon1Rad;

  double a = sin(deltaLat / 2) * sin(deltaLat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(deltaLon / 2) * sin(deltaLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  totalDistance = EarthRadiusKm * c * 1000;

  latDistance = deltaLat * EarthRadiusKm * 1000;
  lonDistance = deltaLon * EarthRadiusKm * cos((lat1Rad + lat2Rad) / 2) * 1000;

  latDistance = (latDistance > 0) ? fabs(latDistance) : -fabs(latDistance);
  lonDistance = (lonDistance > 0) ? fabs(lonDistance) : -fabs(lonDistance);
}
