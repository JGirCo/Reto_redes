#include <Wire.h>

#define SLAVE_ADDRESS 0x80
#define LON_ADDRESS 0X01
#define LAT_ADDRESS 0X02
#define ALT_ADDRESS 0X03
#define OBJ_LON_ADDRESS 0X04

#define I2C_SDA 26
#define I2C_SCL 27

float latitud, longitud, altitud, latitudObjetivo;

void setup() {
  Serial.begin(115200); // Inicia la comunicación serial
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Ejemplo de conexión al esclavo i2c");
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
    Serial.print("Latitud = ");
    latitud = i2cRead(LAT_ADDRESS);
    Serial.println(latitud, 4);

    Serial.print("Longitud = ");
    longitud = i2cRead(LON_ADDRESS);
    Serial.println(longitud, 4);

    Serial.print("Altitud = ");
    altitud = i2cRead(ALT_ADDRESS);
    Serial.println(altitud, 4);

    Serial.print("Latitud Objetivo = ");
    latitudObjetivo = i2cRead(OBJ_LON_ADDRESS);
    Serial.println(latitudObjetivo, 4);

    delay(1000);
}

float i2cRead(uint8_t address) {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(address); // Escribe al ESP esclavo
  Wire.endTransmission(false);
  Wire.requestFrom(SLAVE_ADDRESS, 4, true); // La latitud es una variable flotante, por lo cual se solicitan 4 bytes

  union {
    float floatValue;
    uint8_t bytes[4];
  } data;

  for (int i = 0; i < 4; i++) {
    data.bytes[i] = Wire.read();
  }

  return data.floatValue;
}
