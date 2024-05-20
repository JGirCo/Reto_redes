#include <Wire.h>

#define SLAVE_ADDRESS 0x80

// TODO: Cambiar estos parámereos por los que se usarán
float latitud = 25.9898797;
float longitud = 180.00675;
float altitud = 5.0;
float longitudObjetivo = 2.0;

// Utilizamos un puntero a función para almacenar la función de respuesta adecuada
void (*responseFunction)() = nullptr;

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
}

void loop() {
  // En este esclavo de i2c solo se ejecuta acción cuando se hace un request.
  // Por lo tanto no hace falta un loop
}

void requestEvent() {
  if (responseFunction != nullptr) {
    responseFunction(); // Llama a la función de respuesta adecuada
  }
}

void receiveEvent(int howMany) {
  if (Wire.available()) {
    byte command = Wire.read(); // Recibe byte como comando
    switch (command) {
      case 0x01: // LAT_ADDRESS
        responseFunction = sendLon;
        break;
      case 0x02: // LON_ADDRESS
        responseFunction = sendLat;
        break;
      case 0x03: // ALT_ADDRESS
        responseFunction = sendAlt;
        break;
      case 0x04: // OBJ_LON_ADDRESS
        responseFunction = sendObjLon;
        break;
      default:
        responseFunction = nullptr; // No se recibió un comando válido
    }
  }
}

// Definimos funciones separadas para cada tipo de dato a enviar
void sendLat() {
  Wire.write((byte *)&latitud,4); //Se divide la variable flotante como un arreglo de bytes y se envía
}

void sendLon() {
  Wire.write((byte *)&longitud,4);
}

void sendAlt() {
  Wire.write((byte *)&altitud,4);
}

void sendObjLon() {
  Wire.write((byte *)&longitudObjetivo,4);
}
