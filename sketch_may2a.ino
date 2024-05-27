#include <HardwareSerial.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <Arduino.h>

#define TX 17
#define RX 16

HardwareSerial SerialPort(2);

//const char* ssid = "Anderson";
//const char* password = "cchr.org";

WebSocketsClient webSocket;
WiFiMulti WiFiMulti;

const char* ssid = "lab120";
const char* password = "labredes120";

void setup() {
  Serial.begin(115200);

  SerialPort.begin(115200, SERIAL_8N1, RX, TX);

  SerialPort.write("AT+BAUD=2\r");
  delay(1000);
  SerialPort.write("AT+FPS=19\r");
  delay(1000);
  SerialPort.write("AT+DISP=5\r");
  delay(1000);
  SerialPort.write("AT+BINN=1\r");
  delay(1000);
  SerialPort.write("AT+UNIT=9\r");
  delay(1000);
  SerialPort.flush();
  delay(1000);


  WiFiMulti.addAP(ssid, password);
  while(WiFiMulti.run() != WL_CONNECTED) {
    Serial.println("Não consegui conectar ao WIFI");
		delay(100);
	}

  webSocket.begin("150.162.235.38", 999, "/");

  webSocket.setReconnectInterval(100);
}

uint16_t unpack_unsigned_short(const uint8_t primeiro, const uint8_t segundo) {
  // Assuming raw_data is a pointer to a byte array

  // Extract the two bytes
  uint8_t byte1 = primeiro;
  uint8_t byte2 = segundo;

  // Combine the bytes into a 16-bit unsigned integer
  uint16_t result = (byte2 << 8) | byte1;

  return result;
}



void loop(){
  webSocket.loop();
  if(SerialPort.available()){
    uint8_t primeiro;
    uint8_t segundo;

    primeiro = SerialPort.read();
    segundo = SerialPort.read();

    if(primeiro == 0 && segundo == 255){
      uint8_t primiero_tamanho = SerialPort.read();
      uint8_t segundo_tamanho = SerialPort.read();

      uint16_t tamanho_dados = unpack_unsigned_short(primiero_tamanho, segundo_tamanho);

      if (tamanho_dados != 2516 && tamanho_dados != 10016 && tamanho_dados != 641) {
        Serial.print("Wrong size found !: ");
        return;
      }

      //Serial.println("Frame found as correct size");

      byte* data = new byte[tamanho_dados];
      SerialPort.readBytes(data, tamanho_dados);
      //Lendo todos os dados do pacote

      byte* image_data = new byte[tamanho_dados - 16];
      //Lendo agora somente os dados da imagem, não vamos usar as info das 16 primeiras posições
      for (int i = 0; i < tamanho_dados - 16; i++) {
        image_data[i] = data[i + 16];
      }
      
      if(webSocket.isConnected()){
          Serial.println("sending data to the websocket");
          webSocket.sendBIN(image_data,tamanho_dados - 16);
      }

      delete[] data;
      delete[] image_data;
    }
}
}
