# trab_ubiquos

## Como adicionar o código no Arduino?

Primeiro, é necessário instalar a Arduino IDE através do seguinte link: [Arduino IDE](https://www.arduino.cc/en/software).

Após a instalação, clique em `File` e em seguida em `Preferences`. No campo "Additional boards manager URLs" adicione o seguinte link: `https://espressif.github.io/arduino-esp32/package_esp32_index.json` e clique em `OK`. Isso baixará as bibliotecas do ESP32 para o Arduino.

Para carregar o código para o ESP, é necessário clicar em "Upload", indicado por uma seta apontando para a direita. Fique atento ao `Output`, que irá informar quando será necessário apertar o botão de boot do ESP para que o código seja baixado pela placa. Caso contrário, não será possível carregar o código no ESP. Além disso, verifique se a porta conectada ao ESP está correta.

### Atenção extra:

Ao plugar o ESP no PC com Windows, vá em "Device Manager". Caso a aba "Ports (COM & LPT)" não apareça, isso indica que falta um driver para ser instalado. Na aba "Other Devices", você irá se deparar com uma porta parecida com este nome: "CP2102 USB to UART Bridge Controller", o que indica que o ESP foi detectado mas precisa do driver correto. Vá para o seguinte link [CP2102 USB to UART Bridge VCP Drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads) e realize o download dos drivers. Após isso, a comunicação funcionará corretamente.
