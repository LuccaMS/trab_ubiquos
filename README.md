# trab_ubiquos

## Como adicionar o código no Arduino?

Primeiro, é necessário instalar a Arduino IDE através do seguinte link: [Arduino IDE](https://www.arduino.cc/en/software).

Após a instalação, clique em `File` e em seguida em `Preferences`. No campo "Additional boards manager URLs" adicione o seguinte link: `https://espressif.github.io/arduino-esp32/package_esp32_index.json` e clique em `OK`. Isso baixará as bibliotecas do ESP32 para o Arduino.

Para carregar o código para o ESP, é necessário clicar em "Upload", indicado por uma seta apontando para a direita. Fique atento ao `Output`, que irá informar quando será necessário apertar o botão de boot do ESP para que o código seja baixado pela placa. Caso contrário, não será possível carregar o código no ESP. Além disso, verifique se a porta conectada ao ESP está correta.

### Atenção extra:

Ao plugar o ESP no PC com Windows, vá em "Device Manager". Caso a aba "Ports (COM & LPT)" não apareça, isso indica que falta um driver para ser instalado. Na aba "Other Devices", você irá se deparar com uma porta parecida com este nome: "CP2102 USB to UART Bridge Controller", o que indica que o ESP foi detectado mas precisa do driver correto.  [CP2102 USB to UART Bridge VCP Drivers](https://www.silabs.com/documents/public/software/CP210x_Windows_Drivers.zip) e realize o download dos drivers. Após isso, a comunicação funcionará corretamente.

## Como configurar o código?

O arquivo `sketch_may2a.ino` contém o código que o ESP utilizará. Nele, possuímos duas variáveis principais: `ssid` e `ssipasswordd`, que são as senhas do Wi-Fi que o ESP irá se conectar e devem ser configuradas de acordo com as redes disponíveis no momento. Além disso, na linha 48 do código, em `webSocket.begin("150.162.235.38", 999, "/");`, o ESP se conecta com um websocket em um IP da rede local e na porta 999. Esses parâmetros são dependentes do servidor de websocket. Tendo em vista que o ESP é um cliente, esses parâmetros foram definidos no arquivo `client.ipynb`, que usa o Python para manter um servidor ouvindo na porta 999 e no IP da máquina. Sendo assim, esses dois parâmetros devem ser alterados dependendo do servidor websocket.

## O que o código envia?

O código enviará para o servidor websocket uma lista de bytes, que pode ser convertida para uma imagem.

## Como configurar o código Python?

No Windows, é necessário abrir um prompt de comando e digitar o comando `ipconfig`. Através dele, é possível obter o endereço de IPv4, que será utilizado no código para definir o IP local de acesso. Além disso, uma porta deve ser selecionada. Vale salientar que ambos esses parâmetros devem ser iguais no ESP e no arquivo Python para a comunicação funcionar.
