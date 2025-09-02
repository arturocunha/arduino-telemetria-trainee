// C++ code
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>
#include <DHT.h>

const char* ssid = "ERUS 2.4GHz";
const char* senha = "ultrabots3";

const char* websocket_server_host = "192.168.0.117";
const uint16_t websocket_server_port = 4000;

WebSocketsClient webSocket;
StaticJsonDocument<200> jsonDoc;

//Definição dos pinos do LCD, sendo, na ordem: (RS, E, DB4, DB5, DB6, DB7).
LiquidCrystal lcd(5, 19, 13, 27, 26, 25);

// --- Variáveis Globais ---
bool socketIOConnected = false; // Variável para controlar o estado do handshake do Socket.IO

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WSc] Desconectado!");
            socketIOConnected = false;
            break;

        case WStype_CONNECTED:
            Serial.printf("[WSc] Conexão física estabelecida com o URL: %s\n", payload);
            break;

        case WStype_TEXT:
            Serial.printf("[WSc] Mensagem recebida: %s\n", payload);

            // Passo 1: Servidor envia o pacote OPEN (começa com '0')
            if (payload[0] == '0') {
                Serial.println("[WSc] Pacote OPEN recebido. Enviando pacote CONNECT (40)...");
                // Passo 2: Cliente responde com o pacote CONNECT ('40')
                webSocket.sendTXT("40");
            }
            // Passo 3: Servidor confirma o CONNECT (enviando '40' de volta)
            else if (payload[0] == '4' && payload[1] == '0') {
                Serial.println("[WSc] Handshake completo! Conexão Socket.IO estabelecida.");
                socketIOConnected = true;
            }
            // Passo 4: Servidor envia PING para manter a conexão viva (começa com '2')
            else if (payload[0] == '2') {
                // Passo 5: Cliente responde com PONG ('3')
                webSocket.sendTXT("3");
            }
            break;

        // Os casos abaixo são para depuração
        case WStype_PING:
            Serial.println("[WSc] Ping (nativo) recebido.");
            break;
        case WStype_PONG:
            Serial.println("[WSc] Pong recebido.");
            break;
        default:
            break;
    }
}

// Pinos dos componentes
//LED vemelho == desligado, LED verde == ligado
#define DHTTYPE DHT22
#define DHTPIN 4
DHT dht(DHTPIN, DHTTYPE, 15);

const int pinoAnalogico = 32; //pino para a detectar o acelerador
const int botaoPin = 2;       // Botão conectado ao pino D2
const int ledVermelho = 15;   // LED vermelho conectado ao pino D7
const int ledVerde = 22; // LED verde conectado ao pino D8
const int Lcd = 23; //Coloca o led do LCD na porta 3

// --- Variáveis Globais ---
float temperatura = 0.0; //Inicializa a variável de temperatura
float tensao; //Inicializa a variável de tensao
int pedal; //Inicializa a variável de pedal
bool sensorOk = false; // Flag para verificar o estado do sensor

// --- Variáveis para o temporizador do sensor DHT ---
unsigned long ultimaLeituraSensor = 0;
const long intervaloLeitura = 2000; // Intervalo de 2 segundos entre leituras do DHT

byte segF[8] =
{
  B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111
};
byte segA[8] =
{
  B11111, B11111, B11111, B11111, B00000, B00000, B00000, B00000
};
byte segB[8] =
{
  B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111
};
byte segC[8] =
{
  B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100
};
byte segD[8] =
{
  B00000, B00000, B00000, B00000, B11111, B11111, B11111, B11111
};
byte segE[8] =
{
  B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111
};
byte simbolograus[8] = {
  B00100, B01010, B00100, B00000, B00000, B00000, B00000, B00000
};
byte separar[8] = {
  B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111
}; //Inicializa o caracter separar



// Variáveis para o controle do botão (debounce)
long ultimaMudanca = 0;
long delayBotao = 50;  // 50ms para evitar debounce
int estadoAnteriorBotao = LOW; // Começa como LOW por causa do INPUT_PULLUP

// Variável para o controle dos LEDs
bool led1Ligado = true; // Começa com o LED vermelho ligado

void setup() {
  // --- Inicia a comunicação serial para depuração ---
  Serial.begin(115200);
  Serial.println("Sistema iniciado. Aguardando comando...");

  dht.begin();

  pinMode(Lcd, OUTPUT); //Define o LED do LCD como saida

  lcd.createChar(0,segF); //Criação do caracter de segmento F
  lcd.createChar(1,segA); //Criação do caracter de segmento A
  lcd.createChar(2,segB); //Criação do caracter de segmento B
  lcd.createChar(3,segE); //Criação do caracter de segmento E
  lcd.createChar(4,segD); //Criação do caracter de segmento D
  lcd.createChar(5,segC); //Criação do caracter de segmento C
  lcd.createChar(6,simbolograus); //Criação do caracter de simbolo de graus celsius (º)
  lcd.createChar(7,separar); //Cria o caracter separar


  lcd.begin(16, 2);  // Inicializa o lcd com 16 colunas e 2 linhas
  lcd.display(); // Liga o display do LCD
  lcd.setCursor(0, 0); //Seta o cursor no ponto inicial do LCD
  lcd.clear(); //Limpa qualquer lixo de memória do LCD

  // Configura os pinos dos LEDs como saída
  pinMode(ledVermelho, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  // Configura o pino do botão como entrada e ativa o resistor pull-up interno
  pinMode(botaoPin, INPUT_PULLUP);

  // Define o estado inicial: o LED vermelho já começa ligado
  digitalWrite(ledVermelho, HIGH);
  digitalWrite(ledVerde, LOW);


  WiFi.begin(ssid, senha);
  Serial.println("Conectando ao wifi...");

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); // Espera 1 segundo
    Serial.print(".");
    
    tentativas++;
    if (tentativas > 20) { // Se não conectar em 20 segundos, desiste
        Serial.println("Nao foi possivel conectar ao WiFi.");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Falha no WiFi!");
        while(true); // Trava o programa aqui
    }
  }

  Serial.println("\nWIFI CONECTADO!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.print("WiFi Conectado!");
  delay(2000);
  lcd.clear();

  webSocket.begin(websocket_server_host, websocket_server_port, "/socket.io/?EIO=4&transport=websocket");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {

  webSocket.loop();

  int estadoBotao = digitalRead(botaoPin);
  int valorLido = analogRead(pinoAnalogico);//pega o valor da tensao do potenciometro
  tensao = valorLido * (3.3 / 4095.0);//transfoma do valor do arduino para tensao
  tensao = ((tensao - 0.2)*10)/(3.1 - 0.2);//regra de 3 para detectar a posicao do pedal

  // Garante que o valor do pedal fique sempre entre 0 e 10
  if (tensao < 0) {
    tensao = 0;
  }
  if (tensao > 10) {
    tensao = 10;
  }
  pedal = tensao; // transforma o valor da tensao para o valor inteiro da posicao do pedal

  // Lógica para debounce do botão
  if (estadoBotao != estadoAnteriorBotao && millis() - ultimaMudanca > delayBotao) {
    if (estadoBotao == LOW) { // Se o botão foi pressionado
        // Inverte o estado dos LEDs
        led1Ligado = !led1Ligado;

        // Acende ou apaga os LEDs com base no novo estado
        if (led1Ligado) {
            digitalWrite(ledVermelho, HIGH); // Acende o LED vermelho
            digitalWrite(ledVerde, LOW);  // Apaga o LED verde
            digitalWrite(Lcd, LOW); //Desliga o LCD
            lcd.clear(); //Limpa a tela do LCD

            if(socketIOConnected){
              jsonDoc.clear();
              jsonDoc["temperatura"] = sensorOk ? 0 : -99.9; // Envia um valor de erro se o sensor falhar
              jsonDoc["velocidade"] = 0;
              
              String output;
              serializeJson(jsonDoc, output);

              String pacote = "42[\"sensorData\"," + output + "]";
              webSocket.sendTXT(pacote);
              Serial.print("Pacote enviado: ");
              Serial.println(pacote);
            }
        } else {
            digitalWrite(ledVermelho, LOW);  // Apaga o LED vermelho
            digitalWrite(ledVerde, HIGH);// Acende o LED verde
            digitalWrite(Lcd, HIGH);//Liga o LED do LCD
            lcd.clear();       // limpa o display
            lcd.setCursor(5,0);  // define onde escrever
            lcd.write((byte)0);  //F
            lcd.write((byte)1);  //A
            lcd.write((byte)2);  //B
            lcd.setCursor(5, 1);
            lcd.write((byte)3);  //E
            lcd.write((byte)4);  //D
            lcd.write((byte)5);  //C
            // escreve "ON"
            lcd.setCursor(8,0);
            lcd.write((byte)7); //Cria barras de separação no LCD
            lcd.setCursor(8,1);
            lcd.write((byte)7); //Cria barras de separação no LCD
            lcd.setCursor(9,0);
            lcd.write((byte)1); //A
            lcd.setCursor(10,0);
            lcd.write((byte)7); //Utiliza a separação como perna do N
            lcd.setCursor(10,1);
            lcd.write((byte)7);//Utiliza a separação como perna do N
            delay(1000);
            lcd.clear();
        }
    }

    // Atualiza as variáveis de controle do botão
    // Agora, o estado anterior é corretamente atualizado com o estado atual, fazendo o debounce funcionar.
    estadoAnteriorBotao = estadoBotao;
    ultimaMudanca = millis();
  }

  // Se o sistema estiver ligado (led verde aceso)
  if(!led1Ligado){
    if (millis() - ultimaLeituraSensor >= intervaloLeitura) {
        ultimaLeituraSensor = millis(); // Atualiza o tempo da última leitura
        float h = dht.readHumidity();
        float t = dht.readTemperature();


        // É uma boa prática ler a umidade e a temperatura em sequência.
       if(socketIOConnected){ // Usa a flag de conexão do Socket.IO
            jsonDoc.clear();
            jsonDoc["temperatura"] = sensorOk ? temperatura : -99.9; // Envia um valor de erro se o sensor falhar
            jsonDoc["velocidade"] = pedal;
            
            String output;
            serializeJson(jsonDoc, output);

            // O formato do pacote para emitir um evento no Socket.IO é 42["event_name", {payload}]
            String pacote = "42[\"sensorData\"," + output + "]";
            webSocket.sendTXT(pacote);
            Serial.print("Pacote enviado: ");
            Serial.println(pacote);
        } else {
            Serial.println("Websocket (Socket.IO) não conectado. Aguardando handshake..."); 
        }
        // Verifica se qualquer uma das leituras falhou.
        if (isnan(h) || isnan(t)) {
            Serial.println("Falha ao ler o sensor DHT!");
            sensorOk = false; // Mantém a flag de erro
        } else {
            temperatura = t; // Atualiza a variável global de temperatura
            sensorOk = true; // Define a flag como OK
        }
    }

    // --- Lógica de exibição no LCD ---
    lcd.setCursor(7,0); //seta a posição de cursor na coluna 7, linha 0 no LCD
    lcd.write((byte)7); //Escreve o o caracter de separação em cima
    lcd.setCursor(7,1); //seta a posição de cursor na coluna 7, linha 1 no LCD
    lcd.write((byte)7); //Escreve o o caracter de separação em baixo
    lcd.setCursor(0,0);//seta a posição de cursor na coluna 0, linha 0 no LCD
    lcd.print("TEMP"); //Escreve "TEMP" de temperatura no lcd
    lcd.write((byte)6); // escreve o símbolo de grau
    lcd.setCursor(0,1);//seta a posição de cursor na coluna 0, linha 1 no LCD
    lcd.print("      ");//Escreve espacos vazios para a limpeza da variaveis no lcd
    lcd.setCursor(0,1);//seta a posição de cursor na coluna 0, linha 1 no LCD

    // Mostra "Erro" no LCD se a leitura falhar
    if (sensorOk) {
        lcd.print(temperatura,1); //Escreve a variavel de temperatura no lcd
    } else {
        lcd.print("Erro");
    }

    lcd.setCursor(10,0);//seta a posição de cursor na coluna 10, linha 0 no LCD
    lcd.print("PEDAL"); //Escreve "PEDAL" no lcd
    lcd.setCursor(12,1);//seta a posição de cursor na coluna 10, linha 1 no LCD
    lcd.print("    ");//Escreve espacos vazios para a limpeza da variaveis no lcd
    lcd.setCursor(10,1);//seta a posição de cursor na coluna 10, linha 1 no LCD
    lcd.print("N:");//Escreve "N" de nivel no lcd
    lcd.setCursor(12,1);//seta a posição de cursor na coluna 12, linha 1 no LCD
    lcd.print(pedal);//Escreve a variavel do pedal no lcd
  }
}