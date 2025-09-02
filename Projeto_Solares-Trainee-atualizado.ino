// C++ code
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>
#include <DHT.h>

const char* ssid = "nome da rede";
const char* senha = "senha da rede";

const char* websocket_server_host = "ip da rede";
const uint16_t websocket_server_port = 3001;

WebSocketsClient webSocket;
StaticJsonDocument<200> jsonDoc;

//Definição dos pinos do LCD, sendo, na ordem: (RS, E, DB4, DB5, DB6, DB7).
LiquidCrystal lcd(5, 19, 13, 27, 26, 25);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) { // Verifica qual tipo de evento ocorreu.
        case WStype_DISCONNECTED: // Caso a conexão seja perdida.
            lcd.print("[WSc] Desconectado!");
            break;
        case WStype_CONNECTED: // Caso a conexão seja estabelecida com sucesso.
            lcd.print("[WSc] Conectado!"); // 'payload' aqui contém a URL do servidor.
            webSocket.sendTXT("ESP32 Conectado"); // Envia uma mensagem de texto simples ao servidor para avisar que conectou.
            break;
        case WStype_TEXT: // Caso receba uma mensagem de texto do servidor.
            lcd.print("[WSc] Recebeu!");
            break;
        // Os outros casos são para cenários mais avançados (dados binários, fragmentação, etc.) que não usaremos aqui.
        case WStype_ERROR:
        case WStype_BIN:
        // ...
            break;
    }
}
// Pinos dos componentes
//LED vemelho == desligado, LED verde == ligado
#define DHTTYPE DHT22
#define DHTPIN 4
DHT dht(DHTPIN, DHTTYPE, 15);

const int pinoAnalogico = 32; //pino para a detectar o acelerador
const int botaoPin = 2;   // Botão conectado ao pino D2
const int ledVermelho = 15;    // LED vermelho conectado ao pino D7
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
int estadoAnteriorBotao = LOW; // Corrigido para HIGH devido ao INPUT_PULLUP

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
  lcd.print("Conectando ao wifi...");
  while(WiFi.status()!= WL_CONNECTED){
    delay(500);
    lcd.print(".");
  }
  lcd.clear();
  lcd.print("WIFI CONECTADO!");
  delay(1000);
  lcd.print("Endereço IP: ");
  delay(1000);
  lcd.clear();
  lcd.print(WiFi.localIP());
  delay(1000);
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
      } else {
        digitalWrite(ledVermelho, LOW);  // Apaga o LED vermelho
        digitalWrite(ledVerde, HIGH);// Acende o LED verde
        digitalWrite(Lcd, HIGH);//Liga o LED do LCD
        lcd.clear();         // limpa o display
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
    estadoAnteriorBotao = estadoBotao;
    ultimaMudanca = millis();
  }

  // Se o sistema estiver ligado (led verde aceso)
  if(!led1Ligado){
    // --- [INÍCIO DA CORREÇÃO] ---
    // A leitura do sensor de temperatura agora é feita de forma mais robusta.
    if (millis() - ultimaLeituraSensor >= intervaloLeitura) {
      ultimaLeituraSensor = millis(); // Atualiza o tempo da última leitura
      float h = dht.readHumidity();
      float t = dht.readTemperature();


      // É uma boa prática ler a umidade e a temperatura em sequência.
      if(webSocket.isConnected()){

      jsonDoc.clear();
      jsonDoc["temperatura"] = t;
      jsonDoc["velocidade"] = pedal;
      
      String output;
      
      serializeJson(jsonDoc, output);

      String informações = "42[\"sensorData\"," + output + "]";

      webSocket.sendTXT(informações);}
      else{
        Serial.println("Websocket deu ruim");
      }

      // Verifica se qualquer uma das leituras falhou.
      // A função isnan() retorna 'true' se o valor não for um número válido,
      // que é o que a biblioteca DHT retorna quando ocorre um erro de leitura.
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
    lcd.print("TEMP");//Escreve "TEMP" de temperatura no lcd
    lcd.write((byte)6); // escreve o símbolo de grau
    lcd.setCursor(0,1);//seta a posição de cursor na coluna 0, linha 1 no LCD
    lcd.print("       ");//Escreve espacos vazios para a limpeza da variaveis no lcd
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
    lcd.print("   ");//Escreve espacos vazios para a limpeza da variaveis no lcd
    lcd.setCursor(10,1);//seta a posição de cursor na coluna 10, linha 1 no LCD
    lcd.print("N:");//Escreve "N" de nivel no lcd
    lcd.setCursor(12,1);//seta a posição de cursor na coluna 12, linha 1 no LCD
    lcd.print(pedal);//Escreve a variavel do pedal no lcd
  }
}
