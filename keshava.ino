#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// --------------------------------------------------
// GPS OBJECTS
// --------------------------------------------------
TinyGPSPlus gps;
HardwareSerial GPS(1);

static const int buttonPin = 22;
static const int gsrPin = 34;
Servo myservo;
static const int servoPin = 18;
const int buzzer = 5;
const int vibrator=14;

const int heartRate[] = {70,90,110,150,90,110,160,70,80,100,180};

// --------------------------------------------------
// NETWORK CREDENTIALS
// --------------------------------------------------
const char* ssid     = "POCO X4 Pro 5G";
const char* password = "leomalli";
const char* botToken = "8565751312:AAHQtirqvaUjlrMlnHZKUezJe61nKV5xFJw";
const char* chatId   = "5511807066";

volatile int btnFlag = -1;
bool btDone = false;

String incomingData = "";
BluetoothSerial SerialBT;

// --------------------------------------------------
//   TASK: GSR + HEART RATE SIMULATION
// --------------------------------------------------
void gsr(void *parameter) {
  int len = sizeof(heartRate)/sizeof(heartRate[0]);
  int j = 0;

  while (1) {
    long sum = 0;

    for (int i = 0; i < 10; i++) {
      sum += analogRead(gsrPin);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    float avg = sum / 10.0;
    float voltage = avg * (3.3 / 4095.0);
    float humanRes = (3.3 - voltage) * 10000.0 / voltage;
    float humanRes_k = humanRes / 1000.0;

    Serial.printf("Human resistance: %.2f kΩ\n", humanRes_k);
    Serial.printf("Heart rate: %d\n", heartRate[j]);

    j = (j + 1) % len;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// --------------------------------------------------
//   TASK: SERVO + BUZZER CONTROL
// --------------------------------------------------
void deterrent(void *parameter){
  while(1){
    if(btnFlag == 1){
      myservo.write(180);
      digitalWrite(buzzer, HIGH);
      digitalWrite(vibrator,HIGH);
    }
    else if(btnFlag == 0){
      myservo.write(0);
      digitalWrite(buzzer, LOW);
      digitalWrite(vibrator,LOW);
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

// --------------------------------------------------
//   TASK: BUTTON SHORT/LONG PRESS DETECTOR
// --------------------------------------------------
void button(void *parameter) {

  bool lastState = HIGH;
  bool state;
  unsigned long pressTime = 0;
  unsigned long releaseTime = 0;
  const unsigned long longPressTime = 1500;

  while(1){
    state = digitalRead(buttonPin);

    if (lastState == HIGH && state == LOW) {
      pressTime = millis();
    }
    else if (lastState == LOW && state == HIGH) {
      releaseTime = millis();
      unsigned long duration = releaseTime - pressTime;

      if (duration < longPressTime) {
        btnFlag = 1;
        // Serial.println("Short press detected");
      }
      else {
        btnFlag = 0;
        // Serial.println("Long press detected");
      }
    }

    lastState = state;
    vTaskDelay(80 / portTICK_PERIOD_MS);
  }
}

void serialRead(void *parameter){
  while(1){
    if(Serial.available()>0){
      String led_delay1=Serial.readStringUntil('\n');
      btnFlag=led_delay1.toInt();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --------------------------------------------------
//   TASK: SEND TELEGRAM MSG WITH GPS
// --------------------------------------------------
void telegram(void *parameter) {

  float latitude = 15.167845, longitude = 76.850261;

  while (1) {

    while (GPS.available()) {
      gps.encode(GPS.read());
    }

    if (gps.location.isUpdated()) {
      latitude  = gps.location.lat();
      longitude = gps.location.lng();
    }

    if (btnFlag == 1) {

      // Serial.println("Sending Telegram message…");

      WiFiClientSecure client;
      client.setInsecure();

      if (!client.connect("api.telegram.org", 443)) {
        // Serial.println("❌ Telegram connection failed");
        btnFlag = -1;
        vTaskDelay(300 / portTICK_PERIOD_MS);
        continue;
      }

      String text =
        "Emergency Alert!\n"
        "Latitude: " + String(latitude, 6) + "\n"
        "Longitude: " + String(longitude, 6);

      text.replace(" ", "%20");
      text.replace("\n", "%0A");

      String url =
        "/bot" + String(botToken) +
        "/sendMessage?chat_id=" + chatId + "&text=" + text;

      client.println("GET " + url + " HTTP/1.1");
      client.println("Host: api.telegram.org");
      client.println("Connection: close");
      client.println();

      // Serial.println("✔ Telegram message sent");
      btnFlag = -1;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(gsrPin, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(vibrator,OUTPUT);

  myservo.attach(servoPin);
  myservo.write(0);

  SerialBT.begin("ESP32");
  // Serial.println("Bluetooth Initialized");

  // GPS UART
  GPS.begin(9600, SERIAL_8N1, 16, 17);
  // Serial.println("GPS Initialized");

  // WIFI
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(400);
  }

  Serial.println("\n✔ WiFi connected");
}

// --------------------------------------------------
// LOOP ONLY WAITS FOR BLUETOOTH THEN CREATES TASKS
// --------------------------------------------------
void loop() {

  if (!btDone && SerialBT.available()) {

    incomingData = SerialBT.readStringUntil('\n');
    incomingData.trim();

    Serial.print("Raw Received: ");
    Serial.println(incomingData);

    int c1 = incomingData.indexOf(',');
    int c2 = incomingData.indexOf(',', c1 + 1);

    if (c1 > 0 && c2 > c1) {

      String ph1 = incomingData.substring(0, c1);
      String ph2 = incomingData.substring(c1 + 1, c2);
      String ph3 = incomingData.substring(c2 + 1);

      Serial.println("===== Phone Numbers Received =====");
      Serial.println("Phone 1: " + ph1);
      Serial.println("Phone 2: " + ph2);
      Serial.println("Phone 3: " + ph3);
      Serial.println("=================================");

      btDone = true;
    }
    else {
      Serial.println("Invalid data format!");
    }
  }

  // --------------------------------------------------
  // After BT received → Start tasks → Delete loop
  // --------------------------------------------------
  if (btDone) {

    // Serial.println("Starting tasks...");

    xTaskCreatePinnedToCore(gsr, "GSR", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(deterrent, "deterrent", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(button, "Button", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(telegram, "Telegram", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(serialRead,"Serial read",4096,NULL,1,NULL,1);

    // Serial.println("All tasks started. Deleting loop task...");
    if (SerialBT.hasClient()) {
      SerialBT.disconnect();   // force disconnect
    }
    SerialBT.end();            // shutdown SPP
    btStop();                  // turn off BT controller

    // Serial.println("Bluetooth fully stopped. Deleting loop task...");

    vTaskDelete(NULL);   // delete the loop task safely
  }

  vTaskDelay(50 / portTICK_PERIOD_MS);
}
