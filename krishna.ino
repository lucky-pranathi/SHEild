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
static const int servoPin=18;
const int buzzer=5;
const int heartRate[] = {70,90,110,150,90,110,160,70,80,100,180};

// --------------------------------------------------
// NETWORK CREDENTIALS
// --------------------------------------------------
const char* ssid     = "Hare srinivasa";
const char* password = "Prasanna@16";
const char* botToken = "8565751312:AAHQtirqvaUjlrMlnHZKUezJe61nKV5xFJw";
const char* chatId   = "5511807066";

volatile int btnFlag = -1;


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

void deterrent(void *parameter){
  while(1){
    if(btnFlag==1){
      myservo.write(180);
      digitalWrite(buzzer, HIGH);
      
    }else if(btnFlag==0){
      myservo.write(0);
      digitalWrite(buzzer, LOW);
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
  const unsigned long longPressTime = 1500; // ms

  while(1){
    state = digitalRead(buttonPin);

    // Button pressed
    if (lastState == HIGH && state == LOW) {
      pressTime = millis();
    }

    // Button released
    else if (lastState == LOW && state == HIGH) {
      releaseTime = millis();
      unsigned long duration = releaseTime - pressTime;

      if (duration < longPressTime) {
        btnFlag = 1;
        Serial.println("Short press detected");
      }
      else {
        btnFlag = 0;
        Serial.println("Long press detected");
      }
    }

    lastState = state;
    vTaskDelay(80 / portTICK_PERIOD_MS);
  }
}



// --------------------------------------------------
//   TASK: SEND TELEGRAM MSG WITH GPS
// --------------------------------------------------
void telegram(void *parameter) {

  float latitude = 0.0, longitude = 0.0;

  while (1) {

    // GPS reading (non-blocking)
    while (GPS.available()) {
      gps.encode(GPS.read());
    }

    if (gps.location.isUpdated()) {
      latitude  = gps.location.lat();
      longitude = gps.location.lng();
    }

    // Execute only on button short press
    if (btnFlag == 1) {

      Serial.println("Sending Telegram message…");

      WiFiClientSecure client;
      client.setInsecure();

      if (!client.connect("api.telegram.org", 443)) {
        Serial.println("❌ Telegram connection failed");
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

      Serial.println("✔ Telegram message sent");
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
  myservo.attach(servoPin); 
  myservo.write(0);
  pinMode(buzzer, OUTPUT); 

  // GPS UART
  GPS.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS Initialized");

  // WIFI
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(400);
  }

  Serial.println("\n✔ WiFi connected");

  // TASK CREATION
  xTaskCreatePinnedToCore(gsr, "GSR", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(deterrent,"deterrent",4096,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(button, "Button", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(telegram, "Telegram", 4096, NULL, 1, NULL, 1);
}

void loop() { }
