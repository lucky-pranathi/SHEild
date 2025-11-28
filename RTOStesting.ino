#include<TinyGPSPlus.h>
#include<HardwareSerial.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <WiFi.h>        // ADD THIS
#include <WiFiClientSecure.h> // ADD THIS
#include <WebServer.h>     // <---- ADDED

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu=0;
#else
static const BaseType_t app_cpu=1;
#endif

String incomingData = "";
const int gsrPin = 34;
Servo myservo;
static volatile int telegramFlag=0;
static volatile int confirmFlag=1;
String message="Hello, this is a test message from ESP32!";
const int servoPin = 18;
const int buzzer=5;
static volatile uint8_t gsr_flag=0;
static volatile int btnFlag=-1;
static volatile float latitude,longitude;
static volatile uint8_t day,month,year;
static volatile uint8_t hr,minu,sec;
const int heartRate[]={70,90,110,150,90,110,160,70,80,100,180};
const int buttonPin = 15; 

const char* botToken = "8565751312:AAHQtirqvaUjlrMlnHZKUezJe61nKV5xFJw";  // Your Telegram Bot Token
const char* chatId = "5511807066";  // Replace with your Telegram Chat ID
const char* ssid = "POCO X4 Pro 5G";      // CHANGE
const char* password = "leomalli";  // CHANGE

BluetoothSerial SerialBT;   

TinyGPSPlus gps;
HardwareSerial GPS(1);

// ----------------------------------------
//   HTML WEB SERVER FOR DISPLAY
// ----------------------------------------
WebServer server(80);

String htmlStatusPage = "No data yet";

void handleStatus() {
  server.send(200, "text/html", htmlStatusPage);
}

void gsr(void *parameter){
  int sensorValue = 0;
  int gsr_avg = 0;

  // Debounce counters
  int sweatyCounter = 0;
  int noFingerCounter = 0;
  const int needConsecutive = 2; // require 2 consistent readings

  // Thresholds (based on your measurements)
  const int noFingerADCthreshold = 1800;    // ADC >= this => likely no fingers
  const float noFingerRes_kohm = 12.0;      // resistance <= this => treat as no-finger (10 kΩ observed)
  const float wetRes_kohm = 34.0;           // resistance >= this => "wet" (34-66 kΩ reported)
  const float dryResLower_kohm = 24.0;      // low bound for dry range (for reference)
  while(1){
    long sum = 0;
    for (int i = 0; i < 10; i++) {
      sensorValue = analogRead(gsrPin);
      sum += sensorValue;
      vTaskDelay(5/portTICK_PERIOD_MS);
    }
    gsr_avg = sum / 10;

    // ADC -> voltage
    float voltage = gsr_avg * (3.3 / 4095.0);

    // Calculate resistance assuming a 10k pull-down (adjust if your circuit differs)
    float humanRes = (3.3 - voltage) * 10000.0 / voltage; // in ohms
    float humanRes_kohm = humanRes / 1000.0;

    // Print readings
    Serial.print("Gsr average=: ");
    Serial.print(gsr_avg);
    Serial.print("  Human resistance: ");
    Serial.print(humanRes_kohm, 2);
    Serial.println(" kΩ");

    // // --- Decision logic with simple debounce ---
    // bool isNoFingerByADC = (gsr_avg >= noFingerADCthreshold);
    // bool isNoFingerByRes  = (humanRes_kohm <= noFingerRes_kohm);
    // bool isNoFinger       = (isNoFingerByADC || isNoFingerByRes);

    // if (isNoFinger) {
    //   noFingerCounter++;
    //   sweatyCounter = 0; // reset sweaty counter
    // } else {
    //   noFingerCounter = 0;
    // }

    // // Wet detection: user's "wet" range corresponds to HIGHER resistance
    // bool isWet = (humanRes_kohm >= wetRes_kohm);

    // if (isWet && !isNoFinger) {
    //   sweatyCounter++;
    // } else {
    //   sweatyCounter = 0;
    // }

    // // Act only when we have a couple of consistent readings
    // if (noFingerCounter >= needConsecutive) {
    //   // No fingers inserted — print nothing extra (or optionally show a note)
    //   // Serial.println("No fingers detected"); // optional
    // } else if (sweatyCounter >= needConsecutive) {
    //   Serial.println("Hands are sweaty!");
    //   gsr_flag=1;
    // } else {
    //   // considered dry — print nothing extra
    // }

    // Serial.println();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void gpsTask(void *parameter){
  while(1){
    while (GPS.available()) {
      gps.encode(GPS.read());
    }
    if (gps.location.isUpdated()) {
      Serial.print("Latitude  : ");
      Serial.println(gps.location.lat(), 6);
      latitude=gps.location.lat();
      Serial.print("Longitude : ");
      Serial.println(gps.location.lng(), 6);
      longitude=gps.location.lng();
      Serial.print("Date (DD/MM/YY): ");
      if (gps.date.isValid()) {
        Serial.printf("%02d/%02d/%02d\n", gps.date.day(), gps.date.month(), gps.date.year() % 100);
      } else 
      Serial.println("Invalid");
      day=gps.date.day();
      month=gps.date.month();
      year=gps.date.year()%100;
      Serial.print("Time (IST): ");
      if (gps.time.isValid()) {
        int hour = gps.time.hour() + 5;
        int minute = gps.time.minute() + 30;

        if (minute >= 60) { minute -= 60; hour++; }
        if (hour >= 24) hour -= 24;

        Serial.printf("%02d:%02d:%02d\n", hour, minute, gps.time.second());
        hr=hour;
        minu=minute;
      } else 
      Serial.println("Invalid");
      sec=gps.time.second();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void button(void *parameter){
  
  bool lastButtonState = HIGH;     // assuming pull-up
  bool buttonState;
  unsigned long pressedTime = 0;
  unsigned long releasedTime = 0;
  const unsigned long longPressTime = 1500;  // 1.5s long-press duration
  while(1){
    buttonState = digitalRead(buttonPin);

    // Button Pressed (HIGH → LOW)
    if (lastButtonState == HIGH && buttonState == LOW) {
      pressedTime = millis();   // record press moment
    }

    // Button Released (LOW → HIGH)
    else if (lastButtonState == LOW && buttonState == HIGH) {
      releasedTime = millis();
      unsigned long pressDuration = releasedTime - pressedTime;

      if (pressDuration < longPressTime) {
        // SHORT PRESS
        btnFlag = 1;
        Serial.println("Short Press → flag = 1");
      } 
      else {
        // LONG PRESS
        btnFlag = 0;
        Serial.println("Long Press → flag = 0");
      }
    }
    lastButtonState = buttonState;
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void deterrent(void *parameter) {
  while (1) {
    if (btnFlag == 1) {
      myservo.write(180);
      digitalWrite(buzzer, HIGH);
      telegramFlag = 1;
    } else if (btnFlag == 0) {
      myservo.write(0);
      digitalWrite(buzzer, LOW);
      telegramFlag = 0;
      confirmFlag = 1;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void heart(void *parameter){
  int listSize=sizeof(heartRate)/sizeof(heartRate[0]);
  while(1){
    for(int i=0;i<listSize;i++){
      Serial.print("Heart rate: ");
      Serial.println(heartRate[i]);
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
void telegram(void *parameter) {
  while (1) {
    if (telegramFlag == 1 && confirmFlag == 1) {
      
      // ----------------------------------------------
      // Replace Telegram → Display HTML Page
      // ----------------------------------------------

      htmlStatusPage =
        "<html><body>"
        "<h2>ESP32 ALERT STATUS</h2>"
        "<p><b>GPS :</b><br>"
        "Lat : " + String(latitude, 6) + "<br>"
        "Lon : " + String(longitude, 6) + "<br>"
        "Date : " + String(day) + "-" + String(month) + "-20" + String(year) + "<br>"
        "Time : " + String(hr) + ":" + String(minu) + ":" + String(sec) +
        "</p></body></html>";

      // Serial.println("HTML Updated ✔");
      confirmFlag = 0;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Initializing setup");
  vTaskDelay(100/portTICK_PERIOD_MS);
  GPS.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(buttonPin, INPUT_PULLUP);
  SerialBT.begin("ESP32");
  myservo.attach(servoPin); 
  myservo.write(0);
  pinMode(buzzer, OUTPUT); 

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      Serial.print(" WiFi Status: ");
      Serial.println(WiFi.status());
      vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/status", handleStatus);
  server.begin();
  Serial.println("Web Server Started → Visit: http://<ESP_IP>/status");

  //xTaskCreatePinnedToCore(gsr,"Gsr Sensor Reading",1024,NULL,2,NULL,app_cpu);
  //xTaskCreatePinnedToCore(gpsTask,"GPS data reading",2048,NULL,1,NULL,app_cpu);
  xTaskCreatePinnedToCore(button,"Button",1024,NULL,3,NULL,app_cpu);
  xTaskCreatePinnedToCore(deterrent,"Activate servo and vibrator",4096,NULL,4,NULL,app_cpu);
  //xTaskCreatePinnedToCore(heart,"Heart Rate",768,NULL,2,NULL,app_cpu);
  xTaskCreatePinnedToCore(telegram, "HTML Update", 4096, NULL, 1, NULL, app_cpu);

}

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
  if (SerialBT.available()) {
    incomingData = SerialBT.readStringUntil('\n'); // Read full line
    incomingData.trim();

    Serial.print("Raw Received: ");
    Serial.println(incomingData);

    // Split into 3 phone numbers
    int firstComma = incomingData.indexOf(',');
    int secondComma = incomingData.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      String ph1 = incomingData.substring(0, firstComma);
      String ph2 = incomingData.substring(firstComma + 1, secondComma);
      String ph3 = incomingData.substring(secondComma + 1);

      Serial.println("===== Phone Numbers Received =====");
      Serial.println("Phone 1: " + ph1);
      Serial.println("Phone 2: " + ph2);
      Serial.println("Phone 3: " + ph3);
      Serial.println("=================================");
    }
    else {
      Serial.println("Invalid data format!");
    }
  }
}
