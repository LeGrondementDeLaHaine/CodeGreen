// -------------------------ERa setting--------------------
#define ERA_LOCATION_VN
#define ERA_AUTH_TOKEN "c360f8c0-f9f8-4277-b9d5-69d9237f59fa"
#define ERA_DEBUG
#define DEBUG_SERIAL sereial
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <ESP32Time.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SimpleKalmanFilter.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <ERa.hpp>
#include <Widgets/ERaWidgets.hpp>
#include <DHT.h>
#include <Arduino.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define motor 23
#define A1 32
#define A2 33
#define B1 36
#define B2 34
#define DHT_PIN 4
#define INTERRUPT_PIN 27
#define DHT_TYPE DHT22
#define sub 25
#define sab 26
#define LED_TB 2

ESP32Time rtc(0);


ERA_CONNECTED() {
  ERA_LOG("ERa", "ERa connected!");
}
ERA_DISCONNECTED() {
  ERA_LOG("ERa", "ERa disconnected!");
}
ERaTimer timer;
// -------------------------Weather API--------------------------
String URL = "http://api.openweathermap.org/data/2.5/weather?";
String ApiKey = "d97d901a3844e5232b02f0caa11f55e1";
String city = "Ho Chi Minh";
unsigned long lastTime = 0;
unsigned long timerDelay = 600000;
// Replace with your location Credentials
String lat = "10.871634828225647";
String lon = "106.77983479181722";
bool firstTime = true;
volatile bool motorTriggeredByButton = false;
bool setting1 = HIGH;
bool setting2 = HIGH;
bool menu = false;
int menusel = 0;
bool motorState2 = false;
bool timeon = false;
bool timeoff = false;
int water = 0;
unsigned long checkt = 0;
const char ssid[] = "A12.02";
const char password[] = "A120201082024";
bool checked = false;
// Define NTP Server address
const char* ntpServer = "vn.pool.ntp.org";  //Modify as per your country
const long gmtOffset_sec = 6 * 60 * 60;     // Offset from UTC (in seconds) - GMT +7
const int daylightOffset_sec = 3600;        // Daylight offset (in seconds)

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);
String msg;
String msg2;
String msg3;
String msg4;

float h;
float t;
// Setting time on
int relayOnHour = 0;     // Relay ON
int relayOnMinute = 0;   // Relay ON
int relayOffHour = 0;    // Relay OFF
int relayOffMinute = 0;  // Relay OFF

int Hour;
int Minus;
int Sec;
int mon;
int day;
int year;
int thu;
int leddimming ;
int fandimming ;
volatile bool motorState = false;
volatile bool buttonnow = HIGH;
volatile bool buttonnow2 = HIGH;
bool fanstate = false;
int ledstate = 0;
bool button1 = false;
bool button2 = false;
bool motorTriggerByTimer = false;
unsigned long motortimer = 0;
unsigned long autotimer = 0;
unsigned long timer1 = 0;
unsigned long buttontime = 0;
unsigned long buttontime2 = 0;
String weatherTiengviet;
int16_t dk1;
int16_t dk2;
int level;
DHT dht(DHT_PIN, DHT_TYPE);

// Loc nhieu
float x_kalman1;
float x_kalman2; // kalman filled value
float e;  // Error
SimpleKalmanFilter bo_loc(1, 1, 0.01);

void setup() {
  SPIFFS.begin();
  Serial.begin(115200);
  dht.begin();
  oled.begin();
  oled.clearDisplay();
  pinMode(motor, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(sub, INPUT_PULLUP);
  pinMode(sab, INPUT_PULLUP);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(LED_TB, OUTPUT);
  Serial.println("Connected to WiFi");
  ERa.begin(ssid, password);
  // Start NTP time sync
  timeClient.begin();
  timeClient.update();
  /*---------set internal RTC with NTP---------------*/
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  rtc.setTime(0, 0, 0, 1, 1, 2021);
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
  }
  ERa.addInterval(1000L, timerEvent);
  xTaskCreatePinnedToCore(
    logic,
    "logic",
    5000,
    NULL,
    1,
    NULL,
    0);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), motorbutton, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sub), isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(sab), isr2, FALLING);
  weather();
}

ICACHE_RAM_ATTR void isr1() {
  if (!button1) {
    buttontime = millis();
    button1 = true;
    buttonnow = digitalRead(sub);
  }
}

ICACHE_RAM_ATTR void isr2() {
  if (!button2) {
    buttontime2 = millis();
    button2 = true;
    buttonnow2 = digitalRead(sab);
  }
}

ICACHE_RAM_ATTR void motorbutton() {
  if (!motorState) {
    motorState = true;
    motortimer = millis();
    motorTriggeredByButton = true;
    Serial.println("motor hoat dong");
    timeon = false;
    timeoff = false;
    digitalWrite(motor, HIGH);
  }
}

void loop() {
  ERa.run();
}

void timerEvent() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  ERa.virtualWrite(V0, h);
  ERa.virtualWrite(V1, t);
  ERa.virtualWrite(V5, water);
  ERA_LOG("Timer", "Uptime: %d", ERaMillis() / 1000L);
}

ERA_WRITE(V3){
  int analogB1 = param.getInt();
  Serial.println(analogB1);
  leddimming = analogB1;
  Serial.println("nhan gia tri tu V3" + leddimming);
}
ERA_WRITE(V4){
  int analogA1 = param.getInt();
  Serial.println(analogA1);
  fandimming = analogA1;
  Serial.println("nhan gia tri tu V4" + fandimming);
}

ERA_WRITE(V2) { // hen gio thong qua ERa
  if (!param.isString()) {
    Serial.println("sai cau hinh");
    return;
  }
  ERaJson me = param.toJSON();
  if (me == nullptr) {
    return;
  }
  // me = {"x":0, "y":0, "z":0, "m":0, "msg":1} 1 bat / 2 tat
  int x = me["x"].getInt();
  int y = me["y"].getInt();
  int z = me["z"].getInt();
  int m = me["m"].getInt();
  int msg = me["msg"].getInt();
  if (msg == 1) {
    digitalWrite(LED_TB, HIGH);
    motorState = true;
    motortimer = millis();
    motorTriggeredByButton = true;
    digitalWrite(motor, HIGH);
  } else if (msg == 2) {
    digitalWrite(LED_TB, LOW);
    digitalWrite(motor, LOW);
  }
  relayOnHour = x;
  relayOffHour = y;
  relayOnMinute = z;
  relayOffMinute = m;
}

void sensor() {
  dk1 = analogRead(35);
  dk2 = analogRead(18);
  randomSeed(millis());
  e = (float)random(-20, 20);  // sai so dao dong 20
  dk1 = dk1 + e;
  dk2 = dk2 + e;
  x_kalman1 = bo_loc.updateEstimate(dk1); 
  x_kalman2 = bo_loc.updateEstimate(dk2); // do am dat sau loc
  h = dht.readHumidity();
  t = dht.readTemperature();
  // -------------------- Hen gio bat tat chan relay-------------------------//
  Sec = rtc.getSecond();
  Hour = rtc.getHour(true);
  Minus = rtc.getMinute();
  mon = rtc.getMonth();
  day = rtc.getDay();
  year = rtc.getYear();
  thu = rtc.getDayofWeek() + 1;
  if (!timeon) {
    if (Hour == relayOnHour && Minus == relayOnMinute) {
      digitalWrite(motor, HIGH);
      motorTriggerByTimer = true;
      timeon = true;
      Serial.println("Relay ON");
    }
  }
  if (!timeoff) {
    if (Hour == relayOffHour && Minus == relayOffMinute) {
      motorTriggerByTimer = false;
      digitalWrite(motor, LOW);
      timeoff = true;
      Serial.println("Relay OFF");
    }
  }
  if (x_kalman2 > 7) {
    Serial.println("Water Level: Empty");
    water = 0;
  } else if (x_kalman2 > 4 && x_kalman2 <= 7) {
    Serial.println("Water Level: Low");
    water = 1;
  } else if (x_kalman2 > 1 && x_kalman2 <= 4) {
    Serial.println("Water Level: Medium");
    water = 2;
  } else if (x_kalman2 < 1) {
    Serial.println("Water Level: High");
    water = 3;
  } else {
    Serial.println("Water Level: error");
  }
  msg = x_kalman1 < 3000 ? "AM" : x_kalman1 > 3800 ? "KHO"
                                : "Vua";
  msg2 = x_kalman2 > 4 ? "Thap": x_kalman2 < 1 ? "Cao"
                                : "Vua";
  //  msg3 = t < temperature ? "Thap hon" : " Cao hon" ;
  if (motorTriggeredByButton && (millis() - motortimer >= 5000)) {
    motorState = false;
    motorTriggeredByButton = false;
    Serial.println("May bom ngung hoat dong");
    digitalWrite(motor, LOW);
  }
  if (x_kalman2 < 4 && x_kalman1 > 3800) {
    Serial.println("motor dang hoat dong");
    motorTriggeredByButton = true;
    digitalWrite(motor, HIGH);
  }
  if (fanstate) {
    if (h < 70) {
      Serial.println("Quat xoay ngc chieu");
      analogWrite(A2, 0);
      if(fandimming == true){
        analogWrite(A1, fandimming);
      } else {
      analogWrite(A1, 180);
      }
    } else if (h > 70) {
      Serial.println("Quat xoay chieu dh");
       analogWrite(A1, 0);
       if(fandimming == true){
        analogWrite(A1, fandimming);
      } else {
      analogWrite(A1, 180);
      }
    }
  } else if (!fanstate) {
    Serial.println("Quat ngung hoat dong");
     analogWrite(A1, 0);
     analogWrite(A2, 0);
  }
  if(leddimming == true){
    if(0 < leddimming < 100) ledstate = 1;
    else if( 101 < leddimming < 200) ledstate = 2;
    else if(201 < leddimming < 255) ledstate = 3;
  }
  if (ledstate == true ) {  // Led control
    if (ledstate == 1){
      Serial.print("led bat muc 1");
      analogWrite(B1, 100);
      analogWrite(B2, 0);
    } else if (ledstate == 2) {
      Serial.print("led bat muc 2");
      analogWrite(B1, 160);
      analogWrite(B2, 0);
    } else if (ledstate == 3 ) {
      Serial.print("led bat muc 3");
      analogWrite(B1, 240);
      analogWrite(B2, 0);
    }
  } else if (ledstate == 0) {
    Serial.println("led tat");
    analogWrite(B1, 0);
    analogWrite(B2, 0);
  }
 // oled.clearDisplay();
  Serial.print("water value: ");
  Serial.println(x_kalman2);
  Serial.println(msg);
  Serial.println(x_kalman1);
  oled.setTextSize(1);               // Normal 1:1 pixel scale
  oled.setTextColor(SSD1306_WHITE);  // Draw white text
  oled.setCursor(1, 0);
  oled.print("T:");
  oled.println(thu);
  oled.print(h);
  oled.println(" %");
  oled.print(t);
  oled.println(" C");
  Serial.print(h);
  Serial.println("%");
  Serial.print(t);
  Serial.println(" C");
  oled.drawLine(55, 8, 55, 26, SSD1306_WHITE);
  oled.drawLine(0, 26, 128, 26, SSD1306_WHITE);
  oled.setCursor(32, 27);
  oled.print(city);
  oled.setCursor(64, 16);
  oled.print(Hour);
  oled.print(":");
  oled.print(Minus);
  oled.print(":");
  oled.print(Sec);
  oled.setCursor(64, 8);
  oled.print(day);
  oled.print("/");
  oled.print(mon + 1);
  oled.print("/");
  oled.print(year);
  oled.setCursor(32, 0);
  oled.println(weatherTiengviet);
  oled.display();
}

//------------------ thong bao thoi tiet --------------------------
void weather() {
  HTTPClient http;
  String weatherURL = URL + "lat=" + lat + "&lon=" + lon + "&appid=" + ApiKey;
  http.begin(weatherURL);
  int httpResponseCode = http.GET();
  // if (httpResponseCode > 0) {
  String payload = http.getString();
  Serial.println("HTTP Response code: " + String(httpResponseCode));
  Serial.println("Received payload: " + payload);
  // Use ArduinoJson to parse the JSON
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);
  // if (!error) {
  float temperature = doc["main"]["temp"];
  float humidity = doc["main"]["humidity"];
  String weatherDescription = doc["weather"][0]["description"];
  String place = doc["name"];
  // Convert temperature from Kelvin to Celsius
  temperature -= 273.15;
  // Print weather info to Serial
  Serial.println("Temperature: " + String(temperature) + " °C");
  Serial.println("Humidity: " + String(humidity) + " %");
  if (weatherDescription == "rain") {
    weatherTiengviet = "Mua";
  } else if (weatherDescription == "broken clouds") {
    weatherTiengviet = "Sap mua";
  } else if (weatherDescription == "scattered clouds") {
    weatherTiengviet = "Am u";
  } else if (weatherDescription == "few clouds") {
    weatherTiengviet = "Nang dep";
  } else if (weatherDescription == "clear sky") {
    weatherTiengviet = "Quang dang";
  } else if (weatherDescription == "thunderstorm") {
    weatherTiengviet = "Mua bao";
  } else { weatherTiengviet = "null";}
  city = place;
  checked = true;
  //}
  //}
}

void lcdmenu(int sel) {
  oled.clearDisplay();
  switch (sel) {
    case 1:
    oled.setTextSize(1);               // Normal 1:1 pixel scale
    oled.setTextColor(SSD1306_WHITE);  // Draw white text
      oled.setCursor(0, 0);
      oled.print("TG bat: >");
      oled.print(relayOnHour);
      oled.print(" ");
      oled.print(relayOnMinute);
      oled.setCursor(0, 8);
      oled.print("TG tat: ");
      oled.print(relayOffHour);
      oled.print(" ");
      oled.print(relayOffMinute);
      oled.setCursor(0, 16);
      oled.print("Muc nuoc:");
      oled.println(msg2);
      oled.print("Do am: ");
      oled.print(msg);
      oled.display();
      break;
    case 2:
      oled.setTextSize(1);               // Normal 1:1 pixel scale
      oled.setTextColor(SSD1306_WHITE);  // Draw white text
      oled.setCursor(0, 0);
      oled.print("TG bat: ");
      oled.print(relayOnHour);
      oled.print(" >");
      oled.print(relayOnMinute);
      oled.setCursor(0, 8);
      oled.print("TG tat: ");
      oled.print(relayOffHour);
      oled.print(" ");
      oled.print(relayOffMinute);
      oled.setCursor(0, 16);
      oled.print("Muc nuoc:");
      oled.println(msg2);
      oled.print("Do am: ");
      oled.print(msg);
      oled.display();
      break;
    case 3:
      oled.setTextSize(1);               // Normal 1:1 pixel scale
      oled.setTextColor(SSD1306_WHITE);  // Draw white text
      oled.setCursor(0, 0);
      oled.print("TG bat: ");
      oled.print(relayOnHour);
      oled.print("   ");
      oled.print(relayOnMinute);
      oled.setCursor(0, 8);
      oled.print("TG tat: >");
      oled.print(relayOffHour);
      oled.print("   ");
      oled.print(relayOffMinute);
      oled.setCursor(0, 18);
      oled.print("Muc nuoc:");
      oled.println(msg2);
      oled.print("Do am: ");
      oled.print(msg);
      oled.display();
      break;
    case 4:
      oled.setTextSize(1);               // Normal 1:1 pixel scale
      oled.setTextColor(SSD1306_WHITE);  // Draw white text
      oled.setCursor(0, 0);
      oled.print("TG bat: ");
      oled.print(relayOnHour);
      oled.print("   ");
      oled.print(relayOnMinute);
      oled.setCursor(0, 8);
      oled.print("TG tat: ");
      oled.print(relayOffHour);
      oled.print(" >");
      oled.print(relayOffMinute);
      oled.setCursor(0, 16);
      oled.print("Muc nuoc:");
      oled.println(msg2);
      oled.print("Do am: ");
      oled.print(msg);
      oled.display();
      break;
    case 5:
      oled.setTextSize(1);               // Normal 1:1 pixel scale
      oled.setTextColor(SSD1306_WHITE);  // Draw white text
      oled.setCursor(0, 0);
      oled.print(" FAN: >");
      oled.print(msg3);
      oled.setCursor(0, 8);
      oled.print(" LED: ");
      oled.print(msg4);
      oled.setCursor(0, 16);
      oled.print("Muc nuoc:");
      oled.println(msg2);
      oled.print("Do am: ");
      oled.print(msg);
      oled.display();
      break;
    case 6:
      oled.setTextSize(1);               // Normal 1:1 pixel scale
      oled.setTextColor(SSD1306_WHITE);  // Draw white text
      oled.setCursor(0, 0);
      oled.print(" FAN ");
      oled.print(msg3);
      oled.setCursor(0, 8);
      oled.print(" LED >");
      oled.print(msg4);
      oled.setCursor(0, 16);
      oled.print("Muc nuoc:");
      oled.println(msg2);
      oled.print("Do am: ");
      oled.print(msg);
      oled.display();
      break;
  }
  delay(50);
}

void logic(void* pvParameters) {
  while (1) {
    unsigned long debouce1 = 200;
    unsigned long debouce2 = 200;
    setting1 = HIGH;
    setting2 = HIGH;
    oled.clearDisplay();
//=============================================================
 if (checked == false) {
    weather();
  } else if (checked == true && millis() - checkt >= 60 * 60 * 5) {
    checkt = millis();
    // cap nhat thoi tiet moi 5p
    checked = false;
  }
msg3 = fanstate == true ? "BAT" : "TAT";
msg4 = ledstate == 1 ? "1" : ledstate == 2 ? "2"
                           : ledstate == 3 ? "3"
                           : "Tat";
    if (button1 == true) {
      if (millis() - buttontime > debouce1) {
        setting1 = buttonnow;
        button1 = false;
      } 
    }
    if (button2 == true) {
      if (millis() - buttontime2 > debouce2) {
        setting2 = buttonnow2;
        button2 = false;
      }
    }
    if (setting1 == LOW) {
      Serial.println("nut 1");
      menu = true;
      oled.clearDisplay();
    }
    if (menusel > 6) {
      menusel = 0;
      menu = false;
    }
    if (setting2 == LOW) {
        Serial.println("nut 2");
      if (menusel == 1) {
        oled.clearDisplay();
        relayOnHour++;
        if (relayOnHour > 23) relayOnHour = 0;
      } else if (menusel == 2) {
        oled.clearDisplay();
        relayOnMinute += 5;
        if (relayOnMinute > 60) relayOnMinute = 0;
      } else if (menusel == 3) {
        oled.clearDisplay();
        relayOffHour++;
        if (relayOffHour > 23) relayOffHour = 0;
      } else if (menusel == 4) {
        oled.clearDisplay();
        relayOffMinute += 5;
        if (relayOffMinute > 60) relayOffMinute = 0;
      } else if (menusel == 5) {
        oled.clearDisplay();
        fanstate = !fanstate;
      } else if (menusel == 6) {
        oled.clearDisplay();
        ledstate++;
        if (ledstate > 3) ledstate = 0;
      }
    }
    // ---------------------------- vao menu -------------------------
    if (menu == true) {
      lcdmenu(menusel);
      if (setting1 == LOW) {
        oled.clearDisplay();
        menusel++;  // thoat setting
      }
    } else {
      sensor();
    }
    Serial.println(relayOnHour);
    Serial.println(relayOnMinute);
    Serial.println(msg3);
    Serial.println(msg4);
    Serial.println(leddimming);
    Serial.println(fandimming);
    // Serial.println(xPortGetFreeHeapSize());
    delay(100);
  }
}
