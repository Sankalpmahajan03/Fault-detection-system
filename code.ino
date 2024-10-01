#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Wire.h>           
#include <LiquidCrystal_I2C.h>    
#include <SoftwareSerial.h>

// Insert your network credentials
#define WIFI_SSID "happy home 2G"
#define WIFI_PASSWORD "GBCLASSES"

String distance_Path;
String faulttype_Path;
bool signupOK = false;
LiquidCrystal_I2C lcd(0x27,16,2); 
const int Relay_Neutral = D5;
const int Relay_Green = D6;
const int Relay_Blue = D7;
const int Relay_Red = D8;

int sensor_value;
float current;
const int current_sensor_pin = A0;
const int Reset_Button_Pin = D4;
const int buzzer = D0;
int neutral_current;
int green_current;
int blue_current;
int red_current;
bool Neutral_Line = false;
bool Green_Line = false;
bool Blue_Line = false;
bool Red_Line = false;
bool initial = false;
bool circuit_breakers_trip = false;
bool curr_print = false;
String distance = " ";
int button_state = 0;
int fault_current = 0;
int send_distance = 0;
String send_faulttype = "nof";
double pickup_value;

void setup() {
  Serial.begin(9600);
  lcd.begin();      
  lcd.backlight();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  lcd.clear();
  lcd.print("  TRANSMISSION");
  lcd.setCursor(0, 1);
  lcd.print("      LINE");
  delay(1000);
  lcd.clear();
  lcd.print("FAULT DECTECTION");
  lcd.setCursor (0, 1);
  lcd.print("     SYSTEM");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  pinMode(Reset_Button_Pin, INPUT);
  pinMode(Relay_Neutral, OUTPUT);
  pinMode(Relay_Green, OUTPUT);
  pinMode(Relay_Blue, OUTPUT);
  pinMode(Relay_Red, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // Set PicKUp Current Value by reading normal Load current
  digitalWrite(Relay_Red, HIGH);
  delay(500);
  pickup_value = 2565;
  digitalWrite(Relay_Red, LOW);
}

void loop() {
  button_state = digitalRead(Reset_Button_Pin);
  circuit_breakers_trip = false;
  lines_current_read();
  faulty_lines();
  if (((Neutral_Line) && (Green_Line)) || ((Neutral_Line) && (Blue_Line)) || ((Neutral_Line) && (Red_Line)) || ((Green_Line) && (Blue_Line)) || ((Green_Line) && (Red_Line)) || ((Blue_Line) && (Red_Line))){
    circuit_breakers_trip = true;
    lines_current_read();
    while (button_state == HIGH){
      if ((Red_Line) && (Blue_Line) && (Green_Line)){
        Serial.println("Three Phase Fault Detected");
        send_faulttype = "tpf";
        curr_print = false;
        lcd.clear();
        lcd.print("  THREE PHASE");
        lcd.setCursor(0, 1);
        lcd.print("     FAULT");
        digitalWrite(buzzer, HIGH);
        delay(2000);
        digitalWrite(buzzer, LOW);
        print_3_phase_distances();
      }
      else if (((Red_Line) && (Blue_Line)) || ((Blue_Line) && (Green_Line)) || ((Green_Line) && (Red_Line))){
        curr_print = false;
        if (Neutral_Line){
          Serial.println("Double Line to Ground Fault Detected");
          send_faulttype = "dltg";
          lcd.clear();
          lcd.print(" DOUBLE LINE TO");
          lcd.setCursor(0, 1);
          lcd.print("  GROUND FAULT");
          digitalWrite(buzzer, HIGH);
          delay(2000);
          digitalWrite(buzzer, LOW);
          print_double_line_2_ground_distance();
          }
        else{
          Serial.println("Line to Line Fault Detected");
          send_faulttype = "ltl";
          lcd.clear();
          lcd.print("  LINE TO LINE");
          lcd.setCursor(0, 1);
          lcd.print("     FAULT");
          digitalWrite(buzzer, HIGH);
          delay(2000);
          digitalWrite(buzzer, LOW);
          print_line_2_line_distances();
        }
      }
      else if (((Green_Line) && (Neutral_Line)) || ((Red_Line) && (Neutral_Line)) || ((Blue_Line) && (Neutral_Line))){
        Serial.println("Single Line to Ground Fault Detected");
        send_faulttype = "sltg";
        curr_print = false;
        lcd.clear();
        lcd.print(" SINGLE LINE TO");
        lcd.setCursor(0, 1);
        lcd.print("  GROUND FAULT");
        digitalWrite(buzzer, HIGH);
        delay(2000);
        digitalWrite(buzzer, LOW);
        print_single_line_distances();
      }
      button_state = digitalRead(Reset_Button_Pin);
    }
  }
  else{
    if (!curr_print){
      circuit_breakers_trip = false;
      lcd.clear();
      lcd.print("    NO FAULT");
      Serial.println("No Fault Detected");
      print_loading_line();
      send_distance = 0;
      send_faulttype = "nof";
    }
    curr_print = true;
    lcd.clear();
    lcd.print("RED  GREEN  BLUE");
    print_live_currents();
  }
  reset_faulty_Lines();
}

void reset_faulty_Lines(){
  Neutral_Line = false;
  Green_Line = false;
  Blue_Line = false;
  Red_Line = false;
  initial = false;
  fault_current = 0;
}

void lines_current_read(){
  if (circuit_breakers_trip){
    digitalWrite(Relay_Neutral, LOW);
    digitalWrite(Relay_Green, LOW);
    digitalWrite(Relay_Blue, LOW);
    digitalWrite(Relay_Red, LOW);
  }
  else{
    digitalWrite(Relay_Neutral, HIGH);
    digitalWrite(Relay_Green, LOW);
    digitalWrite(Relay_Blue, LOW);
    digitalWrite(Relay_Red, LOW);
    neutral_current = read_current();
    Serial.print("Neutral Current (A): ");
    Serial.println(neutral_current);
    delay(300);

    digitalWrite(Relay_Neutral, LOW);
    digitalWrite(Relay_Green, HIGH);
    digitalWrite(Relay_Blue, LOW);
    digitalWrite(Relay_Red, LOW);
    green_current = read_current();
    Serial.print("Green Current (A): ");
    Serial.println(green_current);
    delay(300);

    digitalWrite(Relay_Neutral, LOW);
    digitalWrite(Relay_Green, LOW);
    digitalWrite(Relay_Blue, HIGH);
    digitalWrite(Relay_Red, LOW);
    blue_current = read_current();
    Serial.print("Blue Current (A): ");
    Serial.println(blue_current);
    delay(300);

    digitalWrite(Relay_Neutral, LOW);
    digitalWrite(Relay_Green, LOW);
    digitalWrite(Relay_Blue, LOW);
    digitalWrite(Relay_Red, HIGH);
    red_current = read_current();
    Serial.print("Red Current (A): ");
    Serial.println(red_current);
    delay(300);
  }
}

int read_current(){
  delay(30);
  float sensitivity = 0.100;
  float vRef = 1000.0;  // ESP8266 ADC works from 0-1V (1000mV)

  // Offset voltage (0.5V when no current flows after scaling via voltage divider)
  float vZero = vRef / 2.0;
  // Read the analog sensor value
  int sensorValue = analogRead(current_sensor_pin);

  // Convert the analog value to voltage (millivolts)
  float voltage = (sensorValue / 1023.0) * vRef;

  // Subtract the offset to get the voltage related to the current
  float voltageOffset = voltage - vZero;

  // Convert the voltage to current (in Amps)
  float current = voltageOffset / sensitivity;
  return current;
}

void faulty_lines(){
  if (neutral_current <= (pickup_value-15)){
    Neutral_Line = true;
  }
  if (green_current <= pickup_value){
    Green_Line = true;
    if (!initial){
      fault_current = green_current;
      fault_distance();
      initial = true;
    }
  }
  if (blue_current <= pickup_value){
    Blue_Line = true;
    if (!initial){
      fault_current = blue_current;
      fault_distance();
      initial = true;
    }
  }
  if (red_current <= pickup_value){
    Red_Line = true;
    if (!initial){
      fault_current = red_current;
      fault_distance();
      initial = true;
    }
  }
}

void print_3_phase_distances(){
  Serial.println("Red, Green and Blue Shorted");
  lcd.clear();
  lcd.print("RED   --->");
  lcd.setCursor(13, 0);
  lcd.print(distance);
  lcd.setCursor(0, 1);
  lcd.print("GREEN   -->");
  lcd.setCursor(13, 1);
  lcd.print(distance);
  delay(2000);
  lcd.clear();
  lcd.print("BLUE    --->");
  lcd.setCursor(13, 0);
  lcd.print(distance);
}

void print_double_line_2_ground_distance(){
  lcd.clear();
  lcd.print("GREEN   -->");
  lcd.setCursor(13, 0);
  lcd.print(distance);
  delay(2000);
  lcd.clear();
  lcd.print("RED     --->");
  lcd.setCursor(13, 0);
  lcd.print(distance);
}

void print_line_2_line_distances(){
  lcd.clear();
  lcd.print("RED   --->");
  lcd.setCursor(13, 0);
  lcd.print(distance);
  lcd.setCursor(0, 1);
  lcd.print("GREEN   -->");
  lcd.setCursor(13, 1);
  lcd.print(distance);
}

void print_single_line_distances(){
  lcd.clear();
  lcd.print("RED   --->");
  lcd.setCursor(13, 0);
  lcd.print(distance);
}

void fault_distance(){
////  if(fault_current < pickup_value)
//if(fault_current > 390)
//{
//    send_distance = 0;
//    send_faulttype = "nof";
//  }
////  else if (fault_current < (pickup_value - 2))
//  else if (fault_current > 387){
//    distance = "8KM";
//    send_distance = 8;
//  }
//  else if (fault_current < (pickup_value -4)){
//    distance = "6KM";
//    send_distance = 6;
//  }
   if (fault_current >=2565){
    distance = "4KM";
    send_distance = 4;
  }
  else{
    distance = "2KM";
    send_distance = 2;
  }  
}


void print_live_currents(){
  lcd.clear();
  lcd.print("Neutral Current: ");
  lcd.setCursor(13, 0);
  lcd.print(neutral_current);
  lcd.setCursor(0, 1);
  lcd.print("Green Current: ");
  lcd.setCursor(13, 1);
  lcd.print(green_current);
}


void print_loading_line(){
  lcd.setCursor(0, 1);
  delay(187);
  lcd.print("-");
  delay(187);
  lcd.print("--");
  delay(187);
  lcd.print("---");
  delay(187);
  lcd.print("----");
  delay(187);
  lcd.print("-----");
  delay(187);
  lcd.print("------");
  delay(187);
  lcd.print("-------");
  delay(187);
  lcd.print("--------");
  delay(187);
  lcd.print("---------");
  delay(187);
  lcd.print("----------");
  delay(187);
  lcd.print("-----------");
  delay(187);
  lcd.print("------------");
  delay(187);
  lcd.print("-------------");
  delay(187);
  lcd.print("--------------");
  delay(187);
  lcd.print("---------------");
  delay(187);
  lcd.print("----------------");
  lcd.clear();
  lcd.print(" START  CURRENT");
  lcd.setCursor(0, 1);
  lcd.print("   MONITORING");
  delay(2000);
}
