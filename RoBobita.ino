//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤( Motors pins )¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

int m1p1 = 11;
int m1p2 = 10;
int m2p1 = 9;
int m2p2 = 8;

//##############( Ultrasonic sensor pins )##################

int trigger = 6;
int echo = 7;

const int maxDistance = 20;

//// ->->->->->->->->->( Line Tracker )->->->->->->->->->

const int leftSensorPin = A0;
const int middleSensorPin = A1;
const int rightSensorPin = A2;


//◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙( LED Matrix )◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙

#include <LedControl.h>

int CLK = 4;
int CS = 3;
int DIN = 2;
const int TEMP_THRESHOLD = 28;   // Temperature threshold

LedControl Display = LedControl(DIN, CLK, CS, 0);



//◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙( LED Matrix patterns )◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙

//small heart
const uint8_t heart[][8] = { { 0b00011110, 0b00111111, 0b01111111, 0b11111110, 0b11111110, 0b01111111, 0b00111111, 0b00011110 } };
const int heart_LEN = sizeof(heart) / 8;
//big heart
const uint8_t heart1[][8] = { { 0b00000000, 0b00011110, 0b00111110, 0b01111100, 0b01111110, 0b00111110, 0b00011110, 0b00000000 } };
const int heart1_LEN = sizeof(heart1) / 8;
// GO
const uint8_t go[][8] = { { 0b01110000, 0b10001000, 0b10001000, 0b01110000, 0b01100000, 0b10101000, 0b10001000, 0b01110000 } };
const int go_LEN = sizeof(go) / 8;
// 3
const uint8_t trei[][8] = { { 0b00000000, 0b00000000, 0b01101100, 0b10010010, 0b10010010, 0b01000100, 0b00000000, 0b000000000 } };
const int trei_LEN = sizeof(trei) / 8;
// 2
const uint8_t doi[][8] = { { 0b00000000, 0b00000000, 0b10001100, 0b10010010, 0b10100010, 0b11000100, 0b00000000, 0b00000000 } };
const int doi_LEN = sizeof(doi) / 8;
// 1
const uint8_t unu[][8] = { { 0b00000000, 0b00000000, 0b10000000, 0b11111111, 0b10000010, 0b00000100, 0b00000000, 0b00000000 } };
const int unuS_LEN = sizeof(unu) / 8;
// HI :)
const uint8_t HI[][8] = { { 0b00000000, 0b00000100, 0b11101001, 0b00001000, 0b11101000, 0b01001001, 0b11100100, 0b00000000 } };
const int HI_LEN = sizeof(HI) / 8;
// Backward arrow
const uint8_t BACK[][8] = { { 0b00000000, 0b00100000, 0b01000000, 0b11111100, 0b01000000, 0b00100000, 0b00000000, 0b00000000 } };
const int BACK_LEN = sizeof(BACK) / 8;
// Forward arrow
const uint8_t FRONT[][8] = { { 0b00000000, 0b00000100, 0b00000010, 0b00111111, 0b00000010, 0b00000100, 0b00000000, 0b00000000 } };
const int FRONT_LEN = sizeof(FRONT) / 8;
// Right arrow
const uint8_t RIGHT[][8] = { { 0b00010000, 0b00111000, 0b01010100, 0b00010000, 0b00010000, 0b00010000, 0b00000000, 0b00000000 } };
const int RIGHT_LEN = sizeof(RIGHT) / 8;
// Left arrow
const uint8_t LEFT[][8] = { { 0b00000000, 0b00000000, 0b00001000, 0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000 } };
const int LEFT_LEN = sizeof(LEFT) / 8;
// STOP
const uint8_t STP[][8] = { { 0b00111100, 0b01100110, 0b11100111, 0b11100111, 0b11100111, 0b11100111, 0b01100110, 0b00111100 } };
const int STP_LEN = sizeof(STP) / 8;
//HOT
const uint8_t HOT[][8] = {{0b00100000, 0b11100000, 0b00111100, 0b00010100, 0b00011100, 0b00000111, 0b00000010, 0b00000111}};
const int HOT_LEN = sizeof(HOT) / 8;
//COLD
const uint8_t COLD[][8] = {{0b01000000, 0b10100000, 0b11100000, 0b01001110, 0b01111010, 0b00001110, 0b00000101, 0b00000111}};
const int COLD_LEN = sizeof(COLD) / 8;
//OK
const uint8_t OK[][8] = { { 0b01000010, 0b00100100, 0b00011000, 0b01111110, 0b00111100, 0b01000010, 0b01000010, 0b00111100 } };
const int OK_LEN = sizeof(OK)/8;

// -------------------------(IR Receiver)-------------------------

#include <IRremote.h>
#define IR_RECEIVE_PIN A3

#define IRF 6
#define IRB 68
#define IRL 71
#define IRR 64
#define IRSTP 7
#define IROA 29
#define IRLF 31
#define IRSLH 87

int command = 0;

//#####################( Liquid Crystal Display i2c )#####################

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

//⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝( Humidity/Temperature sensor )⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝

#include <DHT.h>
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
int temp=0;

//*****************************(8 LED strip)*****************************

#include <Adafruit_NeoPixel.h>
#define PIN 12
#define N_LEDS 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

//########### Buton #############

const int buttonPin = A4;
/*
int buttonState = LOW; 
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;  
*/
int index = 0;
bool isActive = false;

//Functions prototype
void obsAvoid();
void lineFollower();
void sensorLH();
void RemoteControl();

void setup() {

// Initializare comunicare serial
  Serial.begin(9600);

// Initializare buton
  pinMode(buttonPin, INPUT_PULLUP);

  //►►►►►►►►►►►►►►►► Motors ◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄

  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT);

  //►►►►►►►►►►►►►►►► Ultrasonic sensor ◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  //►►►►►►►►►►►►►►►► Line tracker ◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄

  pinMode(leftSensorPin, INPUT);
  pinMode(middleSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  //►►►►►►►►►►►►►►►► Liquid Crystal Display i2c ◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄

  lcd.begin(16, 2);
  dht.begin();
  lcd.clear();
  lcd.init();
  lcd.backlight();

  //►►►►►►►►►►►►►►►► Initializare matrice 8x8 + welcome ◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄◄

  Display.setIntensity(0, 15);
  Display.shutdown(0, false);
  Display.clearDisplay(0);

  welcome();

// Initializare receiver IR
  IrReceiver.begin(IR_RECEIVE_PIN);
    
  //pixels.begin(); 
}

void loop() {

//obsAvoid();
//lineFollower();
decodeIR();
RemoteControl();
//sensorLH();
}

// --------------------------------------------------
void RemoteControl() {  
//Functie control IR
  if (command == IRF) {
    moveForward();
    printByte(FRONT);
    lcd.setCursor(0, 0);
    lcd.print("   Controlat    ");
    lcd.setCursor(0, 1);
    lcd.print("prin telecomanda");
    delay(10);
  } else if (command == IRB) {
    moveBackward();
    printByte(BACK);
    lcd.setCursor(0, 0);
    lcd.print("   Controlat    ");
    lcd.setCursor(0, 1);
    lcd.print("prin telecomanda");
    delay(10);
  } else if (command == IRL) {
    turnLeft();
    printByte(LEFT);
    lcd.setCursor(0, 0);
    lcd.print("   Controlat    ");
    lcd.setCursor(0, 1);
    lcd.print("prin telecomanda");
    delay(10);
  } else if (command == IRR) {
    turnRight();
    printByte(RIGHT);
    lcd.setCursor(0, 0);
    lcd.print("   Controlat    ");
    lcd.setCursor(0, 1);
    lcd.print("prin telecomanda");

    delay(10);
  } else if(command == IRSTP){
    stop();
    lcd.setCursor(0, 0);
    lcd.print("   Controlat    ");
    lcd.setCursor(0, 1);
    lcd.print("prin telecomanda");
    printByte(STP);
  } else if(command == IROA){
    obsAvoid();
    lcd.setCursor(0, 0);
    lcd.print("    Ocolitor    ");
    lcd.setCursor(0, 1);
    lcd.print("   obstacole    ");
  }else if(command == IRLF){
    lineFollower();
    lcd.setCursor(0, 0);
    lcd.print("   Urmaritor    ");
    lcd.setCursor(0, 1);
    lcd.print("     linie      ");
  }else if(command == IRSLH){
    stop();
    sensorLH();
  }
}
// --------------------------------------------------


//#########################################################################
// Functii Control motorase

void moveForward() {
  // Inainte
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, HIGH);
  digitalWrite(m2p1, HIGH);
  digitalWrite(m2p2, LOW);
}

void moveBackward() {
  // Inapoi
  digitalWrite(m1p1, HIGH);
  digitalWrite(m1p2, LOW);
  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, HIGH);
}

void stop() {
  // STOP
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, LOW);
  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, LOW);
}

void turnRight() {
  // Dreapta complet
  digitalWrite(m1p1, HIGH);
  digitalWrite(m1p2, LOW);
  digitalWrite(m2p1, HIGH);
  digitalWrite(m2p2, LOW);
}

void turnERight() {
  // Dreapta partial
  digitalWrite(m1p1, HIGH);
  digitalWrite(m1p2, LOW);
  digitalWrite(m2p1, HIGH);
  digitalWrite(m2p2, LOW);
  delay(50);
}

void turnLeft() {
  // Stanga complet
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, HIGH);
  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, HIGH);

}

void turnELeft() {
  // Stanga partial
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, HIGH);
  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, HIGH);
  delay(50);
}

//#########################################################################

void obsAvoid() {

   //STRIPRGB(0xFF0F00);

   // Masurarea distantei pana la obstacol
  long duration, distance;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration * 0.0343) / 2; 

  //Verifica daca s-a incalcat limita
  if (distance <= maxDistance) {
    //Opreste
    stop();
    printByte(STP);
    delay(200); // Asteapta 200 milisecunde

    // Intoarce dreapta
    turnRight();
    printByte(RIGHT);
    delay(500); // Asteapta 0.5 secunde pentru rotatie

    // Inainte
    moveForward();
    printByte(FRONT);
    delay(500); // Asteapta 0.5 secunde
  } else {
    // Nu s-a detectat obstacol, mergi inainte
    moveForward();
    printByte(FRONT);
  }
}

//#########################################################################

void lineFollower() {
  //Functie pentru urmaritorul de linie

  int leftSensorValue = digitalRead(leftSensorPin);
  int middleSensorValue = digitalRead(middleSensorPin);
  int rightSensorValue = digitalRead(rightSensorPin);
  
  // Daca toti sensorii sunt activati mergi inainte
  if (leftSensorValue == LOW && middleSensorValue == LOW && rightSensorValue == LOW) {
    moveForward();
    printByte(FRONT);
  }
  // Daca senzorul din stanga este activat mergi stanga
  else if (leftSensorValue == HIGH && middleSensorValue == LOW && rightSensorValue == LOW) {
    turnLeft();
    printByte(LEFT);
  }
  // Daca senzorul din dreapta este activat mergi dreapta
  else if (leftSensorValue == LOW && middleSensorValue == LOW && rightSensorValue == HIGH) {
    turnRight();
    printByte(RIGHT);
  }
  // Daca senzorul central este activat mergi inainte
  else if (leftSensorValue == LOW && middleSensorValue == HIGH && rightSensorValue == LOW) {
    moveForward();
    printByte(FRONT);
  }
  // Daca senzorul din stanga si centru sunt activati mergi stanga
  else if (leftSensorValue == HIGH && middleSensorValue == HIGH && rightSensorValue == LOW) {
    turnLeft();
    printByte(LEFT);
  }
  // Daca senzorul din dreapta si centru sunt activati mergi dreapta
  else if (leftSensorValue == LOW && middleSensorValue == HIGH && rightSensorValue == HIGH) {
    turnRight();
    printByte(RIGHT);
  }
  // Daca nu este nici un senzor activat roteste dreapta si avanseaza putin in cautarea liniei
  else {
    turnRight();
    printByte(RIGHT);
    delay(5);
    moveForward();
     delay(50);    
  }
}

//◙◙◙◙◙◙◙◙◙◙ LED Matrix Display function ◙◙◙◙◙◙◙◙◙◙

void printByte(byte character[][8]) {
  for (int i = 0; i < 8; i++) {
    byte row = 0;
    for (int j = 0; j < 8; j++) {
      row |= (character[0][i] >> (7 - j) & 1) << j;
    }
    Display.setRow(0, i, row);
  }
}
//◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙◙

void welcome() {
// Welcome
  lcd.setCursor(0, 0);
  lcd.print("    Salut ! :)  ");
  lcd.setCursor(0, 1);
  lcd.print("Eu sunt RoBobita");

  printByte(heart);
  delay(300);
  printByte(heart1);
  delay(300);
  printByte(heart);
  delay(300);
  printByte(heart1);
  delay(300);
  printByte(heart);
  delay(300);
  printByte(heart1);
  delay(300);
  printByte(heart);
  delay(300);
  printByte(HI);
  delay(1200);
  printByte(trei);
  delay(800);
  printByte(doi);
  delay(800);
  printByte(unu);
  delay(300);
  printByte(go);
}

void STRIPRGB(uint32_t c){
  //Functie pentru banda LED

  strip.begin();
  for (int i = 0; i < 4; i++) {
  // Aprinde rosu primele 4 leduri
  for (int j = 0; j < 4; j++) {
    strip.setPixelColor(j, strip.Color(80, 0, 0));  // Setare nuanta rosu 
  }
  
  strip.show();
  delay(200);  

  // Opreste luminile rosii
  for (int j = 0; j < 4; j++) {
    strip.setPixelColor(j, 0); 
  }

  strip.show();
  delay(200);  

  // Aprinde albastru pe celelalte 4 leduri
  for (int j = 4; j < 8; j++) {
    strip.setPixelColor(j, strip.Color(0, 0, 80));  // Setare nuanta albastru
  }

  strip.show();
  delay(200);  

  // Opreste luminile albastre
  for (int j = 0; j < 4; j++) {
    strip.setPixelColor(j, 0);  
  }  
  
  strip.show();
  delay(100);  

//Opreste luminile
  for (int j = 4; j < 8; j++) {
    strip.setPixelColor(j, 0);  
  }
  strip.show();
  delay(100);   
 }
}

//⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝

void sensorLH() {
  //Functie pentru senzorul de umiditate si temperatura
 
  int temperature = dht.readTemperature();
  int humidity = dht.readHumidity();

  lcd.setCursor(0, 0);
  lcd.print("Temper.   ");
  lcd.setCursor(10, 0);
  lcd.print("Humid.");
  lcd.setCursor(0, 1);
  lcd.print(temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.print("       ");
  lcd.setCursor(10, 1);
  lcd.print(humidity);
  lcd.println(" %   ");

  if(temperature>TEMP_THRESHOLD){
    printByte(HOT);
  }else if(temperature<=TEMP_THRESHOLD && temperature>=20){
    printByte(OK);
  }else{
    printByte(COLD);
  }
  delay(300);  // Delay pentru stabilitatea senzorului
}

//⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝⃝

void decodeIR() {
  //Functie decodor semnal IR
  if (IrReceiver.decode()) {
    
    command = IrReceiver.decodedIRData.command; 
    IrReceiver.resume();
    Serial.println(command);
  }     
}



