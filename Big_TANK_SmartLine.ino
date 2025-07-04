#include <Wire.h>
#include <Octoliner.h>

Octoliner octoliner(42);

// PID коэффициенты
#define Kp 1.291
#define Kd 20.09

// Настройки скоростей
#define rightMaxSpeed 160
#define leftMaxSpeed 160
#define rightBaseSpeed 100
#define leftBaseSpeed 100
#define SpeedTurn 120

// Пины управления двигателями
#define EN1 6  // Левый двигатель (PWM)
#define IN1 7  // Направление левого
#define EN2 5  // Правый двигатель (PWM)
#define IN2 4  // Направление правого

const int buttonPin = 12;

// Глобальные переменные
int lastError = 0;
boolean buttonState = 0;
boolean flag = 0;
int L, LL, R, RR, CR, CL, error;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  
  for (int i = 4; i <= 7; i++) {
    pinMode(i, OUTPUT);
  }
  
  Wire.begin();
  octoliner.begin(200);

  // Ожидание нажатия кнопки
  while (!flag) {
    flag = digitalRead(buttonPin);
    delay(25);
  }
}

void loop() {
  line_white();
}

//=== Функции движения ===//

void line_white() {
  sensors();
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  // Ограничение скоростей
  if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, leftMotorSpeed);
  analogWrite(EN2, rightMotorSpeed);
}

void line_black() {
  sensors();
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  // Ограничение скоростей
  if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, leftMotorSpeed);
  analogWrite(EN2, rightMotorSpeed);
}

void line_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  delay(10);
  
  analogWrite(EN1, 30);
  analogWrite(EN2, 30);
  delay(20);

  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  delay(100);
}

//=== Функции поворотов ===//

void povorotR() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, SpeedTurn);
  analogWrite(EN2, SpeedTurn);
  delay(150);
  
  line_stop();
  sensors();
  
  while (!((R > 400) && (RR > 400))) {
    sensors();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, SpeedTurn);
    analogWrite(EN2, SpeedTurn);
  }
  
  line_stop();
  sensors();
  
  while (!(CL > 400)) {
    sensors();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, SpeedTurn);
    analogWrite(EN2, SpeedTurn);
  }
  
  line_stop();
}

void povorotL() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, SpeedTurn);
  analogWrite(EN2, SpeedTurn);
  delay(150);
  
  line_stop();
  sensors();
  
  while (!((L > 400) && (LL > 400))) {
    sensors();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, SpeedTurn);
    analogWrite(EN2, SpeedTurn);
  }
  
  line_stop();
  sensors();
  
  while (!(CR > 400)) {
    sensors();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, SpeedTurn);
    analogWrite(EN2, SpeedTurn);
  }
  
  line_stop();
}

//=== Вспомогательные функции ===//

void sensors() {
  RR = octoliner.analogRead(3);
  R  = octoliner.analogRead(4);
  L  = octoliner.analogRead(5);
  LL = octoliner.analogRead(6);
  CR = octoliner.analogRead(2);
  CL = octoliner.analogRead(7);
}
