#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// Инициализация объектов сервоприводов
Servo servo1;
Servo servo2;
Servo servo3;

// Инициализация объекта акселерометра
Adafruit_LSM303_Accel_Unified accel;

// Параметры ПИД-регулятора
double Kp = 0.8;  // Коэффициент пропорциональной составляющей
double Ki = 0.2;  // Коэффициент интегральной составляющей
double Kd = 0.1;  // Коэффициент дифференциальной составляющей

// Переменные для хранения состояний ошибок
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;

// Целевые значения углов сервоприводов
int targetAngle1 = 90;
int targetAngle2 = 90;
int targetAngle3 = 90;

// Пины подключения сервоприводов
int servoPin1 = 2;
int servoPin2 = 3;
int servoPin3 = 4;

// Функция для обновления углов сервоприводов
void updateServoAngles(int angle1, int angle2, int angle3) {
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
}

// Функция для инициализации акселерометра
void initAccelerometer() {
  if (!accel.begin()) {
    Serial.println("Failed to initialize accelerometer!");
    while (1);
  }
}

// Функция для чтения угла наклона от акселерометра
double readAccelerometerAngle() {
  sensors_event_t event;
  accel.getEvent(&event);
  double angle = atan2(event.acceleration.y, event.acceleration.z) * 180 / PI;
  return angle;
}

// Функция для обновления состояния ПИД-регулятора
void updatePID() {
  double currentAngle = readAccelerometerAngle();

  // Вычисление ошибки
  error = targetAngle1 - currentAngle;
  
  // Вычисление интегральной составляющей
  integral += error;
  
  // Ограничение интегральной составляющей
  if (integral > 100) {
    integral = 100;
  } else if (integral < -100) {
    integral = -100;
  }
  
  // Вычисление дифференциальной составляющей
  derivative = error - lastError;
  
  // Обновление углов сервоприводов с использованием ПИД-регулятора
  int angle1 = targetAngle1 + Kp * error + Ki * integral + Kd * derivative;
  int angle2 = targetAngle2 + Kp * error + Ki * integral + Kd * derivative;
  int angle3 = targetAngle3 + Kp * error + Ki * integral + Kd * derivative;
  
  // Ограничение углов сервоприводов
  if (angle1 > 180) {
    angle1 = 180;
  } else if (angle1 < 0) {
    angle1 = 0;
  }
  
  if (angle2 > 180) {
    angle2 = 180;
  } else if (angle2 < 0) {
    angle2 = 0;
  }
  
  if (angle3 > 180) {
    angle3 = 180;
  } else if (angle3 < 0) {
    angle3 = 0;
  }
  
  // Обновление углов сервоприводов
  updateServoAngles(angle1, angle2, angle3);
  
  // Сохранение текущей ошибки
  lastError = error;
}

void setup() {
  // Инициализация сервоприводов
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  
  // Инициализация акселерометра
  initAccelerometer();
  
  // Начальное положение сервоприводов
  updateServoAngles(targetAngle1, targetAngle2, targetAngle3);
  
  // Настройка скорости обновления ПИД-регулятора
  // Вы можете изменить этот параметр в зависимости от требований вашей системы
  // Чем меньше значение, тем чаще будет обновляться состояние ПИД-регулятора
  int pidUpdateInterval = 10;  // Обновление каждые 10 миллисекунд
  
  // Установка интервала обновления
  // Функция updatePID() будет вызываться каждые 10 миллисекунд
  setInterval(updatePID, pidUpdateInterval);
}

void loop() {
  // Оставьте пустой цикл loop()
  // Вся обработка происходит в функции updatePID(),
  // которая вызывается с заданным интервалом
}
