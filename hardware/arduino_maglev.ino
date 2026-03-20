/*
  Arduino sample code for a magnetic levitation setup.
  This example reads a sensor, estimates position, and drives an electromagnet with PWM.
  Adapt pins, scaling, and safety interlocks to your hardware.
*/

const int SENSOR_PIN = A0;
const int PWM_PIN = 9;
const float DT = 0.01f;

float kp = 45.0f;
float ki = 120.0f;
float kd = 1.5f;
float targetPosition = 0.015f;
float integralTerm = 0.0f;
float previousError = 0.0f;

float readPositionMeters() {
  int raw = analogRead(SENSOR_PIN);
  float voltage = raw * (5.0f / 1023.0f);
  return 0.005f + (voltage / 5.0f) * 0.03f;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  float position = readPositionMeters();
  float error = targetPosition - position;
  integralTerm += error * DT;
  float derivative = (error - previousError) / DT;

  float command = kp * error + ki * integralTerm + kd * derivative;
  if (command < 0.0f) command = 0.0f;
  if (command > 255.0f) command = 255.0f;

  analogWrite(PWM_PIN, (int)command);
  previousError = error;

  Serial.print("position=");
  Serial.print(position, 5);
  Serial.print(", pwm=");
  Serial.println(command, 2);

  delay((int)(DT * 1000.0f));
}
