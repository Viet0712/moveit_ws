#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// I2C pins for ESP32-S3
const int I2C_SDA = 8;
const int I2C_SCL = 9;

// PCA9685 config
const int SERVO_FREQ = 50;

// Servo pulse range
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_PERIOD_US = 20000;

String inputLine = "";

void processCommand(String cmd);
void writeServoByChannel(int channel, float degree);
uint16_t microsecondsToTicks(int pulse_us);

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("ESP32-S3 PCA9685 Servo Controller Ready");
  Serial.println("Format: S,<channel>,<degree>");
  Serial.println("Example: S,0,150");

  Wire.begin(I2C_SDA, I2C_SCL);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  // Initial positions
  writeServoByChannel(0, 0);    // gripper close
  writeServoByChannel(1, 90);   // joint1
  writeServoByChannel(2, 90);   // joint2
  writeServoByChannel(3, 90);   // joint3/

  Serial.println("Initial position set");
}

void loop()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n')
    {
      inputLine.trim();

      Serial.print("Full command: [");
      Serial.print(inputLine);
      Serial.println("]");

      if (inputLine.length() > 0)
      {
        processCommand(inputLine);
      }

      inputLine = "";
    }
    else if (c == '\r')
    {
      // Ignore CR
    }
    else
    {
      inputLine += c;
    }
  }
}

void processCommand(String cmd)
{
  cmd.trim();

  if (!cmd.startsWith("S,"))
  {
    Serial.print("Invalid command: ");
    Serial.println(cmd);
    return;
  }

  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0)
  {
    Serial.print("Invalid format: ");
    Serial.println(cmd);
    return;
  }

  int channel = cmd.substring(firstComma + 1, secondComma).toInt();
  float degree = cmd.substring(secondComma + 1).toFloat();

  degree = constrain(degree, 0.0, 180.0);

  writeServoByChannel(channel, degree);
}

void writeServoByChannel(int channel, float degree)
{
  if (channel < 0 || channel > 15)
  {
    Serial.print("Invalid PCA9685 channel: ");
    Serial.println(channel);
    return;
  }

  degree = constrain(degree, 0.0, 180.0);

  int pulse_us = SERVO_MIN_US +
                 (int)((degree / 180.0) * (SERVO_MAX_US - SERVO_MIN_US));

  uint16_t ticks = microsecondsToTicks(pulse_us);

  pwm.setPWM(channel, 0, ticks);

  Serial.print("PCA9685 CH");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(degree);
  Serial.print(" deg, pulse=");
  Serial.print(pulse_us);
  Serial.print(" us, ticks=");
  Serial.println(ticks);
}

uint16_t microsecondsToTicks(int pulse_us)
{
  pulse_us = constrain(pulse_us, SERVO_MIN_US, SERVO_MAX_US);

  // PCA9685 has 4096 ticks per PWM period
  uint16_t ticks = (uint16_t)((pulse_us * 4096.0) / SERVO_PERIOD_US);

  return ticks;
}