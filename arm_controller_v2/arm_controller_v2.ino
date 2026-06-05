#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

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

void processJsonCommand(String jsonStr);
void writeServoByChannel(int channel, float degree);
uint16_t microsecondsToTicks(int pulse_us);

void sendOkServo(const char *cmd, int channel, float degree);
void sendOkSimple(const char *cmd);
void sendError(const char *message);
void sendErrorWithRaw(const char *message, String raw);

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("{\"status\":\"ready\",\"device\":\"esp32s3_servo\"}");

  Wire.begin(I2C_SDA, I2C_SCL);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  writeServoByChannel(0, 0);    // gripper close
  writeServoByChannel(1, 90);   // joint1
  writeServoByChannel(2, 90);   // joint2
  writeServoByChannel(3, 90);   // joint3

  Serial.println("{\"status\":\"ok\",\"cmd\":\"init\",\"message\":\"initial_position_set\"}");
}

void loop()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n')
    {
      inputLine.trim();

      if (inputLine.length() > 0)
      {
        processJsonCommand(inputLine);
      }

      inputLine = "";
    }
    else if (c == '\r')
    {
      // ignore CR
    }
    else
    {
      inputLine += c;

      // Chống trường hợp packet lỗi quá dài làm đầy RAM
      if (inputLine.length() > 512)
      {
        sendErrorWithRaw("input_too_long", inputLine);
        inputLine = "";
      }
    }
  }
}

void processJsonCommand(String jsonStr)
{
  StaticJsonDocument<512> doc;

  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error)
  {
    sendErrorWithRaw("json_parse_failed", jsonStr);
    return;
  }

  const char *cmd = doc["cmd"];

  if (cmd == nullptr)
  {
    sendError("missing_cmd");
    return;
  }

  // Ping test
  if (strcmp(cmd, "ping") == 0)
  {
    Serial.println("{\"status\":\"ok\",\"cmd\":\"ping\",\"message\":\"pong\"}");
    return;
  }

  // Single servo command:
  // {"cmd":"servo","channel":1,"degree":120}
  if (strcmp(cmd, "servo") == 0)
  {
    int channel = doc["channel"] | -1;
    float degree = doc["degree"] | -999.0;

    if (channel < 0 || channel > 15)
    {
      sendError("invalid_channel");
      return;
    }

    if (degree < 0.0 || degree > 180.0)
    {
      sendError("invalid_degree");
      return;
    }

    writeServoByChannel(channel, degree);
    sendOkServo(cmd, channel, degree);
    return;
  }

  // Batch servo command:
  // {"cmd":"set_all","servos":[{"channel":1,"degree":120},{"channel":2,"degree":60}]}
  if (strcmp(cmd, "set_all") == 0)
  {
    JsonArray servos = doc["servos"].as<JsonArray>();

    if (servos.isNull())
    {
      sendError("missing_servos");
      return;
    }

    if (servos.size() == 0)
    {
      sendError("empty_servos");
      return;
    }

    // Kiểm tra toàn bộ dữ liệu trước
    for (JsonObject servo : servos)
    {
      int channel = servo["channel"] | -1;
      float degree = servo["degree"] | -999.0;

      if (channel < 0 || channel > 15)
      {
        sendError("invalid_channel_in_set_all");
        return;
      }

      if (degree < 0.0 || degree > 180.0)
      {
        sendError("invalid_degree_in_set_all");
        return;
      }
    }

    // Dữ liệu hợp lệ rồi mới ghi servo
    for (JsonObject servo : servos)
    {
      int channel = servo["channel"] | -1;
      float degree = servo["degree"] | -999.0;

      writeServoByChannel(channel, degree);
    }

    sendOkSimple("set_all");
    return;
  }

  // Deactivate command:
  // {"cmd":"deactivate","channel":1}
  if (strcmp(cmd, "deactivate") == 0)
  {
    int channel = doc["channel"] | -1;

    if (channel < 0 || channel > 15)
    {
      sendError("invalid_channel");
      return;
    }

    // Tắt xung PWM kênh đó
    pwm.setPWM(channel, 0, 0);

    sendOkSimple("deactivate");
    return;
  }

  sendError("unknown_cmd");
}

void writeServoByChannel(int channel, float degree)
{
  if (channel < 0 || channel > 15)
  {
    return;
  }

  degree = constrain(degree, 0.0, 180.0);

  int pulse_us = SERVO_MIN_US +
                 (int)((degree / 180.0) * (SERVO_MAX_US - SERVO_MIN_US));

  uint16_t ticks = microsecondsToTicks(pulse_us);

  pwm.setPWM(channel, 0, ticks);
}

uint16_t microsecondsToTicks(int pulse_us)
{
  pulse_us = constrain(pulse_us, SERVO_MIN_US, SERVO_MAX_US);

  uint16_t ticks = (uint16_t)((pulse_us * 4096.0) / SERVO_PERIOD_US);

  return ticks;
}

void sendOkServo(const char *cmd, int channel, float degree)
{
  StaticJsonDocument<128> doc;

  doc["status"] = "ok";
  doc["cmd"] = cmd;
  doc["channel"] = channel;
  doc["degree"] = degree;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendOkSimple(const char *cmd)
{
  StaticJsonDocument<128> doc;

  doc["status"] = "ok";
  doc["cmd"] = cmd;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendError(const char *message)
{
  StaticJsonDocument<128> doc;

  doc["status"] = "error";
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendErrorWithRaw(const char *message, String raw)
{
  StaticJsonDocument<512> doc;

  doc["status"] = "error";
  doc["message"] = message;
  doc["raw"] = raw;

  serializeJson(doc, Serial);
  Serial.println();
}