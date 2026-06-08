#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// I2C pins for ESP32-S3
const int I2C_SDA = 8;
const int I2C_SCL = 9;

// PCA9685 config
const int SERVO_FREQ = 50;

// Servo pulse range - hẹp hơn để tránh ép biên servo
const int SERVO_MIN_US = 600;
const int SERVO_MAX_US = 2400;
const int SERVO_PERIOD_US = 20000;

// Smooth control config
const int SERVO_COUNT = 16;
const float DEFAULT_SPEED_DPS = 90.0;     // degree per second
const unsigned long UPDATE_INTERVAL_MS = 15;

float currentDegree[SERVO_COUNT];
float targetDegree[SERVO_COUNT];
float speedDps[SERVO_COUNT];
bool servoActive[SERVO_COUNT];

unsigned long lastUpdateMs = 0;

String inputLine = "";

void processJsonCommand(String jsonStr);
void updateServosSmooth();

void writeServoRaw(int channel, float degree);
void setServoTarget(int channel, float degree, float speed = DEFAULT_SPEED_DPS);
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

  // Init mặc định cho toàn bộ 16 channel
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    currentDegree[i] = 90.0;
    targetDegree[i] = 90.0;
    speedDps[i] = DEFAULT_SPEED_DPS;
    servoActive[i] = false;
  }

  // Vị trí ban đầu của arm
  currentDegree[0] = 0.0;     // gripper close
  targetDegree[0] = 0.0;
  servoActive[0] = true;

  currentDegree[1] = 90.0;    // joint1
  targetDegree[1] = 90.0;
  servoActive[1] = true;

  currentDegree[2] = 90.0;    // joint2
  targetDegree[2] = 90.0;
  servoActive[2] = true;

  currentDegree[3] = 90.0;    // joint3
  targetDegree[3] = 90.0;
  servoActive[3] = true;

  currentDegree[4] = 90.0;    // joint4
  targetDegree[4] = 90.0;
  servoActive[4] = true;

  currentDegree[5] = 90.0;    // joint5
  targetDegree[5] = 90.0;
  servoActive[5] = true;

  // Ghi vị trí ban đầu ra servo
  writeServoRaw(0, currentDegree[0]);   // gripper
  writeServoRaw(1, currentDegree[1]);   // joint1
  writeServoRaw(2, currentDegree[2]);   // joint2
  writeServoRaw(3, currentDegree[3]);   // joint3
  writeServoRaw(4, currentDegree[4]);   // joint4
  writeServoRaw(5, currentDegree[5]);   // joint5

  lastUpdateMs = millis();

  Serial.println("{\"status\":\"ok\",\"cmd\":\"init\",\"message\":\"initial_position_set\"}");
}

void loop()
{
  // Luôn cập nhật servo mượt trong loop
  updateServosSmooth();

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

  // Ping test:
  // {"cmd":"ping"}
  if (strcmp(cmd, "ping") == 0)
  {
    Serial.println("{\"status\":\"ok\",\"cmd\":\"ping\",\"message\":\"pong\"}");
    return;
  }

  // Single servo command:
  // {"cmd":"servo","channel":1,"degree":120}
  // hoặc:
  // {"cmd":"servo","channel":1,"degree":120,"speed":60}
  if (strcmp(cmd, "servo") == 0)
  {
    int channel = doc["channel"] | -1;
    float degree = doc["degree"] | -999.0;
    float speed = doc["speed"] | DEFAULT_SPEED_DPS;

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

    if (speed <= 0.0 || speed > 720.0)
    {
      sendError("invalid_speed");
      return;
    }

    setServoTarget(channel, degree, speed);
    sendOkServo(cmd, channel, degree);
    return;
  }

  // Batch servo command:
  // {"cmd":"set_all","servos":[{"channel":1,"degree":120},{"channel":2,"degree":60}]}
  // hoặc có speed riêng:
  // {"cmd":"set_all","servos":[{"channel":1,"degree":120,"speed":60},{"channel":2,"degree":60,"speed":90}]}
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

    // Kiểm tra toàn bộ trước
    for (JsonObject servo : servos)
    {
      int channel = servo["channel"] | -1;
      float degree = servo["degree"] | -999.0;
      float speed = servo["speed"] | DEFAULT_SPEED_DPS;

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

      if (speed <= 0.0 || speed > 720.0)
      {
        sendError("invalid_speed_in_set_all");
        return;
      }
    }

    // Dữ liệu hợp lệ rồi mới set target
    for (JsonObject servo : servos)
    {
      int channel = servo["channel"] | -1;
      float degree = servo["degree"] | -999.0;
      float speed = servo["speed"] | DEFAULT_SPEED_DPS;

      setServoTarget(channel, degree, speed);
    }

    sendOkSimple("set_all");
    return;
  }

  // Deactivate command:
  // {"cmd":"deactivate","channel":1}
  // Lưu ý: tắt PWM thì servo mất lực giữ, bật lại có thể giật.
  if (strcmp(cmd, "deactivate") == 0)
  {
    int channel = doc["channel"] | -1;

    if (channel < 0 || channel > 15)
    {
      sendError("invalid_channel");
      return;
    }

    pwm.setPWM(channel, 0, 0);
    servoActive[channel] = false;

    sendOkSimple("deactivate");
    return;
  }

  // Activate lại kênh:
  // {"cmd":"activate","channel":1}
  if (strcmp(cmd, "activate") == 0)
  {
    int channel = doc["channel"] | -1;

    if (channel < 0 || channel > 15)
    {
      sendError("invalid_channel");
      return;
    }

    servoActive[channel] = true;
    writeServoRaw(channel, currentDegree[channel]);

    sendOkSimple("activate");
    return;
  }

  sendError("unknown_cmd");
}

void updateServosSmooth()
{
  unsigned long now = millis();

  if (now - lastUpdateMs < UPDATE_INTERVAL_MS)
  {
    return;
  }

  float dt = (now - lastUpdateMs) / 1000.0;
  lastUpdateMs = now;

  for (int ch = 0; ch < SERVO_COUNT; ch++)
  {
    if (!servoActive[ch])
    {
      continue;
    }

    float diff = targetDegree[ch] - currentDegree[ch];

    if (abs(diff) < 0.05)
    {
      currentDegree[ch] = targetDegree[ch];
      continue;
    }

    float maxStep = speedDps[ch] * dt;

    if (abs(diff) <= maxStep)
    {
      currentDegree[ch] = targetDegree[ch];
    }
    else
    {
      if (diff > 0)
      {
        currentDegree[ch] += maxStep;
      }
      else
      {
        currentDegree[ch] -= maxStep;
      }
    }

    writeServoRaw(ch, currentDegree[ch]);
  }
}

void setServoTarget(int channel, float degree, float speed)
{
  if (channel < 0 || channel > 15)
  {
    return;
  }

  degree = constrain(degree, 0.0, 180.0);
  speed = constrain(speed, 1.0, 720.0);

  targetDegree[channel] = degree;
  speedDps[channel] = speed;
  servoActive[channel] = true;
}

void writeServoRaw(int channel, float degree)
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