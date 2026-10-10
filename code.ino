/*
  ============================================================
        NEXUSTEKFUSION SMART CCTV / SAFETY HELMET
                    ESP32 DEVKIT
  ============================================================
  FEATURES:
    - Fixed ESP32 IP (no DHCP guessing)
    - Hardcoded PC IP for mic streaming (no broadcast)
    - KY-037 / KY-038 analog mic  *** ALWAYS ON + CLEAR VOICE ***
        -> UDP 5005 to PC (4-byte seq + 256 int16 PCM)
    - MAX98357A speaker  <- UDP 5000 from PC
    - Sensor data        -> UDP 5002 to PC
    - MQ-2 / MQ-7 gas sensors
    - DHT11 temp + humidity
    - Vibration sensor + vibration motor
    - Buzzer alarm
    - MAX6675 K-Type thermocouple
    - SSD1306 OLED (I2C)

  CLEAR-VOICE DSP CHAIN:
      raw -> DC removal -> HP 2nd-order -> LP 2nd-order
          -> soft noise gate -> AGC -> gain -> clip
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <max6675.h>
#include "driver/i2s.h"
#include <math.h>

// ============================================================
// WIFI CREDENTIALS
// ============================================================
const char* WIFI_SSID     = "Airtel_sahi_0849";
const char* WIFI_PASSWORD = "air99772";

// ============================================================
// FIXED IP ADDRESSES  *** CHANGE THESE ***
// ============================================================
IPAddress ESP32_IP   (192, 168, 1, 7);
IPAddress ESP32_GW   (192, 168, 1, 1);
IPAddress ESP32_MASK (255, 255, 255, 0);

// Your PC (running the Python dashboard) — the mic stream goes here
IPAddress PC_IP      (192, 168, 1, 5);  // <-- CHANGE to YOUR PC's IP

// ============================================================
// UDP PORTS
// ============================================================
#define MIC_PORT       5005
#define SPEAKER_PORT   5000
#define SENSOR_PORT    5002

WiFiUDP micUDP;
WiFiUDP speakerUDP;
WiFiUDP sensorUDP;

// ============================================================
// KY-037 / KY-038 MICROPHONE  — CLEAR VOICE PRESET
// ============================================================
#define MIC_AO 33
#define MIC_DO 36

const uint32_t SAMPLE_RATE        = 16000;   // 16 kHz for voice clarity
const int      SAMPLES_PER_PACKET = 256;
const int      MIC_PACKET_SIZE    = 4 + SAMPLES_PER_PACKET * 2;

// Tunable voice parameters (also adjustable via Serial)
float micGain       = 12.0f;
int   noiseGate     = 200;       // soft-gate threshold in RMS units
bool  filterEnabled = true;      // HP+LP biquads ON for clarity

// ---- Filter state ----
float micDcOffset    = 2048.0f;
float hp_x1 = 0, hp_x2 = 0;      // 2nd-order HP delay line
float hp_y1 = 0, hp_y2 = 0;
float lp_x1 = 0, lp_x2 = 0;      // 2nd-order LP delay line
float lp_y1 = 0, lp_y2 = 0;
float agcGain        = 1.0f;     // running AGC gain
float envFollower    = 0.0f;     // envelope for gate + AGC
const float envAttack  = 0.35f;
const float envRelease = 0.02f;

// Legacy single-pole state kept for filter=off compatibility
float previousInput  = 0;
float previousOutput = 0;

float blockRMS = 0;

int16_t  micSamples[SAMPLES_PER_PACKET];
uint8_t  micPacket[MIC_PACKET_SIZE];
uint32_t micSequence = 0;

// ---- Non-blocking mic scheduling ----
const uint32_t MIC_SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE;
const uint32_t MIC_BATCH_SAMPLES      = 32;
int            micBatchIndex          = 0;
uint64_t       micSumSquaresAccum     = 0;
uint32_t       nextSampleTime         = 0;

bool micStreaming = true;

// ============================================================
// MAX98357A SPEAKER (I2S_NUM_1)
// ============================================================
#define SPK_I2S_PORT  I2S_NUM_1
#define SPK_BCLK      26
#define SPK_LRC       25
#define SPK_DIN       22

uint8_t speakerBuffer[2048];

// ============================================================
// SENSOR PINS
// ============================================================
#define MQ2_PIN               34
#define MQ7_PIN               35
#define DHT_PIN               4
#define DHT_TYPE              DHT11
#define VIBRATION_SENSOR_PIN  13
#define BUZZER_PIN            27
#define VIBRATION_MOTOR_PIN   12
#define MAX6675_SCK           18
#define MAX6675_CS            5
#define MAX6675_SO            19

// ============================================================
// OLED
// ============================================================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_SDA       21
#define OLED_SCL       16
#define OLED_RESET     -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================
// SENSOR OBJECTS / VARIABLES
// ============================================================
DHT      dht(DHT_PIN, DHT_TYPE);
MAX6675  thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);

float dhtTemperature          = 0.0;
float humidity                = 0.0;
float thermocoupleTemperature = 0.0;
int   mq2Value                = 0;
int   mq7Value                = 0;
int   vibrationValue          = 0;
bool  alarmState              = false;
bool  wifiConnected           = false;

int   MQ2_ALARM_THRESHOLD  = 2500;
int   MQ7_ALARM_THRESHOLD  = 2500;
float TEMPERATURE_ALARM    = 70.0;

// ============================================================
// TIMERS
// ============================================================
unsigned long lastSensorRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastSensorSend = 0;
unsigned long lastWiFiCheck  = 0;

#define SENSOR_INTERVAL       1000
#define OLED_INTERVAL         500
#define SENSOR_SEND_INTERVAL  1000
#define WIFI_CHECK_INTERVAL   5000

// ============================================================
// WEB SERVER
// ============================================================
WebServer server(80);

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>NEXUSTEKFUSION CCTV</title>
<style>
body { font-family: Arial; max-width: 560px; margin: 30px auto; padding: 20px; background: #f3f5f7; }
.box { background: white; padding: 22px; border-radius: 12px; }
code { background:#eef; padding:2px 6px; border-radius:4px; }
</style>
</head>
<body>
<div class="box">
<h2>NEXUSTEKFUSION CCTV</h2>
<p>ESP32 IP: <code>%ESP_IP%</code></p>
<p>Mic streaming: <b>ALWAYS ON</b> (clear voice DSP)</p>
<p>Mic target (PC): <code>%PC_IP%</code> : <code>5005</code></p>
<p>Speaker listen port: <code>5000</code></p>
<p>Sensor send port: <code>5002</code></p>
</div>
</body>
</html>
)rawliteral";

String serialCommand = "";

// ============================================================
// WEB HANDLER
// ============================================================
void handleRoot() {
  String page = FPSTR(webpage);
  page.replace("%ESP_IP%", WiFi.localIP().toString());
  page.replace("%PC_IP%",  PC_IP.toString());
  server.send(200, "text/html", page);
}

// ============================================================
// SERIAL HELP / STATUS
// ============================================================
void printHelp() {
  Serial.println();
  Serial.println("===== MICROPHONE CONTROLS =====");
  Serial.println("g=12         Set gain (0.5 to 30)");
  Serial.println("gate=200     Set soft noise gate (0 to 8000)");
  Serial.println("filter=on    Enable HP+LP voice filters");
  Serial.println("filter=off   Disable all filters");
  Serial.println("status       Show current settings");
  Serial.println("help         Show commands");
  Serial.println("===============================");
  Serial.println();
}

void printStatus() {
  Serial.println();
  Serial.println("----- CURRENT SETTINGS -----");
  Serial.printf("Gain: %.1f\n", micGain);
  Serial.printf("Noise gate: %d\n", noiseGate);
  Serial.printf("Filter: %s\n", filterEnabled ? "ON (HP+LP)" : "OFF");
  Serial.printf("DC offset: %.1f\n", micDcOffset);
  Serial.printf("AGC gain: %.2f\n", agcGain);
  Serial.printf("Envelope: %.1f\n", envFollower);
  Serial.printf("Block RMS: %.1f\n", blockRMS);
  Serial.printf("Sample rate: %u Hz\n", SAMPLE_RATE);
  Serial.printf("Mic streaming: ALWAYS ON\n");
  Serial.print ("ESP32 IP: "); Serial.println(WiFi.localIP());
  Serial.print ("Mic target: "); Serial.println(PC_IP);
  Serial.println("----------------------------");
}

// ============================================================
// SERIAL COMMANDS
// ============================================================
void processSerialCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith("g=")) {
    float v = cmd.substring(2).toFloat();
    if (v >= 0.5f && v <= 30.0f) { micGain = v; Serial.printf("Gain set to %.1f\n", micGain); }
    else Serial.println("Gain must be 0.5..30.");
  }
  else if (cmd.startsWith("gate=")) {
    int v = cmd.substring(5).toInt();
    if (v >= 0 && v <= 8000) { noiseGate = v; Serial.printf("Gate set to %d\n", noiseGate); }
    else Serial.println("Gate must be 0..8000.");
  }
  else if (cmd == "filter=on")  {
    filterEnabled = true;
    hp_x1 = hp_x2 = hp_y1 = hp_y2 = 0;
    lp_x1 = lp_x2 = lp_y1 = lp_y2 = 0;
    previousInput = previousOutput = 0;
    Serial.println("Voice filters ON (HP+LP)");
  }
  else if (cmd == "filter=off") {
    filterEnabled = false;
    Serial.println("Filters OFF");
  }
  else if (cmd == "status")     { printStatus(); }
  else if (cmd == "help")       { printHelp(); }
  else if (cmd.length() > 0)    { Serial.println("Unknown command. Type help."); }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialCommand.length() > 0) {
        processSerialCommand(serialCommand);
        serialCommand = "";
      }
    } else if (serialCommand.length() < 80) {
      serialCommand += c;
    }
  }
}

// ============================================================
// WIFI (FIXED IP)
// ============================================================
void connectWiFi()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("Connecting to WiFi (fixed IP)...");
  Serial.print("SSID: "); Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.config(ESP32_IP, ESP32_GW, ESP32_MASK);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("WiFi CONNECTED");
    Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway:  "); Serial.println(WiFi.gatewayIP());
    Serial.print("RSSI: ");     Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println("WiFi CONNECTION FAILED — check SSID/password/gateway");
  }
  Serial.println("================================");
}

// ============================================================
// SPEAKER SETUP (MAX98357A)
// ============================================================
void setupSpeaker()
{
  Serial.println("Starting MAX98357A...");

  i2s_config_t speakerConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = SAMPLES_PER_PACKET,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t speakerPins = {
    .bck_io_num   = SPK_BCLK,
    .ws_io_num    = SPK_LRC,
    .data_out_num = SPK_DIN,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  esp_err_t result = i2s_driver_install(SPK_I2S_PORT, &speakerConfig, 0, NULL);
  if (result != ESP_OK) { Serial.println("MAX98357 driver error"); return; }
  result = i2s_set_pin(SPK_I2S_PORT, &speakerPins);
  if (result != ESP_OK) { Serial.println("MAX98357 pin error"); return; }

  i2s_zero_dma_buffer(SPK_I2S_PORT);
  Serial.println("MAX98357A READY");
}

// ============================================================
// SENSORS / OLED
// ============================================================
void setupDHT() { dht.begin(); delay(1000); Serial.println("DHT11 READY"); }

void setupOLED()
{
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED NOT FOUND (SDA=21 SCL=16 0x3C)");
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(15, 10); display.println("NEXUSTEKFUSION");
  display.setCursor(20, 26); display.println("SMART CCTV");
  display.setCursor(30, 42); display.println("SYSTEM");
  display.display();
  delay(1500);
  Serial.println("OLED READY");
}

void setupSensors()
{
  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);
  analogReadResolution(12);
  Serial.println("SENSORS READY");
}

// ============================================================
// READ SENSORS
// ============================================================
void readSensors()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) humidity = h;
  if (!isnan(t)) dhtTemperature = t;

  mq2Value = analogRead(MQ2_PIN);
  mq7Value = analogRead(MQ7_PIN);
  vibrationValue = digitalRead(VIBRATION_SENSOR_PIN);

  double tc = thermocouple.readCelsius();
  if (!isnan(tc)) thermocoupleTemperature = tc;

  checkAlarm();
}

void checkAlarm()
{
  bool gasAlarm       = (mq2Value >= MQ2_ALARM_THRESHOLD) || (mq7Value >= MQ7_ALARM_THRESHOLD);
  bool heatAlarm      = (thermocoupleTemperature >= TEMPERATURE_ALARM);
  bool vibrationAlarm = (vibrationValue == HIGH);
  alarmState = gasAlarm || heatAlarm || vibrationAlarm;

  digitalWrite(BUZZER_PIN,          alarmState ? HIGH : LOW);
  digitalWrite(VIBRATION_MOTOR_PIN, alarmState ? HIGH : LOW);
}

// ============================================================
// OLED UPDATE
// ============================================================
void updateOLED()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);  display.println("NEXUSTEKFUSION CCTV");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 13);
  display.print("T:"); display.print(dhtTemperature, 1);
  display.print("C H:"); display.print(humidity, 0); display.println("%");

  display.setCursor(0, 25);
  display.print("K:"); display.print(thermocoupleTemperature, 1); display.println("C");

  display.setCursor(0, 37);
  display.print("MQ2:"); display.print(mq2Value);
  display.print(" MQ7:"); display.print(mq7Value);

  display.setCursor(0, 49);
  display.print("VIB:"); display.print(vibrationValue == HIGH ? "YES" : "NO");
  display.setCursor(75, 49);
  display.print(alarmState ? "ALARM!" : "OK");

  display.display();
}

// ============================================================
// CLEAR-VOICE MIC PIPELINE
//   raw -> DC removal -> HP 2nd-order -> LP 2nd-order
//       -> soft noise gate -> AGC -> gain -> clip
// ============================================================
int16_t processMicSample(int raw)
{
  // ---- 1. Adaptive DC removal ----
  micDcOffset += 0.0005f * ((float)raw - micDcOffset);
  float x = (float)raw - micDcOffset;

  if (filterEnabled) {
    // ---- 2. 2nd-order high-pass (biquad, ~200 Hz @ 16 kHz) ----
    const float b0 =  0.9203f;
    const float b1 = -1.8406f;
    const float b2 =  0.9203f;
    const float a1 = -1.8268f;
    const float a2 =  0.8549f;

    float y = b0 * x + b1 * hp_x1 + b2 * hp_x2
                     - a1 * hp_y1 - a2 * hp_y2;
    hp_x2 = hp_x1; hp_x1 = x;
    hp_y2 = hp_y1; hp_y1 = y;
    x = y;

    // ---- 3. 2nd-order low-pass (biquad, ~3400 Hz @ 16 kHz) ----
    const float lb0 = 0.2673f;
    const float lb1 = 0.5347f;
    const float lb2 = 0.2673f;
    const float la1 = -0.2373f;
    const float la2 = 0.3066f;

    float z = lb0 * x + lb1 * lp_x1 + lb2 * lp_x2
                      - la1 * lp_y1 - la2 * lp_y2;
    lp_x2 = lp_x1; lp_x1 = x;
    lp_y2 = lp_y1; lp_y1 = z;
    x = z;
  } else {
    // Legacy single-pole HP when filters are off (keeps DC out only)
    const float alpha = 0.95f;
    float output = alpha * (previousOutput + x - previousInput);
    previousInput  = x;
    previousOutput = output;
    x = output;
  }

  // ---- 4. Envelope follower ----
  float absX = fabsf(x);
  if (absX > envFollower)
    envFollower += envAttack  * (absX - envFollower);
  else
    envFollower += envRelease * (absX - envFollower);

  // ---- 5. Soft noise gate ----
  float gateGain = 1.0f;
  if (noiseGate > 0) {
    float gateLin = (float)noiseGate;
    if (envFollower < gateLin) {
      gateGain = 0.0f;
    } else if (envFollower < gateLin * 1.5f) {
      gateGain = (envFollower - gateLin) / (gateLin * 0.5f);
    }
  }

  // ---- 6. AGC-lite ----
  const float targetEnv = 1200.0f;
  if (envFollower > 100.0f) {
    float desired = targetEnv / envFollower;
    if (desired < 0.5f) desired = 0.5f;
    if (desired > 3.0f) desired = 3.0f;
    agcGain += 0.002f * (desired - agcGain);
  }

  // ---- 7. Apply gate, AGC, static gain ----
  float out = x * gateGain * agcGain * micGain;

  // ---- 8. Clip ----
  if (out >  32767.0f) out =  32767.0f;
  if (out < -32768.0f) out = -32768.0f;

  return (int16_t)out;
}

// ============================================================
// SEND MIC PACKET
// ============================================================
void sendMicPacket()
{
  micPacket[0] = micSequence        & 0xFF;
  micPacket[1] = (micSequence >> 8)  & 0xFF;
  micPacket[2] = (micSequence >> 16) & 0xFF;
  micPacket[3] = (micSequence >> 24) & 0xFF;

  for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
    uint16_t s = (uint16_t)micSamples[i];
    micPacket[4 + i * 2] = s & 0xFF;
    micPacket[5 + i * 2] = (s >> 8) & 0xFF;
  }

  micUDP.beginPacket(PC_IP, MIC_PORT);
  micUDP.write(micPacket, MIC_PACKET_SIZE);
  micUDP.endPacket();
  micSequence++;
}

// ============================================================
// NON-BLOCKING MIC SAMPLER
// ============================================================
void streamMicrophone()
{
  if (!wifiConnected || !micStreaming) return;

  for (uint32_t n = 0; n < MIC_BATCH_SAMPLES; n++) {
    while ((int32_t)(micros() - nextSampleTime) < 0) { /* wait */ }
    nextSampleTime += MIC_SAMPLE_INTERVAL_US;

    int raw = analogRead(MIC_AO);
    int16_t s = processMicSample(raw);

    micSamples[micBatchIndex++] = s;
    micSumSquaresAccum += (uint64_t)((int64_t)s * (int64_t)s);

    if (micBatchIndex >= SAMPLES_PER_PACKET) {
      blockRMS = sqrtf((float)micSumSquaresAccum / SAMPLES_PER_PACKET);

      // (Gate is already applied per sample in processMicSample;
      //  this block-level check is a safety net only.)
      if (noiseGate > 0 && blockRMS < (float)noiseGate * 0.2f) {
        for (int i = 0; i < SAMPLES_PER_PACKET; i++) micSamples[i] = 0;
      }

      sendMicPacket();
      micBatchIndex      = 0;
      micSumSquaresAccum = 0;
    }
  }
}

// ============================================================
// RECEIVE SPEAKER AUDIO
// ============================================================
void receiveSpeaker()
{
  if (!wifiConnected) return;

  int packetSize = speakerUDP.parsePacket();
  if (packetSize <= 0) return;

  if (packetSize > (int)sizeof(speakerBuffer))
    packetSize = sizeof(speakerBuffer);

  int len = speakerUDP.read(speakerBuffer, packetSize);
  if (len > 0) {
    size_t written = 0;
    i2s_write(SPK_I2S_PORT, speakerBuffer, len, &written, 0);
  }
}

// ============================================================
// SEND SENSOR DATA
// ============================================================
void sendSensorData()
{
  if (!wifiConnected) return;

  char packet[400];
  snprintf(packet, sizeof(packet),
    "TEMP=%.2f,HUM=%.2f,TC=%.2f,MQ2=%d,MQ7=%d,VIB=%d,ALARM=%d,RSSI=%d,RMS=%.1f,DC=%.0f",
    dhtTemperature, humidity, thermocoupleTemperature,
    mq2Value, mq7Value, vibrationValue,
    alarmState ? 1 : 0, WiFi.RSSI(),
    blockRMS, micDcOffset
  );

  sensorUDP.beginPacket(PC_IP, SENSOR_PORT);
  sensorUDP.write((uint8_t*)packet, strlen(packet));
  sensorUDP.endPacket();
}

// ============================================================
// SERIAL PRINT
// ============================================================
void printSensorData()
{
  Serial.println();
  Serial.println("========== STATUS ==========");
  Serial.print("DHT11 Temp : "); Serial.print(dhtTemperature, 1); Serial.println(" C");
  Serial.print("Humidity   : "); Serial.print(humidity, 1);       Serial.println(" %");
  Serial.print("MAX6675    : "); Serial.print(thermocoupleTemperature, 1); Serial.println(" C");
  Serial.print("MQ-2 ADC   : "); Serial.println(mq2Value);
  Serial.print("MQ-7 ADC   : "); Serial.println(mq7Value);
  Serial.print("Vibration  : "); Serial.println(vibrationValue);
  Serial.print("Mic RMS    : "); Serial.println(blockRMS);
  Serial.print("Mic DC     : "); Serial.println(micDcOffset, 1);
  Serial.print("AGC gain   : "); Serial.println(agcGain, 2);
  Serial.print("Envelope   : "); Serial.println(envFollower, 1);
  Serial.print("Alarm      : "); Serial.println(alarmState ? "ACTIVE" : "NORMAL");
  Serial.print("WiFi RSSI  : "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.print("ESP32 IP   : "); Serial.println(WiFi.localIP());
  Serial.print("PC target  : "); Serial.println(PC_IP);
  Serial.println("============================");
}

// ============================================================
// WIFI RECONNECT
// ============================================================
void checkWiFi()
{
  if (WiFi.status() == WL_CONNECTED) { wifiConnected = true; return; }

  wifiConnected = false;
  Serial.println("WiFi lost — reconnecting with fixed IP...");
  WiFi.disconnect();
  WiFi.config(ESP32_IP, ESP32_GW, ESP32_MASK);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("######################################");
  Serial.println(" NEXUSTEKFUSION SMART CCTV");
  Serial.println(" ESP32 DEVKIT CONTROLLER");
  Serial.println("######################################");

  pinMode(MIC_AO, INPUT);
  pinMode(MIC_DO, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_AO, ADC_11db);

  setupSensors();
  setupDHT();
  setupOLED();
  connectWiFi();
  setupSpeaker();

  micUDP.begin(MIC_PORT);
  speakerUDP.begin(SPEAKER_PORT);
  sensorUDP.begin(SENSOR_PORT);

  server.on("/", handleRoot);
  server.begin();

  Serial.println();
  Serial.println("######################################");
  Serial.println(" SYSTEM READY");
  Serial.println("######################################");

  if (wifiConnected) {
    Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
    Serial.print("Open http://"); Serial.println(WiFi.localIP());
    Serial.print("Mic -> PC "); Serial.print(PC_IP);
    Serial.print(":"); Serial.println(MIC_PORT);
    Serial.print("Speaker <- PC UDP port "); Serial.println(SPEAKER_PORT);
    Serial.print("Sensor -> PC UDP port "); Serial.println(SENSOR_PORT);
  }

  micStreaming = true;
  nextSampleTime = micros();
  micBatchIndex = 0;
  micSumSquaresAccum = 0;

  Serial.println(">>> MIC ALWAYS-ON ENABLED (CLEAR VOICE) <<<");
  printHelp();
  printStatus();
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
  server.handleClient();
  handleSerial();

  receiveSpeaker();
  streamMicrophone();

  if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    checkWiFi();
  }
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
    readSensors();
    printSensorData();
  }
  if (millis() - lastOLEDUpdate >= OLED_INTERVAL) {
    lastOLEDUpdate = millis();
    updateOLED();
  }
  if (millis() - lastSensorSend >= SENSOR_SEND_INTERVAL) {
    lastSensorSend = millis();
    sendSensorData();
  }
}