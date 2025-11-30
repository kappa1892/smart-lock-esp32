// スマートロック統合コード（ESP32 + AWS IoT Core + MQTT + バッテリー + サーボ）
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "time.h"   // NTP利用に必要

const char* ntpServer = "ntp.nict.jp";  // 日本のNTPサーバ
const long  gmtOffset_sec = 9 * 3600;   // 日本時間(GMT+9)
const int   daylightOffset_sec = 0;     // サマータイムなし

// 利用する時間帯（例: 出勤 7:10-7:45, 帰宅 18:30-21:00）
const int MORNING_START_HOUR = 7;
const int MORNING_START_MIN  = 10;
const int MORNING_END_HOUR   = 7;
const int MORNING_END_MIN    = 45;

const int EVENING_START_HOUR = 18;
const int EVENING_START_MIN  = 30;
const int EVENING_END_HOUR   = 21;
const int EVENING_END_MIN    = 0;

bool isActiveTime = false;   // 時間帯内かどうか

/************** Wi-Fi設定 **************/
const char* ssid = "JCOM_2CE0";
const char* password = "59631129";

/************** AWS IoT設定 **************/
const char* mqttServer = "a16vsl9h0t1na-ats.iot.ap-northeast-1.amazonaws.com"; // AWS IoT Coreエンドポイント
const int mqttPort = 8883;
const char* mqttClientId = "esp32door";
const char* subscribeTopic = "door/control";   // ← 新しいトピック名
const char* publishTopic   = "door/status";    // ← ステータス送信用トピック
const char* wakeTopic      = "door/wake";   // ← 追加

/************** 証明書 **************/
extern const char caCert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

extern const char clientCert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----

)EOF";

extern const char clientKey[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----

)EOF";

/******** MQTT / ネット ********/
WiFiClientSecure net;
PubSubClient client(net);

/******** ハードウェア ********/
Servo lockServo;
#define SERVO_PIN 18
#define GREEN_LED_PIN 2
#define RED_LED_PIN 4

unsigned long lastActivity = 0;       // 最後に操作があった時刻（ミリ秒）
const unsigned long ACTIVE_TIMEOUT = 60000; // 1分でスリープ

String lockState = "locked"; // "locked" or "unlocked"
const int LOCK_ANGLE   = 0;   // 施錠位置の角度
const int UNLOCK_ANGLE = 90;  // 開錠位置の角度
/******** バッテリー ********/
#define BATTERY_PIN 35
const float LOW_VOLTAGE_THRESHOLD = 3.5;

/******** 自動施錠 ********/
unsigned long unlockTime = 0;
bool autoLockPending = false;

/******** ヘルパー ********/
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return raw * (3.3 / 4095.0) * 2.0;
}

void flashGreenLED(int times = 3) {
  for (int i = 0; i < times; i++) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(200);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(200);
  }
}

void checkBatteryWarning() {
  if (readBatteryVoltage() < LOW_VOLTAGE_THRESHOLD) digitalWrite(RED_LED_PIN, HIGH);
  else digitalWrite(RED_LED_PIN, LOW);
}

bool inActiveTime(struct tm timeinfo) {
  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;
  int now = h * 60 + m;

  int morningStart = MORNING_START_HOUR * 60 + MORNING_START_MIN;
  int morningEnd   = MORNING_END_HOUR * 60 + MORNING_END_MIN;
  int eveningStart = EVENING_START_HOUR * 60 + EVENING_START_MIN;
  int eveningEnd   = EVENING_END_HOUR * 60 + EVENING_END_MIN;

  return (now >= morningStart && now <= morningEnd) ||
         (now >= eveningStart && now <= eveningEnd);
}
/******** ドア操作 ********/
void sendDoorStatus(); // 前方宣言

void unlockDoor() {
  Serial.println("🔓 ドア解錠！");
  lockServo.write(UNLOCK_ANGLE);
  delay(800);
  lockServo.write(UNLOCK_ANGLE);  // 念押しでもう一度角度指定

  flashGreenLED();
  lockState = "unlocked";
  sendDoorStatus();

  // 自動施錠タイマー開始
  unlockTime = millis();
  autoLockPending = true;
  lastActivity = millis();  // 最終操作時刻を更新
}

void lockDoor() {
  Serial.println("🔒 ドア施錠！");
  lockServo.write(LOCK_ANGLE);
  delay(800);
  lockServo.write(LOCK_ANGLE);    // 念押しでもう一度角度指定

  lockState = "locked";
  sendDoorStatus();
  lastActivity = millis();  // 最終操作時刻を更新
}

/******** MQTT コールバック ********/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📥 MQTTメッセージ受信: ");
  Serial.println(topic);

  // メッセージを文字列化
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  Serial.println("📩 内容: " + msg);

  // Try JSON
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    if (doc.containsKey("command")) {
      String command = doc["command"].as<String>();
      Serial.println("🛠 コマンド (JSON): " + command);

      if (command == "unlock") {
        unlockDoor();
        lastActivity = millis();
      }
      else if (command == "lock") {
        lockDoor();
        lastActivity = millis();
      }
      else if (command == "wake") {
        Serial.println("🌙 Wake 受信 → ESP32起動中に保持");
        flashGreenLED(2);
        lastActivity = millis();
      }
      else {
        Serial.println("⚠️ 未知のコマンド (JSON): " + command);
      }
      return;  // JSONの処理はここで終わり
    }

  }

  // フォールバック: プレーンテキスト "unlock" / "lock"
  if (msg == "unlock" || msg == "\"unlock\"") unlockDoor();
  else if (msg == "lock" || msg == "\"lock\"") lockDoor();
  else Serial.println("⚠️ 未知のコマンド: " + msg);
}

/******** MQTT 接続 ********/
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT接続中....");
    if (client.connect(mqttClientId)) {
      Serial.println("✅ MQTT接続成功");
      if (client.subscribe(subscribeTopic)) {
        Serial.print("📡 サブスクライブ成功: ");
        Serial.println(subscribeTopic);
      } else {
        Serial.println("❌ サブスクライブ失敗");
      }
    } else {
      Serial.print("❌ 接続失敗, state=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

/******** 送信 ********/
void sendDoorStatus() {
  if (!client.connected()) {
    Serial.println("⚠️ MQTT未接続: send skipped");
    return;
  }
  float v = readBatteryVoltage();
  StaticJsonDocument<200> doc;
  doc["state"] = lockState;
  doc["battery"] = v;
  char buf[200];
  size_t len = serializeJson(doc, buf);
  boolean ok = client.publish(publishTopic, buf, len);
  Serial.print("📤 状態送信: ");
  Serial.println(buf);
  if (!ok) Serial.println("❌ publish failed");
}

/******** setup / loop ********/
void setup() {
  Serial.begin(115200);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);

  WiFi.begin(ssid, password);
  Serial.println("WiFi 接続中...");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - start > 20000) {
      Serial.println("⚠️ WiFi接続タイムアウト");
      break;
    }
  }
  Serial.println("✅ WiFi接続成功");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("NTP同期中...");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
      Serial.println("⚠️ 時刻取得失敗");
  } else {
    Serial.printf("✅ 現在時刻: %02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min);
  }


  net.setCACert(caCert);
  net.setCertificate(clientCert);
  net.setPrivateKey(clientKey);

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.setKeepAlive(60);

  lockServo.attach(SERVO_PIN);
  lockDoor();
  connectMQTT();
  lastActivity = millis(); // 起動時に初期化
}

unsigned long lastSend = 0;

void loop() {
  if (!client.connected()) connectMQTT();
  client.loop();

  checkBatteryWarning();

  // NTPから時刻取得
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    isActiveTime = inActiveTime(timeinfo);
  }

  // 時間帯外なら Deep Sleep
  if (!isActiveTime) {
    Serial.println("🌙 時間帯外 → Deep Sleep移行");
    client.disconnect();
    WiFi.disconnect(true);

    esp_sleep_enable_timer_wakeup(5 * 60 * 1000000ULL); // 5分後に再起動して再チェック
    esp_deep_sleep_start();
  }

  // 時間帯内なら普通に動作
  if (millis() - lastSend > 5000) {
    sendDoorStatus();
    lastSend = millis();
  }

  // 自動施錠（30秒）
  if (autoLockPending && millis() - unlockTime >= 30000) {
    Serial.println("⏰ 自動施錠実行");
    lockDoor();
    autoLockPending = false;
  }
}
