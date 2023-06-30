#include <DHT.h>
#include <Servo.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

#define WIFI_SSID "Wi-Fi 2"
#define WIFI_PASSWORD "123467890"

#define MQTT_SERVER "6f4d1220c6cb4852ab4266f96abc384e.s2.eu.hivemq.cloud"
#define MQTT_PORT 8883

#define MQTT_USERNAME "hivemq.webclient.1685212985222"
#define MQTT_PASSWORD "2X!M8.43LxhJ>AR;navy"

#define ROOM_PATH "ROOM_1"

// door lock
Servo doorLockServo;
#define SERVO_PIN 5
#define RELAY_PIN 32

bool prevIsDoorLocked = true;
bool isDoorLocked = true;

// leds
#define LED_0 25

#define freq 100
#define resolution 8

#define led0Channel 3

bool isLed0TurnedOn = 0;

int led0Brightness = 0;

unsigned long led0Timer = 0;

//DHT
#define DHTPIN 18
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
unsigned long DHTUpdateInterval = 60000;
unsigned long lastDHTUpdate = 0;     
bool isFirstDHTUpdate = true;

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

const char* ntpServer = "0.id.pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

unsigned long getTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return 0;
  }
  return (unsigned long)time(nullptr);
}

void setup() {
  Serial.begin(115200);

  // configure door lock functionalities
  doorLockServo.attach(SERVO_PIN);
  pinMode(RELAY_PIN, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(led0Channel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LED_0, led0Channel);

  //initialize DHT
  dht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println();
  Serial.print("Connected to WiFi. IP Address: ");
  Serial.println(WiFi.localIP());

  wifiClient.setCACert(root_ca);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker...");
      if (mqttClient.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("Connected to MQTT broker");
        mqttClient.subscribe(("/" + String(ROOM_PATH) + "/#").c_str());
      } else {
        Serial.print("Failed to connect to MQTT broker. Retrying...");
      }
    }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }

  mqttClient.loop();

  unsigned long currentTimestamp = getTime();

  //LED
  if(led0Timer) {
      Serial.println(currentTimestamp);
      Serial.println(led0Timer);
    if (currentTimestamp >= led0Timer) {
      led0Timer = 0;
      isLed0TurnedOn = !isLed0TurnedOn;
      mqttClient.publish(("/" + String(ROOM_PATH) + "/IS_LED_0_TURNED_ON").c_str(), isLed0TurnedOn ? "1" : "0", true);
      mqttClient.publish(("/" + String(ROOM_PATH) + "/LED_0_TIMER").c_str(), "0", true);
    }
  }

  if(isLed0TurnedOn) {
    ledcWrite(led0Channel, led0Brightness);
  } else {
    ledcWrite(led0Channel, 0);
  }

  // Door Lock
  if (prevIsDoorLocked != isDoorLocked) {
    prevIsDoorLocked = isDoorLocked; 
    digitalWrite(RELAY_PIN, HIGH); //Energize solenoid
    delay(500);
    if (isDoorLocked) {
      doorLockServo.write(180); // Sliding door to closed position
      Serial.println("Door locked");
    } else {
      doorLockServo.write(0); // Sliding door to open position
      Serial.println("Door unlocked");
    }
    delay(500);
    digitalWrite(RELAY_PIN, LOW); 
  }

  // Thermohygrometer
  if (isFirstDHTUpdate || (millis() - lastDHTUpdate >= DHTUpdateInterval)) {
    lastDHTUpdate = millis();  // Update the last DHT update time
    isFirstDHTUpdate = false;  // Set the first update flag to false

    // Read temperature and humidity data
    float hum = dht.readHumidity();
    float temp = dht.readTemperature();

    // Publish temperature and humidity data only if valid
    if (!isnan(temp) || !isnan(hum)) {
      mqttClient.publish(("/" + String(ROOM_PATH) + "/TEMPERATURE").c_str(), String(temp).c_str(), true);
      mqttClient.publish(("/" + String(ROOM_PATH) + "/HUMIDITY").c_str(), String(hum).c_str(), true);
      mqttClient.publish(("/" + String(ROOM_PATH) + "/DHT_LAST_UPDATE").c_str(), String(currentTimestamp).c_str(), true);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  if (topicStr == ("/" + String(ROOM_PATH) + "/IS_DOOR_LOCKED").c_str()) {
    isDoorLocked = payloadStr.toInt();
    Serial.print("Door lock state changed to: ");
    Serial.println(isDoorLocked);
  }

  if (topicStr == ("/" + String(ROOM_PATH) + "/IS_LED_0_TURNED_ON").c_str()) {
    isLed0TurnedOn = payloadStr.toInt();
    Serial.print("LED_0 state changed to: ");
    Serial.println(isLed0TurnedOn);
  }

  //Brightness
  if (topicStr == ("/" + String(ROOM_PATH) + "/LED_0_BRIGHTNESS").c_str()) {
    led0Brightness = payloadStr.toInt();
    Serial.print("LED 0 brightness state changed to: ");
    Serial.println(led0Brightness);
  }

  //Timer
  if (topicStr == ("/" + String(ROOM_PATH) + "/LED_0_TIMER").c_str()) {
    led0Timer = payloadStr.toInt();
    Serial.print("LED 0 timer state changed to: ");
    Serial.println(led0Timer);
  } 
}

void reconnect() {
  while (!mqttClient.connected()) {
  Serial.print("Connecting to MQTT broker...");
    if (mqttClient.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(("/" + String(ROOM_PATH) + "/#").c_str());
    } else {
      Serial.print("Failed to connect to MQTT broker. Retrying...");
    }
  }
}
