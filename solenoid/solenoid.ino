#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#else
#error "Board not found"
#endif

#include <PubSubClient.h>

#define Relay1            15
#define Relay2            13
#define Relay3            14
#define Relay4            18
#define Relay5            19
#define Relay6            21
#define Relay7            22
#define Relay8            23
#define Relay9            25
#define Relay10           26
#define Relay11           27
#define Relay12           32

const int relayPins[] = {Relay1, Relay2, Relay3, Relay4, Relay5, Relay6, Relay7, Relay8, Relay9, Relay10, Relay11, Relay12};

// Update these with values suitable for your network.

const char* ssid = "Buttered Crayfish";
const char* password = "!T06MYTA9G2G";
const char* mqtt_server = "test.mosquitto.org"; // Local IP address of Raspberry Pi

// Subscribed Topics

#define sub1 "relay/sump"

#define sub2 "relay/vortex"
#define sub3 "relay/mecha"
#define sub4 "relay/bio"

#define sub5 "relay/waste1"
#define sub6 "relay/waste2"
#define sub7 "relay/waste3"

#define sub8 "relay/azolaIn"
#define sub9 "relay/azolaOut"

#define sub10 "relay/aerator"

#define sub11 "relay/aeratorSensor"
#define sub12 "relay/pump"

const char* subscribedTopics[] = {sub1, sub2, sub3, sub4, sub5, sub6, sub7, sub8, sub9, sub10, sub11, sub12};

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
bool state = false;

// Connecting to WiFi Router

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < sizeof(subscribedTopics) / sizeof(subscribedTopics[0]); ++i) {
    if (strstr(topic, subscribedTopics[i])) {
      toggleRelay(topic, payload);
      break;
    }
  }
}

void toggleRelay(char* topic, byte* payload) {
  for (int i = 0; i < sizeof(subscribedTopics) / sizeof(subscribedTopics[0]); ++i) {
    if (strstr(topic, subscribedTopics[i])) {
      int relayIndex = i; // Use the index of the matched topic
      if ((char)payload[0] == '1') {
        digitalWrite(relayPins[relayIndex], LOW);
      } else {
        digitalWrite(relayPins[relayIndex], HIGH);
      }
      break;
    }
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      // Subscribe to all topics
      for (const char* topic : subscribedTopics) {
        client.subscribe(topic);
      }

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(9600);

  for (int i = 0; i < sizeof(relayPins) / sizeof(relayPins[0]); ++i) {
    pinMode(relayPins[i], OUTPUT);
  }

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop()
{

  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

}