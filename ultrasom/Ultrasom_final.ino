#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// WiFi Settings
const char* ssid = "testelouco";
const char* password = "doiseuros";

// MQTT Settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/ultrasonic/distance";

WiFiClient espClient;
PubSubClient client(espClient);

#define echoPin 2
#define trigPin 4
long duration, distance;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connect("ESP32DistanceSensor")) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32DistanceSensor")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Trigger measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2; // Distance in cm

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Convert to C string for MQTT
  char payload[16];
  dtostrf(distance, 0, 2, payload); // Format as float with 2 decimal places

  // Publish to MQTT
  if (client.publish(mqtt_topic, payload)) {
    Serial.println("MQTT message sent");
  } else {
    Serial.println("MQTT send failed");
  }

  delay(1000); 
}