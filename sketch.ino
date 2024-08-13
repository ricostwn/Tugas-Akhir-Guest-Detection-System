#define BLYNK_TEMPLATE_ID "TMPL6Be0WZbfS"
#define BLYNK_TEMPLATE_NAME "Gerakan Servo"
#define BLYNK_AUTH_TOKEN "BqXX3OI6Y_XUlMCFDzvDG7jyxsPl2_ba"

#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>

const int trigPin = 12;  // Pin Trigger sensor ultrasonik
const int echoPin = 14;  // Pin Echo sensor ultrasonik
const int ledPin = 13;   // Pin LED
const int servoPin = 15; // Pin motor servo
const int buzzerPin = 4; // Pin buzzer

Servo myservo;

bool personDetected = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 1000; // Duration for the buzzer to sound in milliseconds

// WiFi and MQTT configuration
const char* ssid = "Wokwi-GUEST";  // Replace with your WiFi SSID
const char* password = "";  // Replace with your WiFi password
const char* mqtt_server = "public.mqtthq.com";
const int mqtt_port = 1883;
const char* distance_topic = "tekkom/undip/bell/jarak";  // Replace with your MQTT topic for distance
const char* status_topic = "tekkom/undip/bell/status";  // Replace with your MQTT topic for status

WiFiClient espClient;
PubSubClient client(espClient);

BlynkTimer timer;

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
  // Loop until connection is successful
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Set Last Will and Testament (LWT) message
    const char* lwtMessage = "ESP32 Terputus";

    // Try to connect
    if (client.connect("ESP32Client", distance_topic, 2, true, lwtMessage)) {
      Serial.println("connected");

      // If connected, publish connected status to both topics with QoS 2
      client.publish(status_topic, (const uint8_t*)"ESP32 Terhubung", strlen("ESP32 Terhubung"), true);
      client.publish(distance_topic, (const uint8_t*)"ESP32 Terhubung", strlen("ESP32 Terhubung"), true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      // If connection fails, publish disconnected status to both topics with QoS 2
      client.publish(status_topic, (const uint8_t*)"ESP32 Terputus", strlen("ESP32 Terputus"), true);
      client.publish(distance_topic, (const uint8_t*)"ESP32 Terputus", strlen("ESP32 Terputus"), true);

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Function to control servo from Blynk slider
BLYNK_WRITE(V1) {
  int position = param.asInt(); // Get slider value
  myservo.write(position); // Set servo position
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  myservo.attach(servoPin);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  timer.setInterval(1000L, checkDistance); // Check distance every second
}

void checkDistance() {
  long duration, distance;

  // Send trigger signal
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Receive echo signal
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance based on echo time
  distance = (duration * 0.0343) / 2;

  Serial.print("Jarak: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Publish distance to MQTT with QoS 2
  char distanceStr[16];
  snprintf(distanceStr, sizeof(distanceStr), "%ld cm", distance);
  client.publish(distance_topic, (const uint8_t*)distanceStr, strlen(distanceStr), true); // Publish with retain flag and QoS 2

  // Set conditions to activate LED and buzzer
  if (distance <= 200) {
    digitalWrite(ledPin, HIGH);  // Turn on LED

    if (!personDetected) {
      personDetected = true;
      buzzerStartTime = millis();
      tone(buzzerPin, 1000); // Start the buzzer
      Serial.println("Ada orang masuk");
      client.publish(status_topic, (const uint8_t*)"ada orang masuk", strlen("ada orang masuk"), true); // Publish with retain flag and QoS 2
    }

    // Turn off the buzzer after the duration has passed
    if (millis() - buzzerStartTime >= buzzerDuration) {
      noTone(buzzerPin);
    }
  } else {
    digitalWrite(ledPin, LOW);   // Turn off LED
    noTone(buzzerPin);
    if (personDetected) {
      personDetected = false;
      Serial.println("Tidak ada orang masuk");
      client.publish(status_topic, (const uint8_t*)"tidak ada orang masuk", strlen("tidak ada orang masuk"), true); // Publish with retain flag and QoS 2
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  Blynk.run();
  timer.run();
}
