#include <WiFi.h>
#include <esp_now.h>

// ================================
// Wi-Fi Credentials (for RPi link)
// ================================
const char* ssid = "";       // <-- Replace with your Wi-Fi SSID
const char* password = ""; // <-- Replace with your Wi-Fi Password

// ================================
// Slave MAC Address
// ================================
uint8_t slaveAddress[] = {0xCC, 0x7B, 0x5C, 0x35, 0xB0, 0x84};

// ================================
// Relay Pins
// ================================
#define RED_PIN    2
#define YELLOW_PIN 4
#define GREEN_PIN  16

// Light Timings (in seconds)
#define YELLOW_TIME 3
#define RED_TIME    5

// State Variables
String currentState = "RED";
bool canOperate = true;  // True when slave finished its cycle

// ================================
// ESP-NOW Message Structure
// ================================
typedef struct struct_message {
  char cmd[10];
} struct_message;

struct_message message;

// ================================
// CALLBACKS
// ================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success " : "Fail ");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));

  Serial.print(" From Slave: ");
  Serial.println(msg.cmd);

  if (strcmp(msg.cmd, "DONE") == 0) {
    canOperate = true; // Slave finished its cycle
    Serial.println(" Slave finished. Master re-enabled.");
  }
}

// ================================
// LIGHT CONTROL
// ================================
void setLight(String color) {
  // Turn all off first
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);

  // Turn on selected color
  if (color == "RED") digitalWrite(RED_PIN, HIGH);
  else if (color == "YELLOW") digitalWrite(YELLOW_PIN, HIGH);
  else if (color == "GREEN") digitalWrite(GREEN_PIN, HIGH);

  currentState = color;
  Serial.println(" Light: " + color);
}

// Send command to Slave via ESP-NOW
void sendToSlave(const char* command) {
  strcpy(message.cmd, command);
  esp_now_send(slaveAddress, (uint8_t*)&message, sizeof(message));
}

// ================================
// SETUP
// ================================
void setup() {
  Serial.begin(115200);

  // Relay pins as output
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  // Connect to Wi-Fi (for RPi communication)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print(" Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n Wi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW initialization failed!");
    ESP.restart();
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add slave as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(" Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }

  Serial.println(" Master Ready");
  setLight("RED");
}

// ================================
// LOOP
// ================================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (!canOperate) {
      Serial.println(" Waiting for slave to finish...");
      return;
    }

    if (cmd == "GREEN") {
      setLight("GREEN");
      sendToSlave("RED"); // Slave stays red while master green
    } 
    else if (cmd == "YELLOW") {
      setLight("YELLOW");
      delay(YELLOW_TIME * 1000);
      setLight("RED");
      delay(RED_TIME * 1000);

      // Notify slave to start its cycle
        sendToSlave("GREEN");   