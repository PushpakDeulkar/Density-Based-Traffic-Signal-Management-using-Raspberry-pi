#include <WiFi.h>
#include <esp_now.h>

// =========================================================
// RELAY PIN CONFIGURATION
// =========================================================
#define RED_PIN    27
#define YELLOW_PIN 26
#define GREEN_PIN  25

// Light timing durations (in seconds)
#define GREEN_TIME  5
#define YELLOW_TIME 3
#define RED_TIME    5

// Structure for ESP-NOW messages
typedef struct struct_message {
  char cmd[10];
} struct_message;

struct_message message;

// =========================================================
// CALLBACK: Data Sent Confirmation
// =========================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// =========================================================
// CALLBACK: Data Received from Master
// =========================================================
//  Updated for ESP-IDF v5 / Arduino Core v3.x compatibility
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));  // Copy data into structure
  Serial.print(" From Master: ");
  Serial.println(msg.cmd);

  // Check received command
  if (strcmp(msg.cmd, "START") == 0) {
    startCycle();
  } 
  else if (strcmp(msg.cmd, "RED") == 0) {
    setLight("RED");
  }
}

// =========================================================
// FUNCTION: Light Control
// =========================================================
void setLight(String color) {
  // Turn all lights OFF
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);

  // Turn ON only the requested color
  if (color == "RED") digitalWrite(RED_PIN, HIGH);
  else if (color == "YELLOW") digitalWrite(YELLOW_PIN, HIGH);
  else if (color == "GREEN") digitalWrite(GREEN_PIN, HIGH);

  Serial.println(" Light: " + color);
}

// =========================================================
// FUNCTION: Timer-Based Cycle (for Slave-controlled side)
// =========================================================
void startCycle() {
  Serial.println(" Timer-based sequence started...");

  // Step 1: GREEN
  setLight("GREEN");
  delay(GREEN_TIME * 1000);

  // Step 2: YELLOW
  setLight("YELLOW");
  delay(YELLOW_TIME * 1000);

  // Step 3: RED
  setLight("RED");
  delay(RED_TIME * 1000);

  // Notify Master that Slave cycle is complete
  strcpy(message.cmd, "DONE");
  esp_now_send(NULL, (uint8_t*)&message, sizeof(message)); // Broadcast to all peers
  Serial.println(" Cycle complete, notified master.");
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);

  // Set relay pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  // Initialize WiFi in Station Mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW init failed! Restarting...");
    ESP.restart();
  }

  // Register send & receive callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Slave Ready");
  setLight("RED");  // Default state
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  // Nothing here — waits for commands from Master
}