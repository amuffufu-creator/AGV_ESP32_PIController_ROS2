#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h> // Needed for Serial2
#include "thijs_rplidar.h"  // Include the Lidar library
#include <vector>           // Needed for lidar data storage if batching more complexly
#include <RPLidar.h>


// ===============================================================
// ==               NETWORK CONFIGURATION                       ==
// ===============================================================
const char* ssid = "YOUR_WIFI_SSID";         // <-- REPLACE
const char* password = "YOUR_WIFI_PASSWORD"; // <-- REPLACE

WiFiUDP Udp;
IPAddress pcIpAddress = IPAddress(0,0,0,0); // Initialize to invalid
const char* pcIpString = "192.168.100.71"; // <-- REPLACE with your PC's actual IP address

const int pcOdomPort = 8888;  // Port on PC for odometry
const int pcLidarPort = 8890; // Port on PC for Lidar
const int localUdpPort = 8889;  // Port on ESP32 for commands

// ===============================================================
// ==             LIDAR CONFIGURATION & PINS                    ==
// ===============================================================
HardwareSerial& lidarSerial = Serial2;
const int lidarRxPin = 16;
const int lidarTxPin = 17;
const int lidarMotorPin = 4;
RPLidar lidar;
unsigned long last_lidar_send_time = 0;
const int LIDAR_SEND_INTERVAL_MS = 100; // Send lidar data batches ~10 times/sec

// ===============================================================
// ==           MOTOR/ENCODER PINS & ISRs (Corrected)           ==
// ===============================================================
const int ENA_PIN = 14; 
const int IN1_PIN = 27; 
const int IN2_PIN = 26; 
const int ENB_PIN = 32;
const int IN3_PIN = 25;
const int IN4_PIN = 33;
const int ENCODER_A_A_PIN = 18; // Right motor
const int ENCODER_A_B_PIN = 19;
const int ENCODER_B_A_PIN = 22; // Left motor
const int ENCODER_B_B_PIN = 23;
volatile long motorA_counts = 0;
volatile long motorB_counts = 0;

// Correct ISR logic confirmed in previous steps
void IRAM_ATTR motorA_ISR() { // Right motor ISR
  if (digitalRead(ENCODER_A_B_PIN) == LOW) { motorA_counts++; } else { motorA_counts--; }
}
void IRAM_ATTR motorB_ISR() { // Left motor ISR
  if (digitalRead(ENCODER_B_B_PIN) == LOW) { motorB_counts++; } else { motorB_counts--; }
}

// ===============================================================
// ==        PI CONTROLLER VARIABLES (Values might need tuning) ==
// ===============================================================
float Kp = 0.03; // Tuned value from previous steps
float Ki = 0.05; // Tuned value from previous steps
float target_velocity_A = 0.0;
float target_velocity_B = 0.0;
long prevMotorA_counts = 0;
float motorA_pwm = 0.0;
float error_integral_A = 0.0;
long prevMotorB_counts = 0;
float motorB_pwm = 0.0;
float error_integral_B = 0.0;
unsigned long prev_pid_time_ms = 0;
float integral_limit = 400.0;
unsigned long prev_odom_time_ms = 0;

// ===============================================================
// ==                      SETUP FUNCTION                       ==
// ===============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Starting AGV Firmware with Lidar (Corrected)...");

  // --- Connect to WiFi with Timeout ---
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) { // 10-second timeout
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection FAILED. Check credentials or signal.");
    // Decide what to do here - maybe loop forever, maybe try to run offline?
    // For now, we'll just print the error.
  }

  // --- Initialize UDP ---
  Udp.begin(localUdpPort);
  Serial.printf("Listening for UDP commands on port %d\n", localUdpPort);
  pcIpAddress.fromString(pcIpString);

  // --- Initialize Lidar ---
  Serial.println("Initializing Lidar...");
  lidarSerial.begin(115200, SERIAL_8N1, lidarRxPin, lidarTxPin);
  pinMode(lidarMotorPin, OUTPUT);
  digitalWrite(lidarMotorPin, LOW);
  
  if (lidar.begin(lidarSerial, lidarMotorPin)) {
      Serial.println("Lidar initialized.");
      lidar.startMotor();
      delay(1000); 
      if (lidar.startScan()) {
          Serial.println("Lidar scan started.");
      } else {
          Serial.println("Error starting Lidar scan.");
      }
  } else {
      Serial.println("Error initializing Lidar.");
  }

  // --- Standard motor/encoder setup ---
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  const int PWM_FREQUENCY = 5000;   
  const int PWM_RESOLUTION = 8;     
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION); ledcAttachPin(ENA_PIN, 0);
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION); ledcAttachPin(ENB_PIN, 1);
  pinMode(ENCODER_A_A_PIN, INPUT_PULLUP); pinMode(ENCODER_A_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_A_PIN), motorA_ISR, RISING);
  pinMode(ENCODER_B_A_PIN, INPUT_PULLUP); pinMode(ENCODER_B_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_A_PIN), motorB_ISR, RISING);
  motorA_counts = 0; motorB_counts = 0;
  prev_pid_time_ms = millis();
  prev_odom_time_ms = millis();
  Serial.println("Setup Complete.");
}

// ===============================================================
// ==                         MAIN LOOP                         ==
// ===============================================================
void loop() {
  unsigned long now = millis();

  // --- Part 1: Listen for UDP Commands (Safe Buffer Read) ---
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Learn the PC's IP only if we haven't already or if it changes (unlikely)
    IPAddress currentRemoteIp = Udp.remoteIP();
    if (pcIpAddress == IPAddress(0,0,0,0) || pcIpAddress != currentRemoteIp) {
        pcIpAddress = currentRemoteIp;
        Serial.print("Learned PC IP Address: ");
        Serial.println(pcIpAddress);
    }

    char incomingPacket[255]; // Buffer size
    int len = Udp.read(incomingPacket, 254); // Read up to buffer_size - 1
    if (len > 0) {
      incomingPacket[len] = '\0'; // Safe null termination
    }
    String command(incomingPacket);
    
    if (command.startsWith("s")) {
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
            target_velocity_A = command.substring(firstComma + 1, secondComma).toFloat();
            target_velocity_B = command.substring(secondComma + 1).toFloat();
            // Optional: Print received targets
            // Serial.printf("New Targets: A=%.0f, B=%.0f\n", target_velocity_A, target_velocity_B);
        }
    }
  }
  
  // --- Part 2: Run PI Controllers (every 100ms) ---
   const int PID_INTERVAL_MS = 100;
  if (now - prev_pid_time_ms >= PID_INTERVAL_MS) {
    // PI calculations remain the same
    long current_ticks_A, current_ticks_B;
    noInterrupts(); current_ticks_A = motorA_counts; current_ticks_B = motorB_counts; interrupts();
    float delta_time_s = (float)(now - prev_pid_time_ms) / 1000.0;
    float velocity_A = (float)(current_ticks_A - prevMotorA_counts) / delta_time_s;
    float error_A = abs(target_velocity_A) - abs(velocity_A);
    error_integral_A += (error_A * delta_time_s); error_integral_A = constrain(error_integral_A, -integral_limit, integral_limit);
    float base_pwm_A = (target_velocity_A == 0) ? 0 : 180;
    motorA_pwm = base_pwm_A + (Kp * error_A) + (Ki * error_integral_A); motorA_pwm = constrain(motorA_pwm, 0, 255);
    float velocity_B = (float)(current_ticks_B - prevMotorB_counts) / delta_time_s;
    float error_B = abs(target_velocity_B) - abs(velocity_B);
    error_integral_B += (error_B * delta_time_s); error_integral_B = constrain(error_integral_B, -integral_limit, integral_limit);
    float base_pwm_B = (target_velocity_B == 0) ? 0 : 180;
    motorB_pwm = base_pwm_B + (Kp * error_B) + (Ki * error_integral_B); motorB_pwm = constrain(motorB_pwm, 0, 255);
    prevMotorA_counts = current_ticks_A; prevMotorB_counts = current_ticks_B; prev_pid_time_ms = now;
  }

  // --- Part 3: Send Odometry via UDP (every 100ms, with Timestamp) ---
   const int ODOM_INTERVAL_MS = 100;
  if (now - prev_odom_time_ms >= ODOM_INTERVAL_MS && WiFi.status() == WL_CONNECTED) {
    long current_ticks_A, current_ticks_B;
    noInterrupts(); current_ticks_A = motorA_counts; current_ticks_B = motorB_counts; interrupts();
    
    char odomBuffer[60]; // Increased buffer size slightly for timestamp
    sprintf(odomBuffer, "o,%ld,%ld,%lu\n", current_ticks_A, current_ticks_B, now); // Added timestamp (millis())

    if (pcIpAddress != IPAddress(0,0,0,0)) { 
        Udp.beginPacket(pcIpAddress, pcOdomPort);
        Udp.write((uint8_t*)odomBuffer, strlen(odomBuffer));
        Udp.endPacket();
    }
    prev_odom_time_ms = now;
  }

  // --- Part 4: Read Lidar Data, Batch, and Send via UDP ---
  lidar.run(); // Needed for thijses/rplidar library
  
  // Check if it's time to send a batch of lidar data
  if (lidar.isScanning() && (now - last_lidar_send_time >= LIDAR_SEND_INTERVAL_MS) && WiFi.status() == WL_CONNECTED) {
    rplidar_response_measurement_node_t nodes[360]; // Buffer for one scan rotation
    size_t count = 360; // Max number of points to grab

    // Attempt to grab available scan data points
    if (IS_OK(lidar.grabScanData(nodes, count))) { // count might be updated with actual number read
        lidar.ascendScanData(nodes, count); // Sort data by angle

        if (count > 0 && pcIpAddress != IPAddress(0,0,0,0)) {
            char lidarBuffer[1024]; // Large buffer for batching
            int offset = 0;
            offset += sprintf(lidarBuffer + offset, "l,"); // Start of lidar message

            // Add up to ~40 points per packet to stay within safe UDP size limits
            int points_in_packet = 0; 
            for (int i = 0; i < count; i++) {
                int angle = (int)(nodes[i].angle_q6_checkbit / 64.0f + 0.5f);
                int distance = (int)(nodes[i].distance_q2 / 4.0f + 0.5f); 

                // Add point to buffer if valid distance and buffer has space
                if (distance > 0 && (offset < sizeof(lidarBuffer) - 20)) { // Check buffer space (-20 for safety)
                    offset += sprintf(lidarBuffer + offset, "%d,%d;", angle, distance);
                    points_in_packet++;
                }

                // Send packet if it's full enough or if it's the last point
                if (points_in_packet >= 40 || (i == count - 1 && points_in_packet > 0) ) {
                    lidarBuffer[offset-1] = '\n'; // Replace last ';' with newline
                    lidarBuffer[offset] = '\0';   // Null terminate

                    Udp.beginPacket(pcIpAddress, pcLidarPort);
                    Udp.write((uint8_t*)lidarBuffer, offset);
                    Udp.endPacket();
                    
                    // Reset buffer for next packet
                    offset = 0; 
                    offset += sprintf(lidarBuffer + offset, "l,");
                    points_in_packet = 0;
                    // Small delay to prevent flooding router/network? Optional.
                    // delay(1); 
                }
            }
             last_lidar_send_time = now; // Update time only after successfully sending
        }
    } else {
       // Optional: Print error if grabScanData fails
       // Serial.println("Failed to grab scan data"); 
    }
  }

  // --- Part 5: Apply Power to Motors ---
  // Motor A (Right)
  if (target_velocity_A > 0) { digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH); } 
  else if (target_velocity_A < 0) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); }
  ledcWrite(0, (target_velocity_A == 0) ? 0 : (int)motorA_pwm);
  // Motor B (Left)
  if (target_velocity_B > 0) { digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, HIGH); } 
  else if (target_velocity_B < 0) { digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW); }
  ledcWrite(1, (target_velocity_B == 0) ? 0 : (int)motorB_pwm);
}