#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

typedef struct struct_message {
  bool L1Pressed;
  bool L2Pressed;
  bool L3Pressed;
  bool L4Pressed;
  bool R1Pressed;
  bool R2Pressed;
  bool R3Pressed;
  bool R4Pressed;
  int joy1_x;
  int joy1_y;
  int joy2_x;
  int joy2_y;    
} struct_message;

struct_message receivedData;

static const int servoLeftFootPin = 13;   // 360° continuous rotation servo
static const int servoLeftLegPin = 14;    // Standard 180° servo
static const int servoRightFootPin = 12;  // 360° continuous rotation servo 
static const int servoRightLegPin = 27;   // Standard 180° servo

Servo servoLeftFoot;
Servo servoLeftLeg;
Servo servoRightFoot;
Servo servoRightLeg;

bool wasR1Pressed = false;
bool wasR3Pressed = false;
bool wasR4Pressed = false;
bool wasL1Pressed = false;
bool wasL2Pressed = false;
bool wasL3Pressed = false;
bool wasL4Pressed = false;
bool wasR2Pressed = false;
bool R3StateActive = false;
bool manualOverride = false;

const int JOYSTICK_DEADZONE = 10;
const int MAX_SERVO_SPEED = 180;

int LNP = 65;     // Left leg neutral position
int RNP = 117;    // Right leg neutral position

int LL = 100;   // Left leg position LEFT TURN
int LR = 175;   // Right leg position LEFT TURN
int RL = 5;    // Left leg position RIGHT TURN
int RR = 80;    // Right leg position RIGHT TURN

void moveServosSmooth(Servo &servo1, int start1, int end1, Servo &servo2, int start2, int end2, int steps, int delayTime) {
  int diff1 = end1 - start1;
  int diff2 = end2 - start2;
  for (int i = 0; i <= steps; i++) {
    int pos1 = start1 + (diff1 * i / steps);
    int pos2 = start2 + (diff2 * i / steps);
    servo1.write(pos1);
    servo2.write(pos2);
    delay(delayTime);
  }
}

void moveServoSmooth(Servo &servo1, int start1, int end1, int steps, int delayTime) {
  int diff = end1 - start1;

  for (int i = 0; i <= steps; i++) {
    int pos = start1 + (diff * i / steps);
    servo1.write(pos);
    delay(delayTime);
  }
}

void returnToNeutral() {
  int currentLeftLeg = servoLeftLeg.read();
  int currentRightLeg = servoRightLeg.read();
  
  moveServosSmooth(servoLeftLeg, currentLeftLeg, LNP, servoRightLeg, currentRightLeg, RNP, 20, 15);
  
  servoLeftFoot.write(90);
  servoRightFoot.write(90);
}

int mapJoystickToSpeed(int value) {
  if (abs(value) < JOYSTICK_DEADZONE) {
      return 90;  // Środek – zatrzymanie dla serwa 360°
  }

  // Mapowanie nowego zakresu joysticka (-100 do 100) na zakres serwa (0 do 180)
  int mappedSpeed = map(value, -100, 100, 0, 180);
  return mappedSpeed;
}

void rightLegSwing() {
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 100, servoRightLeg, servoRightLeg.read(), 175, 20, 15); // Pozycja lewa
  delay(150);
  servoLeftFoot.write(90+20);  // Obrót lewej stopy serwo 360

  moveServoSmooth(servoRightLeg, 175, 60, 20, 10);
  delay(100);

  for (int i = 0; i < 4; i++) {  
    moveServoSmooth(servoRightLeg, 60, 120, 20, 10);  
    delay(100);
    moveServoSmooth(servoRightLeg, 120, 60, 20, 10);  
    delay(100);
  }

  moveServoSmooth(servoRightLeg, 60, 175, 20, 10);  
  delay(100);
}

void moonWalk() {
  for (int i = 0; i < 1; i++) { 
    moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), RL, servoRightLeg, servoRightLeg.read(), RR, 35, 20);
    moveServoSmooth(servoLeftLeg, servoLeftLeg.read(), LL, 35, 20);
    moveServoSmooth(servoLeftLeg, servoLeftLeg.read(), RL, 35, 20);
    moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), LL, servoRightLeg, servoRightLeg.read(), LR, 35, 20);
    moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), RL, servoRightLeg, servoRightLeg.read(), RR, 35, 20);
    moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), LL, servoRightLeg, servoRightLeg.read(), LR, 35, 20);
    moveServoSmooth(servoRightLeg, servoRightLeg.read(), RR, 35, 20);
    moveServoSmooth(servoRightLeg, servoRightLeg.read(), LR, 35, 20);
  }
}

void steps() {
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 40, servoRightLeg, servoRightLeg.read(), 90, 30, 15);
  delay(100);
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 90, servoRightLeg, servoRightLeg.read(), 142, 30, 15);
  delay(100);
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 40, servoRightLeg, servoRightLeg.read(), 90, 30, 15);
  delay(100);
  moveServoSmooth(servoRightLeg, servoRightLeg.read(), 142, 30, 15);
  delay(100);
  moveServoSmooth(servoLeftLeg, servoLeftLeg.read(), 90, 30, 15);
  delay(500);
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 40, servoRightLeg, servoRightLeg.read(), 90, 30, 15);
  delay(100);
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 90, servoRightLeg, servoRightLeg.read(), 142, 30, 15);
  delay(100);
  moveServoSmooth(servoLeftLeg, servoLeftLeg.read(), 40, 30, 15);
  delay(100);
  moveServoSmooth(servoRightLeg, servoRightLeg.read(), 90, 30, 15);
  delay(500);
  moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 90, servoRightLeg, servoRightLeg.read(), 142, 30, 15);
  delay(100);
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  if (!manualOverride) {
    int leftFootSpeed = mapJoystickToSpeed(receivedData.joy1_y);
    int rightFootSpeed = mapJoystickToSpeed(receivedData.joy2_y);
    
    servoLeftFoot.write(leftFootSpeed);
    servoRightFoot.write(180 - rightFootSpeed);
  }

  // R3 handling (square button)
  if (receivedData.R3Pressed) {
    if (!wasR3Pressed) {
      wasR3Pressed = true;
      if (!R3StateActive) {
        moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 170, servoRightLeg, servoRightLeg.read(), 10, 20, 15);
        R3StateActive = true;
      } else {
        returnToNeutral();
        R3StateActive = false;
      }
    }
  } else {
    wasR3Pressed = false;
  }

  // R1 button handling 
  if (receivedData.R1Pressed) {
    if (!wasR1Pressed) {
      manualOverride = true;  // Blokowanie joysticka
      moveServosSmooth(servoLeftLeg, servoLeftLeg.read(), 100, servoRightLeg, servoRightLeg.read(), 175, 20, 15); // Pozycja idąca lewa
      delay(150);
      servoLeftFoot.write(90 + 25); // Obrót w prawo, ustawienie prędkości serwa 360
      wasR1Pressed = true;
    }
  } else if (wasR1Pressed) {
    servoLeftFoot.write(90); // Natychmiastowe zatrzymanie obrotu
    delay(150);
    returnToNeutral();
    manualOverride = false; // Odblokowanie joysticka
    wasR1Pressed = false;
  }

  // R2 button handling 
  if (receivedData.R2Pressed) {
    if (!wasR2Pressed) {
      manualOverride = true;  // Blokowanie joysticka
      moveServosSmooth(servoRightLeg, servoRightLeg.read(), 80, servoLeftLeg, servoLeftLeg.read(), 5, 20, 15);  // Pozycja idąca prawo
      delay(150);
      servoRightFoot.write(90 - 20); // Obrót w lewo, ustawienie prędkości serwa 360
      wasR2Pressed = true;
    }
  } else if (wasR2Pressed) {
    servoRightFoot.write(90); // Natychmiastowe zatrzymanie obrotu
    delay(150);
    returnToNeutral();
    manualOverride = false; // Odblokowanie joysticka
    wasR2Pressed = false;
  }

  // L1 button handling 
  if (receivedData.L1Pressed) {
    if (!wasL1Pressed) {
      manualOverride = true;  // Blokowanie joysticka
      steps();
      wasL1Pressed = true;
    }
  } else if (wasL1Pressed) {
    returnToNeutral();  // Powrót do neutralnej pozycji po zakończeniu ruchów
    manualOverride = false; // Odblokowanie joysticka
    wasL1Pressed = false;
  }

  // L4 button handling 
  if (receivedData.L4Pressed) {
    if (!wasL4Pressed) {
      manualOverride = true;
      moonWalk();
      wasL4Pressed = true;
    }
  } else if (wasL4Pressed) {
    returnToNeutral();  // Powrót do neutralnej pozycji po zakończeniu ruchów
    manualOverride = false; // Odblokowanie joysticka
    wasL4Pressed = false;
  }

    // L3 button handling 
    if (receivedData.L3Pressed) {
      if (!wasL3Pressed) {
        manualOverride = true;
        rightLegSwing();
        wasL3Pressed = true;
      }
    } else if (wasL3Pressed) {
      returnToNeutral();  // Powrót do neutralnej pozycji po zakończeniu ruchów
      manualOverride = false; // Odblokowanie joysticka
      wasL3Pressed = false;
    }
  

  Serial.println("Stan przycisków:");
  Serial.printf("L1: %d, L2: %d, L3: %d, L4: %d\n", 
      receivedData.L1Pressed, receivedData.L2Pressed, 
      receivedData.L3Pressed, receivedData.L4Pressed);
  Serial.printf("R1: %d, R2: %d, R3: %d, R4: %d\n", 
      receivedData.R1Pressed, receivedData.R2Pressed, 
      receivedData.R3Pressed, receivedData.R4Pressed);

}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  

  if (esp_now_init() != ESP_OK) {
      Serial.println("Błąd inicjalizacji ESP-NOW!");
      return;
  }

  esp_now_register_recv_cb(onDataRecv);

  servoLeftFoot.attach(servoLeftFootPin, 544, 2400);
  servoRightFoot.attach(servoRightFootPin, 544, 2400);
  
  servoLeftLeg.attach(servoLeftLegPin, 544, 2400);
  servoRightLeg.attach(servoRightLegPin, 544, 2400);

  servoLeftFoot.write(90);    // Stop - no rotation
  servoRightFoot.write(90);   // Stop - no rotation
  
  servoLeftLeg.write(LNP);     // Position in degrees
  servoRightLeg.write(RNP);   // Position in degrees

  Serial.println("Ready.");

  delay(300);
}

void loop() {
  // Pusta pętla, dane są obsługiwane w callbacku
}