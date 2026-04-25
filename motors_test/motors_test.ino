// L298N Motor Pins
#define ENA 5
#define IN1 8
#define IN2 9
#define ENB 6
#define IN3 11
#define IN4 12

// Encoder Pins (Nano Interrupts are 2 and 3)
#define M1_ENCODER_A 2
#define M1_ENCODER_B 4
#define M2_ENCODER_A 3
#define M2_ENCODER_B 7

// Volatile variables for interrupt safety
volatile long leftTicks = 0;
volatile long rightTicks = 0;

void setup() {
  Serial.begin(115200); // Higher baud rate for faster printing

  // Motor Pin Modes
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Encoder Pin Modes
  pinMode(M1_ENCODER_A, INPUT_PULLUP);
  pinMode(M1_ENCODER_B, INPUT_PULLUP);
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
  pinMode(M2_ENCODER_B, INPUT_PULLUP);

  // Attach Interrupts for both wheels
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), countRight, RISING);

  // Move Forward Command
  moveForward(180); 
}

void loop() {
  // Print values to Serial Monitor
  Serial.print("Left: ");
  Serial.print(leftTicks);
  Serial.print(" | Right: ");
  Serial.println(rightTicks);
  
  delay(100); // Update display every 100ms
}

// --- Helper Functions ---

void countLeft() {
  if (digitalRead(M1_ENCODER_B) == LOW) leftTicks++;
  else leftTicks--;
}

void countRight() {
  if (digitalRead(M2_ENCODER_B) == HIGH) rightTicks++;
  else rightTicks--;
}

void moveForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}
