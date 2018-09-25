
const int OUTPUT_PIN = 13;
const int SWITCH_PIN = 11;

const int INTERRUPT_TIME_MS = 80;

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  digitalWrite(OUTPUT_PIN, LOW);
}

void loop() {
  if (digitalRead(SWITCH_PIN) == LOW) {
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(INTERRUPT_TIME_MS);
    digitalWrite(OUTPUT_PIN, LOW);
    delay(10000);
  }
}
