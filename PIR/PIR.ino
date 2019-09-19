#define pir  D8

int Spir, Dpir;
void setup() {
  // put your setup code here, to run once:
  pinMode(pir, INPUT_PULLUP);      // GPIO 5 Pir Sensor
  Serial.begin(9600);
}

void loop() {
  Spir = digitalRead(pir);  // Sensor pir
  if (Spir == 1) {
    Dpir = 1;
    Serial.print("Read digital input: ");
    Serial.println(Dpir);
  } else {
    Dpir = 0;
    Serial.print("Read digital input: ");
    Serial.println(Dpir);
  }
  delay(1000);
}
