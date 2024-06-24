int pin5 = 5;
int pin3 = 3;
int pin7 = 7;

void setup() {
  Serial.begin(9600);
  pinMode(pin5, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin7, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    int command = Serial.parseInt();
    digitalWrite(pin5, LOW);
    digitalWrite(pin3, LOW);
    digitalWrite(pin7, LOW);

    switch (command) {
      case 5:
        digitalWrite(pin5, HIGH);
        break;
      case 3:
        digitalWrite(pin3, HIGH);
        break;
      case 7:
        digitalWrite(pin7, HIGH);
        break;
      default:
        break;
    }
  }
}
