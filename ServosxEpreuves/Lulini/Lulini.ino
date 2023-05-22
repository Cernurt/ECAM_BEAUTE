void setup() {
  Serial.begin(9600);
}

void loop() {
  int valeur = analogRead(A8);
  Serial.println(valeur);
}
