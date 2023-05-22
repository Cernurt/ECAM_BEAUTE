void setup() {
  Serial.begin(9600);
}

void loop() {
  int valeur = analogRead(A7);
  Serial.println(valeur);
}
