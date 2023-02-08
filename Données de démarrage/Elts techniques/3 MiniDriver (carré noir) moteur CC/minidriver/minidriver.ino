// la broche vcc du driver recoit l'alimentation qui servira à faire tourner les moteurs :
//    - cette aliomentation peut être le +5 de l'arduino si le moteur est un petit moteur ne demandant pas de puissance
//    - cette alimentation sera  le +6 (rouge) de la batterie 6V si vous voulez un peu de puissance
// La broche gnd du driver reçoit le gnd de l'arduino (plus le noir de la batterie si puissance délivrée par batterie : la masse commune est importante)

// programmation ci-dessous pour 1 seul moteur. Vous pouvez piloter 2 moteur avec ce matériel.
 
// les pins 2 et 3 de l'arduino sur lesquelles sont mises les consignes sont reliées
//    aux broches in1 et in2 du driver
// les in1 et in2 permettent de piloter le moteur relié en out1 et out2 dans un sens ou dans l'autre.
int moteurA1=2;
int moteurA2=3;

// delai pour chaque action : moteur droite, moteur gauche, moteur éteint
int duree=2000;

void setup() {
  // déclaration des ports arduino en "sortie"
  pinMode(moteurA1,OUTPUT);
  pinMode(moteurA2,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // lancement de la fonction tourner
  tourner();
}

void tourner(){

  Serial.println("Tourner à droite");
  digitalWrite(moteurA1,0);
  digitalWrite(moteurA2,120); // valeur max 255
  delay(duree);

  digitalWrite(moteurA1,0);
  digitalWrite(moteurA2,0);
  Serial.println("stop");
  delay(duree);
  
  digitalWrite(moteurA1,120);
  digitalWrite(moteurA2,0);
  Serial.println("Tourner à gauche");
  delay(duree);
  
  digitalWrite(moteurA1,0);
  digitalWrite(moteurA2,0);
  Serial.println("stop");
  delay(duree);


}
