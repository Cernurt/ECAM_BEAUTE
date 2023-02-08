// Branchement et pilotage du driver polol, 2 moteurs A et B

// Référence : figure 
// "Minimal wiring diagram for connecting a microcontroller to a DRV8833 dual motor driver carrier."
// du site https://www.pololu.com/product/2130

// aussi https://arduino103.blogspot.com/2014/09/drv8833-piloter-facilement-des-moteurs.html

// avec les indications complémentaires suivantes :

// - Attention à correctement brancher les bornes de la batterie sur le composant : 
//     !! la sortie rouge de la batterie est le + et est branché sur la broche notée Vcc du composant (VMM sur la figure)
//     !! la sortie rouge de la batterie est le + et est branché aussi sur la broche notée VM du composant (BISEN sur la figure)
//     !! la sortie noire de la batterie est le - et est branché sur un GND du composant
//     !! la masse GND de la carte arduino est branchée sur l'autre GND du composant (besoin de masse commune)
//     Vin du composant non connecté, pas plus que MD du composant.
// Remarque : si vous n'avez pas besoin de puissance, le + rouge d'une batterie peut être remplacé par un 5V de l'arduino.

// - les moteurs sont connectés aux port O1 du composant (AOUT1 sur la figure)et O2 (AOUT2)pour le moteur A 
//         et O1 (BOUT1) O2 (BOUT2) pour le moteur B 

// PWM moteur A
// broches de pilotage du moteur A : sorties arduino 2 et 3 à brancher sur in2EN et IN1PH pour moteur A
int moteurA=2;
int moteurA2=3;

// broches de pilotage du moteur B : sorties arduino 4 et 5 à brancher sur in2EN et IN1PH pour moteur B
int moteurB=4;
int moteurB2=5;

// delai pour chaque action : moteur droite, moteur gauche, moteur éteint
int duree=1000;


void setup() {
  // déclarations des pins de sortie
  pinMode(moteurA,OUTPUT);
  pinMode(moteurB,OUTPUT);
  pinMode(moteurA2,OUTPUT);
  pinMode(moteurB2,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // La fonction suivante enchaine 
  //   des rotations des moteurs (droite gauche pour l'un, puis droite gauche pour l'autre

  tourner();
}

void tourner(){
  // moteur B stop
  analogWrite(moteurB,0);
  analogWrite(moteurB2,0);
  
  // moteur A tourne à droite
  analogWrite(moteurA,0);
  analogWrite(moteurA2,255); // vitesse max = valeur 255 - vitesse 0 = valeur 0
  delay(duree);

  // moteur A tourne à gauche
  analogWrite(moteurA,255);
  analogWrite(moteurA2,0);
  delay(duree);

  // moteur A stop
  analogWrite(moteurA,0);
  analogWrite(moteurA2,0);


  // moteur B tourne à droite
  analogWrite(moteurB,0);
  analogWrite(moteurB2,255);
  delay(duree);

  // moteur B tourne à gauche
  analogWrite(moteurB,255);
  analogWrite(moteurB2,0);
  delay(duree);


}
