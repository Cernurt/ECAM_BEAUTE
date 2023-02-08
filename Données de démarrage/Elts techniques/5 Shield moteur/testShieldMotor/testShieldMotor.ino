// Programme super simple pour piloter 4 moteurs, sans asservissement.
// Cela permet de a minima de tester vos moteurs et votre shield, en lien avec vos cablages
//     Si un moteur ne tourne pas, c'est peut-être un moteur ou un composant défectueux. Faites des tests après vérification de vos cablages.
//     Si un moteur ne tourne pas dans le sens qui semble devoir être celui programmé, vérifiez / modifiez vos connexions moteur

// D'autres codes sont par exemple disponible https://www.lextronic.fr/shield-controleur-de-moteurs-40423.html

// bibliothèque utilisable pour le pilotage du shield moteur
#include <AFMotor.h>

// définition de moteurs
AF_DCMotor motor1(1); //Moteur 1 
AF_DCMotor motor2(2); //Moteur 2
AF_DCMotor motor3(3); //Moteur 3 
AF_DCMotor motor4(4); //Moteur 4

// Remarque 1 : si vous mettez le moteur 1 sur la roue droite d'un véhicule et le moteur 2 sur la seconde roue,
//   et qu'ils sont classiquement montés tête bêche, il faudra les piloter en rotation inverse pour que le 
//   véhicule avance et ne tourne pas.



void setup() {
  // rien de particulier à initialiser ici
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.println("Motor test!");
}

void loop() {
      // tout les moteurs au max de vitesse en rotation dans un même sens
      // vitesse entre -255 et 255 (PWM). 
      // Attention, une valeur trop faible (en valeur absolue) ne démarre pas les moteurs, il y a un effet de seuil
      CommandeMoteur1(255);
      CommandeMoteur2(255);    
      CommandeMoteur3(255);
      CommandeMoteur4(255);
      Serial.println("Tous en avant !");    
 delay(3000); // attente 3 seconde et stop moteur
 // remarque : pour stopper un moteur net, on peut aussi mettre un petit coup en arrière très court
      CommandeMoteur1(0);
      CommandeMoteur2(0);    
      CommandeMoteur3(0);
      CommandeMoteur4(0);
      Serial.println("Tous en stop !");   
 delay(500); // attente 0.5 seconde
    // tout les moteurs au max de vitesse en rotation en sens inverse        
      CommandeMoteur1(-255);
      CommandeMoteur2(-255);    
      CommandeMoteur3(-255);
      CommandeMoteur4(-255);
      Serial.println("Tous en arrière !");
 delay(3000); // attente 3 secondes et stop
      CommandeMoteur1(0);
      CommandeMoteur2 (0);    
      CommandeMoteur3(0);
      CommandeMoteur4(0); 
      Serial.println("Tous en stop !");  
 delay(500); // attente 0.5 seconde
  
}

void CommandeMoteur1(double commande) {
  // Commande PWM
  if (commande >= 0) {
    motor1.run(FORWARD);
    motor1.setSpeed(commande);
  }
  if (commande < 0) {
    motor1.run(BACKWARD);
    motor1.setSpeed(-commande);
  }
}

void CommandeMoteur2(double commande) {
  // Commande PWM
  if (commande >= 0) {
    motor2.run(FORWARD);
    motor2.setSpeed(commande);
  }

  if (commande < 0) {
    motor2.run(BACKWARD);
    motor2.setSpeed(-commande);
  }
}

void CommandeMoteur3(double commande) {
  // Commande PWM
  if (commande >= 0) {
    motor3.run(FORWARD);
    motor3.setSpeed(commande);
  }
  if (commande < 0) {
    motor3.run(BACKWARD);
    motor3.setSpeed(-commande);
  }
}

void CommandeMoteur4(double commande) {
  // Commande PWM
  if (commande >= 0) {
    motor4.run(FORWARD);
    motor4.setSpeed(commande);
  }
  if (commande < 0) {
    motor4.run(BACKWARD);
    motor4.setSpeed(-commande);
  }
}
