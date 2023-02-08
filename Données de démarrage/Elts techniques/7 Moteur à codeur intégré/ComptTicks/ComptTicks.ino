/*
Programme pour afficher le nombre de ticks comptés lors d'une rotation manuelle
des moteurs utilisés pour les roues, intégrant un codeur magnétique
moteur : https://www.aliexpress.com/item/32918974888.html?spm=a2g0o.order_list.0.0.21ef5e5bE0sntD 
*/
/* connection d'un moteur pour ce code :
* fil bleu : VCC de la carte arduino : alimentation pour le codeur  
* fil vert : GND de la carte arduino : alimentation pour le codeur
* fil jaune : Signal A du codeur magnétique
* fil blanc : Signal B du codeur magnétique
* fil rouge : non utilisé ici : alimentation + du moteur
* fil noir : non utilisé ici : alimentation - du moteur
*/

// bibliothèque de gestion de temps
#include <FlexiTimer2.h>



#define Nmoy 10            // Nombre de valeurs moyennées pour compter le nombre de ticks

// Définitions et déclarations pour un codeur magnétique du moteur Droit
#define codeurDroitInterruptionA 3    // interruption 3 = broche n°18 - Arduino MEGA2560 : fil 
#define codeurDroitInterruptionB 2    // interruption 2 = broche n°19 - Arduino MEGA2560
#define codeurDroitPinA 18            // Signal A du codeur magnétique Moteur Droit
#define codeurDroitPinB 19            // Signal B du codeur magnétique Moteur Droit
volatile long ticksCodeurDroit = 0;   //Compteur d'impulsions "ticks" codeur Moteur Droit
                                     // sur une période finie (sur CADENCE_MS millisecondes)

// Définitions et déclarations pour le codeur incrémental du moteur Gauche
#define codeurGaucheInterruptionA 4  // interruption 5 = broche n°20 - Arduino MEGA2560
#define codeurGaucheInterruptionB 5  // interruption 4 = broche n°21 - Arduino MEGA2560
#define codeurGauchePinA 20          // Signal A du codeur magnétique Moteur Gauche
#define codeurGauchePinB 21          // Signal B du codeur magnétique Moteur Gauche
volatile long ticksCodeurGauche = 0; //Compteur d'impulsions "ticks" codeur Moteur Gauche
                                     // sur une période finie (sur CADENCE_MS millisecondes)

// variables pour cumuler les nombre de ticks mesurés périodiquement sur une période finie
//    (tous les CADENCE_MS millisecondes)
int cumulTicksCodeurDroit = 0;
int cumulTicksCodeurGauche = 0;

// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms = cadence de mesure des ticks
#define CADENCE_MS 10
// variables utiles
volatile double dt = CADENCE_MS / 1000.;
volatile double temps = -CADENCE_MS / 1000.;

// Ce programme envoie par liaison série des données à l'ordinateur connecté à la carte arduino Mega.
// Cadence d'envoi des données en ms
#define TSDATA 1000
// variables utiles
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;

// set up du programme : initialisations
void setup() {
  // les notoiions d'interruptions sont à découvrir dans le documentation arduino
  // Exemples : https://www.locoduino.org/spip.php?article64 ou https://www.gcworks.fr/tutoriel/arduino/Lesinterruptions.html 
  
  //**** gestion des interruptions moteur droit
  // Codeur incrémental moteur droit
  pinMode(codeurDroitPinA, INPUT);      // entrée digitale pin A codeur
  digitalWrite(codeurDroitPinA, HIGH);  // activation de la résistance de pullup interne de l'arduino
  pinMode(codeurDroitPinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurDroitPinB, HIGH);  // activation de la résistance de pullup interne de l'arduino
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurDroitPinA (définie à la fin du programme)
  // cette fonction ne fait qu'incrémenter / décrémenter le nombre de bascules haut bas du signal
  attachInterrupt(digitalPinToInterrupt(codeurDroitPinA), GestionInterruptionCodeurDroitPinA, CHANGE);
  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinB (définie à la fin du programme)
  // cette fonction ne fait qu'incrémenter / décrémenter le nombre de bascules haut bas du signal
  attachInterrupt(digitalPinToInterrupt(codeurDroitPinB), GestionInterruptionCodeurDroitPinB, CHANGE);

  //**** gestion des interruptions moteur gauche
  // Codeur incrémental moteur gauche
  pinMode(codeurGauchePinA, INPUT);      // entrée digitale pin A codeur
  digitalWrite(codeurGauchePinA, HIGH);  // activation de la résistance de pullup
  pinMode(codeurGauchePinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurGauchePinB, HIGH);  // activation de la résistance de pullup
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinA (définie à la fin du programme)
  // cette fonction ne fait qu'incrémenter / décrémenter le nombre de bascules haut bas du signal
  attachInterrupt(digitalPinToInterrupt(codeurGauchePinA), GestionInterruptionCodeurGauchePinA, CHANGE);
  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinB (définie à la fin du programme)
  // cette fonction ne fait qu'incrémenter / décrémenter le nombre de bascules haut bas du signal
  attachInterrupt(digitalPinToInterrupt(codeurGauchePinB), GestionInterruptionCodeurGauchePinB, CHANGE);

  
  // gestion du temps : voir doc sur FlexiTimer2
  // la fonction isrt définie plus loin sera appelée automatiquement tout les CADENCE_MS millisecondes
  FlexiTimer2::set(CADENCE_MS, 1 / 1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();



  // init liaison série console
  Serial.begin(19200);  // pour transfert en affichage
  
}


// fonction loop appelée en continu
void loop() {
  // la loop ne fait rien.
  // la progression est assurée via les fonctions appelées lors des interruptions ou selon le timer
}


// Fonction excutée tous les CADENCE_MS millisecondes
void isrt() {

// Compteur de boucle et variable d'activation de l'asservissement
  int i;

  // Compteur de ticks moteur droit : ce cumul n'est utile que pour ce programme
  // Vous tournez la roue droite sur N tours à la main, et l'affichage vous donnera le nombre de ticks cumulTicksCodeurDroit correspondant
  // cumulTicksCodeurDroit / N vous donne le nombre de ticks par tour de votre roue droite
  cumulTicksCodeurDroit = cumulTicksCodeurDroit + ticksCodeurDroit;
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurDroit = 0;

  // Compteur de ticks moteur gauche : ce cumul n'est utile que pour ce programme
  // Vous tournez la roue droite sur N tours à la main, et l'affichage vous donnera le nombre de ticks cumulTicksCodeurGauche correspondant
  // cumulTicksCodeurGauche / N vous donne le nombre de ticks par tour de votre roue gauche
  cumulTicksCodeurGauche = cumulTicksCodeurGauche + ticksCodeurGauche;
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurGauche = 0;

  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Ecriture des données sur la liaison série pour monitoring
    ecritureData(); 

    // Incrémentation du temps courant
    temps += dt;
}


// Ecriture des données en sortie tous les TSDATA millisecondes
void ecritureData(void) {
  // On envoie les données s'il s'est écoulé plus de TSDATA secondes
  // entre l'instant courant et le dernier envoi
  // dans ce programme, on écrit le nombre de ticks cumulés
  tempsCourant = millis();
  if (tempsCourant - tempsDernierEnvoi > TSDATA) {
    Serial.print(temps);
    Serial.print(" ");
    Serial.print(cumulTicksCodeurDroit);
    Serial.print(" ");
    Serial.print(cumulTicksCodeurGauche);
    Serial.print("\n");

    tempsDernierEnvoi = tempsCourant;
  }
}




// fonctions de comptage des changements de niveau sur chaque codeur : roue droite
void GestionInterruptionCodeurDroitPinA() {
  // Routine de service d'interruption attachée à la voie A du codeur incrémental droite
  if (digitalRead(codeurDroitPinA) == digitalRead(codeurDroitPinB)) {
    ticksCodeurDroit++;
  }
  else {
    ticksCodeurDroit--;
  }
}
void GestionInterruptionCodeurDroitPinB() {
  // Routine de service d'interruption attachée à la voie B du codeur incrémental droite
  if (digitalRead(codeurDroitPinA) == digitalRead(codeurDroitPinB)) {
    ticksCodeurDroit--;
  }
  else {
    ticksCodeurDroit++;
  }
}

// fonctions de comptage des changements de niveau sur chaque codeur : roue gauche
void GestionInterruptionCodeurGauchePinA() {
  // Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
  if (digitalRead(codeurGauchePinA) == digitalRead(codeurGauchePinB)) {
    ticksCodeurGauche++;
  }
  else {
    ticksCodeurGauche--;
  }
}
void GestionInterruptionCodeurGauchePinB() {
  // Routine de service d'interruption attachée à la voie B du codeur incrémental gauche
  if (digitalRead(codeurGauchePinA) == digitalRead(codeurGauchePinB)) {
    ticksCodeurGauche--;
  }
  else {
    ticksCodeurGauche++;
  }
}
