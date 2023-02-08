/*
Programme pour verifier le fonctionnement a peu près apairé
des moteurs utilisés pour les roues, intégrant un codeur magnétique
moteur : https://www.aliexpress.com/item/32918974888.html?spm=a2g0o.order_list.0.0.21ef5e5bE0sntD 
Une même tension est appliquée aux deux moteurs, ils doivent tourner à la même vistesse. La
vitesse est mesurée par le nombre de ticks comptés sur intervalles de temps finis.
*/
/***********************************************************************************
 * 
 * ATTENTION : VOTRE MONTAGE INTEGRE UNE SOURCE DE PUISSANCE
 * VOUS DEVEZ IMPERATIVEMENT PRESERVER LES COMPOSANTS EN INTEGRANT
 * EN SERIE DANS VOTRE CIRCUIT UN FUSIBLE A FONTE RAPIDE
 * DE PLUS, DECOUPLEZ BIEN L'ALIMENTATION DE LA CARTE ARDUINO DE L'ALIMENTATION EN PUISSANCE
 * EN OTANT LE CAVALIER DU DRIVER (voir schéma de cablage)
 // pour le schéma de connexion, reportez vous aux documents 
 // dans le répertoire "cablage moteur à codeur intégré"
 * Une première manip réalisable pour vérifier les fonctionnement est la suivante :
 *   - chargez le code sur la carte arduino méga
 *   - débranchez la carte méga du PC pour passer en alimentation sur pile de celle-ci
 *   - ne branchez pas les moteurs
 *   - utilisez vos batteries avec un fusible en série
 *   - en fonctionnement, Verifiez les tensions sur les bornes de sortie du drive qui seront reliées aux moteurs
 *     A priori, à consigne égales, vous devez avoir à peu près les mêmes tensions.
*************************************************************************************/
/*
Pour le test, les moteurs n'ont pas besoin d'être montés effectivement sur le robot. 
La rotation des moteurs est mesurée ici par le nombre de ticks cumulés au fur et à mesure que le moteur tourne.
Idéalement :
- vous avez mesuré le nombre de ticks par tour de roue pour les deux moteurs (programme ComptTicks)
  Soit ND le nombre de ticks par tour du moteur droit
  Soit NG le nombre de ticks par tour du moteur droit
- à tension imposée constante et identique sur chaque moteur (ici je met 6 V, même batterie pour les deux)
  Si les moteurs tournent à la même vitesse, on devrait avoir en permanence exactement 
  un comptage alpha * ND sur le moteur droit et alpha * NG sur le moteur gauche (le même alpha)
  sur un temps de mesure identique (typiquement entre deux affichages).
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

// bibliothèque pour le pilotage des moteurs
#include <AFMotor.h>
AF_DCMotor motorDroit(4); // Moteur Droit : sur la carte,4 sorties pour 4 moteurs, le moteur droit est en M4 
AF_DCMotor motorGauche(3); // Moteur Gauche : sur la carte,4 sorties pour 4 moteurs, le moteur gauche est en M3

// Commande en puissance des moteurs 
// la commande va de -255 (vitesse max en arrière) à +255 (vitesse max en avant), 0 étant normalement stop
//        (attention, une vitesse trop faible, 10 ou 20 par exemple, ne permettra pas de faire tourner le moteur,
//         faute de tension au borne
//        (une commande de X amène à mettre TensionBatterie * X/255 Volt au borne du moteur (si le driver fonctionne bien))
//        (si le driver fonctionne bien, avec une même commande, vous devez avoir la même tension au bornes des moteurs)
int commandeDroit;
int commandeGauche;

// Définitions et déclarations pour un codeur magnétique du moteur Droit
#define codeurDroitInterruptionA 3    // interruption 3 = broche n°18 - Arduino MEGA2560 : fil 
#define codeurDroitInterruptionB 2    // interruption 2 = broche n°19 - Arduino MEGA2560
#define codeurDroitPinA 18            // Signal A du codeur magnétique Moteur Droit
#define codeurDroitPinB 19            // Signal B du codeur magnétique Moteur Droit
volatile long ticksCodeurDroit = 0;   //Compteur d'impulsions "ticks" codeur Moteur Droit
                                     // sur une période finie (sur TSDATA millisecondes (entre 2 affichages))

// Définitions et déclarations pour le codeur incrémental du moteur Gauche
#define codeurGaucheInterruptionA 4  // interruption 5 = broche n°20 - Arduino MEGA2560
#define codeurGaucheInterruptionB 5  // interruption 4 = broche n°21 - Arduino MEGA2560
#define codeurGauchePinA 20          // Signal A du codeur magnétique Moteur Gauche
#define codeurGauchePinB 21          // Signal B du codeur magnétique Moteur Gauche
volatile long ticksCodeurGauche = 0; //Compteur d'impulsions "ticks" codeur Moteur Gauche
                                     // sur une période finie (sur TSDATA millisecondes (entre 2 affichages))


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
  // les notions d'interruptions sont à découvrir dans le documentation arduino
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

  // commande (permanente pour les deux moteurs)

  // appel des fonctions de commande
  commandeDroit = 255;
  commandeGauche = 255;
  CommandeMoteurDroit(commandeDroit);
  CommandeMoteurGauche(commandeGauche); 
  
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
    Serial.print(ticksCodeurDroit);
    Serial.print(" ");
    Serial.print(ticksCodeurGauche);
    Serial.print("\n");
    // Une fois écrit, ce nombre est remis à 0
    ticksCodeurDroit = 0;
    ticksCodeurGauche = 0;

    tempsDernierEnvoi = tempsCourant;
  }
}

// fonction de commande des moteurs
void CommandeMoteurDroit(int commande) {
  // Cette fonction envoie les signaux PWM au pont en H
  // Saturation par sécurité
  if (commande > 255) {commande = 255;}
  if (commande < -255) {commande = -255;}
  // Commande PWM
  if (commande >= 0) {
    motorDroit.run(FORWARD); // on va en avant
    motorDroit.setSpeed(commande); // à cette vitesse là
  }
  if (commande < 0) {
    motorDroit.run(BACKWARD); // on va en arrière
    motorDroit.setSpeed(-commande); // à cette vitesse là
  }
}
void CommandeMoteurGauche(int commande) {
  // Cette fonction  envoie les signaux PWM au pont en H
  // Saturation par sécurité
  if (commande > 255) {commande = 255;}
  if (commande < -255) {commande = -255;}
  // Commande PWM
  if (commande >= 0) {
    motorGauche.run(FORWARD); // on va en avant
    motorGauche.setSpeed(commande); // à cette vitesse là
  }
  if (commande < 0) {
    motorGauche.run(BACKWARD); // on va en arrière
    motorGauche.setSpeed(-commande); // à cette vitesse là
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
