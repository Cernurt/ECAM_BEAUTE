/**************************************************************************************************************/
/* Commande de la vitesse linéaire et de la vitesse de rotation du robot mobile à deux roues (type unicycle)  
/* Conduite différentielle - orientation du robot.                                                            
 *  Ce programme implémente un asservissement de type PI
 */
/* NOTE IMPORTANTE                                                              
 *  Ce programme est le programme d'un robot de l'année passé qui maitrisait parfaitement ses déplacements.
 *  Ce robot utilisait un driver moteur différent de celui qui vous est fourni de base. Ceci vous impose 
 *  quelques modifications de ce programme pour correspondre à vos driver : Le programme n'utilise pas la 
 *  bibliothèque AFmotor.h ==> 
 *    - les "// Définitions et déclarations pour les moteurs à courant continu" ne sont plus utiles
 *      les instructions 
 *          digitalWrite(directionMoteurGaucheouDroite, .);
 *          analogWrite(pwmMoteurGaucheouDroite, .);
 *      sont à remplacer par les instructions (inclure AFmotor.h)
 *      motorGaucheouDroite.run(.); // choix du sens de rotation
 *      motorGaucheouDroite.setSpeed(.); // indication de la vitesse
 *   - freinMoteurGaucheouDroite ; // le frein n'existe pas dans votre driver
 *   
 *  Ce programme ne correspond évidemment pas à votre robot ni à ses trajectoires mais peut vous aider. 
 *  Les constantes de l'algorithme en particulier seront à adapter éventuellement en fonction de votre robot.
 *  Idem certaines dimensions. Idem les branchements des codeurs, ...
 *  Prenez le temps de bien lire en détail ce code pour l'adapter, cette adaptation est obligatoire 
 *  sinon cela ne marchera pas.
 *  
 *  Pour une exploitation progressive, faites des tests simples au départ : tout droit et je n'arrête,
 *     tout droit, je m'arrête et je repars puis m'arrête à nouveau, puis introduisez un virage, etc.
 *  
 *  Versionnez votre code. Vous devez pouvoir revenir à un code précédent qui fonctionnait.
 *  En cas de comportement étrange, essayez de mettre des affichages (fonction ecritureData) qui par
 *    exemple validerait que vous n'êtes pas en saturation et donc algorithme ne fonctionnant plus.
 */
/**************************************************************************************************************/
#include <FlexiTimer2.h>

// ATTENTION: donner la bonne valeur de la tension d'alimentation sur la ligne ci-dessous. Ici, 12V
double tensionAlim = 12.;

int Start = 10;         // Capteur mécanique de Start déclaré connecté broche 10
boolean autorisation = false ;    // mémoire de l'autorisation de la commande du robot


// Définitions et déclarations pour le codeur magnétique du moteur Droit (MA)
#define codeurDroitInterruptionA 3  // interruption 3 = broche n°20 - Arduino MEGA2560
#define codeurDroitInterruptionB 2  // interruption 2 = broche n°21 - Arduino MEGA2560
#define codeurDroitPinA 20          // Signal A du codeur magnétique Moteur Droit
#define codeurDroitPinB 21          // Signal B du codeur magnétique Moteur Droit
#define Nmoy 10
volatile long ticksCodeurDroit = 0; //Compteur d'impulsions "ticks" codeur Moteur Droit
static int indiceTicksCodeurDroit = 0;
static int ticksCodeurDroitTab[Nmoy];
#define codeurGauchePinA 18          // Signal A du codeur magnétique Moteur Gauche
#define codeurGauchePinB 19          // Signal B du codeur magnétique Moteur Gauche
volatile long ticksCodeurGauche = 0; //Compteur d'impulsions "ticks" codeur Moteur Gauche
static int codeurDroitDeltaPos;

// Définitions et déclarations pour le codeur incrémental du moteur Gauche (MB)
#define codeurGaucheInterruptionA 5  // interruption 5 = broche n°18 - Arduino MEGA2560
#define codeurGaucheInterruptionB 4  // interruption 4 = broche n°19 - Arduino MEGA2560
static int indiceTicksCodeurGauche = 0;
static int ticksCodeurGaucheTab[Nmoy];
static int codeurGaucheDeltaPos;

const float rapport_reducteur = 43.7;  // Rapport de réduction du réducteur accouplé au moteur

const int res_codeur_voieAB = 64;      // nombre d'impulsion cumulées sur les deux voies A et B pour 1 tour du codeur magnétique (16 pôles)

// Définitions et déclarations pour les moteurs à courant continu
int directionMoteurDroit = 12 ; // sens de rotation Moteur Droit en broche 12
int pwmMoteurDroit = 3;         // PWMA de commande de vitesse Moteur Droit en broche 3
int freinMoteurDroit = 9;       // Commande de freinage Moteur Droit en broche 9

int directionMoteurGauche = 13; // sens de rotation Moteur Gauche en broche 13
int pwmMoteurGauche = 11;       // PWMB de commande de vitesse Moteur Gauche en broche 11
int freinMoteurGauche = 8;      // Commande de freinage Moteur Gauche en broche 8


// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS / 1000.;
volatile double temps = -CADENCE_MS / 1000.;


// Ce programme envoie par liaison série des données à l'ordinateur connecté à la carte arduino Mega.
// Cadence d'envoi des données en ms
#define TSDATA 1000
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;

// On déclare ci-dessous les variables et paramètres nécessaires au traitement de l'information
static double R = 0.0445;      // Rayon d'une roue en m
static double L = 0.196;       // Largeur du robot en m (entre les deux roues)
static double pi = 3.141592 ; // valeur de pi

// Déclaration des variables concernant la commande et la mesure de vitesse du moteur
static double umax = 12.; // valeur max de la tension de commande du moteur
static double umin = -12.; // valeur min (ou max en négatif) de la tension de commande du moteur
static double vxmes = 0.; // vitesse longitudinale mesurée
static double xidotmes = 0.; // vitesse de rotation mesurée
static double Kpvx = 9.; // gain proportionnel pour l'asservissement de vitesse longitudinale
static double Kivx = 85.; // gain intégral pour l'asservissement de vitesse longitudinale
static double Kpxidot = 1.1; // gain proportionnel pour l'asservissement de rotation
static double Kixidot = 9.1; // gain intégral pour l'asservissement de rotation
static double commande_avant_sat_vx = 0.; // commande avant la saturation pour l'asservissement de vitesse longitudinale
static double commande_vx = 0.; // commande pour l'asservissement de vitesse longitudinale
static double commande_avant_sat_xidot = 0.; // commande avant la saturation pour l'asservissement de rotation
static double commande_xidot = 0.; // commande pour l'asservissement de rotation
static double P_vx = 0.; // action proportionnelle pour l'asservissement de vitesse longitudinale
static double I_vx = 0.; // action intégrale pour l'asservissement de vitesse longitudinale
static double P_xidot = 0.; // action proportionnelle pour l'asservissement de rotation
static double I_xidot = 0.; // action intégrale pour l'asservissement de rotation

// Déclarations pour la commande des moteurs
static double omegaDroit = 0.;    // rotation angulaire de la roue Droite en rd/s
static double omegaGauche = 0.;   // rotation angulaire de la roue Gauche en rd/s
static double commandeDroit = 0.;
static double commandeGauche = 0.;

// Déclarations pour des inforamtions de vitesse (non nécessaire au fonctionnement du programme)
static double NDroit = 0.;      // vitesse de rotation de la roue Droite en tr/min
static double NGauche = 0.;     // vitesse de rotation de la roue Gauche en tr/min
static double VDroit = 0.;      // vitesse lineaire de la roue droite en m/s
static double VGauche = 0.;     // vitesse lineaire de la roue Gauche en m/s

// Déclarations pour les consignes de mouvement
static double vxref = 0.;        // vitesse de déplacement en m/s (maximum 1.18m/s)
static double xidotref = 0.;     // orientation du robot en rd/s  (maximum 3 rd/s)
static double Tmax = 0.;          // temps du cycle de déplacement

// Déclarations des temps pour le déplacement du robot (float car plus rapide et moins lourd que double)
float TL1 = 1.5033; // temps de déplacement linéaire de départ
float TR1 = 1.6008; // temps de déplacement rotation pour rentrer dans tp lum
float TL2 = 1.5; // temps de déplacement linéaire vers tp lum
float TARR1 = 3.; // temps de d'arret 1
float TACT1 = 1.8; // temps de deplacement vers interrupteurs
float TARR2 = 6.; // temps d'arret2
float T1 = 0.;
float T2 = 0.;
float TREC1 = 0.6;; // temps de déplacement action1
float TR2 = 2.25 ; //temps de rotation après remise en place au tp lum
float TL3 = 2.1; //temps de ligne droite vers tp mat
float TR3 = 0.78; // temps de rotation pour s'aligner a la barre de tp
float TACT2 = 0.8; // temps de ligne droite pour faire le tp mat
float TR4 = 1.108; //temps de rotation pour sortir du tp mat
float TL4 = 3.; //temps de ligne droite pour faire présentiel en tp MSC
float TR5 = 0.58; // temps de rotation pour faire le palet de présentiel du tp MSC
float TL5 = 1.65; // temps de ligne droite pour faire le palet de présentiel du tp MSC
float TR6 = 3.4; // temps de rotation pour ressortir du tp MSC
float TL6 = 1.55; // temps pour se caler au tp GI
float TACT3 = 25.; // temps d'arret pour faire le tp GI
float TL7 = 1.3; // temps pour aller faire les palets
float TR7 = 1.1; // temps pour aller faire les palets
float TACT4 = 1.; // temps pour emmener les palets
float TREC2 = TACT4; // temps pour reculer après les palets
float TR8 = 3.; // temps pour tourner après les palets
float TL8 = 0.77; //temps pour aller faire action de comm
float TACT5 = 20.;//temps pour faire action de comm
float TACT6 = 8.;//temps pour faire funny action
float Ttot = TL1 + TR1 + TL2 + TR2 + TL3 + TR3 + TACT2 + TR4 + TL4 + TR5 + TL5 + TR6 + TL6 + TACT3 + TL7 + TR7 + TACT4 + TREC2 + TR8 + TL8 + TACT5 + TACT6 ; // temps de déplacement total


// sorties permettant de piloter le démarrage d'actions gérées par une autre carte
int TPGI = 34;
int actCom = 36;
int funAct = 38;

char Direction = ""; 

// Initialisations

void setup(void) {

  // Codeur incrémental moteur droit
  pinMode(codeurDroitPinA, INPUT);      // entrée digitale pin A codeur
  digitalWrite(codeurDroitPinA, HIGH);  // activation de la résistance de pullup interne de l'arduino
  pinMode(codeurDroitPinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurDroitPinB, HIGH);  // activation de la résistance de pullup interne de l'arduino
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurDroitPinA (définie à la fin du programme)
  attachInterrupt(codeurDroitInterruptionA, GestionInterruptionCodeurDroitPinA, CHANGE);
  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinB (définie à la fin du programme)
  attachInterrupt(codeurDroitInterruptionB, GestionInterruptionCodeurDroitPinB, CHANGE);

  // Codeur incrémental moteur gauche
  pinMode(codeurGauchePinA, INPUT);      // entrée digitale pin A codeur
  digitalWrite(codeurGauchePinA, HIGH);  // activation de la résistance de pullup
  pinMode(codeurGauchePinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurGauchePinB, HIGH);  // activation de la résistance de pullup
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinA (définie à la fin du programme)
  attachInterrupt(codeurGaucheInterruptionA, GestionInterruptionCodeurGauchePinA, CHANGE);
  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinB (définie à la fin du programme)
  attachInterrupt(codeurGaucheInterruptionB, GestionInterruptionCodeurGauchePinB, CHANGE);

  pinMode(Start, INPUT);      // bouton de Start
  digitalWrite (Start, HIGH); // activation de la résistance de pullup


  // Moteur à courant continu droit
  pinMode(directionMoteurDroit, OUTPUT);
  pinMode(pwmMoteurDroit, OUTPUT);
  pinMode(freinMoteurDroit, OUTPUT);

  digitalWrite(directionMoteurDroit, 0);  // a l'arret
  digitalWrite(directionMoteurDroit, HIGH);
  digitalWrite(freinMoteurDroit, LOW);  // frein off



  // Moteur à courant continu gauche
  pinMode(directionMoteurGauche, OUTPUT);
  pinMode(pwmMoteurGauche, OUTPUT);
  pinMode(freinMoteurGauche, OUTPUT);

  digitalWrite(directionMoteurGauche, 0);  // a l'arret
  digitalWrite(directionMoteurGauche, LOW);
  digitalWrite(freinMoteurGauche, LOW);  // frein off

  // Liaison série.
  Serial.begin(9600);

  // Compteur d'impulsions des encodeurs
  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;

  /* Le programme principal est constitué:
     - de la traditionnelle boucle "loop" des programmes Arduino, dans laquelle on envoi en permanance la commande en tension du moteur via le potentiomètre de commande
     - de la fonction "isrt", dont l'exécution est définie à cadence fixe par les deux instructions
       ci-dessous. La bonne méthode pour exécuter une fonction à cadence fixe, c'est de l'exécuter sur interruption
       on n'obtiendra jamais une exécution à cadence fixe.
       Il est nécessaire d'exécuter la fonction "isrt" à cadence fixe car cette fonction:
       - doit renvoyer des données à cadence fixe au programme exécuté sur l'ordinateur connecté à la carte
       - calcule la vitesse de rotation du moteur en comptant le nombre d'impulsions du codeur pendant un temps fixe

  */
  FlexiTimer2::set(CADENCE_MS, 1 / 1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

  
  //Déclaration des variables actions
  pinMode(TPGI,OUTPUT);
  digitalWrite(TPGI,LOW);
  pinMode(actCom,OUTPUT);
  digitalWrite(actCom,LOW);
  pinMode(funAct,OUTPUT);
  digitalWrite(funAct,LOW);
}


// Boucle principale
void loop() {

  ecritureData();
  if (digitalRead(Start) == LOW) // Si il y a un appui sur le Bouton Poussoir
  {
    autorisation = !autorisation; // change l'état de l'autorisation
    delay(500);
  }

}

// Fonction excutée sur interruption
void isrt() {

  // Compteur de boucle et variable d'activation de l'asservissement
  int i;

  // Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurDroitPinA et GestionInterruptionCodeurDroitPinA,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i = 0; i < Nmoy; i++) {
    ticksCodeurDroitTab[i] += ticksCodeurDroit;
  }
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurDroit = 0;

  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurDroitDeltaPos = ticksCodeurDroitTab[indiceTicksCodeurDroit];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurDroitTab[indiceTicksCodeurDroit] = 0;

  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurDroit++;
  if (indiceTicksCodeurDroit == Nmoy) {
    indiceTicksCodeurDroit = 0;
  }

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurGauchePinA et GestionInterruptionCodeurGauchePinA,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i = 0; i < Nmoy; i++) {
    ticksCodeurGaucheTab[i] += ticksCodeurGauche;
  }
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurGauche = 0;

  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurGaucheDeltaPos = ticksCodeurGaucheTab[indiceTicksCodeurGauche];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurGaucheTab[indiceTicksCodeurGauche] = 0;

  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurGauche++;
  if (indiceTicksCodeurGauche == Nmoy) {
    indiceTicksCodeurGauche = 0;
  }

  // Calcul de la vitesse de rotation. C'est le nombre d'impulsions converti en radian, divisé par la période d'échantillonnage
  // On fait un calcul glissant sur Nmoy échantillons, d'où le Nmoy*dt
  omegaDroit = 1 * ((2.*pi * ((double)codeurDroitDeltaPos)) / (res_codeur_voieAB * rapport_reducteur)) / (Nmoy * dt); // en rad/s
  omegaGauche = -1 * ((2.*pi * ((double)codeurGaucheDeltaPos)) / (res_codeur_voieAB * rapport_reducteur)) / (Nmoy * dt); // en rad/s


  // Consignes
  // - vxref permet de définir la consigne de vitesse longitudinale (en m/s)
  //   vxref doit rester inférieur à 0.8 m/s
  // - xidotref permet de définir la consigne de vitesse de rotation (rad/s) autour de l'axe vertical
  //   xidotref doit rester inférieur à 3 rad/s

  // Ci-dessous, le robot décrit un 8

  if (autorisation)
  {
    if (temps > Tmax) {
      Tmax = temps + Ttot + 0.5; // Le mvt dure 39.5 s
    }
    else if (temps < (Tmax - (Ttot - TL1 + 0.5))) { // La ligne droite dure 1.3833s
      vxref = 0.3;
      xidotref = 0.;
    } 
    else if (temps < (Tmax - (Ttot  - TL1 - TR1 + 0.5))){ // tourne pendant 1.5708 s 
      vxref = 0.;
      xidotref = 0.97; 
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 + 0.5))) { // La ligne droite dure 1.s
     vxref = 0.3;
     xidotref = 0.;
    } 
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 + 0.5))) { // La rotation après remise en place au tp lumiere dure 1.38s 
     vxref = 0.05;
     xidotref = -0.95;
    }
    
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 + 0.5))) { // La ligne droite pour aller au tp mat dure 2.1s
     vxref = 0.3;
     xidotref = 0.;
    }  
    
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 + 0.5))) { // La rotation pour s'aligner au tp mat dure 0.58s
     vxref = 0.;
     xidotref = -0.97;
    }  
    
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 + 0.5))) { //  tp mat dure 0.6s
     vxref = 0.3;
     xidotref = 0.;
    }  
    
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 + 0.5))) { // la rotation pour sortir du tp mat dure 0.58s
     vxref = 0.1;
     xidotref = -0.97;
    }  
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 + 0.5))) { // la ligne droite pour faire le présentiel tp ms dure 3s
     vxref = 0.3;
     xidotref = 0.;
    } 
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 + 0.5))) { // la rotation pour faire le présentiel tp ms dure 0.38s
     vxref = 0.;
     xidotref = -0.97;
    } 
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 + 0.5))) { // la ligne droite pour faire le présentiel tp ms dure 0.38s
     vxref = 0.3;
     xidotref = 0.;
    } 
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 + 0.5))) { // la rotation pour sortir du tp ms dure 1s
     vxref = 0.05;
     xidotref = 0.97;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 + 0.5))) { // la ligne droite pour aller au tp GI dure 1.2s
     vxref = 0.3;
     xidotref = 0.;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 + 0.5))) { // la ligne droite pour aller au tp GI dure 1.2s
     vxref = 0.;
     xidotref = 0.;
     digitalWrite(TPGI,HIGH);
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 + 0.5))) { // la ligne droite pour aller aux palets dure 1.5s
     vxref = 0.3;
     xidotref = 0.25;
     digitalWrite(TPGI,LOW);
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 + 0.5))) { // la rotation pour aller aux palets dure 0.78s
     vxref = 0.;
     xidotref = -0.97;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 - TACT4 + 0.5))) { // l'action pour mettre les palets dure 1.2s
     vxref = 0.2;
     xidotref = 0.;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 - TACT4 - TREC2 + 0.5))) { // le recul pour sortir des palets dure 0.4s
     vxref = - 0.2;
     xidotref = 0.;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 - TACT4 - TREC2 - TR8 + 0.5))) { // la rotation pour aller a l'action de comm dure 1.2s
     vxref = 0.;
     xidotref = 0.97;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 - TACT4 - TREC2 - TR8 - TL8 + 0.5))) { // la ligne droite pour aller au tp GI dure 1.2s
     vxref = 0.2;
     xidotref = -0.01;
    }
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 - TACT4 - TREC2 - TR8 - TL8 - TACT5 + 0.5))) { // le stop pour action de comm' dure
     digitalWrite(actCom,HIGH);
     vxref = 0.;
     xidotref = 0.;
    }    
    else if (temps < (Tmax - (Ttot - TL1 - TR1 - TL2 - TR2 - TL3 - TR3 - TACT2 - TR4 - TL4 - TR5 - TL5 - TR6 - TL6 - TACT3 - TL7 - TR7 - TACT4 - TREC2 - TR8 - TL8 - TACT5 - TACT6 + 0.5))) { // le stop pour funact dure
     digitalWrite(actCom,LOW);
     digitalWrite(funAct,HIGH);
     vxref = 0.;
     xidotref = 0.;
    }  
    else { // désactivation de autorisation et arrêt car temps = Tmax - 0,5s
      vxref = 0.;
      xidotref = 0.;
      autorisation = !autorisation; // change l'état de l'autorisation
    }
 }

  // Définition des entrées de la fonction d'asservissement
  vxmes = (omegaDroit + omegaGauche) * R / 2;
  xidotmes = (omegaDroit - omegaGauche) * R / L;

  /******* Calcul du PI sur vx *******/

  // Terme proportionnel (la transformation de la commande par retour d'état en PI
  // conduit à une référence nulle, d'où le 0.*vxref)
  P_vx = Kpvx * (0.*vxref - vxmes);

  // Calcul de la commande avant saturation
  commande_vx = P_vx + I_vx;


  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_vx = I_vx + Kivx * dt * (vxref - vxmes);

  /******* Fin Calcul du PI sur vx *******/

  /******* Calcul du PI sur xidot *******/
  // Terme proportionnel
  P_xidot = Kpxidot * (xidotref - xidotmes);

  // Calcul de la commande avant saturation
  commande_xidot = P_xidot + I_xidot;


  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_xidot = I_xidot + Kixidot * dt * (xidotref - xidotmes);

  /******* Fin Calcul du PI sur xidot *******/


  // Calcul des commandes des moteurs
  commandeDroit = (commande_vx + commande_xidot);
  commandeGauche = (commande_vx - commande_xidot);

  // Application des commandes aux moteurs
  CommandeMoteurDroit(commandeDroit, tensionAlim);
  CommandeMoteurGauche(commandeGauche, tensionAlim);

  // Ecriture des données sur la liaison série pour monitoring
  ecritureData();

  // Incrémentation du temps courant
  temps += dt;

}




void CommandeMoteurDroit(double commande, double tensionBatterie) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int tension_int;
  double tension;

  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  tension_int = (int)(255 * (commande / tensionBatterie));

  // Saturation par sécurité
  if (tension_int > 255) {
    tension_int = 255;
  }

  if (tension_int < -255) {
    tension_int = -255;
  }

  // Commande PWM
  if (tension_int >= 0) {
    digitalWrite(directionMoteurDroit, LOW);
    analogWrite(pwmMoteurDroit, tension_int);
  }

  if (tension_int < 0) {
    digitalWrite(directionMoteurDroit, HIGH);
    analogWrite(pwmMoteurDroit, -tension_int);
  }
}

void CommandeMoteurGauche(double commande, double tensionBatterie) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int tension_int;
  double tension;

  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  tension_int = (int)(255 * (commande / tensionBatterie));

  // Saturation par sécurité
  if (tension_int > 255) {
    tension_int = 255;
  }

  if (tension_int < -255) {
    tension_int = -255;
  }

  // Commande PWM
  if (tension_int >= 0) {
    digitalWrite(directionMoteurGauche, HIGH);
    analogWrite(pwmMoteurGauche, tension_int);
  }

  if (tension_int < 0) {
    digitalWrite(directionMoteurGauche, LOW);
    analogWrite(pwmMoteurGauche, -tension_int);
  }
}


void ecritureData(void) {
  // Ecriture des données en sortie tous les TSDATA millisecondes

  // On envoie les données s'il s'est écoulé plus de TSDATA secondes
  // entre l'instant courant et le dernier envoi
  tempsCourant = millis();
  if (tempsCourant - tempsDernierEnvoi > TSDATA) {

    Serial.print(tempsCourant / 1000);
    Serial.print(",");
    Serial.print(commandeDroit);
    Serial.print(",");
    Serial.print(commandeGauche);
    Serial.print(",");
    Serial.print(omegaDroit);
    Serial.print("rd/s ,");
    Serial.print(omegaGauche);
    Serial.print("rd/s ,");


    VDroit = R * omegaDroit ;
    VGauche = R * omegaGauche ;
    Serial.print(VDroit);
    Serial.print("m/s ,");
    Serial.print(VGauche);
    Serial.print("m/s ,");

    Serial.print("\r");
    Serial.print("\n");

    tempsDernierEnvoi = tempsCourant;
  }
}




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
