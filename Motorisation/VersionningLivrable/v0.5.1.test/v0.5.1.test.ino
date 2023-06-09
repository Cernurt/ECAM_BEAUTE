//*****************************************************************************************************************************************

//            Coefficients fixes et à déterminer empiriquement mais Classe pour les moteurs et asservissement en vitesse ok

//            Parcours OK
//            Programme nettoyé

//            Trouver un moyen de se décaller de la borne

//            

//*****************************************************************************************************************************************

#include <AFMotor.h>
#include <TimerOne.h>
#include <Servo.h>
#include <QTRSensors.h>

#define ENCODEURR 19
#define ENCODEURL 18



//____________________________________________________LaGrandeClasse___________________________________________

class ClasseMoteur{

  private:
    float _kp;
    float _ki;
    float _kd;
    double _somme_erreur;
    double _delta_erreur;
    double _erreur_precedente;
    double _speedDelta;
    double _MotorPWM;
    double _tickperround;
    float _rRoues;
    
  
  public:

    
    double speedM;
    long ttlTicks;
    double cibleVitesse;
    AF_DCMotor motor;

    ClasseMoteur(unsigned int tickperround,float rRoues, AF_DCMotor motor,float kp, float ki, float kd){
      
      this->motor = motor;
      this->_rRoues = rRoues;
      this->_tickperround = tickperround;
      this->speedM = 0;
      this->ttlTicks = 0;
      this->_kp = kp;
      this->_ki = ki;
      this->_kd = kd;
      this->cibleVitesse = 0;
      this->_somme_erreur=0;
      this->_delta_erreur=0;
      this->_erreur_precedente=0;
      this->_MotorPWM=0;
    }


    void routine(int hallTicks){
      MotorAsserv();
      speedM = (hallTicks/_tickperround)/0.02; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20
      ttlTicks += hallTicks;
    }
  
    void MotorAsserv(){ // Fonction qui asservit les moteurs aux variables cibleVitesseL et cibleVitesseR
     
      
      _speedDelta = cibleVitesse - speedM;
      _somme_erreur += _speedDelta;
      _delta_erreur = _speedDelta - _erreur_precedente;
      _erreur_precedente = _speedDelta;
      
      _MotorPWM = _kp * _speedDelta + _ki * _somme_erreur + _kd * _delta_erreur;
      
      if (_MotorPWM > 255) _MotorPWM = 255;
      else if (_MotorPWM < 0) _MotorPWM = 0;
      
      motor.setSpeed(_MotorPWM);
    }

    double distanceParcourue(){
      return ((ttlTicks/_tickperround)*2*3.1416*_rRoues);
    }

    double MMtoTicks(float temp){
      return ((temp/(2*3.1416*_rRoues))*_tickperround);
    }

    double TickstoMM(float temp){
      return ((temp/_tickperround)*2*3.1416*_rRoues);
    }

    void datSpeed(float cible){ // EN tour de roue par secondes
      //resetParamAsserv();
      cibleVitesse = cible;
    }

    void resetParamAsserv(){
      _somme_erreur=0;
      _delta_erreur=0;
      _erreur_precedente=0;
    }

};

//____________________________________________________Fin de LaGrandeClasse___________________________________________

//___________________________________________________Création objets/variables_________________________________________________________
bool debug=false;

double tickperround = 600; // Valeur à changer en fonction du capteur à effet hall
volatile int hallTicksR;
volatile int hallTicksL;
double rRoues = 34; // en mm

int Start = 34;     // Capteur méanique de Start déclaré connecté broche 34

float deltaS;
float ratio;
float cibleR;
float cibleL;

unsigned int moyLumi;

double distempR;
double distempL;

//Variables PID suivi de ligne
int ballez = 0;
float Varia;
int sommeErreur = 0;
int lastError;
int positionLigne;
int error;
float KP = 0.000214;
float KI = 0;
float KD = 0.000400;
unsigned int sensors[8];

int PinLumi = 39;

int sensibilite = 50;

int PinEol = 43;

unsigned int tempos;

boolean borne = false;

Servo servolver;   // Servo du 360
Servo servopousse;
Servo servoeol;

QTRSensors qtr;
uint16_t sensorValues[8];

//Creation des objets moteurs
AF_DCMotor motorR = AF_DCMotor(3, MOTOR34_64KHZ); // Initalisation du moteur branchements sur M1
AF_DCMotor motorL = AF_DCMotor(2, MOTOR12_64KHZ); // Initalisation du moteur branchements sur M1

AF_DCMotor brosse = AF_DCMotor(4, MOTOR34_64KHZ); // Initialisation sur M4 de la brosse

ClasseMoteur MDROIT = ClasseMoteur(600,rRoues, motorR, 200, 30, 0);
ClasseMoteur MGAUCHE = ClasseMoteur(600,rRoues, motorL, 200, 30, 0); // Coefficients d'asservissement à parametrer ! kp,ki,kd

//____________________________________________________Def Fonctions___________________________________________________

void tourneCont(float vitessedem, String sens){
  if (sens == "d"){
    MDROIT.motor.run(BACKWARD);   
    MGAUCHE.motor.run(FORWARD);
  } else if (sens == "g"){
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(BACKWARD);
  } else {
    MDROIT.motor.run(RELEASE);   
    MGAUCHE.motor.run(RELEASE);
  }
  MDROIT.datSpeed(vitessedem);
  MGAUCHE.datSpeed(vitessedem);
  
}


void SuiviDligne(float ciblasse){
  
  positionLigne = qtr.readLineBlack(sensorValues);
  error = positionLigne - 3500;
  sommeErreur += error;
  
    
  Varia = KP * error + KI * sommeErreur + KD * (error - lastError);
  lastError = error;
 

  MDROIT.datSpeed(ciblasse - Varia);
  MGAUCHE.datSpeed(ciblasse + Varia); 
}

void calibmax(){                                                                                       //Asservissement
  for (uint16_t i = 0; i < 100; i++)
  {
    if (i < 25){
      tourneCont(0.40, "g");
    }
    else if (i < 71){
      tourneCont(0.40, "d");
    }
    else if (i < 100){
      tourneCont(0.40, "g");
    }
    
    qtr.calibrate();
  }
  MDROIT.datSpeed(0);
  MGAUCHE.datSpeed(0);
}


void ToutDroitCapitaine(float ciblasse, float ticksR, float ticksL){
  
  deltaS = ticksR - ticksL;
  
  cibleR = ciblasse*(1-deltaS*0.001); // Si DeltaS = 500, cibleR = ciblasse*0.5
  cibleL = ciblasse*(1+deltaS*0.001);

  MDROIT.datSpeed(cibleR); 
  MGAUCHE.datSpeed(cibleL);

  if (debug){
    Serial.print("Vitesse cible gauche : ");
    Serial.print(cibleL);
    Serial.print("Vitesse cible droite : ");
    Serial.println(cibleR);
  } else { delay(1); } // Ne fonctionne pas sans ce délai, pas d'idée pourquoi.
}

int capteurLumiere(){
  int lumi = analogRead(A7);
  return lumi;
}

void avanceDeMmLigne(float distDem, float vitesseDem){
  
  distempR = MDROIT.ttlTicks;
  distempL = MGAUCHE.ttlTicks;

  MDROIT.motor.run(FORWARD);    
  MGAUCHE.motor.run(FORWARD);

  while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(distDem)){
    SuiviDligne(vitesseDem);
    MDROIT.motor.run(FORWARD);    
    MGAUCHE.motor.run(FORWARD);
  }

  ToutDroitCapitaine(0,0,0);
  
}

void avanceDeMmInterLigne(float vitesseDem){

  MDROIT.motor.run(FORWARD);     
  MGAUCHE.motor.run(FORWARD);
  positionLigne = qtr.readLineBlack(sensorValues);

  while(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 4750){ // Mettre la sensibilité en variable ?
    SuiviDligne(vitesseDem);
    positionLigne = qtr.readLineBlack(sensorValues);
    MDROIT.motor.run(FORWARD);     
    MGAUCHE.motor.run(FORWARD);
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] == 8000){
      for (int i=0; i<8; i++){
        sensorValues[i] = 0;
      }
    }
  }
  ToutDroitCapitaine(0,0,0);
}


void heartbeat(){
  int pos;
  for (pos = 90; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servopousse.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
    servopousse.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void pliedeplie(){
  if (servopousse.read() == 0){
    servopousse.write(180);    
  }
  else if (servopousse.read() == 180){
    servopousse.write(0);    
  }
  else {
    servopousse.write(180);
  }
}

void avanceDeMm(float distDem, float vitesseDem){

  if (distDem < 0){
  
    distempR = MDROIT.ttlTicks;
    distempL = MGAUCHE.ttlTicks;
  
    while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(abs(distDem))){
      
      ToutDroitCapitaine(vitesseDem, MDROIT.ttlTicks - distempR, MGAUCHE.ttlTicks - distempL);
      MDROIT.motor.run(BACKWARD);   
      MGAUCHE.motor.run(BACKWARD);
      
    }
    
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(FORWARD);
    
  } else {
  
    distempR = MDROIT.ttlTicks;
    distempL = MGAUCHE.ttlTicks;
  
    while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(distDem)){
      ToutDroitCapitaine(vitesseDem, MDROIT.ttlTicks - distempR, MGAUCHE.ttlTicks - distempL);
      MDROIT.motor.run(FORWARD);   
      MGAUCHE.motor.run(FORWARD);
    }
  }
  
  ToutDroitCapitaine(0,0,0);
  
}

void tourne(float vitessedem, String sens){

  distempR = MDROIT.ttlTicks;
  distempL = MGAUCHE.ttlTicks;

  if (sens == "d"){
    MDROIT.motor.run(BACKWARD);   
    MGAUCHE.motor.run(FORWARD);
  } else if (sens == "g"){
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(BACKWARD);
  } else {
    MDROIT.motor.run(RELEASE);   
    MGAUCHE.motor.run(RELEASE);
    //Serial.println("renseigner le sens 'g' ou 'd' en deuxieme argument");
  }

  while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(197)){
    ToutDroitCapitaine(vitessedem, MDROIT.ttlTicks - distempR, MGAUCHE.ttlTicks - distempL);
  }

  ToutDroitCapitaine(0,0,0);
  
}

void leptitgrain(){
  
  if (ballez == 2){
    servolver.write(180);
    ballez += 1;
  } else if (ballez == 1){
    servolver.write(90);
    ballez += 1;
  } else if (ballez == 0){
    servolver.write(0);
    ballez += 1;
  }  

  delay(500);
  
}

void bornemuch(float temp){

  unsigned int lumi = analogRead(A7);

  if (lumi > temp * sensibilite){
    MDROIT.datSpeed(0.00);
    MGAUCHE.datSpeed(0.00);
    
    digitalWrite(PinLumi, HIGH);
    delay(500);
    digitalWrite(PinLumi, LOW);
    delay(500);
    digitalWrite(PinLumi, HIGH);
    delay(500);
    digitalWrite(PinLumi, LOW);
    delay(500);
    digitalWrite(PinLumi, HIGH);
    delay(500);
    digitalWrite(PinLumi, LOW);

    borne = true;

    //MDROIT.datSpeed(0.00);
    //MGAUCHE.datSpeed(0.00);
    
  } 
}

void eolmonte(){
  servoeol.write(45);
  delay(4000);
  servoeol.write(90);
}

void eoldescend(){
  servoeol.write(130);
  delay(3000);
  servoeol.write(90);
}


void timerSpeed(){

  MDROIT.routine(hallTicksR);
  hallTicksR = 0;
  MGAUCHE.routine(hallTicksL); //Coeffshlag
  hallTicksL = 0;

  if (debug){
    Serial.print("Vitesses: Gauche = ");
    Serial.print(MGAUCHE.speedM);
    Serial.print(" - Droite = ");
    Serial.println(MDROIT.speedM);
  }
}

void compteurR(){hallTicksR++;}
void compteurL(){hallTicksL++;}

//____________________________________________________ Fonctions étapes ______________________________________________________

void parcoursAD(){
  
  avanceDeMmLigne(280, 1.00); // A REGLER

  avanceDeMmInterLigne(0.75);

  distempR = MDROIT.ttlTicks;

  delay(1000);

  leptitgrain();

  delay(3000);
  
  //delay(500);

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);
  
  tourne(0.75, "d");

  avanceDeMmInterLigne(0.75);

  distempR = MDROIT.ttlTicks;
  
  //delay(500);

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);

  tourne(0.75, "d");


  brosse.run(BACKWARD);
  brosse.setSpeed(255);

  
  avanceDeMmLigne(250, 0.75);
  
  avanceDeMmInterLigne(0.75);

  brosse.run(RELEASE);

  distempR = MDROIT.ttlTicks;
  
  //delay(500);

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);
  
  tourne(0.75, "g");

}

void parcoursDE(){

  brosse.run(BACKWARD);
  brosse.setSpeed(255);
  
  avanceDeMmLigne(500, 1.25);

  brosse.setSpeed(RELEASE);

  avanceDeMmLigne(360, 1.50);

  avanceDeMmLigne(350, 1.25);

  servopousse.write(90);

  avanceDeMmLigne(150, 0.75);
  
  delay(1000);

  leptitgrain();

  delay(3000);

  avanceDeMmLigne(100, 0.75);

  avanceDeMmLigne(100, 1.25);

  servopousse.write(0);
  
  avanceDeMmInterLigne(1.00);
  
  distempR = MDROIT.ttlTicks;
  //delay(500);
  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);
  
  tourne(0.75, "g");   // -------------------------------- Fin de la premiere grande ligne droite ---------------------------------------------
}

void parcoursEF(){

  brosse.run(BACKWARD);
  brosse.setSpeed(255);
  
  avanceDeMmInterLigne(1.00);
  distempR = MDROIT.ttlTicks;
  
  //delay(500);

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);
  tourne(0.75, "g");

  brosse.run(RELEASE);

}

void parcoursFC(){

  avanceDeMmLigne(460, 1.00);

  delay(1000);

  leptitgrain();

  delay(3000);

  avanceDeMmLigne(100, 1.00);

  MDROIT.datSpeed(0);
  MGAUCHE.datSpeed(0);

  servopousse.write(90);

  delay(1000);

  avanceDeMm(-200, 0.75);

  MDROIT.datSpeed(0);
  MGAUCHE.datSpeed(0);

  servopousse.write(180);

  delay(1000);

  avanceDeMmLigne(100, 1.00);
  
  avanceDeMmLigne(840, 1.25);
  avanceDeMmInterLigne(0.75);
  distempR = MDROIT.ttlTicks;
  
  //delay(500);

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);
  tourne(0.75, "g");
  
}

void parcoursCD(){

  avanceDeMmLigne(150, 1.00);
  avanceDeMmInterLigne(0.75);
  distempR = MDROIT.ttlTicks;
  
  //delay(500);

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  //delay(500);
  tourne(0.75, "g");
  
}

void parcoursDG(){

  avanceDeMmLigne(500, 1.25);

  avanceDeMmLigne(360, 1.50);

  avanceDeMmLigne(500, 1.25);

  avanceDeMmInterLigne(0.75);
  
  //delay(500);

  brosse.run(BACKWARD);
  brosse.setSpeed(255);

  avanceDeMmLigne(300, 0.75); // avance de Mm Ligne à mettre

  brosse.run(RELEASE);
  //delay(500);
  avanceDeMmInterLigne(0.75);
  
  distempR = MDROIT.ttlTicks;

  MDROIT.datSpeed(0);
  MGAUCHE.datSpeed(0);

  delay(500);  

  avanceDeMm(-30-MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.75); // avance de Mm Ligne à mettre
  //delay(500);
  tourne(0.75, "g");  
  
}

void parcoursGH(){

  avanceDeMm(610, 0.75);

  eoldescend();
  
}

//____________________________________________________Debut programme_________________________________________________________



void setup() {
  //Serial.begin(115200);           // set up Serial library at 115200

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, 8);
  qtr.setEmitterPin(47);

  pinMode(Start, INPUT);
  digitalWrite (Start, HIGH);

  pinMode(PinLumi, OUTPUT);

  servolver.attach(9);
  servolver.write(45);
  delay(500);
  servopousse.attach(10);
  servopousse.write(180);

  servoeol.attach(PinEol);
  servoeol.write(90);

  // Zone de test ___________________________

  
  
  // _______________________________________

  //Serial.println("Attente d'appui sur Start");

  if (digitalRead(Start)==HIGH){         //
    while(digitalRead(Start)==HIGH);     //    Lancement si l'interrupteur de start change d'état
  } else if (digitalRead(Start)==LOW){   //
    while(digitalRead(Start)==LOW);      //
  }

  
  
  pinMode(ENCODEURR, INPUT_PULLUP);
  pinMode(ENCODEURL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), compteurR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), compteurL, CHANGE);

  Timer1.initialize(20000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 

  calibmax();

  MDROIT.motor.run(FORWARD);     // ^
  MGAUCHE.motor.run(FORWARD);    // | Exemple de commande des moteurs
  avanceDeMmLigne(0, 1);         // v   (distanceEnMm, vitesse en tr/sec)
}


void loop() {

  //tempos = positionLigne;
  
  parcoursAD();
  
  //delay(500);
  
  parcoursDE();

  //delay(500);

  parcoursEF();

  //delay(500);

  parcoursFC();
  
  //delay(500);

  parcoursCD();
 
  //delay(500);

  parcoursDG();

  //delay(500);

  parcoursGH();
  
  //delay(500);

  while(true){
      
  }
}
