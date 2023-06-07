//*****************************************************************************************************************************************

//            Coefficients fixes et à déterminer empiriquement mais Classe pour les moteurs et asservissement en vitesse ok

//            Parcours OK

//            A faire PWM sur les capteurs pour limiter consommation

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
int PinLumi = 39;   // Led pour la recharge photovoltaique

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

unsigned int tempos;

Servo servolver;   // Servo du 360
Servo servopousse;

QTRSensors qtr;
uint16_t sensorValues[8];

//Creation des objets moteurs
AF_DCMotor motorR = AF_DCMotor(3, MOTOR34_64KHZ); // Initalisation du moteur branchements sur M1
AF_DCMotor motorL = AF_DCMotor(2, MOTOR12_64KHZ); // Initalisation du moteur branchements sur M1

AF_DCMotor brosse = AF_DCMotor(4, MOTOR34_64KHZ); // Initialisation sur M4 de la brosse

ClasseMoteur MDROIT = ClasseMoteur(600,rRoues, motorR, 200, 30, 0);
ClasseMoteur MGAUCHE = ClasseMoteur(600,rRoues, motorL, 200, 30, 0); // Coefficients d'asservissement à parametrer ! kp,ki,kd

//____________________________________________________Def Fonctions___________________________________________________




void ElVirage(float ciblasse, String sens){ //WORK IN PROGRESS
  if (sens == "g"){
    MDROIT.datSpeed(ciblasse * 1.50);
    MGAUCHE.datSpeed(ciblasse * 0.50);

    tempos = positionLigne;
    while (tempos <= 6500){ // Le robot quitte la ligne
      uint16_t positionLigne = qtr.readLineBlack(sensorValues);
      tempos = positionLigne;
      MDROIT.motor.run(FORWARD);   
      MGAUCHE.motor.run(FORWARD);
    }
    while (tempos > 4000){
      uint16_t positionLigne = qtr.readLineBlack(sensorValues);
      tempos = positionLigne;
      MDROIT.motor.run(FORWARD);   
      MGAUCHE.motor.run(FORWARD);
    }
    
  } else if (sens == "d") {
    MDROIT.motor.run(FORWARD);    
    MGAUCHE.motor.run(FORWARD);
    MDROIT.datSpeed(ciblasse * 0.50);
    MGAUCHE.datSpeed(ciblasse * 1.50);
    
    tempos = positionLigne;
    while (tempos >= 1500){ // Le robot quitte la ligne
      uint16_t positionLigne = qtr.readLineBlack(sensorValues);
      tempos = positionLigne;
      MDROIT.motor.run(FORWARD);   
      MGAUCHE.motor.run(FORWARD);
    }
    while (tempos < 3000){
      uint16_t positionLigne = qtr.readLineBlack(sensorValues);
      tempos = positionLigne;
      MDROIT.motor.run(FORWARD);   
      MGAUCHE.motor.run(FORWARD);
    }
    
  }
}

void DemiToru(float ciblasse, float theta){ //WORK IN PROGRESS
  distempR = MDROIT.ttlTicks;
  distempL = MGAUCHE.ttlTicks;
  MGAUCHE.datSpeed(ciblasse * 1.65);
  MDROIT.datSpeed(ciblasse * 0.45);
  
  while (((MGAUCHE.ttlTicks - distempL)-(MDROIT.ttlTicks - distempR)) / MDROIT.MMtoTicks(250) <= theta){ // Le robot fait theta radians en tournant
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(FORWARD);
  }   

  ToutDroitCapitaine(0,0,0);
  
}


void DecaleDligne(float ciblasse){

  positionLigne = qtr.readLineBlack(sensorValues);
  error = positionLigne - 6500;
  sommeErreur += error;
  
    
  Varia = KP * error + KI * sommeErreur + KD * (error - lastError);
  lastError = error;
 

  MDROIT.datSpeed(ciblasse - Varia);
  MGAUCHE.datSpeed(ciblasse + Varia);

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

void capteurLumiere(int sensibilite){
  float temp = 0;
  for (int i = 0; i<100; i++){
    unsigned int lumi = analogRead(A7);
    temp += lumi;
  }
  temp = temp/100;

  if (temp-moyLumi > sensibilite){
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
  } else {

    moyLumi = temp;
   
  }
  
}

uint16_t lectureCapteur(){
  qtr.emittersOn();
  delayMicroseconds(15);
  uint16_t positionLigne = qtr.readLineBlack(sensorValues);
  qtr.emittersOff();  
  return positionLigne;
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
  uint16_t positionLigne = qtr.readLineBlack(sensorValues);

  while(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 4750){ // Mettre la sensibilité en variable ?
    SuiviDligne(vitesseDem);
    uint16_t positionLigne = qtr.readLineBlack(sensorValues);
    MDROIT.motor.run(FORWARD);     
    MGAUCHE.motor.run(FORWARD);
  }
  ToutDroitCapitaine(0,0,0);
}


void heartbeat(){
  int pos;
  for (pos = 130; pos <= 170; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servopousse.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 170; pos >= 130; pos -= 1) { // goes from 180 degrees to 0 degrees
    servopousse.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
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

void tourneLigneG(float vitessedem){
  
  MDROIT.motor.run(FORWARD);   
  MGAUCHE.motor.run(BACKWARD);
  MDROIT.datSpeed(vitessedem);
  MGAUCHE.datSpeed(vitessedem);
  uint16_t positionLigne = qtr.readLineBlack(sensorValues);

  while(sensorValues[0] < 400){
    uint16_t positionLigne = qtr.readLineBlack(sensorValues);
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(BACKWARD);
  }
  
  while(sensorValues[3] < 800){
    uint16_t positionLigne = qtr.readLineBlack(sensorValues);
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(BACKWARD);
  }

  MDROIT.motor.run(FORWARD);   
  MGAUCHE.motor.run(FORWARD);
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
    Serial.println("renseigner le sens 'g' ou 'd' en deuxieme argument");
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
}

void timerSpeed(){

  MDROIT.routine(hallTicksR);
  hallTicksR = 0;
  MGAUCHE.routine(hallTicksL); 
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

//____________________________________________________Debut programme_________________________________________________________



void setup() {
  Serial.begin(115200);           // set up Serial library at 115200

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, 8);
  qtr.setEmitterPin(47);

  pinMode(Start, INPUT);
  digitalWrite (Start, HIGH);

  moyLumi = analogRead(A7);
  pinMode(PinLumi, OUTPUT);            // Led du capteur de lumiere

  servolver.attach(9);
  servolver.write(45);
  servopousse.attach(10);
  servopousse.write(179);

  Serial.println("Attente d'appui sur Start");

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

   
  tempos = positionLigne;
  avanceDeMmLigne(300, 1.50);
  DemiToru(1.20, 3.1416);
  avanceDeMmInterLigne(1.00);

  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMm(70 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  
  
  tourneLigneG(0.75);
  
  
  
  avanceDeMmLigne(250, 0.75);  // -------------------------------- Grande ligne droite ------------------------------------------------
  avanceDeMmLigne(1050, 1.50);
  
  
  avanceDeMmInterLigne(1.00);
  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  
  tourneLigneG(0.75);   // -------------------------------- Fin de la premiere grande ligne droite ---------------------------------------------
  

  avanceDeMmInterLigne(1.00);
  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  
  tourneLigneG(0.75);
  
                // -----------------------------------------Deuxieme grande ligne droite --------------------------------------------

  avanceDeMmLigne(150, 1.00);
  avanceDeMmLigne(350, 1.50);
  avanceDeMmLigne(800, 2.00);
  avanceDeMmInterLigne(0.75);
  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  
  tourneLigneG(0.75);
  
                // ------------------------------------------- Fin de la deuxieme grande ligne droite ----------------------------------

  avanceDeMmInterLigne(1.00);
  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  
  tourneLigneG(0.75);
  

                // ------------------------------------------- Troisième grande ligne droite -------------------------------------------

  avanceDeMmInterLigne(1.50);
  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMmLigne(100, 1.50); // avance de Mm Ligne à mettre
  

                // -------------------------------------------- Fin de la troisième ligne droite --------------------------------------

  avanceDeMmInterLigne(0.75);
  distempR = MDROIT.ttlTicks;
  
  

  avanceDeMm(60 - MDROIT.TickstoMM(MDROIT.ttlTicks - distempR), 0.70); // avance de Mm Ligne à mettre
  
  tourneLigneG(0.75);
  

                // ------------------

  avanceDeMmLigne(400, 1.00);
  

  while(true){
    heartbeat();    
  }

  /*
  ElVirage(1.00, "d");
  
  servopousse.write(170);
  
  avanceDeMmLigne(1820, 1.50); 

  ElVirage(1.00, "d");
  
  avanceDeMmInterLigne(1.00);
  
  delay(20000);

  */
  
}
