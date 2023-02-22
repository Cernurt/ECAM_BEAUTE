#include <AFMotor.h>

#define ENCODEURA 19
#define ENCODEURB 38

AF_DCMotor motor(4); // Initalisation du moteur branchements sur M4
uint8_t i;
int hallticks;
 
void setup() {
 pinMode(ENCODEURA, INPUT_PULLUP);
 pinMode(ENCODEURB, INPUT_PULLUP);
 
 attachInterrupt(digitalPinToInterrupt(19), test, CHANGE);


  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Et c'est partiiii !");

  // turn on motor
  motor.setSpeed(200);
 
  motor.run(RELEASE);
}

void loop() {
 
  Serial.print("Tourne en avant - ");
  
  motor.run(FORWARD);
  motor.setSpeed(200);
  delay(670);     // Delai a ajuster pour que ca fasse pile un tour
  Serial.println(hallticks);
  motor.run(RELEASE);
  hallticks = 0;
  delay(4000);

}

void test(){
  hallticks = hallticks + 1;
}
