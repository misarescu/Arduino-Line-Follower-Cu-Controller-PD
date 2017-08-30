/**
 * Line Follower care are integrat 
 * in el un sistem de control de 
 * tip PD(Proportional si Derivata).
 * 
 * Sistemul PD este un sistem de 
 * control ce preia controlul
 * robotului atat proportional 
 * cu eroarea, dar si prevede
 * schimbarile sistemului folosind
 * derivata proportiei.
 */


/**
 * Librariile sunt folosiste pentru
 * senzori (8QTR-RC de la pololu)
 * si shield pentru motoare care
 * foloseste driver L293D pentru 
 * motoare.
 */
#include <AFMotor.h>
#include <QTRSensors.h>

/**
 * Pinii definiti pentru arrayul 
 * de senzori.
 */
#define s1 A5
#define s2 A4
#define s3 A3
#define s4 A2
#define s5 A1
#define s6 A0  
#define NUM_OF_SENSORS 6
/**
 * Variabilele ce controleaza 
 * viteza motoarelor.
 * @DEFAULT_SPEED 
 * -viteza de mers in fata
 * @MAX_SPEED
 * -viteza maxima de mers
 * in curbe
 * @MIN_SPEED
 * -viteza minima de mers
 * in curbe, de preferat 0
 * dar se poate sa fie si negativa
 */
#define DEFAULT_SPEED 75
#define MAX_SPEED 100
#define MIN_SPEED 0
/**
 * Reprezinta tinta pentru senzori
 * senzorul 0 are valoarea liniei 0
 * senzorul 1 are valoarea liniei 1
 * valorea intre senzorii 0 si 1 este 500.
 */
#define TARGET 2500
/**
 * Constantele care influenteaza 
 * sistemul PD.
 * @KP
 * -constanta proportionalei
 * @KD
 * -constanta derivatei
 * nota: KD > KP
 */
#define KP 0.04
#define KD 0.2

/**
 * error si prevError retin eroarea
 * curenta si cea anterioara
 */
int error = 0;
int prevError = 0;

/**
 * Directia in care se va misca robotul.
 */
int direct;

/**
 * Motoarele dc, conectate pe sloturile
 * 1 si 2, la o frecventa PWM de 64khz
 * nota: pe sloturile 1 si 2 frecventele
 * sunt de 1khz, 2khz, 8khz, si 64khz
 * iar pe sloturile 3 si 4 frecventele 
 * sunt de 1,8 si 64 khz.
 */
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);

//Bara de senzori
QTRSensorsRC senzori((unsigned char[]) {s1,s2,s3,s4,s5,s6}, NUM_OF_SENSORS);
//Valorile citite de senzori, rol de debug
unsigned int valori[NUM_OF_SENSORS];

/**
 * Clamp se asigura ca viteza robotului
 * nu depaseste valorile min sau max.
 * @val
 * -valoarea vitezei transmise
 */
void Clamp(int &val){
  if(val > MAX_SPEED);
    val = MAX_SPEED;
  if(val < MIN_SPEED)
    val = MIN_SPEED;
}

/**
 * Go seteaza viteza motoarelor si
 * directia inc care motorul se invarte
 * aceasta realizandu-se prin constantele 
 * FORWARD si BACKWARD ale librariei 
 * AFMotor. 
 * @Speed
 * -Viteza motorului, valori intre 0 si 255
 * @motor
 * -Motorul care trebuie controlat
 */
void Go(int Speed, AF_DCMotor motor){
  if(Speed > 0){
    Clamp(Speed);
    motor.run(FORWARD);
    motor.setSpeed(Speed);
  }
  else{
    motor.run(BACKWARD);
    motor.setSpeed(-Speed);
  }
  
}

void setup() {
  //setup pentru debug
  delay(500);
  Serial.begin(9600);

  senzori.emittersOn();

  /**
   * Calibrarea senzorilor, se face de 
   * fiecare data cand se porneste robotul.
   */
  Serial.print("begin calibration\n");
  for(int i = 0; i < 50; i++){
    senzori.calibrate();
   // Serial.print("Calibrating\n");
    delay(50);
  }

  Go(-50, motor1);
  Go(-50, motor2);
  delay(100);

  for(int i = 1; i <= 5; i++){
    Go(50-i*10, motor1);
    Go(50-i*10, motor2);
    delay(5);
  }
  
  delay(100);

  for(int i = 0; i < 50; i++){
    senzori.calibrate();
   // Serial.print("Calibrating\n");
    delay(50);
  }


  Go(50, motor1);
  Go(50, motor2);
  delay(100);

  for(int i = 1; i <= 5; i++){
    Go(50-i*10, motor1);
    Go(50-i*10, motor2);
    delay(5);
  }

  /**
   * Finaalizarea calibrarii
   * unii parametrii pot fi modificati,
   * de asemenea si modul in care robotul
   * face "un pas" in spate si altul in fata
   * se poate modifica in functie de 
   * caracteristicile fizice ale robotului.
   */

  /**
   * Debug, afisarea valorilor minime si maxime
   * inregistrate de senzori.
   */
  for (int i = 0; i < NUM_OF_SENSORS; i++)
  {
    Serial.print(senzori.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_OF_SENSORS; i++)
  {
    Serial.print(senzori.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  delay(5000);
  
}

void loop() {
  
  /**
   * Calcularea directiei in care ar trebui
   * sa se miste robotul. Pozitia curenta 
   * este citita de la senzori prin readLine,
   * eroarea este calculata prin diferenta
   * dintre pozitia tinta(TARGET) si pozitia
   * curenta(position). Directia este data
   * de suma valorii proportionale si cea
   * derivata. Valoarea proportionala este
   * data de formula KP * error, iar 
   * valoarea derivata este data de formula
   * KD * (error - prevError).
   */
  int position = senzori.readLine(valori);
  error = TARGET - position;
  direct = (int)((double)(KP * error + KD * (error - prevError)));

  /**
   * Robotul se va misca in functie de
   * valoarea variabilei direct.
   * nota:
   * direct = 0 -> robotul va merge drept
   * direct > 0 -> robotul va merge in stanga
   * direct < 0 -> robotul va merge in dreapta
   */
  Go(DEFAULT_SPEED + direct, motor1);
  Go(DEFAULT_SPEED - direct, motor2);

  /**
   * Dupa terminarea calcularii directiei si 
   * a efectuarii mersului motoarelor in 
   * functie de aceasta, erroarea curenta
   * devine eroarea anterioara.
   */
  prevError = error;

  /** 
   * Debug, afisare valori senzori, pozitie, 
   * eroare si directie.
   */
  for(int i=0; i < NUM_OF_SENSORS; i++){
    Serial.print(valori[i]);
    Serial.print(' ');
  }

  Serial.print('\n');
  Serial.print(position);
  Serial.print(' ');
  Serial.print(error);
  Serial.print(' ');
  Serial.print(direct);
  Serial.print('\n');
}
