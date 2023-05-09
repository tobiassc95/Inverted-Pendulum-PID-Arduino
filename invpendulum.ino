//INVERTED PENDULUM. BY TOBIAS SCALA.

//Define Pins and Stuff.
#define MODE 12 //switch of the board (to select the pendulum mode).
#define MOTOR1 11
#define MOTOR2 10
#define REFPOT A1 //potenciometer of the board (to set a reference in equilibrist mode).
#define ANGPOT A0 //potenciometer of the pendulum (to measure the angle).

//Enumerations
enum {OSCILAT, EQUILIB};

//Functions.
int PIDcontroller(const double Kp, const double Ki, const double Kd);

//Global Variables and Constants.
const unsigned int T = 10; //sample period (millis).
double err; //error (ref - pendpot).
unsigned long cnt = 0;

void setup() {
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MODE, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  bool mode; //inverted pendulum mode (0 -> oscilation, 1 -> equilibrist)
  double ref; //reference (set point).
  double ang; //angle of the inv pendulum (input).
  int pwm; //signal to the motors (output).
  
  static unsigned long nT = 0; //distret time (millis).
  unsigned long t = millis(); //current time (millis).
  if(t - nT >= T) {
    nT = t;

//    if(cnt > 500) {
    mode = digitalRead(MODE);
    if (mode == OSCILAT)
      ref = 0;
    else if (mode == EQUILIB)
      ref = (double)(analogRead(REFPOT) - 512)/732; //reference normalized ([-0.7, 0.7] approx).
    ang = (double)(analogRead(ANGPOT) - 485)/111; //angle normalized ([-1, 1] approx). 
    err = ref - ang;

    if (mode == OSCILAT)
      pwm = PIDcontroller(160,800,100);
    else if (mode == EQUILIB) {
      if (ref > -0.05 && ref < 0.05)
        pwm = PIDcontroller(250,0,70);
      else
        pwm = PIDcontroller(250,100,80);
    }

    if (pwm > 255)
      pwm = 255;
    else if (pwm < -255)
      pwm = -255;
    if (pwm > 10) {
      analogWrite(MOTOR1, 0);
      analogWrite(MOTOR2, pwm);
    }
    else if (pwm < -10) {
      analogWrite(MOTOR1, -pwm);
      analogWrite(MOTOR2, 0);
    }
    else {
      analogWrite(MOTOR1, 0);
      analogWrite(MOTOR2, 0);
    }
    
    Serial.print("err = ");
    Serial.println(err);
//    Serial.print("pwm = ");
//    Serial.println(pwm);
//    Serial.print("ref = ");
//    Serial.println(ref);
//    Serial.print("ang = ");
//    Serial.println(ang);
//    if(cnt < 1501) {
//      cnt++;
//      Serial.print(ang);
//      Serial.print(',');
//    }
//    }
//    else
//    cnt++;
  }
}

int PIDcontroller(const double Kp, const double Ki, const double Kd) {
  double P; //output from the proportional controller.
  double I; //output from the integrative controller.
  double D; //output from the derivative controller.
  static double lasterr = 0; //last error (e(n-1)).
  static double lastI = 0; //last output from the integrative controller (I(n-1)).

  if (Ki == 0)
    lastI = 0; //empty the control I when it is not being used.
  
  //Proportional Controller.
  P = Kp*err;
  //Integrative Controller.
  I = lastI + Ki*T*(err + lasterr)/(2*1000);
  //Derivative Controller.
  D = Kd*1000*(err - lasterr)/T;

  //avoiding windup.
  if (I > 255)
    I = 255;
  else if (I < -255)
    I = -255;

//  Serial.print("P = ");
//  Serial.println(P);
//  Serial.print("I = ");
//  Serial.println(I);
//  Serial.print("D = ");
//  Serial.println(D);
  
  lasterr = err;
  lastI = I;

  return P + I + D;
}

//void loop() {
//  unsigned int pwm = analogRead(ANGPOT);
//  pwm = map(pwm,0,1023,0,255);
//  analogWrite(MOTOR2, pwm);
//}
