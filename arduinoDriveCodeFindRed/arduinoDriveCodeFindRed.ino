/*
  SparkFun Inventor’s Kit
  Circuit 5B - Remote Control Robot

  Control a two wheeled robot by sending direction commands through the serial monitor.
  This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
  Check out the rest of the book at
  https://www.sparkfun.com/products/14326

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/
#include <Pixy2.h>

Pixy2 pixy;

int m_speed = 20;
int m_slow = 15 ;
int m_fast = 15;


//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN1 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN2 = 8;           //control pin 1 on the motor driver for the left motor

//line sensor pins
const int lf1 = 7;
const int lf2 = 6;
const int lf3 = 5;
const int lf4 = 4;

//distance sensor pins
const int trigPin = 24;           //connects to the trigger pin on the distance sensor
const int echoPin = 22;           //connects to the echo pin on the distance sensor

const int trigPin2 = 25;
const int echoPin2 = 23;

const int Brack = 29;
const int Frack = 31;
const int ShootPin = 33;
const int confirmPin = 41;

const int buttonPin = 3;
/********************************************************************************/
void setup()
{
  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  //set the linefollower pins as inputs
  pinMode(lf1, INPUT);
  pinMode(lf2, INPUT);
  pinMode(lf3, INPUT);
  pinMode(lf4, INPUT);

  //set the distance sensor io
  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(Brack, OUTPUT);
  pinMode(Frack, OUTPUT);
  pinMode(ShootPin, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(confirmPin, INPUT_PULLUP);

  Serial.begin(115200);
  pixy.init();
  pixy.changeProg("color_connected_components");
}

double lineDist[6] = {1, 1, 1, 1, -1, -1};

float distance = 0;               //stores the distance measured by the distance sensor
float distance2 = 0;
float range = 0.05;
double T = 1000;
double sendDelay = 0; //delay in ms
double tempT = 0;

String readStr = "";
bool distRead = false;
int timeOut = 15000;

const int orienting = 1;
const int positioning = 2;
bool positioned = false;
const int targeting = 3;
const int reloading = 5;

int state = 0;

int shots = 12;
double t_shoot = -1000.0;
double shootTime = 3;

//straightness tolerance
double tol = 0.5;


double R = 5.0;
//true center 159.5
const int targetCenter = 153.0;

bool shoot = false;
bool handshake = false;
bool red = false;


/********************************************************************************/
void loop()
{
  while (!handshake) {
    digitalWrite(ShootPin, HIGH);
    digitalWrite(Frack, HIGH);
    digitalWrite(Brack, HIGH);
    delay(1000);
    handshake = true;
  }


  //  getData();
  //DRIVER LOOP
  switch (state) {
    case orienting:
      Serial.println("orienting");
      orientation();
      state = positioning;

      break;

    case positioning:
      Serial.println("positioning");
      findPosition();
      positioned = true;
      state = targeting;
      break;

    case targeting:
      Serial.println("targeting");
      target();
      if (shots <= 0) {
        state = reloading;
      }
      break;

    case reloading:
      Serial.println("reloading");
      state = 0;
      break;

    default:
      Serial.println("waiting for start condition");
      while (digitalRead(buttonPin) != 0) {
        Serial.println("waiting for start condition");
        digitalWrite(Brack, LOW);
        delay(10);
        digitalWrite(Brack, HIGH);
        delay(10);
      }
      while (digitalRead(buttonPin) == 0) {
        delay(1);
      }

      if (positioned) {
        state = targeting;
        shots = 12;
      }
      else {
        state = orienting;
      }
      delay(10);
      digitalWrite(Frack, LOW);
      delay(10);
      digitalWrite(Frack, HIGH);
      delay(10);
  }
}

void getData() {
  lineDist[0] = digitalRead(6), lineDist[1] = digitalRead(7), lineDist[2] =  digitalRead(5), lineDist[3] =  digitalRead(4);

  if (distRead) {
    lineDist[4] = getDistance();   //variable to store the distance measured by the sensor
    lineDist[5] = getDistance2();
  }
  else {
    lineDist[4] = -1;   //variable to store the distance measured by the sensor
    lineDist[5] = -1;
  }

  if (T > 0) {
    String sendStr = "<" + String(lineDist[0]) + "," + String(lineDist[1]) + "," + String(lineDist[2]) + "," + String(lineDist[3]) + "," + String(lineDist[4]) + "," + String(lineDist[5]) + ">";
    Serial.println(sendStr);
    T = 0;
    tempT = millis();
  }
  else {
    T = millis() - tempT;
  }

}

void orientation() {
  bool oriented = false;
  int dir = -1;
  bool doubleCheck = false;

  distRead = true;
  while (!oriented) {
    getData();

    if (abs(lineDist[4] - lineDist[5]) < tol && lineDist[4] < 13 && lineDist[4] >= 0) {
      if (doubleCheck){
      oriented = true;
      }
      else {
        doubleCheck = true;
      }
    }
    else if (dir == -1) {
      if (lineDist[4] < lineDist[5]) {
        dir = 1;
        motorDrive("M_cw", m_speed, m_speed);
      }
      else if (lineDist[4] == lineDist[5]) {
        dir = 1;
        motorDrive("M_cw", m_speed, m_speed);
      }
      else {
        dir = 2;
        motorDrive("M_cc", m_speed, m_speed);
      }
    }
    else{
      doubleCheck = false;
    }
  }

  motorDrive("M_st", 0, 0);
  distRead = false;
}


void findPosition() {

  getData();
  motorDrive("M_re", m_speed, m_speed);

  while (lineDist[2] == 1 || lineDist[3] == 1) {
    getData();
    Serial.println("back");
  }
  delay(100);

  motorDrive("M_st", 0, 0);
  delay(1);

  int vision = pixyData();
  motorDrive("M_cw", m_speed, m_speed);
  while (lineDist[0] == 1) {
    getData();
    Serial.print("left");
    vision = pixyData();

    if (vision == -1){
      red = true;
    }

  }
  motorDrive("M_st", 00, 00);


  if (red) {
    Serial.print("red");

    motorDrive("M_cc", m_speed, m_speed);
    while (lineDist[1] == 1) {
      getData();

      Serial.print("find corner");

      if (lineDist[0] == 1) {
        motorDrive("M_cw", 0, 2 * m_speed);
      }
      else {
        motorDrive("M_cc", 2 * m_speed, 0);
      }

      delay(10);
    }

    motorDrive("M_st", 0, 0);

    motorDrive("M_cc", m_speed, m_speed);
    while (lineDist[2] == 1) {
      getData();
      Serial.print("face enemy");
    }

  }
  else {
    Serial.print("non-red");

    motorDrive("M_cc", m_speed, m_speed);
    while (lineDist[2] == 1) {
      getData();

      Serial.print("find corner");

      if (lineDist[3] == 1) {
        motorDrive("M_cc", 0, 2 * m_speed);
      }
      else {
        motorDrive("M_cw", 2 * m_speed, 0);
      }

      delay(10);
    }

    motorDrive("M_st", 0, 0);

    motorDrive("M_cc", m_speed, m_speed);
    while (lineDist[1] == 1) {
      getData();
      Serial.print("face enemy");
    }

    //    motorDrive("M_cc", m_speed, m_speed);
    //    while (lineDist[1] == 1) {
    //      getData();
    //      Serial.print("right");
    //    }
    //
    //    motorDrive("M_st", 0, 0);
    //
    //    motorDrive("M_cw", m_speed, m_speed);
    //    while (lineDist[0] == 1) {
    //      getData();
    //
    //      Serial.print("find corner");
    //
    //      if (lineDist[1] == 1) {
    //        motorDrive("M_cc", 2 * m_speed, 0);
    //      }
    //      else {
    //        motorDrive("M_cw", 0, 2 * m_speed);
    //      }
    //
    //      delay(10);
    //    }
    //
    //    motorDrive("M_st", 0, 0);
    //
    //    motorDrive("M_cw", m_speed, m_speed);
    //    while (lineDist[3] == 1) {
    //      getData();
    //      Serial.print("face enemy");
    //    }
  }
}

//vision system to be implimented
int pixyData() {
  // grab blocks!
  pixy.ccc.getBlocks();
  double x_index;

  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
      if (pixy.ccc.blocks[i].m_signature == 1) {

        x_index = pixy.ccc.blocks[i].m_x;


        if (x_index > (targetCenter) + R) {
          return 1;
          //right
        }
        else if (x_index < (targetCenter) - R) {
          return 3;
          //left
        }
        else {
          return 2;
          //fire
        }
      }
      //red area
      else if (pixy.ccc.blocks[i].m_signature == 3) {
        return -1;
      }
    }
  }
  return 0;
}

void target() {
  String dir = "M_cw";

  if (red){
    dir = "M_cc";
  }
  double sweep = 2.5;
  float delayTime = 0.1;

  int vision = pixyData();


  if (vision == 0) {
    double t1 = millis();
    motorDrive(dir, m_fast, m_fast);

    while (vision == 0) {
      vision = pixyData();

      if (millis() - t1 >= sweep * 1000) {
        if (dir == "M_cw") {
          dir = "M_cc";
          Serial.println("right");

        }
        else {
          dir = "M_cw";
          Serial.println("left");

        }

        t1 = millis();
        motorDrive(dir, m_fast, m_fast);
      }
    }

  }
  motorDrive("M_st", 00, 00);

  shoot = false;

  while (!shoot) {
    vision = pixyData();

    if (vision == 2) {
      motorDrive("M_st", 00, 00);

      double t2 = millis();

      while (vision == 2 && !shoot) {
        motorDrive("M_st", 00, 00);

        vision = pixyData();

        if (millis() - t2 > delayTime * 1000) {
          if (shots < 12) {
            if (millis() - t_shoot > shootTime * 1000) {
              digitalWrite(ShootPin, LOW);
              delay(10);
              digitalWrite(ShootPin, HIGH);
              Serial.println("shoot");

              //SHOOT COMMAND
              t_shoot = millis();
              shoot = true;
              shots = shots - 1;
            }
          }
          else {
            digitalWrite(ShootPin, LOW);
            delay(10);
            digitalWrite(ShootPin, HIGH);
            t_shoot = millis();

            shoot = true;
            Serial.println("shoot");
            shots = shots - 1;
          }
        }
      }
    }
    else if (vision == 1) {
      motorDrive("M_cc", m_slow, m_slow);
      Serial.println("finding");
    }
    else {
      motorDrive("M_cw", m_slow, m_slow);
      Serial.println("finding");
    }
  }

  delay(100);
  Serial.println("move back");
  digitalWrite(Brack, LOW);
  delay(10);
  digitalWrite(Brack, HIGH);

  if (digitalRead(confirmPin) == 1) {
    while (digitalRead(confirmPin) == 1) {
    }
  }

  if (shots > 0) {
    Serial.println("forward rack");
    digitalWrite(Frack, LOW);
    delay(10);
    digitalWrite(Frack, HIGH);
    if (digitalRead(confirmPin) == 1) {
      while (digitalRead(confirmPin) == 1) {
      }
    }
  }
}



void motorDrive(String cmd, int val1, int val2)
{
  if (cmd == "M_cw") {
    //set right motor off
    digitalWrite(AIN1, LOW);                         //set pin 1 to high
    digitalWrite(AIN2, HIGH);                          //set pin 2 to low

    //DRIVE left motor forward
    digitalWrite(BIN1, LOW);                         //set pin 1 to high
    digitalWrite(BIN2, HIGH);                          //set pin 2 to low
  }
  else if (cmd == "M_cc") {
    //DRIVE right motor forward
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low

    //set left motor off
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (cmd == "M_fw") {

    //DRIVE right motor forward
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low

    //DRIVE left motor forward
    digitalWrite(BIN1, LOW);                         //set pin 1 to high
    digitalWrite(BIN2, HIGH);                          //set pin 2 to low
  }
  else if (cmd ==  "M_re") {
    //DRIVE right motor backward
    digitalWrite(AIN1, LOW);                         //set pin 1 to high
    digitalWrite(AIN2, HIGH);                          //set pin 2 to low

    //DRIVE left motor ba backwards
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else {
    //turn right motor off
    digitalWrite(AIN1, LOW);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low

    //turn left motor off
    digitalWrite(BIN1, LOW);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }

  analogWrite(PWMA, abs(val1) * 255.0 / 100.0);             //now that the motor direction is set, drive
  analogWrite(PWMB, abs(val2) * 255.0 / 100.0);             //now that the motor direction is set, drive
}


//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH, timeOut);      //use the pulsein command to see how long it takes for the
  //pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  if (calculatedDistance == 0) {
    calculatedDistance = -2;
  }
  return calculatedDistance;              //send back the distance that was calculated
}

float getDistance2()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  echoTime = pulseIn(echoPin2, HIGH, timeOut);     //use the pulsein command to see how long it takes for the
  //pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  if (calculatedDistance == 0) {
    calculatedDistance = -2;
  }
  return calculatedDistance;              //send back the distance that was calculated
}
