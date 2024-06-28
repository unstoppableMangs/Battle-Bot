#define START_MARKER '<'
#define END_MARKER '>'
#define COMMAND_SEP '|'
#define VALUE_SEP ','


const int basePin = 7;
const int Fpin = 6;
const int Bpin = 5;
const int confirmPin = 12;

const int Frack = 4;
const int Brack = 3;
const int Shoot = 2;

const int firePin = 11;

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

double T = 1000;
double sendDelay = 0; //delay in ms
double tempT = 0;
/********************************************************************************/

void setup()
{
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(basePin, OUTPUT);
  pinMode(Fpin, INPUT_PULLUP);
  pinMode(Bpin, INPUT_PULLUP);

  pinMode(firePin, INPUT_PULLUP);

  Serial.begin(115200);

  pinMode(Frack, INPUT_PULLUP);
  pinMode(Brack, INPUT_PULLUP);
  pinMode(Shoot, INPUT_PULLUP);
  pinMode(confirmPin, OUTPUT);
}

//command inputCMD;

int Flim = 1;
int Blim = 1;
bool loaded = false;
int firingDelay = 20;
int motorSpeed = 250;

char inputCMD = '_';
bool handshake = false;

/********************************************************************************/
void loop()
{
  digitalWrite(confirmPin, HIGH);
  //
  //if (handshake == false){
  //  while (!handshake){
  //      Serial.println("handshake");
  //
  //    if (digitalRead(Frack) && digitalRead(Brack)&& digitalRead(Shoot)){
  //      handshake = true;
  //    }
  //  }
  //}

  if (digitalRead(Frack) == 0) {
    inputCMD = 'F';

  } else if (digitalRead(Brack) == 0) {
    inputCMD = 'B';

  } else if (digitalRead(Shoot) == 0) {
    inputCMD = 'S';
  }

  else {
    inputCMD = '_';
  }

  Serial.println(String(digitalRead(Shoot)) + ',' + String(digitalRead(Brack)) + ',' + String(digitalRead(Frack)));
  Flim = digitalRead(Fpin);
  Blim = digitalRead(Bpin);

  switch (inputCMD)
  {
    case 'F':

        f_Rack();
        loaded = true;

        digitalWrite(confirmPin, LOW);
        delay(10);
        digitalWrite(confirmPin, HIGH);


      break;

    case 'S':
      if (loaded) {
        fire();
        loaded = false;
      }

      break;
    case 'B':
      b_Rack();
      digitalWrite(confirmPin, LOW);
      delay(10);
      digitalWrite(confirmPin, HIGH);
      break;
    default:
      break;
  }

  inputCMD = '_';
  //  sendData();
}

void sendData()
{
  String tempStr = '<' + String(Flim) + ',' + String(Blim) + ',' + String(loaded) + '>';

  if (T > sendDelay) {
    Serial.println(tempStr);
    T = 0;
    tempT = millis();
  }
  else {
    T = millis() - tempT;
  }

}

//load the dart
void f_Rack()
{
  Flim = digitalRead(Fpin);
  Blim = digitalRead(Bpin);

  // Load a dart by moving the rack forward
  if (Flim != 0) {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
    analogWrite(PWMB, motorSpeed);             //now that the motor direction is set, drive

    while (Flim != 0) {
      Flim = digitalRead(Fpin);
      Blim = digitalRead(Bpin);
      Serial.println("Frack");
      sendData();
    }

    digitalWrite(BIN1, LOW);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
    analogWrite(PWMB, 0);             //stop the motor
  }
}

void b_Rack()
{
  Flim = digitalRead(Fpin);
  Blim = digitalRead(Bpin);

  // Load a dart by moving the rack forward
  if (Blim != 0) {
    digitalWrite(BIN1, LOW);                         //set pin 1 to high
    digitalWrite(BIN2, HIGH);                          //set pin 2 to low
    analogWrite(PWMB, motorSpeed);             //now that the motor direction is set, drive

    while (Blim != 0) {
      Blim = digitalRead(Bpin);
      Flim = digitalRead(Fpin);
      Serial.println("Brack");

      sendData();
    }

    digitalWrite(BIN1, LOW);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
    analogWrite(PWMB, 0);             //stop the motor
  }
}

// fire the dart
void fire() {
  digitalWrite(basePin, HIGH);
  delay(firingDelay);
  digitalWrite(basePin, LOW);
}
