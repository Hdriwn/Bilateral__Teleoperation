
double mts=0; // first double to receive
double mvh=0; // second double to receive

#include <Wire.h>
#include <math.h>
const int ledPin =13;
const int rgb = A0;
int cnt=0;
int rotpin =5;

const byte sencoderPinA = 2;//soutputA digital pin21
const byte sencoderPinB = 3; //soutoutB digital pin20

volatile long int scount = 0;
long int sconstCount = 0;

long int previousCount = 0;
#define CR 4//complete rotation
int i =0;

//define gains
float kp=0.03;
float ki=0;
//kp 0.07,ki 0.0001

#define sreadA bitRead(PIND,2)//faster than digitalRead()
#define sreadB bitRead(PIND,3)

// Pin declares
int pwmPin = 6; // PWM output pin for motor 1
int dirPin1 = 8;
int dirPin2 = 9; 

// Pin declares
int fsrPin = A3; // input pin for FSR sensor


      double mposition = 0; // read position value from master
      double mvelocity = 0; // read velocity value from master
// Position tracking variables

int srawPos = 0;     // current raw reading from MR sensor

// Kinematics variables
double xh = 0;           // position of the handle [m]
double lastXh = 0;     //last x position of the handle    
double slastts = 0;   
//slaves velocity variables
double sts =0;
double svh = 0;         //velocity of the handle
double slastVh = 0;     //last velocity of the handle
double slastLastVh = 0; //last last velocity of the handle

double kkk=0;
// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor
float xhp=0;
float xhh=0;
float xx_wall=0;
float x_wall=0;
float x=0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  pinMode(ledPin,OUTPUT);
  pinMode(rgb,OUTPUT);
  //Encoder statements
  Wire.begin(8); // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  //encoder slave
  pinMode(sencoderPinA, INPUT_PULLUP);
  pinMode(sencoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(sencoderPinA), isrAs, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sencoderPinB), isrBs, CHANGE);
  // // Set PWM frequency 
  // setPwmFrequency(pwmPin,1); 
  digitalWrite(rgb,LOW);
  // Input pins

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT); 
  // // Initialize motor 
  // analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin1, LOW);  // set direction
  

double lastTimeAtSurface = 0;
// 
}

void loop()
{
  

  noInterrupts();

  //determining slaves position
  sconstCount = scount;
  srawPos = sconstCount; 
  interrupts();

    sts = 0.35*srawPos; // slave angular position

    // xx_wall= 200;
    double rh = 0.089;   //[m] handle radius
    //  simulatewall(xx_wall,mts);
    xh = rh*(sts*0.00605/0.075);      


     //compute slaves handle velocity
     svh = -(.95*.95)*slastLastVh + 2*.95*slastVh + (1-.95)*(1-.95)*(sts-slastts)/.0001;  // filtered velocity (2nd-order filter)
     slastts = sts;
     slastLastVh = slastVh;
     slastVh = svh;


  // Define kinematic parameters you may need
     double rp = 0.004191;   //[m]pulley radius
     double rs = 0.073152;   //[m]sector radius
controlgains();
force = -kp*(sts-mts) + ki*(mvh);
     Tp = rp/rs * rh * force;    // Compute the require motor pulley torque (Tp) to generate that force
 Serial.println(sts);
  Serial.print(',');
// Serial.print(sts);
// Serial.write("\r\n"); 
// Serial.write(13); // Carriage Return
//   Serial.write(10); // Linefeed
// Serial.print(',');
// Serial.print(mts);
// Serial.print(',');
// Serial.print(force);
//Serial.println();


  if(force < -1) { 
    
    digitalWrite(dirPin1, LOW);//CLOCKWISE
    digitalWrite(dirPin2,HIGH);
  } else if(force>1) {
    digitalWrite(dirPin2,LOW);//ANTICLOCKWISE
    digitalWrite(dirPin1, HIGH);
  }
  else{
    digitalWrite(dirPin2,LOW);
    digitalWrite(dirPin1, LOW);
  
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  

     xhp=xh;
     
}


void isrAs() {
  if (sreadA == LOW) { 
    if (sreadB == LOW) {
      scount++;
    } else {
      scount--;
    }
  } else { 
    if (sreadB == LOW) {
      scount--;
    } else {
      scount++;
    }
  }
}

void isrBs() {
  if (sreadB == LOW) {
    if (sreadA == LOW) {
      scount--;
    } else {
      scount++;
    }
  } else {
    if (sreadA == LOW) {
      scount++;
    } else {
      scount--;
    }
  }
}

// function that executes whenever data is received from master
void receiveEvent(int howMany) {
    if (howMany == sizeof(mts) + sizeof(mvh)) {
        uint8_t mts1Bytes[sizeof(mts)];
        for (int i = 0; i < sizeof(mts); i++) {
            mts1Bytes[i] = Wire.read();
        }
        memcpy(&mts, mts1Bytes, sizeof(mts));

        uint8_t mts2Bytes[sizeof(mvh)];
        for (int i = 0; i < sizeof(mvh); i++) {
            mts2Bytes[i] = Wire.read();
        }
        memcpy(&mvh, mts2Bytes, sizeof(mvh));
    }
}

// function that executes whenever data is requested by master
void requestEvent() {
    uint8_t stsBytes[sizeof(sts)];
    memcpy(stsBytes, &sts, sizeof(sts));
    for (int i = 0; i < sizeof(sts); i++) {
        Wire.write(stsBytes[i]);
    }

    uint8_t svhBytes[sizeof(svh)];
    memcpy(svhBytes, &svh, sizeof(svh));
    for (int i = 0; i < sizeof(svh); i++) {
        Wire.write(svhBytes[i]);
    }
}


void controlgains(){
if (Serial.available()) {
    // read the first character from the serial port
    char c = Serial.read();

    // check if the character is 'p', 'i', or 'd'
    if (c == 'k' || c == 'l') {
      // read the rest of the line from the serial port
      String line = Serial.readStringUntil('\n');

      // convert the line to a float value
      float value = line.toFloat();

      // update the corresponding PID parameter
      if (c == 'k') {
        kp = value;
      } else if (c == 'l') {
        ki = value;
      } 
    } 
  }
}