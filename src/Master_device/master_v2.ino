

#include <Wire.h>
#include <math.h>

double sts=0; // first double to receive
double svh=0; // second double to receive

double kp=0;//0.03;
double ki=0;
const int ledPin =13;
const int rgb = A0;
int cnt=0;
int rotpin =7;
const byte mencoderPinA = 2;//moutputA digital pin2
const byte mencoderPinB = 3;//moutoutB digital pin3

volatile long int mcount = 0;

long int mconstCount = 0;
long int sconstCount = 0;

long int previousCount = 0;
#define CR 5//complete rotation
int i =0;
#define mreadA bitRead(PIND,2)//faster than digitalRead()
#define mreadB bitRead(PIND,3)

// Pin declares
int pwmPin = 6; // PWM output pin for motor 1
int dirPin1 = 8;
int dirPin2 = 9; 

// Pin declares
int fsrPin = A3; // input pin for FSR sensor
//ss
double pulley = 3.79;
double sector = 75;
// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int mrawPos = 0;  
// Kinematics variables
double xh = 0;           // position of the handle [m]
double lastXh = 0;     //last x position of the handle    
double mlastts = 0;   
//masters velocity variables
double mvh = 0;         //velocity of the handle
double mlastVh = 0;     //last velocity of the handle
double mlastLastVh = 0; //last last velocity of the handle
double mts =0;
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

void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  pinMode(ledPin,OUTPUT);
  pinMode(rgb,OUTPUT);
  //Encoder statements
  //encoder master
  pinMode(mencoderPinA, INPUT_PULLUP);
  pinMode(mencoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(mencoderPinA), isrAm, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mencoderPinB), isrBm, CHANGE);

  // // Set PWM frequency 
  // setPwmFrequency(pwmPin,1); 
  digitalWrite(rgb,LOW);
  // Input pins
 Wire.begin(); // join i2c bus as master
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
  //determining masters position
  mconstCount = mcount;
  mrawPos = mconstCount; 

  interrupts();
  // angular displacement
     mts = .35*mrawPos;   //master angular position

    // xx_wall= 200;
    double rh = 0.089;   //[m] handle radius
    //  simulatewall(xx_wall,mts);
    xh = rh*(mts*0.00605/0.075);      

  //compute masters handle velocity
     mvh = -(.95*.95)*mlastLastVh + 2*.95*mlastVh + (1-.95)*(1-.95)*(mts-mlastts)/.0001;  // filtered velocity (2nd-order filter)
     mlastts = mts;
     mlastLastVh = mlastVh;
     mlastVh = mvh;


  // Define kinematic parameters you may need
     double rp = 0.004191;   //[m]pulley radius
     double rs = 0.073152;   //[m]sector radius


transmission();
reception();

controlgains();
force = -kp*(mts-sts) + ki*(svh);
Tp = rp/rs * rh * force;    // Compute the require motor pulley torque (Tp) to generate that force
  // rpulley*theta pulley= rsector*theta sector   
  // rpullet=6.2, rsector=76     
  // thethasector =(rpulley/rsector)*thetapulley
  
  double tpulley1 = pulley/sector * mts;
  double tpulley2 = pulley/sector * sts;
  
  // Serial.print(mts);
  // Serial.print(',');
  Serial.print(tpulley1);
  Serial.print(',');
  Serial.print(tpulley2);
  Serial.println();


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



void isrAm() {
  if (mreadA == LOW) { 
    if (mreadB == LOW) {
      mcount++;
    } else {
      mcount--;
    }
  } else { 
    if (mreadB == LOW) {
      mcount--;
    } else {
      mcount++;
    }
  }
}

void isrBm() {
  if (mreadB == LOW) {
    if (mreadA == LOW) {
      mcount--;
    } else {
      mcount++;
    }
  } else {
    if (mreadA == LOW) {
      mcount++;
    } else {
      mcount--;
    }
  }
}


void controlgains(){
if (Serial.available()) {
    // read the first character from the serial port
    char c = Serial.read();

    // check if the character is 'p', 'i', or 'd'
    if (c == 'k' || c == 'l'|| c=='p' || c=='s') {
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
      else if(c=='p'){
        pulley = value;
      }
      else if(c=='s'){
        sector = value;
      }
    } 
  }
}


  void transmission(){
  Wire.beginTransmission(8); // transmit to device #8

  uint8_t mts1Bytes[sizeof(mts)];
  memcpy(mts1Bytes, &mts, sizeof(mts));
  for (int i = 0; i < sizeof(mts); i++) {
    Wire.write(mts1Bytes[i]);
  }

  uint8_t mts2Bytes[sizeof(mvh)];
  memcpy(mts2Bytes, &mvh, sizeof(mvh));
  for (int i = 0; i < sizeof(mvh); i++) {
    Wire.write(mts2Bytes[i]);
  }

  Wire.endTransmission(); // stop transmitting
  }

  
void reception(){
  // request data from slave
  Wire.requestFrom(8, sizeof(sts) + sizeof(svh)); // request data from slave device #8

  uint8_t sposition1Bytes[sizeof(sts)];
  for (int i = 0; i < sizeof(sts); i++) {
    sposition1Bytes[i] = Wire.read();
  }
  memcpy(&sts, sposition1Bytes, sizeof(sts));

  uint8_t mposition2Bytes[sizeof(svh)];
  for (int i = 0; i < sizeof(svh); i++) {
    mposition2Bytes[i] = Wire.read();
  }
  memcpy(&svh, mposition2Bytes, sizeof(svh));
}