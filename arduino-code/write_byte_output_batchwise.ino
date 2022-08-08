// I2C and Lidar Libs
#include <Wire.h>
#include <VL53L0X.h>
#include "TimerOne.h" // Interupt

#define XSHUT_pin8 10
#define XSHUT_pin7 9
#define XSHUT_pin6 8
#define XSHUT_pin5 7
#define XSHUT_pin4 6
#define XSHUT_pin3 5
#define XSHUT_pin2 4
//#define XSHUT_pin1 3 //not required for address change

//ADDRESS_DEFAULT 0b0101001 or 41
//#define Sensor1_newAddress 41 //not required for address change
#define Sensor2_newAddress 42 
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
#define Sensor6_newAddress 46
#define Sensor7_newAddress 47
#define Sensor8_newAddress 48

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
VL53L0X Sensor5;
VL53L0X Sensor6;
VL53L0X Sensor7;
VL53L0X Sensor8;

// Define Motor Pins
#define enA 11
#define in1 12
#define in2 13
int pinEncoderPos = 2; // digital sensor input

//Define Variables we'll be connecting to
int motor_power = 150; //original motor power
double goal_mspr = 500; // we want two rotations per second
volatile int mspr; // ms per rotation

// Lidar distance variables
int dist1;
int dist2;
int dist3;
int dist4;
int dist5;
int dist6;
int dist7;
int dist8;

// Lidar sensor angle variables
int angle1;
int angle2;
int angle3;
int angle4;
int angle5;
int angle6;
int angle7;
int angle8;

int max_range = 3000;

// for the interupt
volatile int angle = 0;
volatile float ratio = 0;
volatile bool zeroPos = false;
volatile int tickDurationLastTime = 10;
volatile int tickDuration = 1;
volatile int tickAngle = 0;
volatile unsigned long tickLastTime = 0;
volatile unsigned long msLastTime = 0;

//############################################
//##               SETUP                    ##
//############################################
void setup() 
{
  /*WARNING*/
  
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  //pinMode(XSHUT_pin1, OUTPUT);//not required for address change
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  pinMode(XSHUT_pin5, OUTPUT);
  pinMode(XSHUT_pin6, OUTPUT);
  pinMode(XSHUT_pin7, OUTPUT);
  pinMode(XSHUT_pin8, OUTPUT);

  //Motor
  pinMode(enA, OUTPUT); //PWM for rotation speed
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Sensor
  Serial.begin(115200);
  pinMode (pinEncoderPos, INPUT); 

  // start I2C for lidars
  Wire.begin();

  //Change address of sensor and power up next one  
  Sensor8.setAddress(Sensor8_newAddress);
  pinMode(XSHUT_pin8, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  
  Sensor7.setAddress(Sensor7_newAddress);
  pinMode(XSHUT_pin7, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"  

  Sensor6.setAddress(Sensor6_newAddress);
  pinMode(XSHUT_pin6, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  
  Sensor5.setAddress(Sensor5_newAddress);
  pinMode(XSHUT_pin5, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"

  Sensor4.setAddress(Sensor4_newAddress);
  pinMode(XSHUT_pin4, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  
  Sensor3.setAddress(Sensor3_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"

  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"

  //Sensor1.setAddress(Sensor1_newAddress);
  //pinMode(XSHUT_pin1, INPUT);
  //delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"


  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor4.init();
  Sensor5.init();
  Sensor6.init();
  Sensor7.init();
  Sensor8.init();

  
  Sensor1.setMeasurementTimingBudget(20000);
  Sensor2.setMeasurementTimingBudget(20000);
  Sensor3.setMeasurementTimingBudget(20000);
  Sensor4.setMeasurementTimingBudget(20000);
  Sensor5.setMeasurementTimingBudget(20000);
  Sensor6.setMeasurementTimingBudget(20000);
  Sensor7.setMeasurementTimingBudget(20000);
  Sensor8.setMeasurementTimingBudget(20000);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor3.startContinuous();
  Sensor4.startContinuous();
  Sensor5.startContinuous();
  Sensor6.startContinuous();
  Sensor7.startContinuous();
  Sensor8.startContinuous();

  // Spin Motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  Timer1.initialize(1389); // 720deg in 1 second, so 1 deg every 1/720 sec
  Timer1.attachInterrupt(timer);  
    
  //Serial.println("Ready");
  attachInterrupt(digitalPinToInterrupt(pinEncoderPos), encoderTick, RISING);    

  // init at half speed
  analogWrite(enA, motor_power); // 0-255
  delay(200);

}


//############################################
//##           EXTRA FUNCTIONS              ##
//############################################
void timer(){
  angle++; // set to next angle step
}

// Only triggered when switch High->Low (so if a tooth comes into view) (gap is HIGH)
void encoderTick(){
  volatile unsigned long ms = millis();
  volatile int tickDuration = ms-tickLastTime;   // compute transition time    

  // Filter False Interupts
  if (tickDuration > 1){ // DO NOT COMMENT OUT!
    // This is a hardcoded value depending on the 2Hz frequency, adjust accordinly for slower or faster rotations
    if (2*tickDuration>3*tickDurationLastTime && tickDuration>50){
        // zero pos      
        zeroPos = true;
        tickAngle = 0;     // reset angle to zero                     
        angle = tickAngle;
        mspr = ms-msLastTime; // Milliseconds per rotations
        msLastTime = ms;
    } else {
      tickAngle+=24; // 360deg divided by the "15" teeth
      angle = tickAngle; // We are at a tooth, so pull the angle 
    }
    
    tickLastTime = ms; // remember transition time  
    tickDurationLastTime = tickDuration; // remember duration time
 
  }

}



//############################################
//##               MAIN LOOP                ##
//############################################

void loop()
{

  if (zeroPos){
  
      zeroPos = false;

      // send the zero sync
      Serial.write(254); // START Codon 1
      Serial.write(253); // START Codon 2

      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);

      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
      
      Serial.write(252); // END Codon 1
      Serial.write(251); // END Codon 2
      Serial.flush();    

      if(mspr>goal_mspr+5){ // we take too long for one rotation, speed up
        motor_power++;
      }
      else if(mspr<goal_mspr-5){ // we take too short for one rotation, slow down
        motor_power--;
      }

      analogWrite(enA, int(motor_power)); 

          
  }

  dist1 = Sensor1.readRangeContinuousMillimeters();
  angle1 = angle;
  
  dist2 = Sensor2.readRangeContinuousMillimeters();
  angle2 = angle;
  
  dist3 = Sensor3.readRangeContinuousMillimeters();
  angle3 = angle;

  dist4 = Sensor4.readRangeContinuousMillimeters();
  angle4 = angle;

  dist5 = Sensor5.readRangeContinuousMillimeters();
  angle5 = angle;

  dist6 = Sensor6.readRangeContinuousMillimeters();
  angle6 = angle;

  dist7 = Sensor7.readRangeContinuousMillimeters();
  angle7 = angle;

  dist8 = Sensor8.readRangeContinuousMillimeters();
  angle8 = angle;

  if (dist1 > max_range){dist1=max_range;}
  if (dist2 > max_range){dist2=max_range;}
  if (dist3 > max_range){dist3=max_range;}
  if (dist4 > max_range){dist4=max_range;}
  if (dist5 > max_range){dist5=max_range;}
  if (dist6 > max_range){dist6=max_range;}
  if (dist7 > max_range){dist7=max_range;}
  if (dist8 > max_range){dist8=max_range;}

  Serial.write(254); // START Codon 1
  Serial.write(253); // START Codon 2

  Serial.write(angle1 >> 8);  
  Serial.write(angle1 & 0xFF);    
  Serial.write(dist1 >> 8);  
  Serial.write(dist1 & 0xFF);    

  Serial.write(angle2 >> 8);  
  Serial.write(angle2 & 0xFF);    
  Serial.write(dist2 >> 8);  
  Serial.write(dist2 & 0xFF);     

  Serial.write(angle3 >> 8);  
  Serial.write(angle3 & 0xFF);    
  Serial.write(dist3 >> 8);  
  Serial.write(dist3 & 0xFF);  

  Serial.write(angle4 >> 8);  
  Serial.write(angle4 & 0xFF);    
  Serial.write(dist4 >> 8);  
  Serial.write(dist4 & 0xFF);       

  Serial.write(angle5 >> 8);  
  Serial.write(angle5 & 0xFF);    
  Serial.write(dist5 >> 8);  
  Serial.write(dist5 & 0xFF);   

  Serial.write(angle6 >> 8);  
  Serial.write(angle6 & 0xFF);    
  Serial.write(dist6 >> 8);  
  Serial.write(dist6 & 0xFF);    

  Serial.write(angle7 >> 8);  
  Serial.write(angle7 & 0xFF);    
  Serial.write(dist7 >> 8);  
  Serial.write(dist7 & 0xFF);    

  Serial.write(angle8 >> 8);  
  Serial.write(angle8 & 0xFF);    
  Serial.write(dist8 >> 8);  
  Serial.write(dist8 & 0xFF);    

  Serial.write(252); // END Codon 1
  Serial.write(251); // END Codon 2
  Serial.flush();
}
