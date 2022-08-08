#include <Arduino.h>
#include "esp_timer.h"
#include <Wire.h>
#include <VL53L0X.h>

 
#define XSHUT_pin7 23  // N/A for Arduino         // Cable colors lightBlue-Blue
#define XSHUT_pin6 19  //9                        // Cable colors White-White
#define XSHUT_pin5 18  //8                        // Cable colors Brown-Brown
#define XSHUT_pin4 5   //7                        // Cable colors Orange-Orange
#define XSHUT_pin3 17  //6                        // Cable colors Yellow-Yellow
#define XSHUT_pin2 16  //5                        // Cable colors Green-LightGreen
#define XSHUT_pin1 4   //4                        // Cable colors Blue-Blue
#define XSHUT_pin0 0    

//ADDRESS_DEFAULT 0b0101001 or 41
#define Sensor0_newAddress 42 
#define Sensor1_newAddress 43 
#define Sensor2_newAddress 44
#define Sensor3_newAddress 45
#define Sensor4_newAddress 46
#define Sensor5_newAddress 47
#define Sensor6_newAddress 48
#define Sensor7_newAddress 41

VL53L0X Sensor0;
VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
VL53L0X Sensor5;
VL53L0X Sensor6;
VL53L0X Sensor7;

// Define Motor Pins
#define enA 25 // analog output (Uno: 11 / ESP32: 25
#define in1 33 // digital output (Uno: 12 / ESP32: 33 (über 25)
#define in2 32 // digital output (Uno: 13 / ESP32: 32 (über 33)
int pinEncoderPos = 13; //Uno: 2; // digital sensor input

//Define Variables we'll be connecting to
int motor_power = 150; //original motor power
int goal_mspr = 500; // we want two rotations per second
volatile int mspr; // ms per rotation

// Lidar distance variables for each channel
int dist0;
int dist1;
int dist2;
int dist3;
int dist4;
int dist5;
int dist6;
int dist7;

// Lidar sensor angle variables for each channel
int angle0;
int angle1;
int angle2;
int angle3;
int angle4;
int angle5;
int angle6;
int angle7;

int max_range = 3000;

// for the interupt
volatile int angle = 0;
volatile float ratio = 0;
volatile bool zeroPos = false;
volatile bool tick = false;
volatile int tickDurationLastTime = 10;
volatile int tickAngle = 0;
volatile int tickLastTime = 0;
volatile int msLastTime = 0;

//// Timer 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//############################################
//##           EXTRA FUNCTIONS              ##
//############################################
void onTimer(){
  angle++; // set to next angle step
}

// Only triggered when switch High->Low (so if a tooth comes into view) (gap is HIGH)
void IRAM_ATTR encoderTick(){
    volatile unsigned long ms = millis();
    volatile int tickDuration = ms-tickLastTime;   // compute transition time 

    // This is a hardcoded value depending on the 2Hz frequency, adjust accordinly for slower or faster rotations
    if (2*tickDuration>3*tickDurationLastTime && tickDuration>50){
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

//############################################
//##               SETUP                    ##
//############################################
void setup() 
{
  /*WARNING*/
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(XSHUT_pin0, OUTPUT);
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  pinMode(XSHUT_pin5, OUTPUT);
  pinMode(XSHUT_pin6, OUTPUT);
  pinMode(XSHUT_pin7, OUTPUT); 

  //Motor
  pinMode(enA, OUTPUT); //PWM for rotation speed
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Serial Output
  Serial.begin(115200);

  // Rotation encoder
  pinMode (pinEncoderPos, INPUT); 

  // start I2C for lidars
  Wire.begin();

  //Change address of sensor and power up next one  
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

  Sensor1.setAddress(Sensor1_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"

  Sensor0.setAddress(Sensor0_newAddress);
  pinMode(XSHUT_pin0, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"


  Sensor0.init();
  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor4.init();
  Sensor5.init();
  Sensor6.init();
  Sensor7.init();

  Sensor0.setMeasurementTimingBudget(20000);
  Sensor1.setMeasurementTimingBudget(20000);
  Sensor2.setMeasurementTimingBudget(20000);
  Sensor3.setMeasurementTimingBudget(20000);
  Sensor4.setMeasurementTimingBudget(20000);
  Sensor5.setMeasurementTimingBudget(20000);
  Sensor6.setMeasurementTimingBudget(20000);
  Sensor7.setMeasurementTimingBudget(20000);


  // Lets use a timeout
  Sensor0.setTimeout(0);
  Sensor1.setTimeout(0);
  Sensor2.setTimeout(0);
  Sensor3.setTimeout(0);
  Sensor4.setTimeout(0);
  Sensor5.setTimeout(0);
  Sensor6.setTimeout(0);
  Sensor7.setTimeout(0);

  
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor0.startContinuous(10); // do not change to zero, ESP32 tends to hang infinitely when changed to zero
  Sensor1.startContinuous(10);
  Sensor2.startContinuous(10);
  Sensor3.startContinuous(10);
  Sensor4.startContinuous(10);
  Sensor5.startContinuous(10);
  Sensor6.startContinuous(10);
  Sensor7.startContinuous(10);

  // Start Spinning Motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // ESP32 Timer
  timer = timerBegin(0, 80, true); // prescaler of 80 is specifying that value in microseconds (CPU is 80MHz)
  timerAttachInterrupt(timer, &onTimer, false);
  timerAlarmWrite(timer, 1389, true); // 720deg in 1 second, so 1 deg every 1/720 sec  (1/1/720*1000*1000)
  timerAlarmEnable(timer);

  // Attach the interupt for rotation encoder
  attachInterrupt(pinEncoderPos, encoderTick, RISING);

  // init at half speed
  analogWrite(enA, motor_power); // 0-255
  delay(200); // give some time before starting just in case
}

//############################################
//##               MAIN LOOP                ##
//############################################

void loop()
{

  // if the encoder defined the zero degree position
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

  dist0 = Sensor7.readRangeContinuousMillimeters();
  if (Sensor7.timeoutOccurred()) { dist0 = 0; }
  angle0 = angle;

  dist1 = Sensor0.readRangeContinuousMillimeters();
  if (Sensor0.timeoutOccurred()) { dist1 = 0; }
  angle1 = angle;

  dist2 = Sensor1.readRangeContinuousMillimeters();
  if (Sensor1.timeoutOccurred()) { dist2 = 0; }
  angle2 = angle;

  dist3 = Sensor2.readRangeContinuousMillimeters();
  if (Sensor2.timeoutOccurred()) { dist3 = 0; }
  angle3 = angle;

  dist4 = Sensor3.readRangeContinuousMillimeters();
  if (Sensor3.timeoutOccurred()) { dist4 = 0; }
  angle4 = angle;

  dist5 = Sensor4.readRangeContinuousMillimeters();
  if (Sensor4.timeoutOccurred()) { dist5 = 0; }
  angle5 = angle;

  dist6 = Sensor5.readRangeContinuousMillimeters();
  if (Sensor5.timeoutOccurred()) { dist6 = 0; }
  angle6 = angle;

  dist7 = Sensor6.readRangeContinuousMillimeters();
  if (Sensor6.timeoutOccurred()) { dist6 = 0; }
  angle7 = angle;


  // Values over max_range are unreliable, set to defined max range
  if (dist0 > max_range){dist0=max_range;}
  if (dist1 > max_range){dist1=max_range;}
  if (dist2 > max_range){dist2=max_range;}
  if (dist3 > max_range){dist3=max_range;}
  if (dist4 > max_range){dist4=max_range;}
  if (dist5 > max_range){dist5=max_range;}
  if (dist6 > max_range){dist6=max_range;}
  if (dist7 > max_range){dist7=max_range;}

  Serial.write(254); // START Codon 1
  Serial.write(253); // START Codon 2

  // Angle and Distance can be higher than 255, we need two bytes to send the values
  // split the values into the upper and lower bytes, send them in a fixed order

  Serial.write(angle0 >> 8);
  Serial.write(angle0 & 0xFF);    
  Serial.write(dist0 >> 8);  
  Serial.write(dist0 & 0xFF);    

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

  Serial.write(252); // END Codon 1
  Serial.write(251); // END Codon 2       
  Serial.flush();  
}
