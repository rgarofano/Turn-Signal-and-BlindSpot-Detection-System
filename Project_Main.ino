/*-----------------------------------------------------------
 * Project_Main.ino 
 *    software to be uploaded to Arduino Uno for bike helmet turn signal indicator 
 *    and blind spot detection system.
 * 
 * 27 NOV 2019  Group Omnicron 
 * 
 ----------------------------------------------------------*/

 /***PIN MAPPINGS*****************************************/
 //LEFT LED --------------> PIN 11
 //RIGHT LED----------------> PIN 10
 //PROXIMITY NOTIFIER------> PIN 9
 //IMU SCL-----------------> A5
 //IMU SDA-----------------> A4
 //PROXY DETECTOR ECHO ----> PIN 5 
 //PROXY DETECTOR TRIG ----> PIN 6 

/***INCLUDES**********************************************/
#include <Wire.h>
#include "SR04.h"


/***DEFINES**********************************************/
#define MPU9250_ADDRESS     0x68
#define ACC_SCALE_4_G       0x08

#define DIST_TRIG_PIN       6
#define DIST_ECHO_PIN       5
#define PROXIMITY_PIN       9
#define MIN_PROXIMITY_CM    300

#define LED_RIGHT           11
#define LED_LEFT            10

#define NO_TURN             0
#define TURN_RIGHT          1
#define TURN_LEFT           2

#define MIN_CONSEC_SIGNALS  3

#define MAG_ADDRESS 0x0C
 
#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18


/***GLOBAL VARIABLES**********************************************/
int             currentLED = NO_TURN;
unsigned long   Signal_Timer_Start = 0; 
unsigned long   Current_Timer = 0; 

SR04            sr04(DIST_ECHO_PIN, DIST_TRIG_PIN); 

/***FUNCTION DECLARATIONS**********************************************/
void I2Cread(uint8_t, uint8_t, uint8_t, uint8_t*);
void I2Cwritebyte(uint8_t, uint8_t, uint8_t);
int determineTurnType(uint8_t, uint8_t, uint8_t);
void setLEDs(int); 


/***FUNCTION DEFINITIONS**********************************************/

/////////////////////////////////////////
///
/// Main function for arduino code 
/// All initalization of sensors occurs here
///
/////////////////////////////////////////
void setup() {
  
  // Arduino initializations
  Wire.begin();
  Serial.begin(9600);

  //Configure pins for output
  pinMode(PROXIMITY_PIN,OUTPUT);
  pinMode(LED_RIGHT,OUTPUT);
  pinMode(LED_LEFT,OUTPUT);


  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_SCALE_4_G);
  

  return;
}

/////////////////////////////////////////
///
/// Where action of code takes place. This function
/// is called on a loop until program termination
///
/////////////////////////////////////////
void loop() 
{
  //Define Local Variables
  int         distanceReading = 0; 

  uint8_t     IMU_measurement[14];
  int16_t     accel_x = 0;
  int16_t     accel_y = 0;
  int16_t     accel_z = 0;

  int         Turn_Type = NO_TURN;
  
  static int  LeftTurnCount = 0; 
  static int  RightTurnCount = 0; 


  //--------------------------------------
  // ::: ultrasonic sensor :::
  //--------------------------------------

  //Take Measurement from distance sensor in cm
  distanceReading = sr04.Distance();

  //Turn on notifier if object is within 300 cm
  if (distanceReading < MIN_PROXIMITY_CM)
    digitalWrite(PROXIMITY_PIN,HIGH);
  else 
    digitalWrite(PROXIMITY_PIN,LOW);


  //--------------------------------------
  // ::: accelerometer :::
  //--------------------------------------
   
  // Read IMU accelerometer values
  I2Cread(MPU9250_ADDRESS,0x3B,14,IMU_measurement);
  
  //Create 16 bits values from measurement array 
  accel_x = IMU_measurement[0] << 8 | IMU_measurement[1];
  accel_y = IMU_measurement[2] << 8 | IMU_measurement[3];
  accel_z = IMU_measurement[4] << 8 | IMU_measurement[5];
  
  // change acceleration values to m/s^2
  accel_x /= 980;
  accel_y /= 980;
  accel_z /= 980;

  // call compare function to determine if there is a left, right, or no turn
  Turn_Type = determineTurnType(accel_x, accel_y, accel_z);

  
  Serial.print("IMU Measurements: ");
  Serial.print(" X: ");
  Serial.print(accel_x);
  Serial.print(" Y: ");
  Serial.print(accel_y);
  Serial.print(" Z: ");
  Serial.print(accel_z);
  Serial.println(' ');
 


  //--------------------------------------
  // ::: LEDs :::
  //--------------------------------------

  
  if (Turn_Type == TURN_RIGHT)
  {
    //Reset left turn couter since we have a right turn signal 
    LeftTurnCount = 0; 
    
    if (++RightTurnCount >= MIN_CONSEC_SIGNALS)
    {
      //Send message to turn on LED for right turn
      setLEDs(TURN_RIGHT);

      //Reset right turn counter
      RightTurnCount = 0;
    }
  }
  else if (Turn_Type == TURN_LEFT)
  {
    //Reset right turn couter since we have a left turn signal 
    RightTurnCount = 0; 
    
    if (++LeftTurnCount >= MIN_CONSEC_SIGNALS)
    {
      //Send message to turn on LED for left turn
      setLEDs(TURN_LEFT);

      //Reset turn signal counter
      LeftTurnCount = 0;
    }
  }
  else //Turn_Type == NO_TURN
  {
    //No turn signal was recorded, reset both turn signal counters
    LeftTurnCount  = 0; 
    RightTurnCount = 0; 
  }

  //Check if LED timer has expired and LEDs need to be turned off
  if (currentLED == TURN_RIGHT || currentLED == TURN_LEFT)
  {
    Current_Timer = millis();
    
    if (Current_Timer - Signal_Timer_Start >= 5000) //If 5sec has elapsed
    {
      //Turn off LEDs
      currentLED = NO_TURN; 
      setLEDs(NO_TURN); 
    }
  }


  Serial.print("Results: ");
  Serial.print(Turn_Type);
  Serial.print('\t');
  Serial.print(distanceReading);
  Serial.println(' ');
  
  delay(200); //want to sample sensors at frequency of 5Hz
  return;
}


///////////////////////////////////////////////////
///
/// Accept acceleration values and determine which 
/// type of turn is being signalled
///
///////////////////////////////////////////////////
int determineTurnType(int16_t x, int16_t y, int16_t z)
{
  int rc;  //return commmand

  //take only magnitude of sensor values to compare 
  x = abs(x);
  y = abs(y);
  z = abs(z);
  
  if (x > y && x > z)
    rc = TURN_RIGHT;  
  else if(y > x && y > z)
    rc = TURN_LEFT;
  else 
    rc = NO_TURN; 

  return rc;
}

///////////////////////////////////////
///
/// Accept the turn command and set the LEDs 
/// accordingly based on current state
///
///////////////////////////////////////
void setLEDs(int Turn_Type)
{

  switch(Turn_Type)
  {
    case NO_TURN: 
    {
      if (currentLED != NO_TURN)
        break;  //ignore and allow sequence to complete 
      else 
      {
        //Ensure all LEDs are off and break
        digitalWrite(LED_LEFT,LOW);
        digitalWrite(LED_RIGHT,LOW);
        break;
      }
        
      break;    //Should not execute, here for safety 
    }
    case TURN_RIGHT: 
    {
      if (currentLED == TURN_RIGHT)
      {
        //Reset timer and break
        Signal_Timer_Start = millis(); 
        break; 
      }
      else
      {
        //Ensure LEFT_TURN is off and turn on RIGHT_TURN 
        digitalWrite(LED_RIGHT,HIGH);
        digitalWrite(LED_LEFT,LOW);

        currentLED = TURN_RIGHT; 
        Signal_Timer_Start = millis(); 
        break;
      }

      break;    //Should not execute, here for safety 
    }
    case TURN_LEFT:
    {
      if (currentLED == TURN_LEFT)
      {
        //Reset timer and break
        Signal_Timer_Start = millis(); 
        break; 
      }
      else
      {
        //Ensure RIGHT_TURN is off and turn on LEFT_TURN 
        digitalWrite(LED_LEFT,HIGH);
        digitalWrite(LED_RIGHT,LOW);

        currentLED = TURN_LEFT; 
        Signal_Timer_Start = millis(); 
        break;
      }

      break;    //Should not execute, here for safety 
    }
    default: 
    {
      //Should not execute
      //Turn off all LEDs
      digitalWrite(LED_LEFT,LOW);
      digitalWrite(LED_RIGHT,LOW);
      break;
    }
  }

  return; 
}


/////////////////////////////////////////
///
/// This function read Nbytes bytes from I2C device at address Address. 
/// Put read bytes starting at register Register in the Data array. 
///
/////////////////////////////////////////
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
   
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
  Data[index++] = Wire.read();

  return; 
}
 
///////////////////////////////////////
///
/// Write a byte (Data) in device (Address) at register (Register)
///
///////////////////////////////////////
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();

  return;
}
