/*

Microcontroller code for the Powerbot. This code is based off the work presented at;

http://rcarduino.blogspot.com/2012/05/interfacing-rc-channels-to-l293d-motor.html

TODO
------
- Add calibration button (someday/maybe)
- Possible velocity smoothing or attenuation (someday/maybe, trim on controller works for the moment)
- Directions are reversed in the mapping for moving turns (Revesed mapped axes, needs testing, 2015-04-18 15:40:09)
- When moving at full throttle, the inside wheel on a turn buries to zero velocity despite a very light turn input on the controller
- Narrow the velocity range for sit and spin (Reduced IDLE_MAX from 80 to 60, test for suitability, 2015-04-18 15:47:02)
- Calibration mode exits early (~5 seconds) on Max32 platform
- Upon leaving calibration mode, the robot goes into full spin in one direction. We've contacted Digilent for support (2015-04-23 19:55:00)

Pins;
0 - 
1 - 
2 - THROTTLE_IN
3 - STEERING_IN
4 - 
5 - L_DIR_P
6 - L_INHIBIT_P
7 - R_DIR_P
8 - R_INHIBIT_P
9 - PROGRAM_PIN (future)
10 - L_PWM_P
11 - R_PWM_P
12 - 
13 -

*/

#include <EEPROM.h>

// Calibration defaults
#define RC_NEUTRAL 1500
#define RC_MAX 3000
#define RC_MIN 0
#define RC_DEADBAND 40

#define PWM_MIN 0
#define PWM_MAX 255

#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2
#define IDLE_MAX 60

// Calibration variables
uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringCenter = RC_NEUTRAL;
uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleCenter = RC_NEUTRAL;

// Assign pins
#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3
#define LEFT_DIR 5
#define LEFT_INHIBIT 6
#define RIGHT_DIR 7
#define RIGHT_INHIBIT 8
#define PROGRAM_PIN 9
#define PWM_SPEED_LEFT 10
#define PWM_SPEED_RIGHT 11

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

volatile uint8_t bUpdateFlagsShared;

// Local copies of interrupt variables
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define MODE_RUN 0
#define MODE_PROGRAM 1

uint8_t gMode;
uint32_t ulProgramModeExitTime; 

/* Auto-calibration
// Index into the EEPROM Storage assuming a 0 based array of uint16_t
// Data to be stored low byte, high byte
#define EEPROM_INDEX_STEERING_MIN 0
#define EEPROM_INDEX_STEERING_MAX 1
#define EEPROM_INDEX_STEERING_CENTER 2
#define EEPROM_INDEX_THROTTLE_MIN 3
#define EEPROM_INDEX_THROTTLE_MAX 4
#define EEPROM_INDEX_THROTTLE_CENTER 5
*/

void setup()
{
  Serial.begin(9600);

  Serial.println("RCChannelsTo293");

  attachInterrupt(0,calcThrottle,CHANGE); // INT0 = THROTTLE_IN_PIN
  attachInterrupt(1,calcSteering,CHANGE); // INT1 = STEERING_IN_PIN

// Set I/O pin directions
  pinMode(PWM_SPEED_LEFT,OUTPUT);
  pinMode(PWM_SPEED_RIGHT,OUTPUT);
  pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_DIR,OUTPUT);
  pinMode(LEFT_INHIBIT,OUTPUT);
  pinMode(RIGHT_INHIBIT,OUTPUT);
  // pinMode(PROGRAM_PIN, INPUT); Add back at some point

// Set Inhibit pins static low
  digitalWrite(LEFT_INHIBIT,LOW);
  digitalWrite(RIGHT_INHIBIT,LOW);
  
  // readSettingsFromEEPROM(); Auto-calibration for the moment
  ulProgramModeExitTime = millis() + 10000;
  gMode = MODE_PROGRAM;
}

void loop()
{
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint8_t bUpdateFlags;

  // Check shared update flags to see if any channels have a new signal, update locals
  if(bUpdateFlagsShared)
  {
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
    
    bUpdateFlagsShared = 0;

    interrupts();
  }
  
  // Calibration mode, automatically entered on boot
  if(gMode == MODE_PROGRAM)
  {    
    unThrottleCenter = unThrottleIn;
    unSteeringCenter = unSteeringIn;
    
    gDirection = DIRECTION_STOP;
    
    delay(20);
    
   if(ulProgramModeExitTime < millis())
   {
     ulProgramModeExitTime = 0;
     gMode = MODE_RUN;
     // writeSettingsToEEPROM(); Auto-calibration
   }
   else
   {
     if(unThrottleIn > unThrottleMax && unThrottleIn <= RC_MAX)
     {
       unThrottleMax = unThrottleIn;
     }
     else if(unThrottleIn < unThrottleMin && unThrottleIn >= RC_MIN)
     {
       unThrottleMin = unThrottleIn;
     }
     
     if(unSteeringIn > unSteeringMax && unSteeringIn <= RC_MAX)
     {
       unSteeringMax = unSteeringIn;
     }
     else if(unSteeringIn < unSteeringMin && unSteeringIn >= RC_MIN)
     {
       unSteeringMin = unSteeringIn;
     }
   }
  }
  
  if(gMode == MODE_RUN)
  {
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
      
      if(unThrottleIn > unThrottleCenter)
      {
        gThrottle = map(unThrottleIn,unThrottleCenter,unThrottleMax,PWM_MIN,PWM_MAX);
        gThrottleDirection = DIRECTION_FORWARD;
      }
      else
      {
        gThrottle = map(unThrottleIn,unThrottleMin,unThrottleCenter,PWM_MAX,PWM_MIN);
        gThrottleDirection = DIRECTION_REVERSE;
      }
  
      if(gThrottle < IDLE_MAX)
      {
        gGear = GEAR_IDLE;
      }
      else
      {
        gGear = GEAR_FULL;
      }
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      uint8_t throttleLeft = gThrottle;
      uint8_t throttleRight = gThrottle;
  
      gDirection = gThrottleDirection;
      
      unSteeringIn = constrain(unSteeringIn,unSteeringMin,unSteeringMax);
  
      switch(gGear)
      {
        
      case GEAR_IDLE: // Idle causes the robot to sit in place and spin
        if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_RIGHT;
          throttleRight = throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,PWM_MIN,PWM_MAX);
        }
        else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_LEFT;
          throttleRight = throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MAX,PWM_MIN);
        }
        break;
      
      case GEAR_FULL: // If not idle proportionally restrain inside track to turn vehicle around it
        if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          throttleRight = map(unSteeringIn,unSteeringCenter,unSteeringMax,gThrottle,PWM_MIN);
        }
        else if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MIN,gThrottle);
        }
        break;
      }
      analogWrite(PWM_SPEED_LEFT,throttleLeft);
      analogWrite(PWM_SPEED_RIGHT,throttleRight);
    }
  }
  
  if((gDirection != gOldDirection) || (gGear != gOldGear))
  {
    gOldDirection = gDirection;
    gOldGear = gGear;

    digitalWrite(LEFT_DIR,HIGH);
    digitalWrite(RIGHT_DIR,LOW);

    switch(gDirection)
    {
    case DIRECTION_FORWARD:
      digitalWrite(LEFT_DIR,HIGH);
      digitalWrite(RIGHT_DIR,HIGH);
      break;
    case DIRECTION_REVERSE:
      digitalWrite(LEFT_DIR,LOW);
      digitalWrite(RIGHT_DIR,LOW);
      break;
    case DIRECTION_ROTATE_LEFT:
      digitalWrite(LEFT_DIR,HIGH);
      digitalWrite(RIGHT_DIR,LOW);
      break;
    case DIRECTION_ROTATE_RIGHT:
      digitalWrite(LEFT_DIR,LOW);
      digitalWrite(RIGHT_DIR,HIGH);
      break;
    case DIRECTION_STOP:
      digitalWrite(LEFT_DIR,HIGH);
      digitalWrite(RIGHT_DIR,LOW);
      break;
    }
  }

  bUpdateFlags = 0;
}

// Throttle ISR
void calcThrottle()
{
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

// Steering ISR
void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

/*

This section of code is irrelevant until we move away from auto-calibration.

// Not currently used, auto-calibration
void readSettingsFromEEPROM()
{
  unSteeringMin = readChannelSetting(EEPROM_INDEX_STEERING_MIN);
  if(unSteeringMin < RC_MIN || unSteeringMin > RC_NEUTRAL)
  {
    unSteeringMin = RC_MIN;
  }
  Serial.println(unSteeringMin);

  unSteeringMax = readChannelSetting(EEPROM_INDEX_STEERING_MAX);
  if(unSteeringMax > RC_MAX || unSteeringMax < RC_NEUTRAL)
  {
    unSteeringMax = RC_MAX;
  }
  Serial.println(unSteeringMax);
  
  unSteeringCenter = readChannelSetting(EEPROM_INDEX_STEERING_CENTER);
  if(unSteeringCenter < unSteeringMin || unSteeringCenter > unSteeringMax)
  {
    unSteeringCenter = RC_NEUTRAL;
  }
  Serial.println(unSteeringCenter);

  unThrottleMin = readChannelSetting(EEPROM_INDEX_THROTTLE_MIN);
  if(unThrottleMin < RC_MIN || unThrottleMin > RC_NEUTRAL)
  {
    unThrottleMin = RC_MIN;
  }
  Serial.println(unThrottleMin);

  unThrottleMax = readChannelSetting(EEPROM_INDEX_THROTTLE_MAX);
  if(unThrottleMax > RC_MAX || unThrottleMax < RC_NEUTRAL)
  {
    unThrottleMax = RC_MAX;
  }
  Serial.println(unThrottleMax);
  
  unThrottleCenter = readChannelSetting(EEPROM_INDEX_THROTTLE_CENTER);
  if(unThrottleCenter < unThrottleMin || unThrottleCenter > unThrottleMax)
  {
    unThrottleCenter = RC_NEUTRAL;
  }
  Serial.println(unThrottleCenter);
}

// Not currently used, auto-calibration
void writeSettingsToEEPROM()
{
  writeChannelSetting(EEPROM_INDEX_STEERING_MIN,unSteeringMin);
  writeChannelSetting(EEPROM_INDEX_STEERING_MAX,unSteeringMax);
  writeChannelSetting(EEPROM_INDEX_STEERING_CENTER,unSteeringCenter);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_MIN,unThrottleMin);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_MAX,unThrottleMax);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_CENTER,unThrottleCenter);
            
  Serial.println(unSteeringMin);
  Serial.println(unSteeringMax);
  Serial.println(unSteeringCenter);
  Serial.println(unThrottleMin);
  Serial.println(unThrottleMax);
  Serial.println(unThrottleCenter);
}

// Not currently used, auto-calibration
uint16_t readChannelSetting(uint8_t nStart)
{
  uint16_t unSetting = (EEPROM.read((nStart*sizeof(uint16_t))+1)<<8);
  unSetting += EEPROM.read(nStart*sizeof(uint16_t));

  return unSetting;
}

// Not currently used, auto-calibration
void writeChannelSetting(uint8_t nIndex,uint16_t unSetting)
{
  EEPROM.write(nIndex*sizeof(uint16_t),lowByte(unSetting));
  EEPROM.write((nIndex*sizeof(uint16_t))+1,highByte(unSetting));
}
*/
