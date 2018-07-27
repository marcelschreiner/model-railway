//**************************************************************************************************
// Turntable DCC Decoder
//**************************************************************************************************
// This decoder is for turntables with 48 positions and it uses the protocoll from DSD2010.
// To get started you need to set the following things: 

// _______Address___________________________________________________________________________________
#define   DCC_ADDRESS             97        // Start Address (as defined in Central Station 2)

// _______Stepper_Position__________________________________________________________________________
#define   HOME_OFFSET_FULL_STEPS  13        // Set the offset to the home sensor
#define   FULL_STEPS_PER_POS      200L      // # of steps to go from one track/position to the next 

// _______#_of_microsteps___________________________________________________________________________
#define   NUM_MICROSTEPS          4         // Number of microsteps per full step

// _______Stepper_Speed_____________________________________________________________________________
#define   MAX_SPEED               300       // Sets the maximum permitted speed
#define   ACCELARATION            50        // Sets the acceleration/deceleration rate

// _______Pins_____________________________________________________________________________________
          //DCC_IN_PIN            2         // This pis fixed to D2
#define   HOME_SENSOR_PIN         3
#define   DRIVER_STEP_PIN         4
#define   DRIVER_DIRECTION_PIN    5
#define   DRIVER_ENABLE_PIN       6
#define   LIGHT_PIN               13

//**************************************************************************************************
// URLs of sources:
// https://www.dccinterface.com/how-to/assemblyguide/
// http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// http://mrrwa.org/download/
// http://mrrwa.org/2017/12/23/dcc-controlled-turntable-stepper-motor-driver/ 
//**************************************************************************************************

// Includes
#include  "AccelStepper.h"
#include  "NmraDcc.h"

// Random defines
#define   CLOCKWISE             false
#define   ANTI_CLOCKWISE        true
#define   ABSOLUTE              false
#define   RELATIVE              true
#define   SENSOR_ACTIVE_STATE   HIGH

bool      TurnDirection         = CLOCKWISE;
bool      FirstRunFlag          = true;
long      TargetPosition        = 0;
long      CurrentPosition       = 0;
bool      lastIsRunningState;               // True if motor is running
uint16_t  AddrShad              = 0xFFFF;
uint8_t   DirectionShad         = 0xFF;
uint8_t   OutputPowerShad       = 0xFF;

// Speed and Acceleration
#define   STEPPER_MAX_SPEED     (NUM_MICROSTEPS * MAX_SPEED)      // Sets the maximum permitted speed
#define   STEPPER_ACCELARATION  (NUM_MICROSTEPS * ACCELARATION)   // Sets the acceleration/deceleration rate
#define   STEPPER_SPEED         (STEPPER_MAX_SPEED)               // Sets the desired constant speed for use with runSpeed()

#define STEPS_PER_POSITION      (FULL_STEPS_PER_POS * NUM_MICROSTEPS)
#define STEPS_FULL_ROTATION     (STEPS_PER_POSITION * 48)
#define STEPS_HALF_ROTATION     (STEPS_PER_POSITION * 24)

// Setup the AccelStepper object for the A4988 Stepper Motor Driver
AccelStepper stepper(AccelStepper::DRIVER, DRIVER_STEP_PIN, DRIVER_DIRECTION_PIN);

// Dcc Accessory Decoder object
NmraDcc  Dcc ;

// **********************************************************************************************************************
// Move the Turntable     PositionType = ABSOLUTE/RELATIVE      Position = -24...24
// **********************************************************************************************************************
void moveTurntable( bool PositionType, int8_t Position )
{
  if(PositionType==ABSOLUTE)
  {
    // Calculate target position (in steps)
    TargetPosition = constrain(Position, 0, 23)*STEPS_PER_POSITION;

    Serial.print(F(" - Move to position "));
    Serial.print(constrain(Position, 0, 23));
    
    stepper.enableOutputs();

    if(TurnDirection==CLOCKWISE){
      Serial.print(F(", clockwise"));
      if(stepper.currentPosition()<=TargetPosition){
        stepper.moveTo(TargetPosition);
      }else{
        stepper.moveTo(STEPS_HALF_ROTATION+TargetPosition);
      }
    }else{
      Serial.print(F(", anti-clockwise"));
      if(stepper.currentPosition()>=TargetPosition){
        stepper.moveTo(TargetPosition);
      }else{
        stepper.moveTo(TargetPosition-STEPS_HALF_ROTATION);
      }
    }
  }
  
  else if(PositionType==RELATIVE)
  {
    TargetPosition = constrain(Position, -24, 24) * STEPS_PER_POSITION + stepper.targetPosition();
    stepper.enableOutputs();

    Serial.print(F(" - Move relative "));
    Serial.print(constrain(Position, -24, 24));
    
    stepper.moveTo(TargetPosition);
  }

  Serial.print(F(" ("));
  Serial.print(stepper.targetPosition());
  Serial.println(F(")"));
}



// **********************************************************************************************************************
// This function is called whenever a normal DCC Turnout Packet is received
// **********************************************************************************************************************
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  // Make Direction valid with all inputs
  if(Direction!=0){
    Direction = 1;
  }
  
  // Check if address or direction changed
  if ((AddrShad!=Addr)||(DirectionShad!=Direction)||(OutputPowerShad!=OutputPower))
  {
    // Check if Address is in the valid range and OutputPower == 1
    if((Addr>=DCC_ADDRESS)&&(Addr<=(DCC_ADDRESS+15))&&OutputPower)
    {
      
      // Turn the light on or off
      if(Addr==DCC_ADDRESS){
        digitalWrite(LED_BUILTIN, Direction);
      }

      // Check if "step right" or "step left" 
      else if(Addr==(DCC_ADDRESS+1)){
        if(Direction){
          moveTurntable(RELATIVE,-1);
        }else{
          moveTurntable(RELATIVE,1);
        }
      }

      // Check if "turn 180" 
      else if(Addr==(DCC_ADDRESS+2)){
        if(Direction){
          moveTurntable(RELATIVE,-24);
        }else{
          moveTurntable(RELATIVE,24);
        }
      }
      
      // Set the turn direction
      else if(Addr==(DCC_ADDRESS+3)){
        TurnDirection = Direction;
      }

      // Set Postiton
      else if((Addr>=(DCC_ADDRESS+4))&&(Addr<=(DCC_ADDRESS+15))){
        // Calculate position
        moveTurntable(ABSOLUTE,(((Addr-DCC_ADDRESS-4)*2)+Direction));
      }
      
    }
    
    // Save the current address and direction for the next time
    AddrShad        = Addr ;
    DirectionShad   = Direction ;
    OutputPowerShad = OutputPower;
  }
}



// **********************************************************************************************************************
// SETUP  -  Stepper Driver
// **********************************************************************************************************************
void setupStepperDriver()
{
  stepper.setPinsInverted(true, false, true);   // Its important that these commands are in this order
  stepper.setEnablePin(DRIVER_ENABLE_PIN);        // otherwise the Outputs are NOT enabled initially   
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);        // Sets the maximum permitted speed
  stepper.setAcceleration(STEPPER_ACCELARATION); // Sets the acceleration/deceleration rate
  stepper.setSpeed(STEPPER_SPEED);               // Sets the desired constant speed for use with runSpeed()

  lastIsRunningState = stepper.isRunning();
  Serial.println(F(" - Stepper driver complete"));
}



// **********************************************************************************************************************
// SETUP  -  Home Position
// **********************************************************************************************************************
bool moveToHomePosition()
{
  Serial.print(F(" - Homing the turntable..."));

  // Turn Table untill home sensor is active
  stepper.move(STEPS_FULL_ROTATION * 2);
  while(digitalRead(HOME_SENSOR_PIN) != SENSOR_ACTIVE_STATE){
    stepper.run();

    // Check if home sensor was not found before stepper stops
    if(!stepper.isRunning())
    {
      Serial.println(F("  FAILED"));
      Serial.print(F("   THE HOME SENSOR WAS NOT FOUND :("));
      while(1){}
    }
  }
  stepper.setCurrentPosition(0);

  
  // Backup again (blocking function)
  stepper.runToNewPosition(-STEPS_PER_POSITION);


  // Move slow to home position
  stepper.setMaxSpeed((int)(STEPPER_MAX_SPEED/10));
  stepper.moveTo(STEPS_PER_POSITION);
  while(digitalRead(HOME_SENSOR_PIN) != SENSOR_ACTIVE_STATE){
    stepper.run();

    // Check if home sensor was not found before stepper stops
    if(!stepper.isRunning())
    {
      Serial.println(F("  FAILED"));
      Serial.print(F("   THE HOME SENSOR WAS NOT FOUND :("));
      while(1){}
    }
  }

  // Home position was reached
  stepper.setCurrentPosition(HOME_OFFSET_FULL_STEPS * NUM_MICROSTEPS);
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);

  // Move to position zero
  stepper.runToNewPosition(0);
    
  Serial.println(F(" complete!"));
}



// **********************************************************************************************************************
// SETUP  -  DCC Decoder
// **********************************************************************************************************************
void setupDCCDecoder()
{
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  Serial.println(F(" - DCC decoder complete"));
}



// **********************************************************************************************************************
// Entry Piont of Programm
// **********************************************************************************************************************
void setup()
{
  delay(1000);
  
  Serial.begin(115200);

  Serial.println(F("***************************"));
  Serial.println(F("     Turntable Decoder"));
  Serial.println(F("***************************"));
  Serial.println(F("Settings:"));
  Serial.print  (F(" - DCC Address: "));
  Serial.println(DCC_ADDRESS);
  Serial.print  (F(" - Microsteps: 1/"));
  Serial.println(NUM_MICROSTEPS);
  Serial.print  (F(" - Full steps per position: "));
  Serial.println(FULL_STEPS_PER_POS);
  Serial.print  (F(" - Home offset in steps: "));
  Serial.println(HOME_OFFSET_FULL_STEPS);
  Serial.println(F("Setup:"));

  // Init pins
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
  Serial.println(F(" - Gpio complete"));
  
  setupStepperDriver();

  moveToHomePosition();

  setupDCCDecoder();

  Serial.println(F("Debug information:"));
}

void loop()
{
  Dcc.process();

  stepper.run();

  if(!stepper.isRunning())
  {
    stepper.disableOutputs();

    // Reset the step position to prevent overflow
    CurrentPosition = stepper.currentPosition();
    while(CurrentPosition<0){
      CurrentPosition += STEPS_HALF_ROTATION;
    }
    stepper.setCurrentPosition(CurrentPosition%STEPS_HALF_ROTATION);
  }
}
