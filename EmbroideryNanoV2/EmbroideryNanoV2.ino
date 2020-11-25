
#include "motor.h"
#include "powercontrol.h"
#include "needle.h"

#define EN        8       // stepper motor enable, low level effective
#define X_DIR     5       //X axis, stepper motor direction control 
#define Y_DIR     6       //y axis, stepper motor direction control
#define Z_DIR     7       //z axis, stepper motor direction control
#define X_STP     2       //x axis, stepper motor control
#define Y_STP     3       //y axis, stepper motor control
#define Z_STP     4       //z axis, stepper motor control
#define X_HOME    9       // x axis home
#define Y_HOME    10      // y1 axis home
#define Z_HOME    11      // y2 axis home

#define Z_CROSS   12      // the input pin that recieves the zero crossing pulses from our power control hardware 
#define TRIAC     A0      // the output pin that triggers the triac 
#define NEEDLE    A1      // the input pin that detects the needle!

int m_phase_a = 0;

CMotor XMotor(X_DIR, X_STP, X_HOME, 0); // this is the x motor drive
CMotor YMotor(Y_DIR, Y_STP, Y_HOME, 0); // this is the y motor drive - note that we later add a second axis during setup

CPowerControl power(Z_CROSS, TRIAC);    // class that controls the triac power to the needle motor

CNeedle needle(NEEDLE);                 // simple class that maintains the needle state.

// Get data from the serial port - including checksum
int GetRXData(byte init, void * data, int len)
{
  byte checksum = init;
  if(Serial.readBytes((byte * )data, len) == len)
  {
    byte chk_sum;
    if(Serial.readBytes(&chk_sum, 1) == 1)
    {
      // get here and we have required bytes with a check sum so lets check it out
      for(int count = 0; count < len; count++) checksum += ((byte *) data)[count];
      checksum = ~checksum;
      if(checksum == chk_sum) return 0;
    }
  }
  Serial.write('!'); // bad data
  return -1;
}

// send data to the serial port - including checksum
int PutTXData(byte * data, int len)
{
  byte checksum = 0;
  for(int count = 0; count < len; count++)
  {
    Serial.write(data[count]);
    checksum += data[count]; 
  }
  Serial.write(~checksum);
  return 0;
}

// simple move request structure with needle power and needle function
struct move {
  int x_dem;
  int y_dem;
  byte power;
  char code;
};

// we can have eight moves in the buffer at any time
struct moveBuffer {
  struct move moves[8];
  char inpos;
  char outpos;
  char length;
} buffer;

// structure to transfer current status information from the motors
struct {
  byte x_stat, y_stat, mstat, bstat;
  int xdem, xpos, ydem, ypos;
  int xabs, yabs;
} status; 

// structure to transfer homed status information from the motors
struct {
  byte x_home_stat, y_home_stat;
  int x_home_offset, y_home_offset;
} home_status;

// simple 2D coordinate structure
struct {
  int x_coord, y_coord;
} D2Coord;

// Another for motor limits!
struct {
  int x_min, x_max;
  int y_min, y_max;
} limits;

// machine setup
struct {
  int set_ok;                              // set to 1 when the machine setup has been sucesssfully received
  int x_limit, y_limit;                    // the main axis limits - should always be positive
  int power_boost_cycles;                  // the number of half cycles used to boost the machine motor
} machine_setup;

// hoop fitted to the machine - setup
struct {
  int set_ok;                               // set to 1 when the machine setup has been sucesssfully received
  int x_min, x_max, x_off;                  // the hoop X limits and offset - should always be positive
  int y_min, y_max, y_off;                  // the hoop Y limits and offset - should always be positive
} hoop_setup;

int buffer_phase = 0;                       // stores the current state of the buffer
int needle_up_stop = 0;                     // stores a request to stop the needle motor with the needle raised

int power_safety_count = 0;                 // used to ensure the mains motor power is removed in the event of a needle stall SAFETY (10s)!

#define MOTOR_SAFETY_LIMIT 60L * 60L * 500L // set to auto power off the stepper motors after 60 minutes of no movement SAFETY!
long motor_safety_limit = 0;                // the counter for the motor safety limit.

#define VERSION 0x10                        // version code is always useful!

void setup() {

  pinMode(EN, OUTPUT);                      
  digitalWrite(EN, HIGH);                   // disable the stepper Motors

  pinMode(LED_BUILTIN, OUTPUT);             // useful indicator for when the needle is up

  memset(&machine_setup, sizeof(machine_setup), 0);  // clear the machine setup

  noInterrupts();           // disable all interrupts
  
  // use the 1ms timer to generate an interrupt using the match register A:
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A); // enable it
  TCCR0A = 0;

  YMotor.initialiseSecondAxis(Z_DIR, Z_STP, Z_HOME, 1); // add the second Y axis using the Z drive

  Serial.begin(38400); // this matches the bluetooth device

  power.initialise();  // initiliase the triac power object

  needle.ClearStitch();  // clear possible stitch request

  buffer.inpos = 0;
  buffer.outpos = 0;
  buffer.length = 0;      // flush the buffer

  interrupts();           // enable all interrupts
}

void loop() {
  // parse the control codes
  parseOriginal();
  //
  // Simple motor safety operations
  //
  if(XMotor.isMoving() || YMotor.isMoving())
  {
    motor_safety_limit = millis() + MOTOR_SAFETY_LIMIT;
  }
  else
  {
    if(millis() > motor_safety_limit)  digitalWrite(EN, 1); // safety exceeded so turn off the motor
  }
}


void parseOriginal()
{
  byte buf[2];
  struct move mrequest;
  int return_int;

  if(Serial.peek() >= 0)
  {
    // there is a character in the buffer!
    char com = Serial.read(); // grab the byte
    switch(com) {
      case 'd' :
        if(!GetRXData('d', buf, 1))
        {
          power.SetMicroSecondOnDelay(buf[0] * 50);
          power_safety_count = 5000;
          Serial.write('*');
        }
        break;
      case 'p' : // set motor power!
        if(!GetRXData('p', buf, 2))
        {
          digitalWrite(EN, buf[0] || buf[1] ? 0 : 1);
          if(buf[0] || buf[1]) motor_safety_limit = millis() + MOTOR_SAFETY_LIMIT;
          Serial.write('*');
        }
        break;
      case 'r' : // reset command - effectively E-Stop
        if(!GetRXData('r', NULL, 0))
        {
           noInterrupts();           // disable all interrupts
           needle.ClearStitch();  // clear possible stitch request
           buffer.inpos = 0;
           buffer.outpos = 0;
           buffer.length = 0;     // clear the buffer
           buffer_phase = 0;      // make sure it is off!
           interrupts();           // reeanble all interrupts
           Serial.write('*');
        }
        break;
      case 'h' : // home command!
        if(!GetRXData('h', NULL, 0))
        {
           if(!buffer.length && !XMotor.isMoving() && !YMotor.isMoving())
           {
             XMotor.StartHoming();
             YMotor.StartHoming();
           }
           Serial.write('*');
        }
        break;
      case 'z' : // zero command
        if(!GetRXData('z', NULL, 0))
        {
           XMotor.SetZero();
           YMotor.SetZero();
           Serial.write('*');
        }
        break;
      case 'i' : // incremental move
        if(!GetRXData('i',&D2Coord, 4))
        {
           needle.ClearStitch();  // clear possible stitch request
           XMotor.IncrementDemand(D2Coord.x_coord);
           YMotor.IncrementDemand(D2Coord.y_coord);
           Serial.write('*');
        }
        break;
      case 'm' : // demanded move
        if(!GetRXData('m', &D2Coord, 4))
        {
           needle.ClearStitch();  // clear possible stitch request
           XMotor.SetDemand(D2Coord.x_coord);
           YMotor.SetDemand(D2Coord.y_coord);
           Serial.write('*');
        }
        break;
      case 'l' : // set current position
        if(!GetRXData('l', &D2Coord, 4))
        {
           XMotor.SetPosition(D2Coord.x_coord);
           YMotor.SetPosition(D2Coord.y_coord);
           Serial.write('*');
        }
        break;
      case 'b' : // add to the buffer
        if(!GetRXData('b', &mrequest, sizeof(mrequest)))
        {
           if(buffer.length < 8)
           {
               buffer.length++;
               buffer.moves[buffer.inpos].x_dem = mrequest.x_dem;
               buffer.moves[buffer.inpos].y_dem = mrequest.y_dem;
               buffer.moves[buffer.inpos].power = mrequest.power;
               buffer.moves[buffer.inpos].code = mrequest.code;
               buffer.inpos = (buffer.inpos + 1) % 8;
               Serial.write('*');
           }
           else Serial.write('!');
        }
        break;
      case 's' : // get the system status
        status.x_stat = (XMotor.inPosition() ? 0 : 1) | (digitalRead(EN) ? 0 : 2);
        status.y_stat = (YMotor.inPosition() ? 0 : 1) | (digitalRead(EN) ? 0 : 2);
        status.mstat = !needle.getState();
        status.bstat = buffer.length;
        status.xdem = XMotor.GetDemand();
        status.xpos = XMotor.GetPosition();
        status.ydem = YMotor.GetDemand();
        status.ypos = YMotor.GetPosition();
        status.xabs = XMotor.GetAbsPosition();
        status.yabs = YMotor.GetAbsPosition();
        PutTXData((byte* ) &status, sizeof(status));  // write it!
        break;
      case 'g' :  // get the home status!
          home_status.x_home_stat = XMotor.GetHomedStatus() |  (XMotor.isHoming() ? 0 : 128);
          home_status.y_home_stat = YMotor.GetHomedStatus() |  (YMotor.isHoming() ? 0 : 128);
          home_status.x_home_offset = XMotor.GetHomedDelta();
          home_status.y_home_offset = YMotor.GetHomedDelta();
          PutTXData((byte* ) &home_status, sizeof(home_status));  // write it!
          break;
        break;
      case 'c' : // get the stitch count
        return_int = needle.getStitchCount();
        PutTXData((byte* ) &return_int, 1);  // write it!
        break;
      case 'u' : // get the total up ticks
        return_int = needle.getNeedleUpCount();
        PutTXData((byte* ) &return_int, 2);  // write it!
        break;
      case 'a' : // get the AC cross count
        return_int = power.getCrossCount();
        PutTXData((byte* ) &return_int, 1);  // write it!
        break;
      case '?' : // get the firmware version
        return_int = VERSION;
        PutTXData((byte* ) &return_int, 1);  // write it!
        break;
      case 'x' : // set up stop
        if(!GetRXData('x', NULL, 0))
        {
           if(!buffer.length && !buffer_phase)
           {
             needle.InitialiseStitch(); 
             needle_up_stop = 1;
           }  
           Serial.write('*');
        }
        break;
      case 'k' : // konfigure the machine setup
        machine_setup.set_ok = 0; // always clear this
        if(!GetRXData('k', &machine_setup, sizeof(machine_setup)))
        {
          // ok we have received a setup so lets use it
          XMotor.SetMaximum(machine_setup.x_limit);
          YMotor.SetMaximum(machine_setup.y_limit);
          power.SetPowerBoost(machine_setup.power_boost_cycles);
          machine_setup.set_ok = 1; // machine setup is OK!
          Serial.write('*');
        }
        else Serial.write('!');
        break;
      case 'q' : // configure the hoop limits
        hoop_setup.set_ok = 0; // always clear this
        if(!GetRXData('q', &hoop_setup, sizeof(hoop_setup)))
        {
          // ok we have received a setup so lets use it
          XMotor.SetHoopLimits(hoop_setup.x_min, hoop_setup.x_max, hoop_setup.x_off);
          YMotor.SetHoopLimits(hoop_setup.y_min, hoop_setup.y_max, hoop_setup.y_off);
          hoop_setup.set_ok = 1; // hoop setup is OK!
          Serial.write('*');
        }
        else Serial.write('!');
        break;
      case 't' : // get traverse limits!
        limits.x_min = XMotor.GetPosMin();
        limits.x_max = XMotor.GetPosMax();
        limits.y_min = YMotor.GetPosMin();
        limits.y_max = YMotor.GetPosMax();
        PutTXData((byte* ) &limits, sizeof(limits));  // write it!
        break;
        break;
    }
  }
}

// We will get here every 1ms
SIGNAL(TIMER0_COMPA_vect) 
{
  if(!(m_phase_a % 2))
  {
     // get here every other cycle (2ms)
     // we must only check whether we move the motors if the needle is raised and the motors are enabled!
      XMotor.doTick(needle.getState() && !digitalRead(EN));
      if(digitalRead(EN)) XMotor.ClearHoming();
      if(digitalRead(EN)) XMotor.Stop();
      YMotor.doTick(needle.getState() && !digitalRead(EN));
      if(digitalRead(EN)) YMotor.ClearHoming();
      if(digitalRead(EN)) YMotor.Stop();
  }
  else
  {
    // get here every other cycle (2ms) opposite phase to above :-)
    // check out the needle
    needle.doTick();
    digitalWrite(LED_BUILTIN, needle.getState());
    if(needle_up_stop)
    {
       if(needle.Completed())
       {
          needle_up_stop = 0;
          power.SetMicroSecondOnDelay(0); // up stop requested so turn off power
       }
    }

    if(power_safety_count) power_safety_count--;
    else power.SetMicroSecondOnDelay(0);
    //
    switch(buffer_phase) {
      case 0:
        // the buffer is idle so see if we should move to the next
        if(!buffer.length) break;
        XMotor.SetDemand(buffer.moves[buffer.outpos].x_dem);
        YMotor.SetDemand(buffer.moves[buffer.outpos].y_dem);
        buffer_phase++;
      case 1: // we are waiting for the moves to complete
        if(!XMotor.inPosition() || !YMotor.inPosition()) break;
        if(buffer.moves[buffer.outpos].code) needle.InitialiseStitch();
        buffer_phase++;
      case 2: // in position so output the power
        power.SetMicroSecondOnDelay(buffer.moves[buffer.outpos].power * 50);
        needle_up_stop = buffer.moves[buffer.outpos].code == 3;
        power_safety_count = 5000;
        buffer_phase++;
        break;
      case 3: // the power has been outputed so wait for the needle to complete
        if(needle.Completed())
        {
          // needle is up and/or completed so work out what we should do
          buffer.outpos = (buffer.outpos + 1) % 8;
          buffer.length--;
          buffer_phase = 0; // lets go again
        }
        break;
     }
  }
  m_phase_a++;
}

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
  power.doZCInt(); // we get here when the zero cross optocupler changes 
}

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{
  power.doTriac();  // trigger the triac
}
 
ISR(TIMER1_COMPB_vect) // timer compare interrupt service routine
{
  power.doPhase();  // perform other operations including noise filtering and power down
}
 


