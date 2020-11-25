#include "Arduino.h"
#include "Motor.h"
     
CMotor::CMotor(int dir, int step, int home, int invert) 
{
  dirAdd = dir;
  stepAdd = step;
  homeAdd = home;
  home2Add = homeAdd;
  CMotor::invert = invert;
  position = 0;
  demand = 0;
  maximum = 0;
  minimum = 0;
  moving = 0;
  motor2 = 0;   // second motor active (in parallel with this one
  step2Add = 0; // step address
  dir2Add = 0;  // direction address
  invert2 = 0;
  homing = 0;
  home_delta = 0;
  zero_offset = 0;

  hoop_min = 0;
  hoop_max = 0;
  hoop_offset = 0;

  pinMode(dirAdd, OUTPUT); 
  pinMode(stepAdd, OUTPUT);
  pinMode(homeAdd, INPUT_PULLUP);
}

 void CMotor::doTick(int run)
  {
    if(in_position_pause) in_position_pause--;
    if(!run)
    {
      if(homing)
      {
         homing = 0; // homing should never be interrupted!
         home_status = 6; // code for killed
      }
      return;
    }
    moving = 0;
    pulse = 0;
    if(!homing)
    {
      if(demand != position)
      {
        if(demand > position)
        {
          if(position < hoop_max )
          {
            digitalWrite(dirAdd, !invert);
            if(motor2) digitalWrite(dir2Add, !invert2);
            position++;
            pulse = 1;
          }
        }
        else
        {
          if(position > hoop_min)
          {
            digitalWrite(dirAdd, invert);
            if(motor2) digitalWrite(dir2Add, invert2);
            position--;
            pulse = 1;
          }
        }
      }
    }
    else
    {
      // get here and we are homing
      int home_switch = digitalRead(homeAdd) || digitalRead(home2Add);
      switch(homing) {
        case 1:
          if(!home_switch) // no homing switch is activated
          {
             digitalWrite(dirAdd, invert); // home switch not hit so drive towards it!
             if(motor2) digitalWrite(dir2Add, invert2);
             home_delta--;
             if((home_delta + maximum + 100) < 0)
             {
                // home failure the axis didnt get back before the counts ran out!
                home_status = 2;
                homing = 0;
                break;
             }
             pulse = 1;
          }
          else 
          {
            homing++; // set next phase
            home_count = 50; // drive 50 pulses off (5mm)
          }
          break;
       case 2:
          digitalWrite(dirAdd, !invert); // drive away from home
          if(motor2) digitalWrite(dir2Add, !invert2);
          pulse = 1;
          home_delta++;
          if(!home_count--) 
          {
            home_count = 100;
            if(home_switch)
            {
              // at least one switch is activated this should not happen
              home_status = 3;
              homing = 0;
              break;
            }
            homing++;
            home_rate = 4;
          }
          break;
       case 3:
          if(!home_switch) // no homing switch is activated
          {
             digitalWrite(dirAdd, invert); // home switch not hit so drive towards it!
             if(motor2) digitalWrite(dir2Add, invert2);
             if(home_count <= 0)
             {
                // home failure the axis didnt get back before the fine counts ran out!
                home_status = 4;
                homing = 0;
                break;
             }
             
             home_rate--;
             if(!home_rate)
             {
               pulse = 1;
               home_rate = 4;
               home_delta--;
               home_count--;
             }
          }
          else 
          {
            homing++; // set next phase
          }
          break;
       case 4:
          if(motor2)
          {
            homing++; // two motor drive so see if we can pull in the second axis!
            max_2motor_align = 50; // max is 5mm to prevent damage (hopefully)
            home_rate = 4;
          }
          else 
          {
            homing = 0; // our work here is done
            home_delta += position;
            position = minimum;
            demand = minimum;
            home_status = 1;
          }
          break;
       case 5: // 2 motor setup so lets try and pull in the non homed axis other axis (max 5mm).
          if(digitalRead(homeAdd) && digitalRead(home2Add))
          {
            // home worked!
            homing = 0;
            home_delta += position;
            position = minimum;
            demand = minimum;
            home_status = 1;
            break;
          }
          if(!max_2motor_align)
          {
             // alignment failure
             homing = 0;
             home_status = 5;
             break;
          }
          home_rate--;
          if(!home_rate)
          {
            home_rate = 4;
            max_2motor_align--;
            if(!digitalRead(homeAdd))
            {
               digitalWrite(dirAdd, invert); // home switch not hit so drive towards it!
               // primary axis not pulled in so give it a go
              digitalWrite(stepAdd, 1);
              delayMicroseconds(2);
              digitalWrite(stepAdd, 0); // step the motor
            }
            if(!digitalRead(home2Add))
            {
               digitalWrite(dir2Add, invert2); // home switch not hit so drive towards it!
               // primary axis not pulled in so give it a go
              digitalWrite(step2Add, 1);
              delayMicroseconds(2);
              digitalWrite(step2Add, 0); // step the motor
          }
        }
      }
    }
    if(pulse)
    {
      digitalWrite(stepAdd, 1);
      if(motor2) digitalWrite(step2Add, 1);
      delayMicroseconds(2);
      digitalWrite(stepAdd, 0); // step the motor
      if(motor2) digitalWrite(step2Add, 0); // and the second if it exists!
      moving = 1;
      in_position_pause = 12;
    }
  }

 void CMotor::initialiseSecondAxis(int dir, int step, int home, int invert)
 {
     motor2 = 1;
     step2Add = step;
     dir2Add = dir;
     home2Add = home;
     pinMode(home2Add, INPUT_PULLUP);
     invert2 = invert;
 }

 void CMotor::StartHoming()
 {
    homing = 1;
    home_status = 0;
    home_delta = 0;
    home_count = 0;
 }

void CMotor::IncrementDemand(int delta)
{
  demand += delta;
  if(demand < hoop_min) demand = hoop_min;
  if(demand > hoop_max) demand = hoop_max; 
}

void CMotor::SetHoopLimits(int min, int max, int offset)
{
  hoop_min = min;
  hoop_max = max;
  zero_offset = offset;
  demand = offset;
}



