#include "Arduino.h"
#include "PowerControl.h"

CPowerControl::CPowerControl(int zc_bit, int triac_bit)
{
   zc_add = zc_bit; 
   triac_add = triac_bit;

   delta_cross_tick_count = 0;

   power_phase = 0;

   boost_count = 0;
   boost_cycles = 0;

}

void CPowerControl::initialise()
{
   pinMode(zc_add, INPUT_PULLUP);
   pinMode(triac_add, OUTPUT);

   *digitalPinToPCMSK(zc_add) |= bit(digitalPinToPCMSKbit(zc_add));
   PCIFR  |= bit (digitalPinToPCICRbit(zc_add)); // clear any outstanding interrupt
   PCICR  |= bit (digitalPinToPCICRbit(zc_add)); // enable interrupt for the group

   TCCR1A = 0;
   TCCR1B = 0;
   TCCR1B |= (1 << CS11);  // 8 prescaler
   TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt (A used for trigger delay)
   TIMSK1 |= (1 << OCIE1B); // enable timer compare interrupt (B used for overflow reset)
   
   usdelay = 0;  
   OCR1A = usdelay;  
   OCR1B = 30000; 
    
}

void CPowerControl::doZCInt()
{

  if(power_phase) return;           // power phase will only be cleared 5ms after the zero cross - this virtually illiminates noisy triggers
  
  digitalWrite(triac_add, 0);       // get here and this is a new trigger phase so clear the triac trigger
  TCNT1 = 0;                        // clear the counter (increments every 0.5us)
  if(boost_count)
  {
    OCR1A = 1; // max power         // if power boost active then max power!
    boost_count--;                  // reduce the count (boost gets set to user value whenever needle first moves)
  }
  else OCR1A = usdelay;             // we will trigger the triac in usdelay seconds (usdelay set by SetMicroSecondOnDelay)
  OCR1B = 10000; // 5ms             // mask out further triggers for 5ms
  power_phase = 1;                  // set power phase to 1 so if noise brings us back we just ignore!
  delta_cross_tick_count++;         // used to work out the mains frequency and determine if power is available
}

void CPowerControl::doTriac()
{
  digitalWrite(triac_add, 1); // trigger the triac
}

void CPowerControl::doPhase()
{
  if(power_phase == 1)
  {
    // get here and we were in the noise delay phase so set up for power failure
    OCR1B = 30000;      // if we trigger in 15ms then power is lost
    power_phase = 0;    // allow the zero cross trigger to activate again
  }
  else
  {
    TCNT1 = 0;                    // get here and power has failed so reset the timer back to zero
    digitalWrite(triac_add, 0);   // and kill the triac drive!
  }
}

void CPowerControl::doReset()
{
  TCNT1 = 0;
  digitalWrite(triac_add, 0);
}


// just return the cross count for information about the mains supply
int CPowerControl::getCrossCount()
{
  int temp = delta_cross_tick_count;
  delta_cross_tick_count = 0;
  if(temp > 255) temp = 0;
  return temp;
}

void CPowerControl::SetMicroSecondOnDelay(int req_delay) 
{
  if(req_delay > 0 && !usdelay) boost_count = boost_cycles;  // if power request and the ac drive is off then activate the boost helper!
  usdelay = req_delay << 1;                                   // calculate the us delay count (just *2 as this timer repeat every 0.5us
}





