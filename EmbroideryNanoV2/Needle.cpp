#include "Arduino.h"
#include "needle.h"

CNeedle::CNeedle(int needle_bit)
{
  needleAdd = needle_bit;
  pinMode(needleAdd, INPUT_PULLUP);
  
  needle_state = 0; // always start with the needle assumed down
  delta_stitch_count = 0;
  delta_up_count = 0;
  filter = 0; 

}

void CNeedle::doTick()
{
  if(!needle_state)
  {
    // needle is lowered so see if raised detected
    if(!digitalRead(needleAdd))
    {
      filter++;
      if(filter >=1) 
      {
        if(needle_state == 0) delta_stitch_count++; // simple stitch counter
        needle_state = 1; // clear / raised
        filter = 0;
      }
    }
    else filter = 0;
  }
  else
  {
    // needle is rasied so see if lowered detected
    if(digitalRead(needleAdd))
    {
      filter++;
      if(filter >=10) 
      {
        needle_state = 0; // lowered
        filter = 0;
      }
    }
    else 
    {
      filter = 0;
    }
  }

  if(needle_state) delta_up_count++;

  switch(stitch_mon) {
    case 0:
      if(!needle_state) stitch_mon++;
      break;
    case 1:
      if(needle_state) stitch_mon++;
      break;
  }
}

int CNeedle::getStitchCount()
{
  int temp =  delta_stitch_count;
  delta_stitch_count = 0;
  if(temp > 255) temp = 0;
  return temp;
}

int CNeedle::getNeedleUpCount()
{
  int temp =  delta_up_count;
  delta_up_count = 0;
  return temp;
}



