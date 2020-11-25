class CMotor {
  public:
      CMotor(int dir, int step, int home, int invert);
      void initialiseSecondAxis(int dir, int step, int home, int invert);
      void doTick(int run);
    
      void SetMaximum(int max) {maximum = max;}
      void SetMinimum(int min) {minimum = min;}
      void SetPosition(int pos) {zero_offset = position - pos;}
      void SetDemand(int dem) {demand = dem + zero_offset;}
      int GetDemand() {return demand - zero_offset;}
      int GetPosition() {return position - zero_offset;}
      int GetAbsPosition() {return position;}
      void IncrementDemand(int delta);
      int GetPosMax() {return hoop_max - zero_offset; }
      int GetPosMin() {return hoop_min - zero_offset; }

      void SetZero() {zero_offset = position;}  // sets the curent position to be the new zero!

      void SetHoopLimits(int min, int max, int offset); // sets the hoop limits and drives to the offset position
      
      bool inPosition() {return !in_position_pause && (position == demand);}
      int isMoving() {return moving;}
      int isHoming() {return !homing;}
      int GetHomedDelta() {return home_delta;}
      int GetHomedStatus() {return home_status;}

      void StartHoming();
      void ClearHoming() {home_status = 0; homing = 0;}

      void Stop() { demand = position; }

  private:
      int position;
      int demand;
      int maximum;
      int minimum;
      int home_status;
      int pulse;
      int moving;   // set to 1 when moving
      int homing;   // set to non zero if homing phases are in progress
      int home_count;
      int home_rate;
      int max_2motor_align;
      int home_delta;
      int zero_offset;

      int hoop_max;
      int hoop_min;
      int hoop_offset;

      int in_position_pause;

  private:
      int stepAdd;  // step address
      int dirAdd;   // direction address
      int homeAdd;   // home address
      int invert;   // set to 1 if the primary motor is inverted
      int motor2;   // second motor active (in parallel with this one
      int step2Add; // step address
      int dir2Add;  // direction address
      int home2Add;   // home address
      int invert2;  // set if second axis direction should be inverted

};


