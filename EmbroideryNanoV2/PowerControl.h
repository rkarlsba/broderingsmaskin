class CPowerControl
{
  public:
  
  CPowerControl(int zc_bit, int triac_bit);

  void doZCInt();
  void doTriac();
  void doReset();
  void doPhase();

  void initialise();
  
  void SetMicroSecondOnDelay(int req_delay);

  int getCrossCount(); // returns the number of mains crosses since the last time it was called

  void SetPowerBoost(int half_cycles) {boost_cycles = half_cycles;}

  private:

  int zc_add;
  int triac_add;

  int usdelay;

  int delta_cross_tick_count;  // used for old school mains frequency

  int power_phase;

  int boost_count;
  int boost_cycles;

};

