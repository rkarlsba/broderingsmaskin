
class CNeedle {
  public:
  CNeedle(int needle_bit);

  void doTick();                            // perform needle operations
  int getState() {return needle_state;}

  void InitialiseStitch(){stitch_mon = 0;}                    // sets up variables to detect a stitch!
  void ClearStitch(){stitch_mon = 3;}                         // sets up variables to clear detection!
  int  Completed(){return stitch_mon >= 2 && needle_state;}   // returns true when a stitch is completed and raised

  int getStitchCount(); // returns the number of stictches since the last time it was called
  int getNeedleUpCount();  // returns the TOTAL number of ticks the needle has been in the air since the last time it was called!

  private:
  int needleAdd;      // the needle address
  int needle_state;   // 0 = down - 1 = raised

  int filter;         // used to filter potentially noisy needly states
  int stitch_mon;

  int delta_stitch_count;  // the number of stitches
  int delta_up_count;      // the total number of ticks the neeedle is up during the above stitch count

};

