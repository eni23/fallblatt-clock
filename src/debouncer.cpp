#include <Arduino.h>

/**
 * Super simple debouncer class
 **/
class Debouncer {
  unsigned long last_active;
  int min_delay;

  public:
  Debouncer(int minimal_delay) {
    last_active = 0;
    min_delay = minimal_delay;
  }

  boolean check(){
    if ( (millis() - last_active) > min_delay ){
      last_active = millis();
      return true;
    }
    else {
      return false;
    }
  }

};
