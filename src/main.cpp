#include "Arduino.h"
#include "config.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "debouncer.cpp"

#include <Rotary.h>   // platformio lib install 275
#include <DS3231.h>   // platformio lib install 1379
#include <Wire.h>

// Serial, RS485
SoftwareSerial rs485(RS485_PIN_RX, RS485_PIN_TX);

// RTC
DS3231 rtc;
uint8_t sec, min, hour;
bool clock_century = false;
bool clock_pm_flag = false;
bool clock_has_pm  = false;

// time setting & encoder
Rotary encoder = Rotary(ENCODER_PIN_UP, ENCODER_PIN_DOWN);
Debouncer btn_debounce( 390 );
uint8_t current_enc_state = 0;
uint8_t current_hour      = 0;
uint8_t current_min       = 0;

// Serial debug macros
#ifdef ENABLE_SERIAL_DBG
  #define serial_debug(args...) Serial.print(args);
  #define serial_debugln(args...) Serial.println(args);
#else
  #define serial_debug(args);
  #define serial_debugln(args);
#endif

//
// Fallblatt module functions
//

// send RS485 break condition
void send_rs485_break( int ms=30 ){
  rs485.end();
  // start in lower baudrate and send zero is basically break
  rs485.begin( 1200 );
  byte x[10];
  memset( x, 0x00, 10 );
  rs485.write( x, 10 );
  rs485.end();
  rs485.begin( 19200 );
}

// set module position
void panel_goto( uint8_t addr, uint8_t position ){
  send_rs485_break();
  rs485.write( 0xFF );
  rs485.write( 0xC0 );
  rs485.write( addr );
  rs485.write( position );
}

// read position from module
uint8_t panel_get_pos( uint8_t addr ){
  uint8_t res = 0;
  send_rs485_break();
  rs485.write( 0xFF );
  rs485.write( 0xD0 );
  rs485.write( addr );
  res = rs485.read();
  return res;
}

// calculate position minute module
uint8_t calc_min_pos( uint8_t pos ){
  if ( pos < 31 ) {
    return pos+30;
  }
  return pos-31;
}


//
// RTC Clock functions
//


void rtc_get(){
    hour  = rtc.getHour(clock_pm_flag, clock_has_pm);
    min   = rtc.getMinute();
    sec   = rtc.getSecond();
}


//
// Encoder functions
//

// gets called when encoder button pressed
void encoder_btn_interrupt(){
  // check if press is bounce
  if ( !btn_debounce.check() ){
    return;
  }
  // if not in time-setting-mode turn on and set to hour
  if ( current_enc_state == 0 ){
    current_hour = hour;
    current_enc_state = 1;
    return;
  }
  // if time setting was on hours set to minute
  if ( current_enc_state == 1 ){
    current_min = min;
    current_enc_state = 2;
    return;
  }
  // if time setting was on hours set save and exit flag
  if ( current_enc_state == 2 ){
    current_enc_state = 3;
    return;
  }
}

// process encoder input
// gets called every loop run
void encoder_loop(){

  // set leds according to state
  if ( current_enc_state == 1 ){
    digitalWrite( LED_BUILTIN_TX, LOW );
  }
  else if ( current_enc_state == 2 ){
    digitalWrite( LED_BUILTIN_TX, HIGH );
    digitalWrite( LED_BUILTIN_RX, LOW);
  }
  // if state is 3, save current config
  else if ( current_enc_state == 3 ){
    rtc.setHour(current_hour);
    rtc.setMinute(current_min);
    rtc.setSecond(0);
    delay(50);
    // exit time-setting mode
    current_enc_state = 0;
    digitalWrite( LED_BUILTIN_RX, HIGH );
  }

  // Check if encoder has moved
  unsigned char enc_dir = encoder.process();

  // if yes, process encoder movement
  if ( enc_dir && current_enc_state > 0 ){

    // state 1 of 2: hour
    if ( current_enc_state == 1 ){
      if (enc_dir == DIR_CW){
        if ( current_hour < 23 ){
          current_hour = current_hour+1;
        }
        else {
          current_hour = 0;
        }
      } else {
        if ( current_hour > 0 ){
          current_hour = current_hour - 1;
        }
        else {
          current_hour = 23;
        }
      }
      serial_debugln(current_hour);
      panel_goto( MODULE_ADDR_HOUR, current_hour );
    }

    // state 2 of 2: minutes
    if ( current_enc_state == 2 ){
      if ( enc_dir == DIR_CW ){
        if ( current_min < 60 ){
          current_min = current_min + 1;
        }
        else {
          current_min = 0;
        }
      } else {
        if ( current_min > 0 ){
          current_min = current_min-1;
        }
        else {
          current_min = 59;
        }
      }
      uint8_t rpos = calc_min_pos( current_min );
      serial_debugln(current_min);
      panel_goto( MODULE_ADDR_MIN, rpos );
    }
  }

}


//
// Setup & main loop
//

// initial setup
void setup() {

  // init encoder
  pinMode( ENCODER_PIN_UP,   INPUT_PULLUP );
  pinMode( ENCODER_PIN_DOWN, INPUT_PULLUP );
  pinMode( ENCODER_PIN_BTN,  INPUT_PULLUP );

  // button press interrupt
  attachInterrupt(
    digitalPinToInterrupt( ENCODER_PIN_BTN ),
    encoder_btn_interrupt,
    CHANGE
  );
  sei();

  // init serial & rs485 SoftwareSerial
  #ifdef ENABLE_SERIAL_DBG
    Serial.begin( SERIAL_BAUD );
  #endif

  rs485.begin( 19200 ); // baudrate for modules is fixed
  rs485.flush();
  Serial.flush();

  // real time clock
  Wire.begin();
  rtc.setClockMode(false);  // set to 24h

  // load time from rtc and display to modules
  delay( 100 );
  rtc_get();
  //rtc.get( &sec, &min, &hour, &day, &month, &year );
  panel_goto( MODULE_ADDR_HOUR, hour );
  uint8_t rpos = calc_min_pos( min );
  panel_goto( MODULE_ADDR_MIN, rpos );
  delay( 100 );

  // turn off leds
  pinMode( LED_BUILTIN_TX, OUTPUT );
  digitalWrite( LED_BUILTIN_TX, HIGH );
  pinMode( LED_BUILTIN_RX, OUTPUT );
  digitalWrite( LED_BUILTIN_RX, HIGH );
}

// loop function
void loop() {

  // process time setting
  encoder_loop();

  // display time from rtc to modules
  // only updating if not in time-setting mode
  if ( current_enc_state == 0 ){
    rtc_get();
    if (sec == 0){
      serial_debug("current time is: ");
      serial_debug(hour);
      serial_debug(":");
      serial_debug(min);
      serial_debug(":");
      serial_debugln(sec);

      panel_goto( MODULE_ADDR_HOUR, hour );
      uint8_t rpos = calc_min_pos(min);
      panel_goto( MODULE_ADDR_MIN, rpos );
    }

  }

}
