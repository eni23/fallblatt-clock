#ifndef __CONFIG_H
  #define __CONFIG_H

  #define SERIAL_BAUD       19200

  #define RS485_PIN_RX      9
  #define RS485_PIN_TX      10

  #define MODULE_ADDR_HOUR  82
  #define MODULE_ADDR_MIN   29

  #define ENCODER_PIN_UP    8
  #define ENCODER_PIN_DOWN  6
  #define ENCODER_PIN_BTN   7 // pin 7 is needed for interrupt

  #define ENABLE_SERIAL_DBG true
  

#endif
