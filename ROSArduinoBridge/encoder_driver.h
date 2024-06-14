/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  // All pins are part of PORTD in your configuration
  #define LEFT_ENC_PIN_A PD2  // Arduino pin D5
  #define LEFT_ENC_PIN_B PD3  // Arduino pin D6
  
  // Adjusting the other encoder pins to use PORTD as well
  #define RIGHT_ENC_PIN_A PC4  // Arduino pin D9
  #define RIGHT_ENC_PIN_B PC5 // Arduino pin D10
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();