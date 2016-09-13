/*
Guitar Looper

Records the guitar signal and stores it in memory.
Recorded sound is stored in PCM format at 22.05kHz 8bit

*/

#include "Arduino.h"

int led_pin = 13;
int counter = 1;


void startTimer(){
  pmc_set_writeprotect(0); //enable the write protect register of the pmc_set_writeprotect
  pmc_enable_periph_clk(ID_TC0); //enable timer counter 0, channel 0

  TC_Configure(TC0,0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  TC_SetRA(TC0, 0, 1000); // sets interrupt rate
  TC_SetRC(TC0, 0, 4000); // sets interrupt rate
  TC_Start(TC0, 0);       // Start the timer

  // enable timer interrupts on the timer
  TC0->TC_CHANNEL[0].TC_IER=TC_IER_CPCS;   // IER = interrupt enable register
  TC0->TC_CHANNEL[0].TC_IDR=~TC_IER_CPCS;  // IDR = interrupt disable register

  NVIC_EnableIRQ(TC0_IRQn);               // Enable interrupt routine

}

void TC0_Handler()
{
  // We need to get the status to clear it and allow the interrupt to fire again
  TC_GetStatus(TC0, 0);
  counter ^= 1;  //toggling between 1 and 0;
  Serial.println(counter);
}

void setup() {
  Serial.begin(9600);
  startTimer();
  PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
  PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral
  Serial.println("finished initialising timer counter");
}

void loop() {
}
