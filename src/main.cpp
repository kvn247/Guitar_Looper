/*
Guitar Looper

Records the guitar signal and stores it in memory.
Recorded sound is stored in PCM format at 22.05kHz 8bit

*/

#include "Arduino.h"

void startTimer(){
  pmc_set_writeprotect(0); //enable the write protect register of the pmc_set_writeprotect
  pmc_enable_periph_clk(ID_TC0); //enable timer counter 0, channel 0

  //Configure TC0 channel 1, Waveform Mode, Reset tc_val on RC compare, chose mclk/2, TIOA toggle on RC compare
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_ACPA_NONE |  TC_CMR_ACPC_TOGGLE);
  TC_SetRC(TC0, 0, 952); // sets interrupt rate; curretly set for 22.05KHz
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

}

void pinInit(){

  //We want to output the counter timer onto pin 2 of the Arduino
  //tc0 outputs to TIOA0, which is on peripheral B; we need to set this up

  PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
  PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral

}

void setup() {
  Serial.begin(9600); //used for debugging only
  startTimer();
  pinInit();
}

void loop() {
}
