# Stopwatch
A stopwatch implementation with a TI msp430g2553 with an attached LTC-4627JR 4 digit 7 segment display

   This code provides a simple stop watch on a MSP430g2553 with custom display
   attached. The device will start at 00:00. Pressing the right most button 
   will start the timer, going up by miliseconds. Pressing the middle button at
   any time will reset the device to 00:00. You can press the left most button
   at any time to pause the timer. Once the timer reaches 99:90 it will stop on
   its own and wait for an input from the user. This all is accomplished using
   both push button interupts for the buttons respectively and using both timers
   on the board as well as using the dadd instruction to increment the value
   displayed



  Both Timers (TimerA_0 and TimerA_1) are set up
  TimerA_0 will be set up to update the display at the rate of one digit
  every 2 mS.  This results in no observable flicker in the multiplexing. 
  Slower rates such as 10 mS per digit result in observable flicker.  



   TimerA_1 will be set up to generate the 100 mS interval interrupts
   for timing for the stopwatch or clock. TimerA_1 has the higher
   priority of the two timers, so the time update has precedence over display
   multiplexing. This is also where DisplayValue is incremented using the dadd
   instruction.
