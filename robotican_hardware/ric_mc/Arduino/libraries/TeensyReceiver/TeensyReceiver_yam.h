/*  PWM (Pulse Width Modulation) sampling done in hardware via pin interrupts and PIT timer.
    
    I am using one, out of 4 PIT timers build into the Teensy 3.0.
    
    Pins used for PWM signal capture are fully definable by the user.
    However, i do recommend avoiding FLEX timer enabled pins, (Teensy 3.0 pin numbering) 5, 6, 9, 10, 20, 21, 22, 23,
    as those are used for PWM signal generation (for Electronic Speed Controllers).
    
    Using the hardware timer for timing / counting the pulses gives us superior accuracy of ~21 nano seconds.
    
    This receiver code also utilizes a more advanced failsafe sequence.
    In case of receiver malfunction / signal cable damage, RX Failasefe will kicks in
    which will start auto-descent sequence.
    
*/
/*
#define RX_CHANNELS 8
uint8_t PWM_PINS[RX_CHANNELS] = {2, 3, 4, 5, 6, 7, 8, 9};

volatile uint16_t RX[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
volatile uint32_t PWM_time[RX_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
*/
#define RX_CHANNELS 6
uint8_t PWM_PINS[RX_CHANNELS] = {2,3,4,5,6,7};

volatile uint16_t RX[RX_CHANNELS] = {1500,1500,1500,1500,1500,1500};
volatile uint32_t PWM_time[RX_CHANNELS] = {0,0,0,0,0,0};


volatile uint16_t RX_failsafeStatus=0;
volatile int16_t RX_signalReceived = 10;

bool failsafeEnabled = true;

void readPWM(uint8_t channel) {
    uint32_t now = PIT_CVAL0; // Current counter value
    uint32_t delta = PWM_time[channel] - now; // Delta (calculated in reverse, because PIT is downcounting timer)
    
    // All of the number below are scaled to use with Teensy 3.0 running at 48Mhz
    if (delta < 100800 && delta > 43200) { // This is a valid pulse //(delta < 120000 && delta > 43200)
        RX[channel] = delta / 48;
        
        // Bring failsafe status flag for current channel down every time we accept a valid signal
        //RX_failsafeStatus &= ~(1 << channel);
     // if (( RX_signalReceived>0) && (RX[0]<2100)&&(RX[0]>900)&&(RX[1]<2100)&&(RX[1]>900)&&(RX[2]<2100)&&(RX[2]>900)&&(RX[3]<2100)&&(RX[3]>900)&&(RX[4]<2100)&&(RX[4]>900)&&(RX[5]<2100)&&(RX[5]>900)) { 
 if ( RX_signalReceived>0) {
RX_signalReceived-=1;

if (failsafeEnabled) {
Serial.print(channel+1);
Serial.print("   ");
Serial.print(delta);
Serial.print("   ");
Serial.println(RX[channel]);
}
}
    } else { // Beginning of the pulse
        PWM_time[channel] = now;
    }
}

// ISRs
void PWM_ISR_0() {
    readPWM(0);
}
void PWM_ISR_1() {
    readPWM(1);
}
void PWM_ISR_2() {
    readPWM(2);
}
void PWM_ISR_3() {
    readPWM(3);
}
void PWM_ISR_4() {
    readPWM(4);
}
void PWM_ISR_5() {
    readPWM(5);
}
void PWM_ISR_6() {
    readPWM(6);
}
void PWM_ISR_7() {
    readPWM(7);
}

void (*PWM_Handlers [])(void) = {
    PWM_ISR_0, 
    PWM_ISR_1, 
    PWM_ISR_2, 
    PWM_ISR_3, 
    PWM_ISR_4, 
    PWM_ISR_5, 
    PWM_ISR_6, 
    PWM_ISR_7
};

void initializeReceiver() {
    // initialize PIT timer (teensy running at 48000000)
	 SIM_SCGC6 |=  SIM_SCGC6_PIT;
    PIT_MCR = 0x00;          // Turn on PIT
    PIT_LDVAL0 = 0xFFFFFFFF; // Load initial value of 4294967295
    PIT_TCTRL0 = 0x01;       // Start the counter
    for (uint8_t i = 0; i < RX_CHANNELS; i++) {
        pinMode(PWM_PINS[i], INPUT);
        attachInterrupt(PWM_PINS[i], PWM_Handlers[i], CHANGE);
    }
}

void RX_failSafe() {
RX_signalReceived+=2;
if (RX_signalReceived>500) RX_signalReceived=500;
if (RX_signalReceived>=50) {
//((RX[0]<900)||(RX[0]>2100)||(RX[1]<900)||(RX[1]>2100)||(RX[2]<900)||(RX[2]>2100)||(RX[3]<900)||(RX[3]>2100)||(RX[4]<900)||(RX[4]>2100)||(RX[5]<900)||(RX[5]>2100)) {

	failsafeEnabled = true;
}
else 
{
	failsafeEnabled = false;
}
/*
    if (RX_failsafeStatus > 0) {
        RX_signalReceived++; // if this flag reaches 10, an auto-descent routine will be triggered.
    } else {

        // Raise the FLAGS
        RX_failsafeStatus |= (1 << RX_CHANNELS) - 1;
        
        // Reset the counter
        RX_signalReceived = 0;
    }
    
    if (RX_signalReceived > 10) {

        RX_signalReceived = 10; // don't let the variable overflow
        failsafeEnabled = true; // this ensures that failsafe will operate in attitude mode
        
     //TODO: FAILSAFE CODE

    } else {
        failsafeEnabled = false;
    }
*/
}
