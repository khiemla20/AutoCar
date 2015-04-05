// PWM Example program using Timer Overflow Interrupts.
// PWM Channel: TPM1_CH1 (PTB1) PWM frequency - 1 kHz; 
// Pulse width oscillates in continuously between 0% and 100% duty cycle 
// where the pulse width is updated in the TPM interrupt handler.

#include "MKL25Z4.h"

volatile unsigned short PW1 = 4500;
volatile char TPMflag = 0;
volatile unsigned short counter_LED;

void TPM1_IRQHandler(void) {
//clear pending IRQ
	NVIC_ClearPendingIRQ(TPM1_IRQn);
	
// clear the overflow mask by writing 1 to TOF
	
	//if (TPM1->SC & TPM_SC_TOF_MASK) 
	//	TPM1->SC |= TPM_SC_TOF_MASK;
	
	if (TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) 
		TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;

// modify pulse width for TPM1_CH1
	
	if (!TPMflag) {
		TPM1->CONTROLS[0].CnV = ++PW1; 	
		//TPM1->CONTROLS[0].CnV = PW1;
		if (PW1 == 6000)
			TPMflag = 1;
	} else {
		TPM1->CONTROLS[0].CnV = --PW1; 	
		//TPM1->CONTROLS[0].CnV = 0;
		if (PW1 == 3000) 
			TPMflag = 0;
	}
	
	counter_LED++;
		if (counter_LED == 50) 
		{
			PTB->PTOR = 1UL << 18;
			counter_LED = 0;
		}
}

void Init_PWM(void) {

// Set up the clock source for MCGPLLCLK/2. 
// See p. 124 and 195-196 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
// TPM clock will be 48.0 MHz if CLOCK_SETUP is 1 in system_MKL25Z4.c.
	
	SIM-> SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	
	
// See p. 207 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; // Turn on clock to TPM1


// See p. 163 and p. 183-184 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
	PORTB->PCR[0] = PORT_PCR_MUX(3); // Configure PTB1 as TPM1_CH1

// Set channel TPM1_CH1 to edge-aligned, high-true PWM
	
  	TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
//	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK | TPM_CnSC_CHIE_MASK;
	TPM1->CONTROLS[0].CnSC |=  TPM_CnSC_CHIE_MASK;
	
// Set period and pulse widths
	
	TPM1->MOD = 60000-1;		// Freq. = (48 MHz / 16) / 3000 = 1 kHz
	
	TPM1->CONTROLS[0].CnV = PW1; 	
	
// set TPM1 to up-counter, divide by 16 prescaler and clock mode
	
	//TPM1->SC = (TPM_SC_TOIE_MASK | TPM_SC_CMOD(1) | TPM_SC_PS(4));
	TPM1->SC = ( TPM_SC_CMOD(1) | TPM_SC_PS(4));

// clear the overflow mask by writing 1 to TOF
	
	//if (TPM1->SC & TPM_SC_TOF_MASK) TPM1->SC |= TPM_SC_TOF_MASK;
	if (TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;

// Enable Interrupts

	NVIC_SetPriority(TPM1_IRQn, 192); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM1_IRQn); 
	NVIC_EnableIRQ(TPM1_IRQn);	
	
	counter_LED=0;

}	

int main (void) {
	
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK
								| SIM_SCGC5_PORTB_MASK
								| SIM_SCGC5_PORTC_MASK
								| SIM_SCGC5_PORTD_MASK
								| SIM_SCGC5_PORTE_MASK );
	
	//SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; //activate portB
	
	PORTB->PCR[18] = 1UL << 8; //PTB0 is GPIO
	PTB->PDOR = 1 ;
	PTB->PDDR = 1UL << 18; // PTB18 is an output
	//PTB->PSOR = 1UL << 18;  // turn off LED Red
	 Init_PWM();

	 while (1) {
			__wfi();
	 }
 }
