#include "MKL25Z4.h"
#include "timers.h"
#include "adc16.h"

volatile unsigned PIT_interrupt_counter = 0;
extern int idxP,idxS;
extern int counterP,counterS;
extern int doneP;
extern int doneS;
extern int pingTurn;
extern int counterGPIO;
extern int doneBoth;

void Init_PIT(unsigned period_us) {
	// Enable clock to PIT module
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

	// Enable module, freeze timers in debug mode
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	PIT->MCR |= PIT_MCR_FRZ_MASK;

	// Initialize PIT0 to count down from argument 
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(period_us * 24); // 24 MHz clock frequency

	// No chaining
	PIT->CHANNEL[0].TCTRL &= PIT_TCTRL_CHN_MASK;

	// Generate interrupts
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;

	/* Enable Interrupts */
	NVIC_SetPriority(PIT_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(PIT_IRQn);
	NVIC_EnableIRQ(PIT_IRQn);
}


void Start_PIT(void) {
	// Enable counter
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void Stop_PIT(void) {
	// Disable counter
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}


void PIT_IRQHandler() {
	idxP = 0;
	idxS = 0;
	//counterP = 1;
	//counterS =1;
	doneP=0;
	doneS=0;
	doneBoth=0;
pingTurn=1;
	counterGPIO=0;
	
	// Asset SI PTD7
	FPTD->PSOR = 1UL << 7;

	// Assert PTB0 for measuring conversion
	FPTB->PSOR = 1UL << 0;

	//clear pending IRQ
	NVIC_ClearPendingIRQ(PIT_IRQn);
	
	//assert CLK signal, PTE1
	FPTE->PSOR = 1UL << 1;
	
	

	// check to see which channel triggered interrupt 
	if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {


		// clear status flag for timer channel 0
		PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;

		// Do ISR work - move next sample from buffer to DAC
		//FPTB->PTOR = 1;							// toggle PTB0

	}
	else if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
		// clear status flag for timer channel 1
		PIT->CHANNEL[1].TFLG &= PIT_TFLG_TIF_MASK;
	}
	
	

	//Deassert PTD7
	FPTD->PCOR = 1UL << 7;
	
	//ADC0->SC1[0] = AIEN_ON | 0x0C; 			
	ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; // select b channel
	// start conversion on channel SE6b (PTD5)
	ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6);
	
	//Deassert CLK, PTE1
	FPTE->PCOR = 1UL << 1;
	

	FPTB->PCOR = 1UL<<0;
}

// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
