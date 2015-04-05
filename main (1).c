// ADC test program using software triggering and polling
// Uses ADC0_SE12 (PTB2)
// Note that ADC0_SE12 is the default pin assignment for PTB2
// Therefore PORTB->PCR[2] doesn't need to be explicitly configured.
// Input signal on ADC0_SE12 (PTB2) should be in 0 - 3V range.


#include <MKL25Z4.H>
#include "adc16.h"

void init_ADC0(void);

const uint32_t led_mask[] = {1UL << 18, 1UL << 19, 1UL << 1};
#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE	 2


void LED_Initialize(void) {

  SIM->SCGC5    |= (SIM_SCGC5_PORTB_MASK 
									| SIM_SCGC5_PORTD_MASK);      			/* Enable Clock to Port B & D */ 
  PORTB->PCR[18] = (1UL <<  8);                      /* Pin PTB18 is GPIO */
  PORTB->PCR[19] = (1UL <<  8);                      /* Pin PTB19 is GPIO */
  PORTD->PCR[1]  = (1UL <<  8);                      /* Pin PTD1  is GPIO */
	
  FPTB->PDOR = (led_mask[0] | led_mask[1] );          /* switch Red/Green LED off  */
  FPTB->PDDR = (led_mask[0] | led_mask[1] );          /* enable PTB18/19 as Output */

  FPTD->PDOR = led_mask[2];            /* switch Blue LED off  */
  FPTD->PDDR = led_mask[2];            /* enable PTD1 as Output */
}


void Init_ADC(void) {
	
	init_ADC0();			// initialize and calibrate ADC0
	ADC0->CFG1 = (ADLPC_LOW | ADIV_1 | ADLSMP_LONG | MODE_8 | ADICLK_BUS_2);	// 8 bit, Bus clock/2 = 12 MHz
	ADC0->SC2 = 0;		// ADTRG=0 (software trigger mode)
}


unsigned int Read_ADC (void) {
	volatile unsigned int res=0;
	
	ADC0->SC1[0] = 0xC; 			// start conversion (software trigger) on AD12 i.e. ADC0_SE12 (PTB2)
	
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		// wait for conversion to complete (polling)

	res = ADC0->R[0];				// read result register
	return res;
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	unsigned char val;
	
	
  LED_Initialize();
	Init_ADC();
	
	while (1) {
		val = Read_ADC();	// read one value from ADC0 using software triggering and polling
		if (val & 0x80)
			  FPTB->PCOR   = led_mask[LED_RED];    /* Red LED On*/
		else
			  FPTB->PSOR   = led_mask[LED_RED];    /* Red LED Off*/

		if (val & 0x40)
			  FPTB->PCOR   = led_mask[LED_GREEN];    /* Green LED On*/
		else
			  FPTB->PSOR   = led_mask[LED_GREEN];    /* Green LED Off*/

		if (val & 0x20)
			  FPTD->PCOR   = led_mask[LED_BLUE];    /* Blue LED On*/
		else
			  FPTD->PSOR   = led_mask[LED_BLUE];    /* Blue LED Off*/
		
	}
}

