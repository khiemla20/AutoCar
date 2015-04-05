/*
Name :
				Khiem Lam
				Samuel Weber
*/


#include "main.h" /* include peripheral declarations */
#include "MKL25Z4.h"
#include "adc16.h"
#include "timers.h"

volatile unsigned short PW1 = 4500, PW2 = 50,PW3=50;
unsigned short MOD1=6000, MOD0=600;

volatile char TPMflag = 0;
volatile unsigned short counter_LED;
volatile int Dec,Inc,redBlink;

unsigned char ADC_Array0[128];// Ping buffer used to store ADC values
unsigned char ADC_Array1[128]; //Pong buffer
unsigned char ADC_Array2[128]; //Sing buffer
unsigned char ADC_Array3[128]; //Song buffer

//int InitArr=0;
int ArrControl[128];


unsigned char First=0, Second=0;
unsigned char FirstCent=0,SecondCent=0;

unsigned char trackBuffer[127];
unsigned char MSB, LSB;

 
int counterP,counterS;
int doneP;
int doneS;
int doneBoth;
int ping,sing;
int pingTurn;
int idxP,idxS;        // Index used to implement circular buffer
int counterGPIO;
int read3, read7, printLast2=0;
unsigned char A_IFB,B_IFB;
unsigned int count1P,count1S;
unsigned char Vol_threshold;

const uint32_t led_mask[] = { 1UL << 18, 1UL << 19, 1UL << 1 };
#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE   2
 
void print(unsigned char , int);


int count0_B(int Arr[128]){
int i=0;
int count0=0;
	while(Arr[i++]==0){ 
		count0++;
	}
	return count0;
}


int count0_E(int Arr[128]){
int i=128;
int count0=0;
	while(Arr[i--]==0){ 
		count0++;
	}
	return count0;
}


void init_ADC0(void);

unsigned char findAvg(unsigned char Arr[128]){
 
    int i, x = 0;
    for (i = 0; i<128; i++)
        x = x + Arr[i];
    x = x / 128;
    return x;
 
}
 
unsigned char findDiff(unsigned char Arr[128], unsigned char *getMin){
    int min, max, i;
    min = Arr[0];
    max = min;
    for (i = 1; i<128; i++){
        if (min>Arr[i]) min = Arr[i];
        if (max< Arr[i]) max = Arr[i];
    }
		*getMin=min;
    return (max - min);
}
 
unsigned char findMin(unsigned char Arr[128]){
	int i;
	int min = Arr[0];
	for(i=0;i<128;i++)
	if (min>Arr[i]) min = Arr[i];
	return min;
}

int algorithmFirst(unsigned char buffer[128]){
	int i,count1=0;
	unsigned char  base,diff;
	diff=findDiff(buffer,&base);
	Vol_threshold=diff/2+0x0A ;
	
	for (i=0;i<128;i++)
	{
		if(buffer[i]<Vol_threshold)  {
			uart0_putchar('0'); //black pixel
			ArrControl[i] = 0;
		}
		else {
			uart0_putchar('1'); //white pixel
			ArrControl[i] = 1;
			count1++;
		}
	 }
	
	uart0_putchar('\r');
	uart0_putchar('\n');
	
	return count1;
}


void Init_ADC(void) {
	
	init_ADC0();			// initialize and calibrate ADC0
	ADC0->CFG1 = (ADLPC_LOW | ADIV_1 | ADLSMP_LONG | MODE_8 | ADICLK_BUS_2);	// 8 bit, Bus clock/2 = 12 MHz
	ADC0->SC2 = 0;		// ADTRG=0 (software trigger mode)
	ADC0->CFG2 = ADC_CFG2_MUXSEL_MASK;
}


unsigned int Read_ADC (void) {
	volatile unsigned int res=0;
	
	ADC0->SC1[0] = 0xC; 			// start conversion (software trigger) on AD12 i.e. ADC0_SE12 (PTB2)
	
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		// wait for conversion to complete (polling)

	res = ADC0->R[0];				// read result register
	return res;
}

void TPM0_IRQHandler(void) {
	
	NVIC_ClearPendingIRQ(TPM0_IRQn);
	
	if(TPM0->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK)
		TPM0->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
	if(TPM0->CONTROLS[2].CnSC & TPM_CnSC_CHF_MASK)
		TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHF_MASK;
	
	TPM0->CONTROLS[0].CnV =PW2;
	TPM0->CONTROLS[2].CnV =PW3;
}
void TPM1_IRQHandler(void) {
//clear pending IRQ
	NVIC_ClearPendingIRQ(TPM1_IRQn);
	
	
// clear the Channel flag mask by writing 1 to CHF
		
	if (TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) 
		TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
			
	TPM1->CONTROLS[0].CnV = PW1;
	
	


}

void Init_PWM(void) {

// Set up the clock source for MCGPLLCLK/2. 
// See p. 124 and 195-196 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
// TPM clock will be 48.0 MHz if CLOCK_SETUP is 1 in system_MKL25Z4.c.
	
	SIM-> SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK | SIM_SOPT2_UART0SRC(1));
	
	
// See p. 207 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; // Turn on clock to TPM1 (servo)
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // Turn on clock to TPM0
	
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK
								| SIM_SCGC5_PORTB_MASK
								| SIM_SCGC5_PORTC_MASK
								| SIM_SCGC5_PORTD_MASK
								| SIM_SCGC5_PORTE_MASK );
	
	
	
	PORTA->PCR[1] = PORT_PCR_MUX(0x2);		// Enable the UART0_RX function on PTA1
	PORTA->PCR[2] = PORT_PCR_MUX(0x2);		// Enable the UART0_TX function on PTA2


		PORTD->PCR[7] = (1UL << 8);    /* Pin PTD7  is GPIO */
    PORTE->PCR[1] = 1UL << 8;              //Pin PTE1 is GPIO
		PORTB->PCR[0] = (1UL << 8);   // Pin PTB0 is GPIO
    FPTD->PDDR |= 1UL << 7; //PTD7 as an output
    FPTE->PDDR |= 1UL << 1; //PTE1 as an output
		FPTD->PDDR |= 1UL << 1;          /* enable PTD1 as Output */                                            
		FPTB->PDOR |= 1;                                                                 // initialize PTB0
    FPTB->PDDR |= 1;                                                                 // configure PTB0 as output

	
	PORTE->PCR[21] = 1UL << 8; //PTE21 is an GPIO /EN pin
	PORTC->PCR[13] = PORT_PCR_MUX(1); //PORT PTC13 is an GPIO pin (SW1)
	PORTC->PCR[17] = PORT_PCR_MUX(1); //PORT PTC17 is an GPIO  pin (SW2)
	//PORTC->PCR[2] = PORT_PCR_MUX(1); //configure PTC2 as GPIO
//	PORTC->PCR[4] = PORT_PCR_MUX(1); //configure PTC4 as GPIO
PORTC->PCR[2] = 1UL<<8;
PORTC->PCR[4] = 1UL<<8;
	
	FPTC->PDDR = 1UL << 2 | 1UL << 4 ; //set PTC2 and PTC4 as an output GPIO
	FPTC->PDDR &=~(1UL<<13 | 1UL<<17) ; // assign 0 to PTC13(SW1) and PTC17(SW2) to set them as an input GPIO
	
	FPTE->PDDR |= 1UL << 21 ; // PTE21 is configured as an output
	FPTE->PCOR = 1UL << 21; //turn PTE21(EN) off /DISABLE H-Bridge
	//PTC->PDOR = 1UL<<2 | 1UL<<4;
	FPTC->PCOR =1UL<<2; // deassert PTC2
	FPTC->PCOR = 1UL<<4 ; // deassert PTC4
	
	
// See p. 163 and p. 183-184 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
	PORTB->PCR[0] = PORT_PCR_MUX(3); // Configure PTB1 as TPM1_CH1 (servo)
	PORTC->PCR[1] = PORT_PCR_MUX(4); //configure PTC1 as TPM0_CH0
	PORTC->PCR[3] = PORT_PCR_MUX(4); //configure PTC3 as TPM0_CH2
	
	
// Set channel TPM1_CH0 to edge-ligned, high-true PWM
	
 	TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;//(servo)
		TPM1->CONTROLS[0].CnSC |=  TPM_CnSC_CHIE_MASK; //(for servo)
		
		// Set channel TPM0_CH1/TPC1 to center-ligned, high-true PWM
		TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK |TPM_CnSC_CHIE_MASK ;

		
		// Set channel TPM0_CH2/TPC3 to center-aligned, high-true PWM
		TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK |TPM_CnSC_CHIE_MASK ;
	
		// Set channel PTC3/TPM0_CH2 to center-aligned, low-true PWM
	//	TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK ;
		
// Set period and pulse widths for TPM1
	
	TPM1->CONTROLS[0].CnV = PW1; 	//(for servo)
	TPM1->MOD = 60000-1;	//5khz	// Freq. = (48 MHz / 16) / 3000 = 1k Hz
	
	// Set period and pulse widths for TPM0 CH1 / LEFT - POT1
	TPM0->MOD = 600-1;	//5khz	// Freq. = (48 MHz / 16) / 3000 =  1k Hz
	TPM0->CONTROLS[0].CnV = PW2;
	

	
	// Set period and pulse widths for TPM0 CH2 / RIGHT - POT2
	TPM0->CONTROLS[2].CnV = PW3;
	
	
// set TPM1 to up-counter, divide by 16 prescaler and clock mode
	TPM1->SC = ( TPM_SC_CMOD(1) | TPM_SC_PS(4));
	
	// set TPM0 to up/down-counter, divide by 16 prescaler and clock mode
	//TPM0->SC = (TPM_SC_CPWMS_MASK | TPM_SC_PS(4));
	TPM0->SC = ( TPM_SC_CMOD(1) | TPM_SC_PS(4));

// clear the Channel flag mask by writing 1 to CHF
	if (TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
	
	// clear the Channel flag mask on CH1 by writing 1 to CHF
	//if (TPM0->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) TPM0->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;//
	if (TPM0->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) TPM0->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
	
	// clear the Channel flag mask on CH2 by writing 1 to CHF
	
		if (TPM0->CONTROLS[2].CnSC & TPM_CnSC_CHF_MASK) TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHF_MASK;

// Enable Interrupts

	NVIC_SetPriority(TPM0_IRQn, 192); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM0_IRQn); 
	NVIC_EnableIRQ(TPM0_IRQn);	
	
	NVIC_SetPriority(TPM1_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM1_IRQn); 
	NVIC_EnableIRQ(TPM1_IRQn);	

	
	
	counter_LED=0;
	redBlink=0;

}	


void put(char *ptr_str)
{
	while(*ptr_str)
		uart0_putchar(*ptr_str++);
}

unsigned char hex_ascii(unsigned char hex){
    unsigned char ascii;
    if (hex>9) ascii = hex - 10 + 'A';
    else ascii = hex + '0';
 
    return ascii;
}
void print(unsigned char hex, int b){
	if(b) hex=hex_ascii((hex>>4) & 0x0f);
	else hex=hex_ascii(hex&0x0f);
	uart0_putchar(hex);
}


void printBuff(unsigned char *ADC_Arr){
	int i;
	for(i=0; i<128;i++){
		print(ADC_Arr[i],1);
		print(ADC_Arr[i],0);
		uart0_putchar(' ');
	}
}
int main(void){
	int uart0_clk_khz;
	char str1[] = "\r\nPlease turn power supply on, then press SW2! \r\n";
	int flag=1;
	unsigned char pot1,pot2;
	int count0_C_R=0, count0_C_L=0;
	int count0_R,count0_L;
	int timer=0;
	int count1_C=0;
	int delTa=0;
	int CURV=0;
	int ST=0, Hill=0;
	int counter, ACCEL=1;
	const int ACCEL_LIM=50;
	const int RATE=2,INIT_RATE=1;
	unsigned short currentDeg,delTa_CURV,delTa_CURV_W;
	int ST_TO_CURV = 0,	CURV_TO_ST = 0;
	const unsigned short centerDeg=4500;
	unsigned short speedST=50;
	
	// Lab 4 variables   
    ping = 1;
		sing =1;
		pingTurn=1; //ping buffer start first

	 Init_PWM();  
	 uart0_clk_khz = (48000000 / 1000); // UART0 clock frequency will equal half the PLL frequency	
	uart0_init (uart0_clk_khz, TERMINAL_BAUD);  

	
// LED_Initialize();
    
    Init_ADC();
    Init_PIT(10000);        // count-down period = 10,000 us
    Start_PIT();
 
    // Enable Interrupts 
    NVIC_SetPriority(ADC0_IRQn, 128); // 0, 64, 128 or 192
    NVIC_ClearPendingIRQ(ADC0_IRQn);
    NVIC_EnableIRQ(ADC0_IRQn);

	
	
	
	put(str1);
		while(!( FPTC->PDIR & 1UL<<17)){}
		
		FPTE->PSOR = 1UL << 21; //enable H-Bridge
	
			//PART 2
			
			while(flag){
					ADC0->SC1[0] = 0xD; 			// start conversion (software trigger) on AD13 i.e. ADC0_SE13 (PTB3)
						while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		// wait for conversion to complete (polling)
						pot1 = ADC0->R[0]; //POT1 left 
							print(pot1,1);
							print(pot1,0);
							uart0_putchar(' ');
						
					ADC0->SC1[0] = 0xC; 			// start conversion (software trigger) on AD13 i.e. ADC0_SE13 (PTB3)
							while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		// wait for conversion to complete (polling)
								pot2 = ADC0->R[0]; //POT2 right
								print(pot2,1);
								print(pot2,0);
								uart0_putchar(' ');
						
					
							if(FPTC->PDIR & 1UL<<13) {
								speedST = 200;
								flag=0;
								counter=3000;
							}
				}
			
				
				while (1) {
  
            						
  if(uart0_getchar_present()== 0){
		
		count1P=0;
		count0_R=0;
		
			//the outer most camera is ping/right camera
					if(doneP){
						if (ping)  {
							count1P=algorithmFirst(ADC_Array0);
							
						
						}
						else {
							count1P=algorithmFirst(ADC_Array1);
							
		
						}
						
								if (ping) ping = 0;
										else ping = 1;
										 
					count0_R = count0_B(ArrControl);
					count0_L = count0_E(ArrControl);
						
						if(count0_C_R ==0) timer=40;
							if(timer>0){
							 count0_C_R=count0_R;
							 count0_C_L=count0_L;
								count1_C = count1P;
								timer--;
							}
							print(count1P,1);
							print(count1P,0);
							uart0_putchar(' ');	
							print(count1_C,1);
							print(count1_C,0);
							
							put(" count0_C_R  ");
							print(count0_C_R,1);
							print(count0_C_R,0);
							put(" count0_R  ");
							print(count0_R,1);
							print(count0_R,0);
							
							
							put(" count0_C_L  ");
							print(count0_C_L,1);
							print(count0_C_L,0);
							put(" count0_L  ");
							print(count0_L,1);
							print(count0_L,0);
							
							put("\r\n");
							doneP = 0;							
					}
		}
	
						while(counter-- >0){}
	
		ACCEL=INIT_RATE;
		//drive motor
		if(timer==0){
			
		//else {
				if(count0_R > count0_C_R  ){ //car mov toward the right, need to turn left
					
					
					ACCEL*=RATE;
					delTa=count0_R - count0_C_R;
					delTa_CURV_W=80+delTa*7;
					delTa_CURV=200+delTa*14;
					
					if(count0_R - count0_C_R <=0){ //straight
							
							ST++;
							
						if(CURV >= 10 && ST>=2){ 
						ST_TO_CURV = 0;
						CURV_TO_ST = 1;
						CURV=0;
					}
					
					
						if(ACCEL>=ACCEL_LIM)ACCEL=ACCEL_LIM;
						
								if(A_IFB>=0x15)Hill=1;
								else Hill=0;
						
						if(!ST_TO_CURV){
							//PW2=PW3 = speedST - ACCEL;
							PW2=PW3 = speedST;
							PW1= centerDeg;
						}
						
					
						
						if(Hill) PW2=PW3 = 30;
					
							}
				else	if(count0_R - count0_C_R <= 2){ //adjust to center
					
								ST++;
					
					if(CURV >= 10 && ST>=2){ 
						ST_TO_CURV = 0;
						CURV_TO_ST = 1;
						CURV=0;
					}
					
					
					if(ACCEL>=ACCEL_LIM)ACCEL=ACCEL_LIM;
					
						if(!ST_TO_CURV){
							//PW2=PW3 = speedST - ACCEL;
							PW2=PW3 = speedST;
							PW1= centerDeg-80;
						}
					
					
					
					if(Hill) PW2=PW3 = 30;
							}
				else if(count0_R - count0_C_R <=4){//adjust to center
							
					
					ST++;
					if(CURV >= 10 && ST>=2){ 
						ST_TO_CURV = 0;
						CURV_TO_ST = 1;
						CURV=0;
					}
					
					
					if(ACCEL>=ACCEL_LIM)ACCEL=ACCEL_LIM;
					
					if(!ST_TO_CURV){
							//PW2=PW3 = speedST - ACCEL;
						PW2=PW3 = speedST;
							PW1= centerDeg-100;
						}
					
					//dealing with hill
					if(Hill) PW2=PW3 = 30;
							}
				
				else if(count0_R - count0_C_R >= 6){ //turn
						CURV++;
					
					if((CURV>=1) && (ST>=20)){
						ST_TO_CURV = 1;
						CURV_TO_ST = 0;
						ST=0;
						}
					
					
						
					//calculating turning deg
					
									if(CURV_TO_ST){//reducing wiggle here
										if(delTa_CURV_W<800)
											PW1= centerDeg + (delTa_CURV_W);
										else PW1=centerDeg + 800;
										
										PW2=PW3 = (speedST)*7/10 ;
									}
									else {//normal turning 
										if(delTa_CURV<1500)
											PW1= centerDeg - delTa_CURV;
										else PW1=centerDeg - 1500;
										
										//PW2=PW3 = (speedST)/delTa + (speedST*5)/10 ;
									
										PW2=PW3 =speedST*8/10-ACCEL;
										
									}
									
									
									currentDeg=PW1;
									
									
									if(Hill) PW1=centerDeg;
									
									
						}//end of turn
				
					} //end of right
								
			else if(count0_L > count0_C_L  ){//car moves towarding left
					ACCEL*=RATE;
				delTa=count0_L - count0_C_L;
				delTa_CURV_W=80+delTa*7;
					delTa_CURV=200+delTa*14;
		
			if(count0_L - count0_C_L <= 0){ //straight
							ST++;
							
						if(CURV >= 10 && ST>=2){ 
						ST_TO_CURV = 0;
						CURV_TO_ST = 1;
						CURV=0;
					}
					
					
						if(ACCEL>=ACCEL_LIM)ACCEL=ACCEL_LIM;
						
								if(A_IFB>=0x15)Hill=1;
								else Hill=0;
						
						if(!ST_TO_CURV){
							//PW2=PW3 = speedST - ACCEL;
							PW2=PW3 = speedST;
							PW1= centerDeg;
						}
						
					
						
						if(Hill) PW2=PW3 = 30;
					
							}
				else	if(count0_L - count0_C_L <= 2){ //adjust to center
					
								ST++;
					
					if(CURV >= 10 && ST>=2){ 
						ST_TO_CURV = 0;
						CURV_TO_ST = 1;
						CURV=0;
					}
					
					
					if(ACCEL>=ACCEL_LIM)ACCEL=ACCEL_LIM;
					
						if(!ST_TO_CURV){
							//PW2=PW3 = speedST - ACCEL;
							PW2=PW3 = speedST;
							PW1= centerDeg+80;
						}
					
					
					
					if(Hill) PW2=PW3 = 30;
							}
				else if(count0_L - count0_C_L<=4){//adjust to center
							
					
					ST++;
					if(CURV >= 10 && ST>=2){ 
						ST_TO_CURV = 0;
						CURV_TO_ST = 1;
						CURV=0;
					}
					
					
					if(ACCEL>=ACCEL_LIM)ACCEL=ACCEL_LIM;
					
					if(!ST_TO_CURV){
							//PW2=PW3 = speedST - ACCEL;
						PW2=PW3 = speedST;
							PW1= centerDeg+100;
						}
					
					//dealing with hill
					if(Hill) PW2=PW3 = 30;
							}
				
				else if(count0_L - count0_C_L >= 6){ //turn
						CURV++;
					
					if((CURV>=1) && (ST>=20)){
						ST_TO_CURV = 1;
						CURV_TO_ST = 0;
						ST=0;
						}
					
					
						
					//calculating turning deg
					
									if(CURV_TO_ST){//reducing wiggle here
										if(delTa_CURV_W<800)
											PW1= centerDeg + (delTa_CURV_W);
										else PW1=centerDeg + 800;
										
										PW2=PW3 = (speedST)*7/10 ;
									}
									else {//normal turning 
										if(delTa_CURV<1500)
											PW1= centerDeg + delTa_CURV;
										else PW1=centerDeg + 1500;
										
										//PW2=PW3 = (speedST)/delTa + (speedST*5)/10 ;
										PW2=PW3 =speedST*8/10-ACCEL;
									}
									
									
									currentDeg=PW1;
									
									
									if(Hill) PW1=centerDeg;
									
									
						}//end of turn
				}//end of left
			else {
			PW1 = currentDeg;
			}
			
				if(ST>20000){ 
					ST=2000;
					
				}
				if(CURV>1000){
					CURV=100;
			
				}
		}//end of timer == 0
	
	} // end of while			
} // end main


	
	
void ADC0_IRQHandler() {
		//Clear interupt request flag
				NVIC_ClearPendingIRQ(ADC0_IRQn);		
				FPTE->PSOR = 1UL<<1; //assert PTE1 /CLK
		
				ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; // select b channel
	
    // read one value from ADC0 using software triggering
				if (ping)
            ADC_Array0[idxP] = ADC0->R[0];     //Read to Ping buffer	
				
				else 
						ADC_Array1[idxP] = ADC0->R[0];    //Read to Pong buffer  
				
				
				//if (counterS<127)					
				if(idxP<128){
					ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6); // start conversion on C2
					
				 if(idxP==127) 
					doneP=1;
				}
				idxP++;       
			FPTE->PCOR = 1UL<<1; // deassert PTE1/ CLK
	
			
			
		
						
   	counterGPIO++;

			if(read7){
				read7=0;
				A_IFB = ADC0->R[0];
				ADC0->CFG2 = ADC_CFG2_MUXSEL_MASK;
				printLast2=1;
			}
			if(read3){
				read3=0;
				B_IFB = ADC0->R[0];		
				ADC0->CFG2 = 0;
				ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(7);
				read7=1;
			
			}
			if(counterGPIO==256){
				FPTB->PCOR = (1UL << 0); //deassert PTB0, start conversion    
				
				//NVIC_ClearPendingIRQ(ADC0_IRQn);
				ADC0->CFG2 = 0; //select a channel for conversion	
				//ADC0->SC1[0]=0x3;
				ADC0->SC1[0]=AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(3);
				read3=1;
			}

			
	
			
			
        
}

