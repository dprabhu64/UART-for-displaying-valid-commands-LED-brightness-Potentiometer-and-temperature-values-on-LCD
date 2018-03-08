/*******************************************************************************
* File Name     : main.c
* Version       : 1.0
* Device(s)     : RX63N
* Tool-Chain    : Renesas RX Standard Toolchain 1.0.0
* OS            : None
* H/W Platform  : YRDKRX63N
* Description   : This is the main sample code. Application demonstrates use of 
*               : the UART to send and receive data over the RS232 interface. 
*		  It also displays the brightness values of LEDs,temperature,
*		  internal referance voltage  on LCD 
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 20.04.2014     1.00        First release
*******************************************************************************/
/*Includes   <System Includes> , "Project Includes"
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <machine.h>
#include "platform.h"
#include "uart.h"
#include <string.h>
#include "r_switches.h"
#include "r_init_clock.h"
#include "r_init_non_existent_port.h"
#include "r_init_stop_module.h"
#include "typedefine.h"
#ifdef __cplusplus
#endif

/*******************************************************************************
Macro definitions
*******************************************************************************/
#define SW_STANDBY  IR(ICU,IRQ15)       /* IRQ15: mode transit SW */
#define SW_ON       1                   /* SW on */
#define SW_OFF      0                   /* SW off */

#define LED_RUN     PORT0.PODR.BIT.B3   /* P03: run LED */
#define LED_ON      0                   /* LED on */
#define LED_OFF     1                   /* LED off */

#define LOOP_COUNT  (96000000L/128)     /* Loop counter: Wait for at least 1/128 seconds (ICLK:96MHz) */
#define MAX_TASKS 4

/*******************************************************************************
Typedef definitions
*******************************************************************************/
/* **** Time data **** */
typedef struct
{
    uint8_t     second;                 /* Second */
    uint8_t     minute;                 /* Minute */
    uint8_t     hour;                   /* Hour */
    uint8_t     dayweek;                /* Day of the week */
    uint8_t     day;                    /* Day */
    uint8_t     month;                  /* Month */
    uint16_t    year;                   /* Year */
} time_bcd_t;

/*******************************************************************************
Exported global variables and functions (to be accessed by other files)
*******************************************************************************/
time_bcd_t      time;                   /* Time data */

typedef struct {
 int initialTimerValue;
 int timer;
 int run;
 int enabled;
 void (* task)(void);
 } task_t;
 task_t GBL_task_table [MAX_TASKS ];

/*Defining Global variables */
 //int rel,release=0 ;
int temp_adc_out,alt_bright=0;
int bright=0,delay=0,delay1=0,chk=0,Pulse=0,flag=0,TFlag,IFlag,Hold1,Hold2;
int i,j,k,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,counter,counter1=0
,counter2=0,check_counter2,temp1,y,z,result,n=0;
char lcd_out[13],s4[100],s5[100],cmp[10];
char *rData,*s2,*s1;
char *LED,*Temp,*a,*Buff,*cmp1;
float Vs=0,T,Vref;
unsigned int temp_new =0,temp_new1=0;
uint8_t sec,min,hr,day,mnth,Rmin=70;


unsigned int temp_old,safe,Pulse_old=0;
unsigned int temp_diff;
temp_old=0;
temp_diff=0;
int dly_ctr;
int k=0;     


/*******************************************************************************
* Prototypes for local functions
*******************************************************************************/
void ADC_Init_POT();
void ADC_temperature_Init();
float temp_calculation();
void ADC_Int_ref_vlt();
float int_ref_vltg_calc();
uint32_t crc32(uint32_t crc, const void *buf, size_t size);
void S12ADC_isr(void);
void parsing1();
void rtc_init(void);
void rtc_time_read(void);
void RTC_Initialization();
void RTC_callback(void);
void RTC_display(void);
void f1_pulse_output();
void f2_get_POT();
void f3_RTC();
void f4_rx_commands_disp();
void init_Task_Timers(void);
void Init_TMR_Interrupt(void);
int Add_Task(void (*task)(void), int time, int priority);
void tick_timer_intr(void);
void Run_RTC_Scheduler(void);
/*****************************************************************************
* Function name: S12ADC_isr
* Description  : ADC Interrupt ISR function
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
*****************************************************************************/

#pragma interrupt (S12ADC_isr(vect=VECT_S12AD0_S12ADI0))
void S12ADC_isr(void)
{
	if(TFlag==1) 
	{
		Hold2=(S12AD.ADTSDR & 0x0FFF );  // Temporarily store converted digital 									ouput of Internal Ref. Voltage
TEMPS.TSCR.BYTE = 0x00;	// stops temperature sensor with output disabled to the ADC
/*Calling the function to initialize setting up of ADC registers for reading Internal Reference voltage*/
		ADC_Int_ref_vlt();
/*Calling the function to calculate Temperature value*/

		T= temp_calculation();
	}	
	if(IFlag==1) 
	{
Hold1=(S12AD.ADOCDR & 0X0FFF); // Temporarily store converted digital ouput of Temperature.


/*Calling the function to initialize setting up of ADC registers for reading Potentiometer values.*/
		ADC_Init_POT();
/*Calling the function to calculate Internal Reference Voltage value*/
		Vref=int_ref_vltg_calc();	
	
	}
}

/******************************************************************************
* Function name: main
* Description  : Main program function
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
*****************************************************************************/
void main(void)
{
	
	
/*Calling the function to initialize setting up of ADC registers for reading 
the temperature sensor value*/     

	ADC_temperature_Init();

/* The three pushbuttons on the YRDK board are tied to interrupt lines, set them up here */
    	R_SWITCHES_Init();

/* Initialize the SCI channel for asynchronous UART communications. */
	sci_uart_init(); 
    	sci_tx_int_enable();
    	sci_rx_int_enable(); 
		
		
/* A Receieve Buffer of 10 bytes created to read data stored in Serial Buffer of 127bytes*/
           Buff = calloc(10, sizeof(char)); //Receieve Buffer dynamically created
           a = calloc(10, sizeof(char)); 	  //Storage Receieve Buffer dynamically created to copy the above Buffer.




//* Initialize the Real Time Clock by calling Initialization Functions. */     
       RTC_Initialization ();
	
/* Initialize LCD function call */
  	   lcd_initialize(); 

/* Clear LCD function call */	
       lcd_clear();
	
/* Display message on LCD */
  lcd_display(LCD_LINE1, "  DARSHAN   ");
  lcd_display(LCD_LINE2, "  DHAIRYA  ");
  lcd_display(LCD_LINE3, "B Value=000");
	
/* Displays Temperature Sensor Output Value*/
    	sprintf(lcd_out,"T Value=%3.1f ",T);
    	lcd_display(LCD_LINE4,lcd_out);

/* Displays Input Reference Voltage Value*/
    	sprintf(lcd_out,"V Value=%3.2fV",Vref);
    	lcd_display(LCD_LINE5,lcd_out);
		
		

Init_TMR_Interrupt(); // Call tmr_interrupt function to init timer0
	
init_Task_Timers(); // initialize the RTC scheduler
//Add 4 Tasks to the RTC scheduler
Add_Task(f1_pulse_output,4000,0);//  Count recorded was 176//2000//3000
Add_Task(f2_get_POT,3116,1);//Count recorded was 107//186/1116//2116
Add_Task(f3_RTC,2416,2);	//121//1416
Add_Task(f4_rx_commands_disp,1611,3);//97//611
	        

Run_RTC_Scheduler(); // Run RTC scheduler 

} /* End of function main() */



/****************************************************************************
* Function name: sw1_callback
* Description  : Callback function that is executed when SW1 is pressed.
*                Called by sw1_isr in r_switches.c
*				 When SW1 is pressed the brightness of the LEDs is increased.
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
******************************************************************************/
void sw1_callback()
{
	
	/*The brightness is incremented*/
	if( bright <=90){
	bright= bright +10;
	/*delay is used to generate ON time delay */
	delay =delay+ 10; 
	/*delay 1 is used to generate OFF time delay*/
	delay1=100-delay;
	}
	
	else if(bright >90 && bright <100)  //Prevents overexceeding of duty cycle above 100%
     {
		 alt_bright = 100 + temp_new ;
		 bright = 100;
		 delay = 100;
		 delay1= 100-delay;
	 }
}
/* End of function sw1_callback() */


/*****************************************************************************
* Function name: sw2_callback
* Description  : Callback function that is executed when SW2 is pressed.
* Called by sw2_isr in r_switches.c When SW2 is pressed the brightness of the LEDs is reduced. 
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
****************************************************************************/
void sw2_callback(void)
{
	
	/*Tp prevent the single digit value of bright to go into negative */
	if( bright>=10 && bright <100)
	{
		bright= bright - 10;
	/*delay is uesd to generate ON time delay */
		delay =delay-10;
    	/*delay is used to generate OFF time delay*/
		delay1= 100-delay;
	}
    	if (bright >= 100)  //Prevents overexceeding of duty cycle above 100%
	{
		if( alt_bright !=0){
	     		bright = alt_bright ;
		}
		 bright=bright-10;
		 delay = bright;
		 delay1 =100-delay;
		 alt_bright=0;
	}		     
 }/* End of function sw2_callback() */
 
 
 
/******************************************************************************
* Function name: sw3_callback
* Description  : Callback function that is executed when SW3 is pressed.
*                Called by sw3_isr in r_switches.c
* Arguments    : none
* Return value : none
****************************************************************************/
void sw3_callback(void)
{

 RTC_display();
return 0;
      
} /* End of function sw3_callback() */



/*****************************************************************************
* Function name: crc32
* Description  : CRC 32 bit Computation. 
* Arguments    : crc,s4,size
* Return value : crc
****************************************************************************/
uint32_t crc32(uint32_t crc, const void *buf, size_t size)
{
static uint32_t crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

	const uint8_t *p;

	p = buf;
	crc = crc ^ ~0U;

	while (size--)
		crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

	return crc ^ ~0U;
}
/******************************************************************************
* Function name: ADC_Init_POT()
* Description  : ADC initialiazation for Potentiometer input 
* Arguments    : none
* Return value : none
*****************************************************************************/
void ADC_Init_POT() {
#ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
#endif
	SYSTEM.MSTPCRA.BIT.MSTPA17 = 0;/* 12 bit ADC module stop state is cancelled
							 i.e it is enabled */
	PORT4.PDR.BIT.B2 = 0; /*Pin selected as input Pin*/
	PORT4.PMR.BIT.B2 = 1; /*Selects the Pin as an I/O pin for peripheral(ADC) 
					functions*/
	
	MPC.P42PFS.BYTE = 0x80; 
	/* P4.2 Pin  Used as analog pin, analog input function selected, 
	Interrupt input function disabled.*/
	


	S12AD.ADCSR.BYTE = 0x4C; /*Continious Scan mode, PCLK enabled*/
	 
	S12AD.ADANS0.WORD = 0x0004; /* channel 2 is selected and subjected to scan 
							conversion */
	S12AD.ADANS1.WORD = 0x0000; // Channel 16-20 deselected
	S12AD.ADEXICR.WORD = 0x0000; /* Internal reference AD conversion , temperature 
							sensor o/p AD conversion deselected*/
	S12AD.ADCER.WORD = 0x0020; 
	/* Automatic Clearing (ACE) enabled, Right Alignment for ADC channel 0 Data
	 Register selected.*/
	
	S12AD.ADCER.BIT.ACE = 1;  /*automatic clearing*/
    //S12AD.ADCER.BIT.ADRFMT = 0;  /*Right Alignment*/

	S12AD.ADSTRGR.BYTE = 0x00; /*Asynchronous Trigger selected */

	S12AD.ADCSR.BIT.ADST = 1;        /*Start  ADC */
#ifdef PLATFORM_BOARD_RDKRX63N
	/* Turn write protection back on for all protected registers */
	SYSTEM.PRCR.WORD = 0xA500; 
/* Enable write protection for all protected registers */	
#endif 
printf("\nADC pot initialized");
 }

/*****************************************************************************
* Function name: ADC_temperature_Init()
* Description  : Temperature sensor output initialization 
* Arguments    : none
* Return value : none
******************************************************************************/
void ADC_temperature_Init(){
	int i;
#ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
#endif
	SYSTEM.MSTPCRA.BIT.MSTPA17 = 0;/* 12 bit ADC module stop state is cancelled
							 i.e it is enabled*/
	SYSTEM.MSTPCRB.BIT.MSTPB8 = 0;/*12 bit ADC module stop state is cancelled i.e 
							it is enabled */
	 
	#ifdef PLATFORM_BOARD_RDKRX63N
	/* Turn write protection back on for all protected registers */
	SYSTEM.PRCR.WORD = 0xA500; 
/* Enable write protection for all protected registers */	
#endif	
	
	TFlag=1;		    //Set Temp. Flag for executing Temp.Code in the ISR
	S12AD.ADANS0.WORD = 0x0000; /* channel 0-15 are de-selected and not subjected to
						 scan conversion */
	S12AD.ADANS1.WORD = 0x0000;	/*channel 16-20 are de-selected and not subjected to 
							scan conversion*/ 
	S12AD.ADCER.BIT.ADRFMT = 0;  /*Right Alignment*/
	S12AD.ADCSR.BYTE = 0x1C; // Single Scan mode selected, ADC Interrupt Enabled
	
	
	S12AD.ADSSTR23.WORD = 0xFF14;// for 255 states 
	S12AD.ADEXICR.WORD = 0x0100; /* Internal reference AD conversion deselected & 
						temperature sensor o/p AD conversion selected*/
	
	
	
	TEMPS.TSCR.BIT.TSEN = 1;/* Starts temperature sensor with output disabled to the 
						ADC*/
	IEN(S12AD0,S12ADI0)=0;   // ADC Interrupt disabled in ICU
	IPR(S12AD0,S12ADI0)=0x03;// ADC Interrupt priority set
	IR(S12AD0,S12ADI0) = 0;  // Any existing ADC Interrupt requests is cleared
	IEN(S12AD0,S12ADI0)=1;	// ADC Interrupt enabled in ICU
	
	// Waiting for the stabilization of reference voltage to the sensor for 30 micro seconds 
	for(i=0;i<3000;i++)
	{
	}
	TEMPS.TSCR.BIT.TSOE = 1;// Enable the output from the temperature sensor to ADC
	
	// Waiting for the stabilization of temperature output from the sensor for 1 micro second
	for(i=0;i<100;i++)
	{
	}
	
	S12AD.ADCSR.BIT.ADST = 1;        /*Start  the ADC */
}


/******************************************************************************
* Function name: temp_calculation 
* Description  : Temperature sensor output calculation  
* Arguments    : none
* Return value : temperature 
*****************************************************************************/
float temp_calculation ()
{

	float V1,Slope,T;
	temp_adc_out = Hold2; // Getting the ADC value from corresponding register.
	
	Vs = (temp_adc_out *3.3)/4096; /* convert to voltage corresponding to temperature 
						output*/
	/* Temperature Slope =4.1 mV/°C, output voltage(V1) @ 25°C is 1.26V.*/
	
	V1=1.26; // voltage @25'C
	Slope = 0.0041; // V /'C
    	T = ((Vs - V1) / Slope) + 25;
	TFlag=0;                  //Clear Temp. Flag for preventing Temp.Code to run in the ISR
	return (T);	
}
/*****************************************************************************
* Function name: ADC_Int_ref_vlt()
* Description  : Input Reference voltage initialization 
* Arguments    : none
* Return value : none
*****************************************************************************/
void ADC_Int_ref_vlt(){
//	int i;
#ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
#endif
	SYSTEM.MSTPCRA.BIT.MSTPA17 = 0;/* 12 bit ADC module stop state is cancelled 
							i.e it is enabled*/
	SYSTEM.MSTPCRB.BIT.MSTPB8 = 0;/* 12 bit ADC module stop state is cancelled i.e 
							it is enabled*/

	#ifdef PLATFORM_BOARD_RDKRX63N
	/* Turn write protection back on for all protected registers */
	SYSTEM.PRCR.WORD = 0xA500; 
/* Enable write protection for all protected registers */	
#endif 
	S12AD.ADCER.BIT.ADRFMT = 0;  /*Right Alignment*/
	S12AD.ADEXICR.WORD = 0x0200; /* Internal reference voltage AD conversion
				 selected & temperature sensor o/p AD conversion deselected*/
	S12AD.ADANS0.WORD = 0x0000; /* channel 0-15 are de-selected and not subjected to
						 scan conversion */
	S12AD.ADANS1.WORD = 0x0000;	/*channel 16-20 are de-selected and not subjected to
						 scan conversion*/ 
	S12AD.ADCSR.BYTE = 0x1C; // Single Scan mode selected , ADC Interrup Enabled
	
	IEN(S12AD0,S12ADI0)=0;  // ADC Interrupt disabled in ICU
	IPR(S12AD0,S12ADI0)=2;	// ADC Interrupt Priority set
	IR(S12AD0,S12ADI0) = 0;	// Any existing ADC Interrupt requests is cleared
	IEN(S12AD0,S12ADI0)=1;  // ADC Interrupt enabled in ICU

	IFlag=1;	//Set Int. Ref. Voltage Flag for executing Int. Ref. Volt. Code in the ISR
	S12AD.ADCSR.BIT.ADST = 1;        /*Start  ADC */
printf("ADC Int Ref Volt initialised");
}

/*****************************************************************************
* Function name: int_ref_vltg_calc(),,,,,,,,,,,,,,,,,,,,,,,,,,
* Description  : Input Reference voltage Calculation  
* Arguments    : none
* Return value : internal reference voltage(float)
****************************************************************************/
float int_ref_vltg_calc()
{

	//int temp_adc_out;
	float Vref;
	int x;
	x = Hold1; // Getting the ADC value from corresponding register
	
	Vref = (x *3.3)/4096; // convert to voltage corresponding to internal reference voltage
	IFlag=0;		/*Clear Int. Ref. Voltage. Flag for preventing Int. Ref. Voltage
				 Code to run in the ISR*/
	return (Vref);
	
	
}



/*****************************************************************************
* Function name:pulse_output,,,,,,,,,,,,,,,,,,,,,,,,,,
* Description  : Input Reference voltage Calculation  
* Arguments    : none
* Return value : internal reference voltage(float)
****************************************************************************/

void RTC_Initialization ()
{
//* Initialize the Real Time Clock by calling Initialization Functions. */     

    	R_INIT_StopModule();   // Stopping the peripherals which start operations
   	R_INIT_NonExistentPort();  
R_INIT_Clock(); 
	 
    	RTC.RCR4.BIT.RCKSEL=0;	// Sub-Clock selected
    	RTC.RCR3.BIT.RTCEN = 1;  // Sub-clock started
    	RTC.RCR2.BIT.START = 0;  // Stopping the RTC for updating registers
    	RTC.RCR4.BIT.RCKSEL=0;	// Sub-Clock selected
RTC.RCR2.BIT.HR24 = 1;         /* Hours Mode: 24-hour mode */
RTC.RCR2.BIT.RESET = 1;	// RTC registers reset
RTC.RCR3.BIT.RTCEN = 1;      // Sub-clock started
    /* ==== Setting the time and date in RTC ==== */
  	RTC.RCR2.BIT.START = 0; // RTC stopped/ disabled for setting values to registers
RTC.RCR2.BIT.RESET = 1; // RTC registers reset
    	RTC.RCR3.BIT.RTCEN = 1; // Sub-clock started	
	RTC.RYRCNT.WORD = 0x0014;           /* Set the BCD-coded value: 14 year */
RTC.RMONCNT.BYTE = 0x04;            /* Set the BCD-coded value: 04 month */
RTC.RDAYCNT.BYTE = 0x29;            /* Set the BCD-coded value: 18 day */
RTC.RHRCNT.BYTE = 0x09;             /* Set the BCD-coded value: 13 hour */
RTC.RWKCNT.BYTE = 0x01;             /* Set the day of the week: Monday */
RTC.RMINCNT.BYTE = 0x25;            /* Set the BCD-coded value: 00 minute */
RTC.RSECCNT.BYTE = 0x00;            /* Set the BCD-coded value: 00 second */
RTC.RCR4.BIT.RCKSEL=0;	// Sub-Clock selected
  	RTC.RCR2.BIT.AADJE=1;		// Automatic Adjustment Enabled
  	RTC.RCR2.BIT.START = 1; 	// Starting the RTC
	
while(RTC.RCR2.BIT.START == 0)
{				// Wait until RTC started
}
IEN(RTC,COUNTUP) = 0;	//RTC Carry Interrupt Disables in ICU
RTC.RCR1.BIT.CIE = 1;	// Carry  Interrupt enabled in RTC	
}




/*****************************************************************************
* Function name:pulse_output,,,,,,,,,,,,,,,,,,,,,,,,,,
* Description  : Input Reference voltage Calculation  
* Arguments    : none
* Return value : internal reference voltage(float)
****************************************************************************/


void f1_pulse_output(){

	
		if(bright !=0) {
			
		temp_old=temp_new; // current adc value (0-9) becomes previous adc value for detecting change in new value.   			
		  /*LED4 -LED15 are turned ON/OFF using commands from console and stored*/ 	
		  LED4=b4;
		  LED5=b5;
		  LED6=b6;
		  LED7=b7;
		  LED8=b8;
		  LED9=b9;
		  LED10=b10;
		  LED11=b11;
		  LED12=b12;
		  LED13=b13;
		  LED14=b14;
		  LED15=b15;
 		 
 /* LEDs are turned ON for certain time period depending on the duty cycle. The for loop generates the ON time period in PWM*/
	      for (dly_ctr = 0;dly_ctr <(4000*delay);dly_ctr++)
	      {
	      }

	      /*LED4 -LED15 are turned Off*/
		  LED4=LED5=LED6=LED7=LED8=LED9=LED10=LED11=LED12=LED13=LED14=LED15=1;

/* LEDs are turned OFF for certain time period depending on the duty cycle. The for loop generates the OFF time period in PWM*/
		  for (dly_ctr = 0;dly_ctr <(4000*delay1);dly_ctr++)
		  {
	      	  }
	   }
	 else
	{
	 	  LED4=LED5=LED6=LED7=LED8=LED9=LED10=LED11=LED12=LED13=LED14=LED15=1;	  		 
		  /* LEDs are turned OFF for certain time period depending on the duty cycle
		  //The for loop generates the OFF time period in PWM*/
	      for (dly_ctr = 0;dly_ctr <(4000*50);dly_ctr++)
	      {
	      }
	      /*LED4 -LED15 are turned Off*/
		  LED4=LED5=LED6=LED7=LED8=LED9=LED10=LED11=LED12=LED13=LED14=LED15=1;
		  
		  /* LEDs are turned OFF for certain time period depending on the duty cycle
		  The for loop generates the OFF time period in PWM*/
		  for (dly_ctr = 0;dly_ctr <(4000*50);dly_ctr++)
		  {
	      }
	 }



	}


/*****************************************************************************
* Function name:get_POT,,,,,,,,,,,,,,,,,,,,,,,,,,
* Description  : Input Reference voltage Calculation  
* Arguments    : none
* Return value : internal reference voltage(float)
****************************************************************************/
void f2_get_POT(){
	
	sprintf(lcd_out,"B Value=%0.3d",bright);	
	lcd_display(LCD_LINE3,lcd_out);
	
  	temp_new1 = (S12AD.ADDR2 & 0X0FFF); // Digitally converted pot value stored in a register.
	temp_new = temp_new1/409.6;	    // to get the current adc value from 0-9 	
	temp_diff= temp_new - temp_old;    // Change in the potentiometer value 
	if(temp_old!=temp_new)
			{
				safe=temp_new;			
			}
 	// To promote one extra decrement when decreasing duty cycle from 100 through 90 via pot
    	if( temp_new == 8 && temp_old ==9 && bright ==100) 
		temp_diff = temp_diff-1;
	
	/*Updating the duty cycle values and the delay below.*/ 
	/*The delay interval value is for LEDs turn OFF and Turn ON times */
	 
	if(Pulse==Pulse_old)
	{ 
	bright= bright + temp_diff;	
	delay = delay + temp_diff;
	delay1 = 100-delay;
	}
	else
	{
	bright= Pulse + safe;
	delay= bright;
	delay1= 100-delay;
	Pulse_old=Pulse;
	}
	

	/*To prevent duty cycle to fall below 0% when sw2 is pressed at duty cyle is 0%*/	
       if (bright <=0)
	{
	      bright=0;
		  delay=0;  
		  delay1=100-delay;
		  temp_old = 0;
	   }
	/*To prevent duty cycle to exceed over 100% when sw1 is pressed*/   
	   else if( bright >= 100){
		alt_bright = 100+temp_new;					    
		bright =100;
		delay=100;
		delay1=0;
		
		//offset= alt_bright-bright;
		}
	temp_old=temp_new;
		
	}

/*****************************************************************************
* Function name:RTC,,,,,,,,,,,,,,,,,,,,,,,,,,
* Description  : Input Reference voltage Calculation  
* Arguments    : none
* Return value : internal reference voltage(float)
****************************************************************************/
void f3_RTC()
{

if (RTC.RMINCNT.BYTE!= Rmin)
						{ 
							Rmin=RTC.RMINCNT.BYTE;		 		
							RTC_display();
						}


/*rel=GBL_task_table[0].timer;	 	
	if(release==0){
	printf("\nf1");
	printf("\nRel = %d \n",rel);
	printf("\n Task 2 done one time \n",GBL_task_table[0].timer);
	release++;
	}*/
	
}

/*****************************************************************************
* Function name:rx_commands_disp,,,,,,,,,,,,,,,,,,,,,,,,,,
* Description  : Input Reference voltage Calculation  
* Arguments    : none
* Return value : internal reference voltage(float)
****************************************************************************/
void f4_rx_commands_disp(){
	
		if (sci_read_count_get()) //Check for any waiting unread character in Serial Buffer
	{

		Buff[k] = sci_get_char();	// Get the char waiting in buffer.
		
		if(Buff[k]!= NULL)		//Check whether the read character is not NULL.
   	
		{ 

		
	
			if('\r' == Buff[k])
        			{
            sci_put_string("\r\n");      // CRLF combo          
        			}
        		else 
        			{
            sci_put_char(Buff[k]);   // Transmit the same character back. 
        			}
		 	if((k % 9 == 0) && k != 0) 
		 	{
            			Buff = (char * ) realloc(Buff, (k + 10));
				a = (char * ) realloc(a, (k + 10));		
		 	}			
		
			counter2++; //To count number of received command characters before ';'
			check_counter2++; /* Counter for checking availability of dynamic space
                                                                                      of atleast 10 bytes.*/
		
			a[k]=Buff[k];
			if (a[k]==';') //Checking for end of command chain.
			{
				counter=counter2; 
				counter2=0;
				k=0;
				free(Buff);		 // Dynamic array is made free
				Buff=  (char * ) calloc(10,sizeof(char));
				parsing1(counter);   /*Calling the String Parsing Function to parse
						individual valid commands from a command-chain*/
				
				bright=Pulse;	
				free(a);			// Dynamic array is made free
// Dynamically allocates array for next string of commands.
				a=  (char * ) calloc(10,sizeof(char));		
			}                       
	  		else 
	  		{
		  		k++;
	  		}
   		} 	
  

	}
 
}





/******************************************************************************
* Function name: parsing1
* Description  : Decodes received command messages separated by comma","			 
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
****************************************************************************/
void parsing1(int counter)
{
	int z1=0;
	char case1[7] = {'P','W','M','0','1','='};
	char case2[4] ={'L','E','D'};
			
	for(i=0;i<counter;i++)	
	{
		if(a[i]==','||a[i]==';')
		{
			if (n==8)
			{
				
				result=strncmp(case1,cmp,6);
				if(!result)
				{
					if(cmp[6]=='0')
					{
						z=cmp[7]-0x30;
						if((z>=0) || (z<=9))
						{
							Pulse=z*10;
						}
					}
					else if (cmp[6]=='1')
					{
						if(cmp[7]=='0')
						{
							Pulse=10*10;
		
						}
					}
				
				}
/*Start of LED Comand Parsing*/
				result =strncmp(case2,cmp,3);
			
				if(!result)
				{
					if (cmp[5]=='=')
					{

/*Start of LED OFF cases */
						if((cmp[6]=='0' & cmp[7]=='0')){

							if(cmp[3]=='0')
							{
								z=cmp[4]-0x30;

								if((z>=4) || (z<=9))
								{
									y=z;
								}
							}
							else if (cmp[3]=='1')
							{
								z=cmp[4]-0x30;
								if(z>=0 || z<=5)
								{
									y=z+10;
								}
							}
							switch(y)
							{
								case 4: b4=1;
								break;
								case 5: b5=1;
								break;
								case 6: b6=1;
								break;
								case 7: b7=1;
								break;
								case 8: b8=1;
								break;
								case 9: b9=1;
								break;
								case 10:b10=1;
								break;
								case 11:b11=1;
								break;
								case 12:b12=1;
								break;
								case 13:b13=1;
								break;
								case 14:b14=1;
								break;
								case 15:b15=1;
								break;
							}	
						}
/*Start of LED ON cases */
	
						else if ((cmp[6]=='0' & cmp[7]=='1'))  								{
							if(cmp[3]=='1')
							{
								z=cmp[4]-0x30;
								if(z>=0 || z<=5)
								{
									y=z+10;
								}
							}
							else if (cmp[3]=='0')
							{
								z=cmp[4]-0x30;
								if((z>=4) || (z<=9))
								{
									y=z;
								}
							}
							switch(y)
							{
								case 4: b4=0;
									break;
								case 5: b5=0;
									break;
								case 6: b6=0;
									break;
								case 7: b7=0;
									break;
								case 8: b8=0;
									break;
								case 9: b9=0;
									break;
								case 10:b10=0;
									break;
								case 11:b11=0;
									break;
								case 12:b12=0;
									break;
								case 13:b13=0;
									break;
								case 14:b14=0;
									break;
								case 15:b15=0;
									break;
							}
						}					
					}	
				}

n=0;  /* Counter, for counting the size of characters of valid command, 
			initialized back to zero.*/
			z1=0; // Offset to array initialized to zero again.
			free (cmp); // Free the space in the array for reuse.
			free(cmp1); // Free the space in the array for reuse.
			}
			else
			{
			n=0;	/* Counter, for counting the size of characters of valid
					 command, initialized back to zero.*/
				z1=0;	// Offset to array initialized to zero again.
				free (cmp);	// Free the space in the array for reuse.
				free(cmp1);	// Free the space in the array for reuse.	
			}
				
		}	
		else 
		{		
			cmp[z1]= a[i];
			z1++;	/* Array offset incremented to point to next sequential character in
					 command.*/
			if(z1==8)
			{
				z1=0;	/* Offset to array initialized to zero, if reached the 
						maximum limit of 8.*/
			}
			n++;	/* Counter, for counting the size of characters of valid command, 
					incremented.*/
		}
				
	}

}




/******************************************************************************
* Function name: init_Task_Timers
* Description  : Timer initialization			 
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
****************************************************************************/


void init_Task_Timers(void){
 int i;
 	
 /* Initialize all tasks */
 	for(i = 0; i < MAX_TASKS; i++){
 		GBL_task_table[i].initialTimerValue = 0;
 		GBL_task_table[i].run = 0;
 		GBL_task_table[i].timer = 0;
 		GBL_task_table[i].enabled = 0;
 		GBL_task_table[i].task = NULL;
 
	}
 }

/******************************************************************************
* Function name: Add_Task
* Description  : Adding Task 			 
* Arguments    : none
* Return value : none
*Authors       : Darshan Prabhu
****************************************************************************/

int Add_Task(void (*task)(void), int time, int priority){
 /* Check for valid priority */
 
 if(priority >= MAX_TASKS || priority < 0)
 	return 0;
 /* Check to see if we are overwriting an already scheduled
task */
 if(GBL_task_table [priority ].task != NULL)/*420 EMBEDDED SYSTEMS USING THERENESASRX63N MICROCONTROLLER*/
 	return 0;
 /* Schedule the task */
 	GBL_task_table[priority].task = task;
 	GBL_task_table[priority].run = 0;
 	GBL_task_table[priority].timer = time;
 	GBL_task_table[priority].enabled = 1;
 	GBL_task_table[priority].initialTimerValue = time;
 	//printf("/n add task %d",priority);
	return 1;
 }

/******************************************************************************
* Function name: Enable Task 
* Description  : Enable Task 			 
* Arguments    : none
* Return value : none
*Authors       : Darshan Prabhu
****************************************************************************/

/*void Enable_Task(int task_number){
GBL_task_table[task_number].enabled = 1;
 }

 void Disable_Task(int task_number){
 GBL_task_table[task_number].enabled = 0;
 }*/
 
/******************************************************************************
* Function name: Enable Task 
* Description  : Enable Task 			 
* Arguments    : none
* Return value : none
*Authors       : Darshan Prabhu
****************************************************************************/
 

void Run_RTC_Scheduler(void){
 
 int i;
 printf("\n rr1");
 /* Loop forever */
 while(1){
 /* Check each task */
 
 	for(i = 0; i < MAX_TASKS; i++){
 /* check if valid task */
 
 		if(GBL_task_table[i].task != NULL){
 /* check if enabled */
 
 			if(GBL_task_table[i].enabled == 1){
 /* check if ready to run */
 
 
 				if(GBL_task_table[i].run == 1){
 /* Reset the run ?ag */
 
 					GBL_task_table[i].run = 0;
 /* Run the task */
 					
					GBL_task_table[i].task();
					
 /* break out of loop to start at entry 0 */
					
				
					break;
 				}
 			}
 		}
	}
	 
 }
 
}

/******************************************************************************
* Function name: Init_TMR_Interrupt 
* Description  : Enable Task 			 
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
****************************************************************************/
void Init_TMR_Interrupt(void)
{

#ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
    #endif
    /* Power up the TMR unit using the appropriate Module Stop Control Register */
	MSTP(TMR0) = 0;
	
	
#ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA500; /* Protect on  */
    #endif	
	
    /* Timer Control Register (TCR) 
	b7      CMIEB
	b6      CMIEA: 1 = compare match A interrupt requests are enabled
	b5      OVIE
    b4:b3   CCLR:  0 = TCNT clearing disabled
    b2:b0   Reserved. The write value should always be 0.
    */
    
    TMR0.TCR.BYTE = 0x48;//COUNTER cleared by compare Match A  interrupt
    TMR1.TCR.BYTE = 0x00;//clearing is disabled
    /* Timer Counter Control Register (TCCR)
	b7      TMRIS
    b6:b5   Reserved. The write value should always be 0.
    b4:b3   CSS: 3 = cascade TMR 0 and TMR 1
    b2:b0   CKS: 2 = Count at PCLK / 8192
    */
    /* cascade TMR0 and TMR1, TMR0 count when TMR1 overflows, TMR1 counts from PCLK  */
	TMR0.TCCR.BYTE = 0x08; /* PCLK selected */
//	TMR01.TCCR = 0x1808; 
    /* Timer Control/Status Register (TCSR)
    b7:b5   Reserved. The write value should always be 1.
    b4      ADTE
	b3:b2   OSB
	b1:b0   OSA
    */
    TMR0.TCSR.BYTE = 0xE0;// No change in Output
    //TMR1.TCSR.BYTE = 0xE0;
    /* Timer Constant Registers (TCORA and/or TCORB)
    *  b15:b8 Timer Constant Register 0
    *  b07:b0 Timer Constant Register 1
    */
	
	TMR0.TCORA = 240; /* delay length approx 5 us in timer counts. */ 
	    /* If PCLK = 48 MHz and we are using PCLK / 8192 to increment the timer then */
		/* one count  = 8,192 /48 = 170.667 uS */
		/* one second = 48,000,000 / 8,192 = 5,859.375 counts */
	TMR01.TCNT = 0x0000;
		
	IR (TMR0,CMIA0) = 0; /* interrupt reset */
    IPR(TMR0,CMIA0) = 4; /* interrupt priority set */
    IEN(TMR0,CMIA0) = 1; /* interrupt enable */
 
}
/******************************************************************************
* Function name: tick_timer_intr 
* Description  : ISR for Timer Interrupt of 5us 			 
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
****************************************************************************/


#pragma interrupt (tick_timer_intr(vect=VECT(TMR0, CMIA0)))
void tick_timer_intr(void) {
static char i;

for (i=0 ; i<MAX_TASKS ;i++) { 
	if (GBL_task_table[i].task != NULL) {
		if (GBL_task_table[i].enabled == 1) {
			if (GBL_task_table[i].timer) { 
				if (--GBL_task_table[i].timer == 0){
					GBL_task_table[i].run = 1;
					GBL_task_table[i].timer = GBL_task_table[i].initialTimerValue;
					printf("\n GBL_task_table[%d].timer=%d",i,GBL_task_table[i].timer);
					}
			} 
		} 
	}
} 
}
/*****************************************************************************
* Function name: RTC_display
* Description  : Display RTC values on host PC console
* Arguments    : none
* Return value : none
*Authors       :  Darshan Prabhu
****************************************************************************/
void RTC_display(void)
{
/*Declaring local variables*/

       volatile uint8_t dummy;
        uint32_t crc;
        size_t size;	
        time.second = RTC.RSECCNT.BYTE;         /* Read the BCD-code second */
        time.minute = RTC.RMINCNT.BYTE;         /* Read the BCD-code minute */
        time.hour = RTC.RHRCNT.BYTE;            /* Read the BCD-coded hour */
        time.dayweek = RTC.RWKCNT.BYTE;         /* Read the day of the week */
        time.day = RTC.RDAYCNT.BYTE;            /* Read the BCD-coded day */
        time.month = RTC.RMONCNT.BYTE;          /* Read the BCD-coded month */
        time.year = 0x2000 | RTC.RYRCNT.WORD;   /* Read the BCD-coded year */

        /*sec=(time.second & 0xF0)>>4;
        time.second=time.second & 0x0F;
	
        min=(time.minute & 0xF0)>>4;
        time.minute=time.minute & 0x0F;
    	
        hr=(time.hour & 0xF0)>>4;
        time.hour=time.hour & 0x0F;
		
        day=(time.day & 0xF0)>>4;
        time.day=time.day & 0x0F;
		
        mnth=(time.month & 0xF0)>>4;
        time.month=time.month & 0x0F;*/

        sci_put_string("\r\n");                  

/* Sending entire string to hyperterminal*/
sprintf(s4,"%x-%0.2x-%0.2xT%0.2x:%0.2x:%0.2x:B Value=%0.3d;T Value=%3.1f;V Value=%3.2f",time.year,time.month,time.day,time.hour,time.minute,time.second,bright,T,Vref);	
//sprintf(s4,"%x-%d%d-%d%dT%d%d:%d%d:%d%d:B Value=%0.3d;T Value=%3.1f;V Value=%3.2f",time.year,mnth,time.month,day,time.day,hr,time.hour,min,time.minute,sec,time.second,bright,T,Vref);

	strcpy(s5,s4);
	crc=0x00000000;
	size=strlen(s5);
	crc=  crc32(crc, s5,size );   // Calculating CRC-32 for the string sent to hyperterminal
	sprintf(s5,"CRC-32:%X",crc); 
	sci_put_string(s4);          // Sending CRC-32 for the string sent to hyperterminal
	sci_put_string("\r\n");
	sci_put_string(s5);
	sci_put_string("\n\r\n");	
}
