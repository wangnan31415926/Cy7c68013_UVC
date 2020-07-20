#pragma NOIV                    // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:      slave.c
//   Contents:  Hooks required to implement USB peripheral function.
//              Code written for FX2 REVE 56-pin and above.
//              This firmware is used to demonstrate FX2 Slave FIF
//              operation.
//   Copyright (c) 2003 Cypress Semiconductor All rights reserved
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"            // SYNCDELAY macro

#define LED_ALL         (bmBIT0 | bmBIT1 | bmBIT2 | bmBIT3)

extern BOOL GotSUD;             // Received setup data flag
extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

BYTE Configuration;             // Current configuration
BYTE AlternateSetting;          // Alternate settings
static WORD xdata LED_Count = 0;
static BYTE xdata LED_Status = 0;
BOOL done_frm_fpga = 0;

// EZUSB FX2 PORTA = slave fifo enable(s), when IFCFG[1:0]=11
//sbit PA0 = IOA ^ 0;             // alt. func., INT0#
//sbit PA1 = IOA ^ 1;             // alt. func., INT1#
// sbit PA2 = IOA ^ 2;          // is SLOE
//sbit PA3 = IOA ^ 3;             // alt. func., WU2
// sbit PA4 = IOA ^ 4;          // is FIFOADR0
// sbit PA5 = IOA ^ 5;          // is FIFOADR1
// sbit PA6 = IOA ^ 6;          // is PKTEND
// sbit PA7 = IOA ^ 7;          // is FLAGD

// EZUSB FX2 PORTC i/o...       port NA for 56-pin FX2
// sbit PC0 = IOC ^ 0;
// sbit PC1 = IOC ^ 1;
// sbit PC2 = IOC ^ 2;
// sbit PC3 = IOC ^ 3;
// sbit PC4 = IOC ^ 4;
// sbit PC5 = IOC ^ 5;
// sbit PC6 = IOC ^ 6;
// sbit PC7 = IOC ^ 7;

// EZUSB FX2 PORTB = FD[7:0], when IFCFG[1:0]=11
// sbit PB0 = IOB ^ 0;
// sbit PB1 = IOB ^ 1;
// sbit PB2 = IOB ^ 2;
// sbit PB3 = IOB ^ 3;
// sbit PB4 = IOB ^ 4;
// sbit PB5 = IOB ^ 5;
// sbit PB6 = IOB ^ 6;
// sbit PB7 = IOB ^ 7;

// EZUSB FX2 PORTD = FD[15:8], when IFCFG[1:0]=11 and WORDWIDE=1
//sbit PD0 = IOD ^ 0;
//sbit PD1 = IOD ^ 1;
//sbit PD2 = IOD ^ 2;
//sbit PD3 = IOD ^ 3;
//sbit PD4 = IOD ^ 4;
//sbit PD5 = IOD ^ 5;
//sbit PD6 = IOD ^ 6;
//sbit PD7 = IOD ^ 7;

// EZUSB FX2 PORTE is not bit-addressable...

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
// The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------
//void LED_Off (BYTE LED_Mask);
//void LED_On (BYTE LED_Mask);

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------
void TD_Init(void)             // Called once at startup
{
   // set the CPU clock to 48MHz
   	CPUCS = 0x12;  	//48MHZ	 CLKOUT ENALBE

   // set the slave FIFO interface to 48MHz
   IFCONFIG |= 0x40;
//   IFCONFIG = 0X03; //选择为外部时钟，且为同步slaveFIFO模式，输入IFCLK:5~48MHz

  // Registers which require a synchronization delay, see section 15.14
  // FIFORESET        FIFOPINPOLAR
  // INPKTEND         OUTPKTEND
  // EPxBCH:L         REVCTL
  // GPIFTCB3         GPIFTCB2
  // GPIFTCB1         GPIFTCB0
  // EPxFIFOPFH:L     EPxAUTOINLENH:L
  // EPxFIFOCFG       EPxGPIFFLGSEL
  // PINFLAGSxx       EPxFIFOIRQ
  // EPxFIFOIE        GPIFIRQ
  // GPIFIE           GPIFADRH:L
  // UDMACRCH:L       EPxGPIFTRIG
  // GPIFTRIG
  
  // Note: The pre-REVE EPxGPIFTCH/L register are affected, as well...
  //      ...these have been replaced by GPIFTC[B3:B0] registers

  // default: all endpoints have their VALID bit set
  // default: TYPE1 = 1 and TYPE0 = 0 --> BULK  
  // default: EP2 and EP4 DIR bits are 0 (OUT direction)
  // default: EP6 and EP8 DIR bits are 1 (IN direction)
  // default: EP2, EP4, EP6, and EP8 are double buffered

  // we are just using the default values, yes this is not necessary...
  SYNCDELAY;        // see TRM section 15.14
  EP2CFG = 0xE0;	// enabled, quad buffered, 512B, IN, bulk fifo, 4 buffer
//	EP2CFG = 0xE8;	// enabled, quad buffered, 1024B, IN, bulk fifo, 4 buffer
  SYNCDELAY;                    
  EP4CFG = 0x00;	// disabled...
  SYNCDELAY;                    
  EP6CFG = 0x00;	// disabled...
  SYNCDELAY;                    
  EP8CFG = 0x00;	// disabled...

  // out endpoints do not come up armed
  
  // since the defaults are double buffered we must write dummy byte counts twice
//  SYNCDELAY;                    
//  EP2BCL = 0x80;                // arm EP2OUT by writing byte count w/skip.
//  SYNCDELAY;                    
//  EP2BCL = 0x80;
//  SYNCDELAY;                    
//  EP4BCL = 0x80;                // arm EP4OUT by writing byte count w/skip.
//  SYNCDELAY;                    
//  EP4BCL = 0x80;    

// Configure the EPxFIFOCFG
	EP2FIFOCFG = 0x00; 	// autoin, 8 Bit Wide
//	EP2FIFOCFG = 0x09; 	// autoin, 16 Bit Wide
	SYNCDELAY;         	
	EP4FIFOCFG = 0x00;	// no-autoOUT, bytewide
	SYNCDELAY;                     
	EP6FIFOCFG = 0x00; 	// no-autoOUT, bytewide
	SYNCDELAY;                     
	EP8FIFOCFG = 0x00;	// no-autoOUT, bytewide
	SYNCDELAY;    

// Configure PIN Polarity
	PORTACFG |= 0x40;	//IFCOG[1:0] = 11(Slave FIFO Mode), Set PORTACFG[6] to USE PA7-SLCS
	SYNCDELAY;	
	FIFOPINPOLAR = 0x00;//BIT[5:0] = {PKTEND, SLOE, SLRD, SLWR, EMPTY, FULL}
						//Set SLWR, PKTEND, SLOE,SLRD EMPTY, FULL Low Active
	SYNCDELAY;

// Configure Autoin package Length
//	EP2AUTOINLENH = 0x02; // EZ-USB automatically commits data in 512-byte chunks
// 	EP2AUTOINLENH = 0x04; // EZ-USB automatically commits data in 1024-byte chunks
//	SYNCDELAY;
// 	EP2AUTOINLENL = 0x00;
//	SYNCDELAY;	
	
// FLAGA - User-Programmable Level; FLAGB - FIFO Full, FLAGC - FIFO Empty: (L: Valid)
	PINFLAGSAB = 0x00;//0x8a;
	SYNCDELAY;  
	PINFLAGSCD = 0x00;//0x08;
	SYNCDELAY;

// Reset FIFO, then clear endpoint
	SYNCDELAY;
	FIFORESET = 0x80;// activate NAK-ALL to avoid race conditions
	SYNCDELAY;
	FIFORESET = 0x02;// reset, FIFO 2
	SYNCDELAY;
 	FIFORESET = 0x04;// reset, FIFO 4
	SYNCDELAY;
	FIFORESET = 0x06;// reset, FIFO 6
	SYNCDELAY;
    FIFORESET = 0x08;// reset, FIFO 8
	SYNCDELAY;
	FIFORESET = 0x00;// deactivate NAK-AL
	SYNCDELAY;

// enable dual autopointer feature
   // AUTOPTRSETUP |= 0x01;

   USBIE |= bmSOF;
   SYNCDELAY;
   //Configure Start Trigger
   OED |= (1<<2);	//PD2	0:Input;	1:output
   PD2 = 0;
  

}

unsigned char BCH=0x8c;
unsigned short wn_33ms=0;
void TD_Poll( void )
{ // Called repeatedly while the device is idle
static unsigned long wn=0;
static unsigned long nan=4;
static BOOL	 flag=0;
unsigned short i=0;

if(wn<0x0000ffff0)
{wn++;}
else
{
//PD2=1;

if(0)//(wn_33ms!=160)
{}
else{
while( !( EP24FIFOFLGS & 0x02 ) )
 { // EP8EF=0, when buffer not empty
 ; // wait ‘til host takes entire FIFO data
 }
 

//if(nan==4)
//{
flag=~flag;
//nan=0;

EP2FIFOBUF[0]=0x0c;//	0c 8d 9c e1  85 20 64 99  ac 20 03 06


	EP2FIFOBUF[1]=BCH;
	if(0)//(flag)
	{}
	else 
	{
	
	 /* Modify UVC header to toggle Frame ID */
	        BCH ^= 0x01;
	
	        /* Indicate End of Frame in the buffer */
 //EP2FIFOBUF[1] |=  0x02;
	}
	EP2FIFOBUF[2]=nan;
	EP2FIFOBUF[3]=0x12;
	EP2FIFOBUF[4]=0x12;
	EP2FIFOBUF[5]=0x12;
	EP2FIFOBUF[6]=(unsigned char)(nan&0x000000ff);;			   
	EP2FIFOBUF[7]=(unsigned char)((nan>>8)&0x000000ff);
	EP2FIFOBUF[8]=(unsigned char)((nan>>16)&0x000000ff);
	EP2FIFOBUF[9]=(unsigned char)((nan>>24)&0x000000ff);
	EP2FIFOBUF[10]=0xc1;
	EP2FIFOBUF[11]=0x05;
//	EP2FIFOBUF[12]++;
//}
   SYNCDELAY;
for(i=6;i<512/2;i++)
{
// if(EP2FIFOBUF[i]>100){EP2FIFOBUF[i]=EP2FIFOBUF[i]%128;}	
// if(EP2FIFOBUF[i]<20){EP2FIFOBUF[i]=EP2FIFOBUF[i]+20;}
  EP2FIFOBUF[2*i]=0xf4;
  EP2FIFOBUF[2*i+1]=0x80;

 // EP2FIFOBUF[3*i+2]=0x80;
}



 nan++;
  SYNCDELAY;
	EP2BCH = 0x01;
	 SYNCDELAY;
			EP2BCL = 0x2c;//0xbc; // Clear bytecount to allow new data in; also stops NAKing
			 SYNCDELAY;
		 INPKTEND=0x02;
  

}
}

}

BOOL TD_Suspend( void )          
{ // Called before the device goes into suspend mode
   return( TRUE );
}

BOOL TD_Resume( void )          
{ // Called after the device resumes
   return( TRUE );
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------
BOOL DR_GetDescriptor( void )
{
   return( TRUE );
}

BOOL DR_SetConfiguration( void )   
{ // Called when a Set Configuration command is received
  
  if( EZUSB_HIGHSPEED( ) )
  { // ...FX2 in high speed mode

	EP2AUTOINLENH = 0x02;   // set core AUTO commit len = 512 bytes
    SYNCDELAY;
	EP2AUTOINLENL = 0x00;

  }
  else
  { // ...FX2 in full speed mode
    EP6AUTOINLENH = 0x00;
    SYNCDELAY;
    EP8AUTOINLENH = 0x00;   // set core AUTO commit len = 64 bytes
    SYNCDELAY;
    EP6AUTOINLENL = 0x40;
    SYNCDELAY;
    EP8AUTOINLENL = 0x40;
  }
      
  Configuration = SETUPDAT[ 2 ];
  return( TRUE );        // Handled by user code
}

BOOL DR_GetConfiguration( void )   
{ // Called when a Get Configuration command is received
   EP0BUF[ 0 ] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);          // Handled by user code
}

BOOL DR_SetInterface( void )       
{ // Called when a Set Interface command is received
   AlternateSetting = SETUPDAT[ 2 ];
   return( TRUE );        // Handled by user code
}

BOOL DR_GetInterface( void )       
{ // Called when a Set Interface command is received
   EP0BUF[ 0 ] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return( TRUE );        // Handled by user code
}

BOOL DR_GetStatus( void )
{
   return( TRUE );
}

BOOL DR_ClearFeature( void )
{
   return( TRUE );
}

BOOL DR_SetFeature( void )
{
   return( TRUE );
}

BOOL DR_VendorCmnd( void )
{
  return( TRUE );
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav( void ) interrupt 0
{
   GotSUD = TRUE;         // Set flag
   EZUSB_IRQ_CLEAR( );
   USBIRQ = bmSUDAV;      // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok( void ) interrupt 0
{
   EZUSB_IRQ_CLEAR( );
   USBIRQ = bmSUTOK;      // Clear SUTOK IRQ
}

void ISR_Sof( void ) interrupt 0
{		 
  if(wn_33ms<273){wn_33ms++;}
  else{wn_33ms=0;}
   EZUSB_IRQ_CLEAR( );
   USBIRQ = bmSOF;        // Clear SOF IRQ
}

void ISR_Ures( void ) interrupt 0
{
   if ( EZUSB_HIGHSPEED( ) )
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }
   
   EZUSB_IRQ_CLEAR( );
   USBIRQ = bmURES;       // Clear URES IRQ
}

void ISR_Susp( void ) interrupt 0
{
   Sleep = TRUE;
   EZUSB_IRQ_CLEAR( );
   USBIRQ = bmSUSP;
}

void ISR_Highspeed( void ) interrupt 0
{
   if ( EZUSB_HIGHSPEED( ) )
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }

   EZUSB_IRQ_CLEAR( );
   USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack( void ) interrupt 0
{
}
void ISR_Stub( void ) interrupt 0
{
}
void ISR_Ep0in( void ) interrupt 0
{
}
void ISR_Ep0out( void ) interrupt 0
{
}
void ISR_Ep1in( void ) interrupt 0
{
}
void ISR_Ep1out( void ) interrupt 0
{
}
void ISR_Ep2inout( void ) interrupt 0
{
}
void ISR_Ep4inout( void ) interrupt 0
{
}
void ISR_Ep6inout( void ) interrupt 0
{
}
void ISR_Ep8inout( void ) interrupt 0
{
}
void ISR_Ibn( void ) interrupt 0
{
}
void ISR_Ep0pingnak( void ) interrupt 0
{
}
void ISR_Ep1pingnak( void ) interrupt 0
{
}
void ISR_Ep2pingnak( void ) interrupt 0
{
}
void ISR_Ep4pingnak( void ) interrupt 0
{
}
void ISR_Ep6pingnak( void ) interrupt 0
{
}
void ISR_Ep8pingnak( void ) interrupt 0
{
}
void ISR_Errorlimit( void ) interrupt 0
{
}
void ISR_Ep2piderror( void ) interrupt 0
{
}
void ISR_Ep4piderror( void ) interrupt 0
{
}
void ISR_Ep6piderror( void ) interrupt 0
{
}
void ISR_Ep8piderror( void ) interrupt 0
{
}
void ISR_Ep2pflag( void ) interrupt 0
{
}
void ISR_Ep4pflag( void ) interrupt 0
{
}
void ISR_Ep6pflag( void ) interrupt 0
{
}
void ISR_Ep8pflag( void ) interrupt 0
{
}
void ISR_Ep2eflag( void ) interrupt 0
{
}
void ISR_Ep4eflag( void ) interrupt 0
{
}
void ISR_Ep6eflag( void ) interrupt 0
{
}
void ISR_Ep8eflag( void ) interrupt 0
{
}
void ISR_Ep2fflag( void ) interrupt 0
{
}
void ISR_Ep4fflag( void ) interrupt 0
{
}
void ISR_Ep6fflag( void ) interrupt 0
{
}
void ISR_Ep8fflag( void ) interrupt 0
{
}
void ISR_GpifComplete( void ) interrupt 0
{
}
void ISR_GpifWaveform( void ) interrupt 0
{
}

// ...debug LEDs: accessed via movx reads only ( through CPLD )
// it may be worth noting here that the default monitor loads at 0xC000
xdata volatile const BYTE LED0_ON  _at_ 0x8000;
xdata volatile const BYTE LED0_OFF _at_ 0x8100;
xdata volatile const BYTE LED1_ON  _at_ 0x9000;
xdata volatile const BYTE LED1_OFF _at_ 0x9100;
xdata volatile const BYTE LED2_ON  _at_ 0xA000;
xdata volatile const BYTE LED2_OFF _at_ 0xA100;
xdata volatile const BYTE LED3_ON  _at_ 0xB000;
xdata volatile const BYTE LED3_OFF _at_ 0xB100;
// use this global variable when (de)asserting debug LEDs...
BYTE xdata ledX_rdvar = 0x00;
BYTE xdata LED_State = 0;
void LED_Off (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_OFF;
		LED_State &= ~bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_OFF;
		LED_State &= ~bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_OFF;
		LED_State &= ~bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_OFF;
		LED_State &= ~bmBIT3;
	}
}

void LED_On (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_ON;
		LED_State |= bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_ON;
		LED_State |= bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_ON;
		LED_State |= bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_ON;
		LED_State |= bmBIT3;
	}
}

