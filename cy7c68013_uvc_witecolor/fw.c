//-----------------------------------------------------------------------------
// File:      fw.c
// Contents:  Firmware frameworks task dispatcher and device request parser
//            source.
//
// indent 3.  NO TABS!
//
// $Revision: 17 $
// $Date: 11/15/01 5:45p $
//
// Copyright (c) 2002 Cypress Semiconductor, Inc. All rights reserved
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
#define DELAY_COUNT   0x9248*8L  // Delay for 8 sec at 24Mhz, 4 sec at 48
#define _IFREQ  24000            // IFCLK constant for Synchronization Delay
#define _CFREQ  48000            // CLKOUT constant for Synchronization Delay

//-----------------------------------------------------------------------------
// Random Macros
//-----------------------------------------------------------------------------
#define   min(a,b) (((a)<(b))?(a):(b))
#define   max(a,b) (((a)>(b))?(a):(b))

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
  
#include "fx2sdly.h"             // Define _IFREQ and _CFREQ above this #include

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
volatile BOOL   GotSUD;
BOOL      Rwuen;
BOOL      Selfpwr;
volatile BOOL   Sleep;                  // Sleep mode enable flag

WORD   pDeviceDscr;   // Pointer to Device Descriptor; Descriptors may be moved
WORD   pDeviceQualDscr;
WORD   pHighSpeedConfigDscr;
WORD   pFullSpeedConfigDscr;   
WORD   pConfigDscr;
WORD   pOtherConfigDscr;   
WORD   pStringDscr;   

//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------
void SetupCommand(void);
void TD_Init(void);
void TD_Poll(void);
BOOL TD_Suspend(void);
BOOL TD_Resume(void);

BOOL DR_GetDescriptor(void);
BOOL DR_SetConfiguration(void);
BOOL DR_GetConfiguration(void);
BOOL DR_SetInterface(void);
BOOL DR_GetInterface(void);
BOOL DR_GetStatus(void);
BOOL DR_ClearFeature(void);
BOOL DR_SetFeature(void);
BOOL DR_VendorCmnd(void);

// this table is used by the epcs macro 
const char code  EPCS_Offset_Lookup_Table[] =
{
   0,    // EP1OUT
   1,    // EP1IN
   2,    // EP2OUT
   2,    // EP2IN
   3,    // EP4OUT
   3,    // EP4IN
   4,    // EP6OUT
   4,    // EP6IN
   5,    // EP8OUT
   5,    // EP8IN
};

// macro for generating the address of an endpoint's control and status register (EPnCS)
#define epcs(EP) (EPCS_Offset_Lookup_Table[(EP & 0x7E) | (EP > 128)] + 0xE6A1)

//-----------------------------------------------------------------------------
// Code
//-----------------------------------------------------------------------------

// Task dispatcher
void main(void)
{
   DWORD   i;
   WORD   offset;
   DWORD   DevDescrLen;
   DWORD   j=0;
   WORD   IntDescrAddr;
   WORD   ExtDescrAddr;
   DWORD   tCount=0;
   // Initialize Global States
   Sleep = FALSE;               // Disable sleep mode
   Rwuen = FALSE;               // Disable remote wakeup
   Selfpwr = FALSE;            // Disable self powered
   GotSUD = FALSE;               // Clear "Got setup data" flag


	
   // Initialize user device
    TD_Init();
   

   // The following section of code is used to relocate the descriptor table. 
   // Since the SUDPTRH and SUDPTRL are assigned the address of the descriptor 
   // table, the descriptor table must be located in on-part memory.
   // The 4K demo tools locate all code sections in external memory.
   // The descriptor table is relocated by the frameworks ONLY if it is found 
   // to be located in external memory.
   pDeviceDscr = (WORD)&DeviceDscr;
   pDeviceQualDscr = (WORD)&DeviceQualDscr;
   pHighSpeedConfigDscr = (WORD)&HighSpeedConfigDscr;
   pFullSpeedConfigDscr = (WORD)&FullSpeedConfigDscr;
   pStringDscr = (WORD)&StringDscr;

   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }

   if ((WORD)&DeviceDscr & 0xe000)
   {
      IntDescrAddr = INTERNAL_DSCR_ADDR;
      ExtDescrAddr = (WORD)&DeviceDscr;
      DevDescrLen = (WORD)&UserDscr - (WORD)&DeviceDscr + 2;
      for (i = 0; i < DevDescrLen; i++)
         *((BYTE xdata *)IntDescrAddr+i) = 0xCD;
      for (i = 0; i < DevDescrLen; i++)
         *((BYTE xdata *)IntDescrAddr+i) = *((BYTE xdata *)ExtDescrAddr+i);
      pDeviceDscr = IntDescrAddr;
      offset = (WORD)&DeviceDscr - INTERNAL_DSCR_ADDR;
      pDeviceQualDscr -= offset;
      pConfigDscr -= offset;
      pOtherConfigDscr -= offset;
      pHighSpeedConfigDscr -= offset;
      pFullSpeedConfigDscr -= offset;
      pStringDscr -= offset;
   }

   EZUSB_IRQ_ENABLE();            // Enable USB interrupt (INT2)
   EZUSB_ENABLE_RSMIRQ();            // Wake-up interrupt

   INTSETUP |= (bmAV2EN | bmAV4EN);     // Enable INT 2 & 4 autovectoring

   USBIE |= bmSUDAV | bmSUTOK | bmSUSP | bmURES | bmHSGRANT;   // Enable selected interrupts
   EA = 1;                  // Enable 8051 interrupts

#ifndef NO_RENUM
   // Renumerate if necessary.  Do this by checking the renum bit.  If it
   // is already set, there is no need to renumerate.  The renum bit will
   // already be set if this firmware was loaded from an eeprom.
   if(!(USBCS & bmRENUM))
   {
       EZUSB_Discon(TRUE);   // renumerate
   }
#endif

   // unconditionally re-connect.  If we loaded from eeprom we are
   // disconnected and need to connect.  If we just renumerated this
   // is not necessary but doesn't hurt anything
   USBCS &=~bmDISCON;

   CKCON = (CKCON&(~bmSTRETCH)) | FW_STRETCH_VALUE; // Set stretch to 0 (after renumeration)

   // clear the Sleep flag.
   Sleep = FALSE;



   // Task Dispatcher
   while(TRUE)               // Main Loop
   {
      if(GotSUD)            // Wait for SUDAV
      {
         SetupCommand();          // Implement setup command
           GotSUD = FALSE;            // Clear SUDAV flag
      }

      // Poll User Device
      // NOTE: Idle mode stops the processor clock.  There are only two
      // ways out of idle mode, the WAKEUP pin, and detection of the USB
      // resume state on the USB bus.  The timers will stop and the
      // processor will not wake up on any other interrupts.
      if (Sleep)
          {
          if(TD_Suspend())
              { 
              Sleep = FALSE;            // Clear the "go to sleep" flag.  Do it here to prevent any race condition between wakeup and the next sleep.
              do
                  {
                    EZUSB_Susp();         // Place processor in idle mode.
                  }
                while(!Rwuen && EZUSB_EXTWAKEUP());
                // Must continue to go back into suspend if the host has disabled remote wakeup
                // *and* the wakeup was caused by the external wakeup pin.
                
             // 8051 activity will resume here due to USB bus or Wakeup# pin activity.
             EZUSB_Resume();   // If source is the Wakeup# pin, signal the host to Resume.      
             TD_Resume();
              }   
          }

	
	
    	TD_Poll();

   }
}


/******************** Added by mddd       ****************/
BYTE bRequest,bmReqType; 
WORD wValue, wIndex, wLength;

#define CY_USB_TYPE_MASK               (0x60)
#define CY_USB_TARGET_MASK             (0x03)
#define CY_USB_STANDARD_RQT            (0X00)
#define CY_USB_CLASS_RQT               (0x20)
#define CY_USB_TARGET_INTF             (0x01)
#define CY_UVC_CTRL_INTERFACE          (0x00)
#define CY_UVC_STREAM_INTERFACE		   (0x01)

typedef unsigned char uint8_t ;

#define CX3_UVC_VS_PROBE_CONTROL                (0x0100)           /* Video Stream Probe Control Request */
#define CX3_UVC_VS_COMMIT_CONTROL               (0x0200)           /* Video Stream Commit Control Request */
#define CX3_UVC_VC_REQUEST_ERROR_CODE_CONTROL   (0x0200)           /* Request Control Error Code*/
#define CX3_UVC_ERROR_INVALID_CONTROL           (0x06)             /* Error indicating invalid control requested*/
#define CX3_UVC_STREAM_INTERFACE                (0x01)             /* Streaming Interface : Alternate setting 1 */


#define CX3_USB_UVC_SET_REQ_TYPE                (uint8_t)(0x21)    /* UVC interface SET request type */
#define CX3_USB_UVC_GET_REQ_TYPE                (uint8_t)(0xA1)    /* UVC Interface GET request type */
#define CX3_USB_UVC_GET_CUR_REQ                 (uint8_t)(0x81)    /* UVC GET_CUR request */
#define CX3_USB_UVC_SET_CUR_REQ                 (uint8_t)(0x01)    /* UVC SET_CUR request */
#define CX3_USB_UVC_GET_MIN_REQ                 (uint8_t)(0x82)    /* UVC GET_MIN Request */
#define CX3_USB_UVC_GET_MAX_REQ                 (uint8_t)(0x83)    /* UVC GET_MAX Request */
#define CX3_USB_UVC_GET_RES_REQ                 (uint8_t)(0x84)    /* UVC GET_RES Request */
#define CX3_USB_UVC_GET_LEN_REQ                 (uint8_t)(0x85)    /* UVC GET_LEN Request */
#define CX3_USB_UVC_GET_INFO_REQ                (uint8_t)(0x86)    /* UVC GET_INFO Request */
#define CX3_USB_UVC_GET_DEF_REQ                 (uint8_t)(0x87)    /* UVC GET_DEF Request */

xdata BYTE glProbeCtrl[34]={
		0x00, 0x00,                         /* bmHint : No fixed parameters */
		0x01,           					/* Corresponding Video format index */
		0x01					,           /* Corresponding Video frame index */
		0x15, 0x16, 0x05, 0x00,             /* Frame interval (in 100ns units): (1/30)x10^7  */
		0x00, 0x00,                         /* Key frame rate in key frame/video frame units */
		0x00, 0x00,                         /* PFrame rate in PFrame / key frame units */
		0x00, 0x00,                         /* Compression quality control */
		0x00, 0x00,                         /* Window size for average bit rate */
		0x00, 0x00,                         /* Internal video streaming i/f latency in ms */
		0x00, 0x60, 0x09, 0x00,             /* Maximum video or still frame size in bytes: 640 x 480 x 2 */
		0x8C, 0x07, 0x00, 0x00,             /* No. of bytes device can rx in single payload-1932 */
		0x00, 0x60, 0xE3, 0x16,             /* Device Clock */
		0x00,                               /* Framing Information - Ignored for uncompressed format */
		0x00,                               /* Preferred payload format version */
		0x00,                               /* Minimum payload format version */
		0x00,                               /* Maximum payload format version */

};
/**************************************************/

void HandleVideoStreamRqts()
{

      if ((wValue == CX3_UVC_VS_PROBE_CONTROL) || (wValue == CX3_UVC_VS_COMMIT_CONTROL))
      {
        switch (bRequest)
        {
        case CX3_USB_UVC_GET_INFO_REQ:
          {
		  	      EP0BUF[0] = 3;
                  EP0BCH = 0;
                  EP0BCL = 1;
          }
          break;

        case CX3_USB_UVC_GET_LEN_REQ:
          {
		  	      EP0BUF[0] = 34;
                  EP0BCH = 0;
                  EP0BCL = 1;
          }
          break;

        case CX3_USB_UVC_GET_CUR_REQ:
        case CX3_USB_UVC_GET_MIN_REQ:
        case CX3_USB_UVC_GET_MAX_REQ:
        case CX3_USB_UVC_GET_DEF_REQ:
          {
                  SUDPTRH = MSB(&glProbeCtrl[0]);
                  SUDPTRL = LSB(&glProbeCtrl[0]);
          }
          break;

        case CX3_USB_UVC_SET_CUR_REQ:
          {
			EP0BCH = 0;
			EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing

          }
          break;

        default:

          break;
        }
      }


}
/********************************************************************************/


// Device request parser
void SetupCommand(void)
{
   void   *dscr_ptr;

/******************** Added by mddd       ****************/                   
   BYTE bType, bTarget;

   bmReqType = SETUPDAT[0];
   bType = (bmReqType & CY_USB_TYPE_MASK);              
   bTarget = (bmReqType & CY_USB_TARGET_MASK);    
   bRequest = SETUPDAT[1];
   wValue = (SETUPDAT[3] << 8) | SETUPDAT[2];
   wIndex = (SETUPDAT[5] << 8) | SETUPDAT[4];
   wLength = (SETUPDAT[7] << 8) | SETUPDAT[6];

   if (bType == CY_USB_CLASS_RQT)
   {
      if (bTarget == CY_USB_TARGET_INTF)
      {
        switch(SETUPDAT[4])
        {
            case CY_UVC_STREAM_INTERFACE:
            	HandleVideoStreamRqts();
            break;

            case CY_UVC_CTRL_INTERFACE:
            break;
        }
      }

   }
/*************************************************/


  if(bType == CY_USB_STANDARD_RQT)
  {
   switch(SETUPDAT[1])
   {
      case SC_GET_DESCRIPTOR:                  // *** Get Descriptor
         if(DR_GetDescriptor())
            switch(SETUPDAT[3])         
            {
               case GD_DEVICE:            // Device
                  SUDPTRH = MSB(pDeviceDscr);
                  SUDPTRL = LSB(pDeviceDscr);
                  break;
               case GD_DEVICE_QUALIFIER:            // Device Qualifier
                  SUDPTRH = MSB(pDeviceQualDscr);
                  SUDPTRL = LSB(pDeviceQualDscr);
                  break;
               case GD_CONFIGURATION:         // Configuration
                  SUDPTRH = MSB(pConfigDscr);
                  SUDPTRL = LSB(pConfigDscr);
                  break;
               case GD_OTHER_SPEED_CONFIGURATION:  // Other Speed Configuration
                  SUDPTRH = MSB(pOtherConfigDscr);
                  SUDPTRL = LSB(pOtherConfigDscr);
                  break;
               case GD_STRING:            // String
                  if(dscr_ptr = (void *)EZUSB_GetStringDscr(SETUPDAT[2]))
                  {

                     SUDPTRH = MSB(dscr_ptr);
                     SUDPTRL = LSB(dscr_ptr);
                  }
                  else 
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
               default:            // Invalid request
                  EZUSB_STALL_EP0();      // Stall End Point 0
            }
         break;
      case SC_GET_INTERFACE:                  // *** Get Interface
         DR_GetInterface();
         break;
      case SC_SET_INTERFACE:                  // *** Set Interface
         DR_SetInterface();
         break;
      case SC_SET_CONFIGURATION:               // *** Set Configuration
         DR_SetConfiguration();
         break;
      case SC_GET_CONFIGURATION:               // *** Get Configuration
         DR_GetConfiguration();
         break;
      case SC_GET_STATUS:                  // *** Get Status
         if(DR_GetStatus())
            switch(SETUPDAT[0])
            {
               case GS_DEVICE:            // Device
                  EP0BUF[0] = ((BYTE)Rwuen << 1) | (BYTE)Selfpwr;
                  EP0BUF[1] = 0;
                  EP0BCH = 0;
                  EP0BCL = 2;
                  break;
               case GS_INTERFACE:         // Interface
                  EP0BUF[0] = 0;
                  EP0BUF[1] = 0;
                  EP0BCH = 0;
                  EP0BCL = 2;
                  break;
               case GS_ENDPOINT:         // End Point
                  EP0BUF[0] = *(BYTE xdata *) epcs(SETUPDAT[4]) & bmEPSTALL;
                  EP0BUF[1] = 0;
                  EP0BCH = 0;
                  EP0BCL = 2;
                  break;
               default:            // Invalid Command
                  EZUSB_STALL_EP0();      // Stall End Point 0
            }
         break;
      case SC_CLEAR_FEATURE:                  // *** Clear Feature
         if(DR_ClearFeature())
            switch(SETUPDAT[0])
            {
               case FT_DEVICE:            // Device
                  if(SETUPDAT[2] == 1)
                     Rwuen = FALSE;       // Disable Remote Wakeup
                  else
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
               case FT_ENDPOINT:         // End Point
                  if(SETUPDAT[2] == 0)
                  {
                     *(BYTE xdata *) epcs(SETUPDAT[4]) &= ~bmEPSTALL;
                     EZUSB_RESET_DATA_TOGGLE( SETUPDAT[4] );
                  }
                  else
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
            }
         break;
      case SC_SET_FEATURE:                  // *** Set Feature
         if(DR_SetFeature())
            switch(SETUPDAT[0])
            {
               case FT_DEVICE:            // Device
                  if(SETUPDAT[2] == 1)
                     Rwuen = TRUE;      // Enable Remote Wakeup
                  else if(SETUPDAT[2] == 2)
                     // Set Feature Test Mode.  The core handles this request.  However, it is
                     // necessary for the firmware to complete the handshake phase of the
                     // control transfer before the chip will enter test mode.  It is also
                     // necessary for FX2 to be physically disconnected (D+ and D-)
                     // from the host before it will enter test mode.
                     break;
                  else
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
               case FT_ENDPOINT:         // End Point
                  *(BYTE xdata *) epcs(SETUPDAT[4]) |= bmEPSTALL;
                  break;
            }
         break;
      default:                     // *** Invalid Command
         if(DR_VendorCmnd())
            EZUSB_STALL_EP0();            // Stall End Point 0
   }
  }
   // Acknowledge handshake phase of device request
   EP0CS |= bmHSNAK;
}

// Wake-up interrupt handler
void resume_isr(void) interrupt WKUP_VECT
{
   EZUSB_CLEAR_RSMIRQ();
}

