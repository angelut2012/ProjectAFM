#include "USBSerial.h"

USBSerial mUSerial;// global

extern DigitalOut *p_LED;//(PORT_LED);

USBSerial::USBSerial(void)
{
	
	//p_LED = new DigitalOut(PORT_LED);
}


USBSerial::~USBSerial(void)	
{
	//delete p_LED;
}

void USBSerial::begin()
{   


	/* Init system */
	TM_RCC_InitSystem();



	/* Init HAL layer */
	HAL_Init();

	/* Init leds */
	//TM_DISCO_LedInit();

	/* Init delay */
	//TM_DELAY_Init();

	/* Init USB peripheral */
	TM_USB_Init();

	/* Init VCP on FS and HS ports.. */
	TM_USBD_CDC_Init(TM_USB_FS);
	//TM_USBD_CDC_Init(TM_USB_HS);

	/* ..or use single call for both modes */
	//TM_USBD_CDC_Init(TM_USB_Both);

	/* Start USB device mode on FS and HS ports.. */
	TM_USBD_Start(TM_USB_FS);
	//TM_USBD_Start(TM_USB_HS);

	/* .. or use single call for both modes */
	//TM_USBD_Start(TM_USB_Both);

}


void  USBSerial::process()
	// time consumption=(1/0.536-1/1.9)/2=0.67 us
	/* Process USB CDC device, send remaining data if needed */
	/* It is better if you call this in periodic timer, like each ms in SYSTICK handler */

{

	TM_USBD_CDC_Process(TM_USB_FS);	
#define __AFM_USB_DEBUG__
#ifdef __AFM_USB_DEBUG__	// commented by jun, 20160321
	if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok && (mCountProcess++ > 40000)) 		
		
		p_LED->write(1);

	else
		
		p_LED->write(0);
#endif // __AFM_DEBUG__
	
//	/* Check if device is ready, if drivers are installed if needed on FS port */
//
//
//
//	/* Check if user has changed parameter for COM port */
//	TM_USBD_CDC_GetSettings(TM_USB_FS, &USB_FS_Settings);
//	
//	/* Check if updated */
//	if (USB_FS_Settings.Updated) 
//	{
//		/* Update settings for UART here if needed */
//		write("USB setting updated.\n");
//	}

}

size_t USBSerial::readBytes( uint8_t *buffer, size_t length)
{
	return TM_USBD_CDC_GetArray(TM_USB_FS,buffer,length);
}

size_t USBSerial::write(uint8_t  num)
{
	return write((char)num);
}

size_t USBSerial::write( char ch)	
{
	return TM_USBD_CDC_Putc(TM_USB_FS,ch);
}

size_t USBSerial::write(const char *buffer) 
{
	return TM_USBD_CDC_Puts(TM_USB_FS, buffer);	
}
size_t USBSerial::write(const uint8_t *buffer, size_t length)
{
	return TM_USBD_CDC_PutArray(TM_USB_FS, (uint8_t *)buffer, length);
}


