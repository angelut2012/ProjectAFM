#include "main_AFM.h"




extern USBSerial mUSerial;// global

//extern DigitalOut p_LED(PA_8);
DigitalOut p_Power_SEM(D_PS_SEM);
DigitalOut p_Power_HV(D_PS_HV);



int  main_AFM(void) 
{	
//}
//int tets()
//{

#ifdef __AFM_test
	extern AFM_test mAFM_test;
	//mAFM_test.test_blink_all_IOs();
//	mAFM_test.test_blink_LED();
	//mAFM_test.test_SEM_ADC_read();
	mAFM_test.test_HV_DAC_write();	
	
//	setup();
//	while (1)
//		loop();
#else

	extern AFM_Core mAFM_Core;
	mAFM_Core.AFM_main_setup();


	while (1)
		mAFM_Core.AFM_main_loop();

#endif	
	return 0;
}
//	/* USB CDC settings */
//	TM_USBD_CDC_Settings_t USB_FS_Settings;
//	char string_array[30];
//	
//
//    /* Init system */
//	TM_RCC_InitSystem();
//    
//	/* Init HAL layer */
//	HAL_Init();
//    
//	/* Init leds */
//	//TM_DISCO_LedInit();
//    
//	/* Init delay */
//	TM_DELAY_Init();
//    
//	/* Init USB peripheral */
//	TM_USB_Init();
//    
//	/* Init VCP on FS and HS ports.. */
//	TM_USBD_CDC_Init(TM_USB_FS);
//	//TM_USBD_CDC_Init(TM_USB_HS);
//    
//	/* ..or use single call for both modes */
//	//TM_USBD_CDC_Init(TM_USB_Both);
//    
//	/* Start USB device mode on FS and HS ports.. */
//	TM_USBD_Start(TM_USB_FS);
//	//TM_USBD_Start(TM_USB_HS);
//    
//	/* .. or use single call for both modes */
//	//TM_USBD_Start(TM_USB_Both);
//	DigitalOut myled(PA_8);   
//
//	while (1) {
//        
//      
//        
//	    /* Process USB CDC device, send remaining data if needed */
//	    /* It is better if you call this in periodic timer, like each ms in SYSTICK handler */
//		TM_USBD_CDC_Process(TM_USB_FS);
//        
//		/* Check if device is ready, if drivers are installed if needed on FS port */
//		if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) {
//			//TM_DISCO_LedOn(LED_GREEN);
//			myled = 1; // LED is ON
//		}
//		else {
//			myled = 0; // LED is ON//TM_DISCO_LedOff(LED_GREEN);
//		}
//        
//		/* Check if user has changed parameter for COM port */
//		TM_USBD_CDC_GetSettings(TM_USB_FS, &USB_FS_Settings);
//        
//		/* Check if updated */
//		if (USB_FS_Settings.Updated) {
//		    /* Update settings for UART here if needed */
//			TM_USBD_CDC_Puts(TM_USB_FS, "USB  test  full speed!\n");
//		}
//        
//
//		
//		/* Check if anything received on FS port */
//		if (TM_USBD_CDC_Getc(TM_USB_FS, string_array)) {
//			//,sizeof(string_array)
//		                   
//		    /* One character received */
//			string_array[0]++;
//			 /* Send it back */
//			TM_USBD_CDC_Putc(TM_USB_FS, string_array[0]);
//		}
//        
//	}	
//	
//	return 0;
//}
//
//

