#ifndef _APPLY_H
#define _APPLY_H
#include <sys.h>	  
#include "screen.h"
void SW_CMDHandler(void);
void TaskProcessing(void);
void PowerStateUpdate(void);
//void UART1_CMDHandler(u8 *DATA);
void UART1_CMDHandler(PCTRL_MSG msg);
void BATCheckDIS(void);
void SystemPowerDown(void);
	
#endif

