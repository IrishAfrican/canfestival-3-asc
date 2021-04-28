  
#ifndef __CM3_H
#define __CM3_H

#ifndef STM32F1
#define STM32F1
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/timer.h>

#include "rtos/FreeRTOS.h"
#include "rtos/task.h"
#include "rtos/queue.h"

#define PARM_SJW	CAN_BTR_SJW_1TQ
#define PARM_TS1	CAN_BTR_TS1_6TQ
#define PARM_TS2	CAN_BTR_TS2_7TQ
#define PARM_BRP	78		// 33.333 kbps

/*********************************************************************
 * @brief The libopencm3 CAN message structure 
 * @ingroup can
 *********************************************************************/
struct s_canmsg {
	uint32_t	msgid;		    // Message ID
	uint32_t	fmi;		    // Filter index
	uint8_t		length;		    // Data length
	uint8_t		data[8];	    // Received data
	uint8_t		xmsgidf : 1;    // Extended message flag
	uint8_t		rtrf : 1;	    // RTR flag
	uint8_t		fifo : 1;	    // RX Fifo 0 or 1
};

// void initialize_can(bool nart, bool locked);
// void can_recv(struct s_canmsg *msg);
// void can_xmit(uint32_t id, bool ext, bool rtr, uint8_t length, void *data);


#endif 

