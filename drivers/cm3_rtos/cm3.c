#include "cm3.h"
#include "canfestival.h"
#include "timer.h"
#include "can.h"

volatile TIMEVAL last_counter_val = 0;
volatile TIMEVAL elapsed_time = 0;

static CO_Data *co_data = NULL;

static QueueHandle_t canrxq = 0;
static QueueHandle_t cantxq = 0;

/*********************************************************************
 * Canfestival 
 * Clear the timer
 *********************************************************************/
void clearTimer(void)
{
	timer_disable_counter(TIM3);;
	nvic_disable_irq(NVIC_TIM3_IRQ);
	rcc_periph_clock_disable(RCC_TIM3);
}


void start_callback(CO_Data* d, UNS32 id)
{

}

/*********************************************************************
 * Canfestival Timer setup
 *********************************************************************/
void initTimer(void) {

	rcc_periph_clock_enable(RCC_TIM3);		// Need TIM3 clock

	// TIM3:
	timer_disable_counter(TIM3);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	rcc_periph_reset_pulse(RST_TIM3);
	
	timer_set_mode(TIM3,
		TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE,
		TIM_CR1_DIR_UP);

	//TIM_CR1_URS
	
	timer_set_prescaler(TIM3, 720 -1);
	/* Disable preload. */
	timer_disable_preload(TIM3);
	timer_continuous_mode(TIM3);
	/* count full range */
	timer_set_period(TIM3, 65535 -1);
	/* Enable Update interrupt */
	timer_enable_irq(TIM3, TIM_DIER_UIE);

	/* Counter enable. */
	timer_enable_counter(TIM3);

	// this is needed for correct canfestival virtual timer management start
	SetAlarm(NULL, 0, start_callback, 0, 0);
}

/*********************************************************************
 * Canfestival 
 * Set the timer for the next alarm.
 *********************************************************************/
void setTimer(TIMEVAL value)
{
	rcc_periph_clock_enable(RCC_TIM3);				// Need TIM3 clock
	timer_disable_counter(TIM3);
  	uint32_t timer = timer_get_counter(TIM3);        // Copy the value of the running timer
	elapsed_time += timer - last_counter_val;
	last_counter_val = 65535 - value;
	timer_set_counter(TIM3, last_counter_val);
	timer_enable_counter(TIM3);
}

/*********************************************************************
 * Canfestival 
 * Return the elapsed time to tell the Stack how much time is spent since last call.
 *********************************************************************/
TIMEVAL getElapsedTime(void)
{
  	uint32_t timer = timer_get_counter(TIM3);        // Copy the value of the running timer
	if(timer < last_counter_val)
		timer += 65535;
	TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
	//printf("elapsed %lu - %lu %lu %lu\r\n", elapsed, timer, last_counter_val, elapsed_time);
	return elapsed;
}

/*********************************************************************
 * Canfestival 
 * This function handles Timer 3 interrupt request.
 *********************************************************************/
void
tim3_isr(void) {
	if (timer_get_flag(TIM3, TIM_SR_UIF)) {

		last_counter_val = 0;
		elapsed_time = 0;
		/* Clear interrupt flag. */
		timer_clear_flag(TIM3, TIM_SR_UIF);

		TimeDispatch();
	}
}

/* prescaler values for 87.5%  sampling point (88.9% at 1Mbps)
   if unknown bitrate default to 50k
*/
// uint16_t brp_from_birate(uint32_t bitrate)
// {
// 	if(bitrate == 10000)
// 		return 225;
// 	if(bitrate == 50000)
// 		return 45;
// 	if(bitrate == 125000)
// 		return 18;
// 	if(bitrate == 250000)
// 		return 9;
// 	if(bitrate == 500000)
// 		return 9;
// 	if(bitrate == 1000000)
// 		return 4;
// 	return 45;
// }

/*********************************************************************
 * Submit each CAN message for transmission handled by libopencm3
 *********************************************************************/
static void
can_tx_task(void *arg __attribute((unused))) {
	struct s_canmsg cmsg;

	for (;;) {
		if ( xQueueReceive(cantxq, &cmsg, portMAX_DELAY) == pdPASS ) {
			while ( can_transmit(CAN1, cmsg.msgid, cmsg.xmsgidf, cmsg.rtrf, cmsg.length, (uint8_t*)&cmsg.data) == -1 )
				taskYIELD();
		}
	}
}

/*********************************************************************
 * Issue canDispatch() for each CAN message received
 *********************************************************************/
static void
can_rx_task(void *arg __attribute((unused))) {
	struct s_canmsg cmsg;

	for (;;) {
		if ( xQueueReceive(canrxq, &cmsg, portMAX_DELAY) == pdPASS ) {
			Message rxm = {0};
			rxm.cob_id = cmsg.msgid;
			rxm.rtr = cmsg.rtrf;
			rxm.len = cmsg.length;
			for (int i=0 ; i<rxm.len ; i++) {
				rxm.data[i] = cmsg.data[i];
			}
			canDispatch(co_data, &rxm);
		}
	}
}

/*********************************************************************
 * Main CAN RX ISR routine for FIFO x
 *********************************************************************/
static void
can_rx_isr(uint8_t fifo,unsigned msgcount) {
        struct s_canmsg cmsg;
        bool xmsgidf, rtrf;

        while ( msgcount-- > 0 ) {
                can_receive(
                        CAN1,
                        fifo,                   // FIFO # 
                        true,                   // Release      
                        &cmsg.msgid,
                        &xmsgidf,               // true if msgid is extended
                        &rtrf,                  // true if requested transmission
                        (uint8_t *)&cmsg.fmi,   // Matched filter index
                        &cmsg.length,           // Returned length
                        cmsg.data,
                        NULL);					// Unused timestamp
                cmsg.xmsgidf = xmsgidf;
                cmsg.rtrf = rtrf;
                cmsg.fifo = fifo;
                // If the queue is full, the message is lost
                xQueueSendToBackFromISR(canrxq, &cmsg, NULL);
        }
}

/*********************************************************************
 * CAN FIFO 0 ISR
 *********************************************************************/
void
usb_lp_can_rx0_isr(void) {
    can_rx_isr(0, CAN_RF0R(CAN1)&3);
}

/*********************************************************************
 * CAN FIFO 1 ISR
 *********************************************************************/
void
can_rx1_isr(void) {
    can_rx_isr(1, CAN_RF1R(CAN1)&3);
}


/*********************************************************************
 * Initialize for CAN I/O
 *********************************************************************/
unsigned char canInit(CO_Data * d, uint32_t bitrate)
{
	bool nart = false;
	bool locked = true;

	/* save the canfestival handle */  
	co_data = d;

	rcc_periph_clock_enable(RCC_AFIO);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_CAN_PB_TX);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_CAN_PB_RX);

	gpio_primary_remap(                         // Map CAN1 to use PB8/PB9
		AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,      // Optional
		AFIO_MAPR_CAN1_REMAP_PORTB);            // CAN_RX=PB8, CAN_TX=PB9

	can_reset(CAN1);
    can_init(
		CAN1,
		false,		// ttcm=off
		false,		// auto bus off management
		true,		// Automatic wakeup mode.
		nart,		// No automatic retransmission.
		locked,		// Receive FIFO locked mode
		false,		// Transmit FIFO priority (msg id)
		PARM_SJW,	// Resynchronization time quanta jump width (0..3)
		PARM_TS1,	// segment 1 time quanta width
		PARM_TS2,	// Time segment 2 time quanta width
		PARM_BRP,	// Baud rate prescaler for 33.333 kbs
		false,		// Loopback
		false);		// Silent

	can_filter_id_mask_16bit_init(
		0,							// Filter bank 0
		0x000 << 5, 0x001 << 5,		// LSB == 0
		0x000 << 5, 0x001 << 5,		// Not used
		0,							// FIFO 0
		true);

	can_filter_id_mask_16bit_init(
		1,							// Filter bank 1
		0x010 << 5, 0x001 << 5,		// LSB == 1 (no match)
		0x001 << 5, 0x001 << 5,		// Match when odd
		1,							// FIFO 1
		true);

	canrxq = xQueueCreate(33, sizeof(struct s_canmsg));
	cantxq = xQueueCreate(33, sizeof(struct s_canmsg));

	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_enable_irq(NVIC_CAN_RX1_IRQ);
	can_enable_irq(CAN1, CAN_IER_FMPIE0|CAN_IER_FMPIE1);

	xTaskCreate(can_rx_task, "canrx", 400, NULL, configMAX_PRIORITIES-1, NULL);

	xTaskCreate(can_tx_task, "cantx", 400, NULL, configMAX_PRIORITIES-1, NULL);

  	return 1;
}

void canClose(void)
{
  	can_reset(CAN1);

	TaskHandle_t taskRxHandle = xTaskGetHandle("canrx");
	if (taskRxHandle) {
		vTaskDelete(taskRxHandle);
	}
	nvic_disable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_disable_irq(NVIC_CAN_RX1_IRQ);
	can_disable_irq(CAN1, CAN_IER_FMPIE0|CAN_IER_FMPIE1);

	vQueueDelete(canrxq);

	TaskHandle_t taskTxHandle = xTaskGetHandle("cantx");
	if (taskTxHandle) {
		vTaskDelete(taskTxHandle);
	}

	vQueueDelete(cantxq);

	rcc_periph_clock_disable(RCC_GPIOB);
    rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
	rcc_periph_clock_disable(RCC_AFIO);
}

// The driver send a CAN message passed from the CANopen stack
unsigned char canSend(CAN_PORT notused, Message *m)
{
	struct s_canmsg cmsg;

	cmsg.msgid = m->cob_id;
	cmsg.rtrf = m->rtr;
	cmsg.length = m->len;
	for (int i=0 ; i< m->len; i++) {
		cmsg.data[i] = m->data[i];
	}

    xQueueSendToBack(cantxq, &cmsg, (TickType_t) 0);

	return 1;
}
