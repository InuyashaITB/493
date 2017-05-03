/**
 ****************************************************************************************
 *
 * @file usr_design.c
 *
 * @brief Product related design.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  USR
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "app_env.h"
#include "led.h"
#include "uart.h"
#include "lib.h"
#include "usr_design.h"
#include "gpio.h"
#include "button.h"
#include "sleep.h"
#include "adc.h"
#include "analog.h"
#include "pwm.h"


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define LED_ON_DUR_ADV_FAST        										 2
#define LED_OFF_DUR_ADV_FAST       										 (uint16_t)((GAP_ADV_FAST_INTV2*0.625)/10)
#define LED_ON_DUR_ADV_SLOW        										 2
#define LED_OFF_DUR_ADV_SLOW       										 (uint16_t)((GAP_ADV_SLOW_INTV*0.625)/10)
#define LED_ON_DUR_CON          											 0xffff
#define LED_OFF_DUR_CON                   						 0
#define LED_ON_DUR_IDLE                   						 0
#define LED_OFF_DUR_IDLE                  						 0xffff

#define APP_HEART_RATE_MEASUREMENT_TO     						 1400 // 14s
#define APP_HRPS_ENERGY_EXPENDED_STEP     						 50

#define EVENT_BUTTON1_PRESS_ID            						 0

///IOS Connection Parameter
#define IOS_CONN_INTV_MAX                              0x0010
#define IOS_CONN_INTV_MIN                              0x0008
#define IOS_SLAVE_LATENCY                              0x0000
#define IOS_STO_MULT                                   0x012c

#define CAR_VOLTAGE_TRIG															 1350
#define AMBIENT_LIGHT																	 500
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct 	usr_env_tag usr_env 	= {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE};
struct 	bd_addr cur_peer_addr, null_addr;
bool		bond 					=	false;

volatile uint32_t adc_done = 0;
volatile uint32_t int_num = 0;
int16_t buf[10];
int16_t amb[10];

adc_read_configuration read_cfg;

bool		activity 			= false;
bool		start_proc		= false;
bool		DRL						= true;
bool		test_led			= true;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void led_on(){
	led_set(1, LED_ON);
	led_set(2, LED_ON);	
	led_set(3, LED_ON);
	led_set(4, LED_ON);
	led_set(5, LED_ON);
	led_set(6, LED_ON);
}

void led_off(){
	led_set(1, LED_OFF);
	led_set(2, LED_OFF);
	led_set(3, LED_OFF);
	led_set(4, LED_OFF);
	led_set(5, LED_OFF);
	led_set(6, LED_OFF);
}

void led_alt(){
	led_toggle(1);
	led_toggle(2);
	led_toggle(3);
	led_toggle(4);
	led_toggle(5);
	led_toggle(6);
}

static void cur_peer_addr_init()
{
	for(int i = 0; i < BD_ADDR_LEN; i++)
		{
			cur_peer_addr.addr[i] = 0;
			null_addr.addr[i] = 0;
		}
}

static void adc_test_cb(void)
{

	adc_done = 1;
}

/**
 ****************************************************************************************
 * @brief   Led1 for BLE status
 ****************************************************************************************
 */
static void usr_led1_set(uint16_t timer_on, uint16_t timer_off)
{
    usr_env.led1_on_dur = timer_on;
    usr_env.led1_off_dur = timer_off;

    if (timer_on == 0 || timer_off == 0)
    {
        if (timer_on == 0)
        {
            led_set(1, LED_OFF);
        }
        if (timer_off == 0)
        {
            led_set(1, LED_ON);
        }
        ke_timer_clear(APP_SYS_LED_1_TIMER, TASK_APP);
    }
    else
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, timer_off);
    }
}

/**
 ****************************************************************************************
 * @brief   Led 1 flash process
 ****************************************************************************************
 */
static void usr_led1_process(void)
{
    if(led_get(1) == LED_ON)
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_off_dur);
    }
    else
    {
        led_set(1, LED_ON);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_on_dur);
    }
}

/**
 ****************************************************************************************
 * @brief   Application task message handler
 ****************************************************************************************
 */
void app_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid)
    {
        case GAP_SET_MODE_REQ_CMP_EVT:
            if(APP_IDLE == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_FAST, LED_OFF_DUR_ADV_FAST);
                ke_timer_set(APP_ADV_INTV_UPDATE_TIMER, TASK_APP, 30 * 100);
            }
            else if(APP_ADV == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_SLOW, LED_OFF_DUR_ADV_SLOW);
            }
            break;

        case GAP_ADV_REQ_CMP_EVT:
            usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);
            ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
            break;

        case GAP_DISCON_CMP_EVT:
            usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);

            // start adv
            app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                    app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                    app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                    GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
            break;

        case GAP_LE_CREATE_CONN_REQ_CMP_EVT:
            if(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.status == CO_ERROR_NO_ERROR)
            {
                if(GAP_PERIPHERAL_SLV == app_get_role())
                {
                    ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
                    usr_led1_set(LED_ON_DUR_CON, LED_OFF_DUR_CON);

                    // Update cnx parameters
                    //if (((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.con_interval >  IOS_CONN_INTV_MAX)
                    {
                        // Update connection parameters here
                        struct gap_conn_param_update conn_par;
                        /// Connection interval minimum
                        conn_par.intv_min = IOS_CONN_INTV_MIN;
                        /// Connection interval maximum
                        conn_par.intv_max = IOS_CONN_INTV_MAX;
                        /// Latency
                        conn_par.latency = IOS_SLAVE_LATENCY;
                        /// Supervision timeout, Time = N * 10 msec
                        conn_par.time_out = IOS_STO_MULT;
                        app_gap_param_update_req(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.conhdl, &conn_par);
                    }
                }
								
								//Get peer BT address
								struct bd_addr peer_addr = ((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.peer_addr;
								
								//If this is our first connection to the device, bond with it
								if(!bond && cur_peer_addr.addr != null_addr.addr)
								{
									app_gap_bond_req(&peer_addr, SMP_OOB_AUTH_DATA_NOT_PRESENT, SMP_AUTH_REQ_NO_MITM_BOND, SMP_IO_CAP_NO_INPUT_NO_OUTPUT);
									setCurrentAddress(peer_addr);
								}
            }
            break;
        case QPPS_DISABLE_IND:
            break;

        case QPPS_CFG_INDNTF_IND:
            break;

        default:
            break;
    }
}

void setCurrentAddress(struct bd_addr ad)
{
	cur_peer_addr = ad;
}

bool compareAddress(uint8_t A[], uint8_t B[])
{
	int i;
	for(i = 0; i < BD_ADDR_LEN; i++)
	{
		if(A[i] != B[i]) return false;
	}
	return true;
}

//Toggle the security measurement of BT Address Check
void bond_check_set(bool val)
{
	bond = val;
}

/**
 ****************************************************************************************
 * @brief Handles LED status timer.
 *
 * @param[in] msgid      APP_SYS_UART_DATA_IND
 * @param[in] param      Pointer to struct app_uart_data_ind
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_led_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(msgid == APP_SYS_LED_1_TIMER)
    {
        usr_led1_process();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles advertising mode timer event.
 *
 * @param[in] msgid     APP_ADV_INTV_UPDATE_TIMER
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that first phase of adversting mode is timeout.
 ****************************************************************************************
 */
int app_gap_adv_intv_update_timer_handler(ke_msg_id_t const msgid, void const *param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(APP_ADV == ke_state_get(TASK_APP))
    {
        usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);

        // Update Advertising Parameters
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE, 
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE), 
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_SLOW_INTV, GAP_ADV_SLOW_INTV);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   Restore peripheral setting after wakeup
 ****************************************************************************************
 */
void usr_sleep_restore(void)
{
#if QN_DBG_PRINT
    uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_9600);
    uart_tx_enable(QN_DEBUG_UART, MASK_ENABLE);
    uart_rx_enable(QN_DEBUG_UART, MASK_ENABLE);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles button press after cancel the jitter.
 *
 * @param[in] msgid     Id of the message received
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_button_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_SYS_BUTTON_1_TIMER:
            // make sure the button is pressed
            if(gpio_read_pin(BUTTON1_PIN) == GPIO_LOW)
            {
                if(APP_IDLE == ke_state_get(TASK_APP))
                {
                    if(!app_qpps_env->enabled)
                    {
                        // start adv
                        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);

#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
                    }
                }
                else if(APP_ADV == ke_state_get(TASK_APP))
                {
                    // stop adv
                    app_gap_adv_stop_req();

#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
                }
            }
            break;

        default:
            ASSERT_ERR(0);
            break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_button1_press_handler(void)
{
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep)
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif

    // delay 20ms to debounce
    ke_timer_set(APP_SYS_BUTTON_1_TIMER, TASK_APP, 2);
    ke_evt_clear(1UL << EVENT_BUTTON1_PRESS_ID);
}

/**
 ****************************************************************************************
 * @brief   Button 1 click callback
 * @description
 *  Button 1 is used to enter adv mode.
 ****************************************************************************************
 */

void usr_button1_cb(void)
{
		start_procedure();
	
	
    /*// If BLE is in the sleep mode, wakeup it.
    if(ble_ext_wakeup_allow())
    {
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
        if (sleep_env.deep_sleep)
        {
            wakeup_32k_xtal_switch_clk();
        }
#endif

        sw_wakeup_ble_hw();

    }
		

    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_BUTTON1_PRESS_ID);*/
}


void test_relays(){
		gpio_set_direction_field(RELAY_PINS, (uint32_t)GPIO_OUTPUT);
		gpio_write_pin_field(RELAY_PINS, GPIO_LOW);
}
enum states {one, two, three, four };
enum states state = one;
void start_procedure_2(){
	adc_done = 0;
	adc_read(&read_cfg, buf, 10, adc_test_cb);
	while(adc_done == 0);
	
	//Initialize timout procedure
	timer_init(QN_TIMER1, timeout_procedure);
	timer_config(QN_TIMER1, 3, TIMER_COUNT_S(1, 3));
	timer_enable(QN_TIMER1, MASK_ENABLE);
	timer_delay(QN_TIMER1, 3, TIMER_COUNT_S(900, 3));
	
	if(state == one){
		//Initialization state - We want to do the first set of commands.
		// Make sure we are connected to correct phone
		if(app_find_bonded_dev(&cur_peer_addr) != 0xff){
				//Check to make sure clutch / parking gear is enabled && car voltage is under 13V
			if(GPIO_HIGH == gpio_read_pin(CLUTCH_PIN) && car_voltage() < CAR_VOLTAGE_TRIG)
			{
				//Start car procedure
				gpio_write_pin(RELAY2_PIN, GPIO_LOW); // Disable Accessory Power
				gpio_write_pin(RELAY6_PIN, GPIO_LOW); // Disable Headlamp Power
				
				gpio_write_pin(RELAY3_PIN, GPIO_HIGH); // Enable Run Power Relay
				
				delay(50000); // Wait .25 seconds
				gpio_write_pin(RELAY4_PIN, GPIO_HIGH); // Enable Solenoid relay
			}
		}
		state = two;
		
		//Now move to next state
		start_procedure_2();
	} else if(state == two) {
		delay(1000000); // Delay for some time to allow car to start
		gpio_write_pin(RELAY4_PIN, GPIO_LOW); // Turn off starter solenoid
		ambient_procedure(); // Decide what to do with the headlamps
		gpio_write_pin(RELAY2_PIN, GPIO_HIGH); // Turn accessories back on
		state = three;
		//Insert Delay Here
		
		//Now move to next state
		start_procedure_2();
	} else if(state == three) {
		
		state = four;
		//Insert Delay Here
		
		//Now move to next state
		start_procedure_2();
	} else if(state == four) {
		
		state = one;
	} else {
			state = one; // Undefined state, initialize to One
	}
	
}
/**
 ****************************************************************************************
 * @brief   Car Start Command Callback
 ****************************************************************************************
 */
void start_procedure()
{
	adc_done = 0;
	adc_read(&read_cfg, buf, 10, adc_test_cb);
  while (adc_done == 0); // Wait for the car bus to be read before proceeding
	
	//Check if we are in the starting phase
	if(start_proc == true)
	{
		gpio_write_pin(RELAY4_PIN, GPIO_LOW); // Turn off Starter relay
		gpio_write_pin(RELAY2_PIN, GPIO_HIGH); // Turn on accessories
		ambient_procedure(); // Follow the ambient procedure for headlamps
		
		gpio_set_interrupt(START_PIN, GPIO_INT_FALLING_EDGE);
		return; // No need to continue with rest of the start_procedure
	}
	
	//Start out with an inital check to make sure that we are connected to the right device
	if(app_find_bonded_dev(&cur_peer_addr) != 0xff)
	{			
		
			//Check to make sure clutch / parking gear is enabled && car voltage is under 13V
			if(GPIO_HIGH == gpio_read_pin(CLUTCH_PIN) && car_voltage() < CAR_VOLTAGE_TRIG)
			{
				//Start car procedure
				gpio_write_pin(RELAY2_PIN, GPIO_LOW); // Disable Accessory Power
				gpio_write_pin(RELAY6_PIN, GPIO_LOW); // Disable Headlamp Power
				
				gpio_write_pin(RELAY3_PIN, GPIO_HIGH); // Enable Run Power Relay
				
				delay(25000); // Wait .25 seconds
				gpio_write_pin(RELAY4_PIN, GPIO_HIGH); // Enable Solenoid relay
				
				gpio_set_interrupt(START_PIN, GPIO_INT_RISING_EDGE);
				start_proc = true;
			}
			
			//If car voltage is greater than 13V and start button is pressed, shut off the car
			else if(car_voltage() > CAR_VOLTAGE_TRIG)
			{
				gpio_write_pin(RELAY3_PIN, GPIO_LOW);
				ambient_procedure();
				
				activity = false;
				timer_init(QN_TIMER1, timeout_procedure);
				timer_delay(QN_TIMER1, 3, TIMER_COUNT_US(15000000, 3));
				timer_enable(QN_TIMER1, MASK_ENABLE);
			}
			
			//if clutch not active, turn on / off accessories
			else if(GPIO_LOW == gpio_read_pin(CLUTCH_PIN) && car_voltage() < CAR_VOLTAGE_TRIG)
				{				
					if(GPIO_LOW == gpio_read_pin(RELAY1_PIN))	
					{
								gpio_write_pin(RELAY1_PIN, GPIO_HIGH); //If accessories are off, turn on
								//Start timeout timer
								timer_init(QN_TIMER0, timeout_procedure);
								timer_delay(QN_TIMER0, 3, TIMER_COUNT_US(900000000, 3));
								timer_enable(QN_TIMER0, MASK_ENABLE);
								activity = false;
					}
					else 	gpio_write_pin(RELAY1_PIN, GPIO_LOW);  //If accessories are on, turn off
			}
	}
}

//car_voltage() returns an integer value of the averaged ADC input reading of the Bus Monitor
int16_t car_voltage()
{
	int16_t temp = 0;
	for(int i = 0; i < 10; i++)
	{
			temp = temp + buf[i];
	}
	return temp / 10;	
}

void timeout_procedure()
{
	if(activity == false) 
		{
			gpio_write_pin(RELAY1_PIN, GPIO_LOW); // Turn off accessories
			timer_enable(QN_TIMER0, MASK_DISABLE); // Disable timers associated with this function
			timer_enable(QN_TIMER1, MASK_DISABLE);
		}
}

/**
 ****************************************************************************************
 * @brief   Ambient Enabled Command Callback
 ****************************************************************************************
 */

void ambient_procedure()
{
	
	if(app_find_bonded_dev(&cur_peer_addr) != 0xff)
	{
		if(!DRL)
		{
			read_cfg.start_ch = AIN1;
			read_cfg.end_ch = AIN1;
			
			adc_done = 0;
			adc_read(&read_cfg, amb, 10, adc_test_cb);
			while (adc_done == 0);
			
			if(ambient_light() < AMBIENT_LIGHT)
				//If Ambient is low, lights should turn on
				gpio_write_pin(RELAY6_PIN, GPIO_HIGH);
			else
				//If Ambient is high, lights should be off
				gpio_write_pin(RELAY6_PIN, GPIO_LOW);
		}
		else
		{
			read_cfg.start_ch = AIN0;
			read_cfg.end_ch = AIN0;
			adc_done = 0;
			adc_read(&read_cfg, buf, 10, adc_test_cb);
			while (adc_done == 0);
			
			if(car_voltage() > CAR_VOLTAGE_TRIG)
				gpio_write_pin(RELAY6_PIN, GPIO_HIGH);
			else
				gpio_write_pin(RELAY6_PIN, GPIO_LOW);
		}
	}
}

int16_t ambient_light()
{
	int16_t temp = 0;
	for(int i = 0; i < 10; i++)
		temp = temp + amb[i];
	
	return temp / 10;
}

/**
 ****************************************************************************************
 * @brief   Clutch/Park Enabled Command Callback (Start Button Illumation Control)
 ****************************************************************************************
 */

void clutch_procedure()
{
	if(app_find_bonded_dev(&cur_peer_addr) != 0xff)
	{
		activity = true;
		//Is the clutch/Parking gear enabled 
		if(GPIO_HIGH == gpio_read_pin(CLUTCH_PIN) || car_voltage() > CAR_VOLTAGE_TRIG)
			gpio_write_pin(RELAY0_PIN, GPIO_HIGH); // LED Illuminator
		else if(GPIO_LOW == gpio_read_pin(CLUTCH_PIN) && car_voltage() < CAR_VOLTAGE_TRIG)
			gpio_write_pin(RELAY0_PIN, GPIO_LOW);
	}
}

/**
 ****************************************************************************************
 * @brief   All GPIO interrupt callback
 ****************************************************************************************
 */
void gpio_interrupt_callback(enum gpio_pin pin)
{
    switch(pin)
    {
        case BUTTON1_PIN:
            //Button 1 is used to enter adv mode.
            //usr_button1_cb();
						start_procedure_2();
            break;

#if (defined(QN_TEST_CTRL_PIN))
        case QN_TEST_CTRL_PIN:
            //When test controll pin is changed to low level, this function will reboot system.
            gpio_disable_interrupt(QN_TEST_CTRL_PIN);
            syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_REBOOT_SYS);
            break;
#endif
				case BUTTON2_PIN:
						//Button 2 interupt test (Toggle LED 1)
						adc_done = 0;
						adc_read(&read_cfg, buf, 10, adc_test_cb);
						while (adc_done == 0);
				
						if(led_get(1) == LED_ON)
						{
							led_set(1, LED_OFF);
						}
						else
						{
							led_set(1, LED_ON);
						}
						break;
						
				case START_PIN:
					read_cfg.start_ch = AIN0;
					read_cfg.end_ch = AIN0;
					start_procedure();
					break;

				case AMBIENT_PIN:
					ambient_procedure();
					break;
				
				case CLUTCH_PIN:
					clutch_procedure();
					break;
				
        default:
            break;
    }
}


/**
 ****************************************************************************************
 * @brief   User initialize
 ****************************************************************************************
 */

void usr_init(void)
{
		cur_peer_addr_init();
		adc_pin_enable(AIN0, MASK_ENABLE);
		adc_pin_enable(AIN1, MASK_ENABLE);
	
	    // ADC initialization    
#if ADC_WORKING_AT_32K==TRUE
    clk32k_enable(__32K_TYPE);
    adc_init(ADC_SINGLE_WITHOUT_BUF_DRV, ADC_CLK32K_16000, ADC_INT_REF, ADC_12BIT);
#else
    adc_init(ADC_SINGLE_WITHOUT_BUF_DRV, ADC_CLK_1000000, ADC_INT_REF, ADC_12BIT);
#endif
	
	// Read configuration
    
	
		read_cfg.trig_src = ADC_TRIG_SOFT;
    
    adc_done = 0;
    
    // modify here
    read_cfg.mode = CONTINUE_MOD;
    read_cfg.start_ch = AIN0;
    read_cfg.end_ch = AIN0;
	
    adc_read(&read_cfg, buf, 10, adc_test_cb);
    while (adc_done == 0);

    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_BUTTON1_PRESS_ID, 
                                            app_event_button1_press_handler))
    {
        ASSERT_ERR(0);
    }
}

/// @} USR

