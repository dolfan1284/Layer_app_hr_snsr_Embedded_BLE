/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      pos_sensor_service.c
 *
 *  DESCRIPTION
 *      This file defines routines for using Position Sensor service.
 *
 ****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *===========================================================================*/

#include <gatt.h>
#include <gatt_prim.h>
#include <buf_utils.h>
#include <thermometer.h>    /* System Thermometer*/
#include <timer.h>
#include <ls_app_if.h>
/*============================================================================*
 *  Local Header Files
 *===========================================================================*/

#include "app_gatt.h"
#include "pos_sensor_service.h"
#include "hr_sensor.h"
#include "nvm_access.h"
#include "app_gatt_db.h"

/*============================================================================*
 *  Private Data Types
 *===========================================================================*/

/* Array variables for tracking a window of potentiometer values
//  We want to hold on to the last NUM_SAMPLES values and perform
//  an average on them to arrive at a 'current' value - this 
//  is called a moving average: http://en.wikipedia.org/wiki/Moving_average
//  NOTE: The larger NUM_SAMPLES, the smoother the values, but also
//  the longer it will take the average to move in response to you turning the
//  potentiometer
   */
#define NUM_SAMPLES 20
uint16 potSamplesLeft[NUM_SAMPLES];
uint16 potSamplesRight[NUM_SAMPLES];
uint8 potIndexLeft = 0;
uint16 totalPotValueLeft = 0;
uint8 potIndexRight = 0;
uint16 totalPotValueRight = 0;

uint16 potRespSamplesLeft[NUM_SAMPLES];
uint8 potRespIndexLeft = 0;
uint16 totalPotRespValueLeft = 0;


/* Change detection variables
//  Even with the averaging, our pot value will likely still flip between
//  neighboring values, so this changeThreshold is used in order to only
//  update the pot value sent out from the arduino if the pot value
//  has changed by a specified amount
   */
int lastAveragePotValueLeft = 0;
int lastAveragePotValueRight = 0;
int lastAveragePotRespValueLeft = 0;
const int CHANGE_THRESHOLD = 5;


/*============================================================================*
 *  Private Definitions
 *===========================================================================*/

/* Number of words of NVM memory used by Battery service */
#define POS_SERVICE_NVM_MEMORY_WORDS              (MAX_SENSOR_LENGTH)/*JDP was 1*/

/* The offset of data being stored in NVM for POS_SENSOR service. This offset is 
 * added to Battery service offset to NVM region (see g_batt_data.nvm_offset) 
 * to get the absolute offset at which this data is stored in NVM
 */
#define POS_SENSOR_NVM_LEVEL_CLIENT_CONFIG_OFFSET     (10)/*JDP made up value*/

/*============================================================================*
 *   Private Function Prototypes
 *===========================================================================*/
/*extern void ResetIdleTimerPos(void);*/

/*============================================================================*
 *  Private Function Implementations
 *===========================================================================*/

/*============================================================================*
 *  Public Function Implementations
 *===========================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryDataInit
 *
 *  DESCRIPTION
 *      This function is used to initialise battery service data 
 *      structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
/*JDP called from hr_sensor.c hrSensorDataInit()*/

extern void Pos_SensorDataInit(void)
{
    if(!AppIsDeviceBonded())
    {
        /* Initialise position sensor level client configuration characterisitic
         * descriptor value only if device is not bonded */
        g_pos_sensor_data.level_client_config = gatt_client_config_none;
    }
    TimerDelete(g_pos_sensor_data.pos_meas_tid);
    g_pos_sensor_data.pos_meas_tid = TIMER_INVALID;
    g_pos_sensor_data.st_ucid = GATT_INVALID_UCID;

    TimerDelete(g_pos_sensor_data.app_tid);
    g_pos_sensor_data.app_tid = TIMER_INVALID;

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryInitChipReset
 *
 *  DESCRIPTION
 *      This function is used to initialise battery service data 
 *      structure at chip reset
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
/*JDP, set this up on first run*/

extern void Pos_SensorInitChipReset(void)
{
    int i;
    /* Initialise position sensor level to 0 percent so that the pos_sensor level 
     * notification (if configured) is sent when the value is read for 
     * the first time after power cycle.
     */
    for(i=0;i<MAX_SENSOR_LENGTH;i++)
        g_pos_sensor_data.value[i] = 0;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appInitExit
 *
 *  DESCRIPTION
 *      This function is called upon exiting from app_state_init state. The 
 *      application starts advertising after exiting this state.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
/*
static void appInitExit(void)
{
        if(g_pos_sensor_data.bonded && 
            (!GattIsAddressResolvableRandom(&g_pos_sensor_data.bonded_bd_addr)))
        {
        */
            /* If the device is bonded and bonded device address is not
             * resolvable random, configure White list with the Bonded 
             * host address 
             */
         /*   if(LsAddWhiteListDevice(&g_pos_sensor_data.bonded_bd_addr)!=
                ls_err_none)
            {
                ReportPanic(app_panic_add_whitelist);
            }
        }
        
}
*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      appAdvertisingExit
 *
 *  DESCRIPTION
 *      This function is called while exiting app_state_fast_advertising and
 *      app_state_slow_advertising states.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void appAdvertisingExit(void)
{
        /* Cancel advertisement timer */
        TimerDelete(g_pos_sensor_data.app_tid);
        g_pos_sensor_data.app_tid = TIMER_INVALID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      idleDormantTimerExpiryHandler
 *
 *  DESCRIPTION
 *      This function contains handles the timer expiry for the timer which the 
 *      application starts on entering the idle mode. The application enters 
 *      the dormant mode in this function and wakes up only on getting input on
 *      the HR input PIO
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void idleDormantTimerExpiryHandler(timer_id tid)
{
    /* Check if it is the NO HR input idle dormant timer which has expired */
    if(tid == g_pos_sensor_data.idle_dormant_tid)
    {
        g_pos_sensor_data.idle_dormant_tid = TIMER_INVALID;

        /* Put the HR sensor into Dormant mode.
         * The application is expecting the Wake pin to be in 
         * shorted state with the HR Input Pio. So the second 
         * parameter shall be set in accordance with the default
         * pull mode on the HR input pio. If it is pulled up,
         * we need to configure wake on low value(2nd parameter)
         * Note: Third parameter gets ignored in Dormant state.
         */
        SleepRequest(sleep_state_dormant, WAKE_SIGNAL_LEVEL, NULL);

    } /* Else ignore timer */

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppSetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
 
extern void AppSetStatePos(app_state new_state)
{
    /* Check if the new state to be set is not the same as the present state
     * of the application. */
    app_state old_state = g_pos_sensor_data.state;
    
    if (old_state != new_state)
    {
        /* Handle exiting old state */
        switch (old_state)
        {
            case app_state_init:
                /*appInitExit();*/
            break;

            case app_state_disconnecting:
                /* Common things to do whenever application exits
                 * app_state_disconnecting state.
                 */

                /* Initialise HR Sensor and used services data structure 
                 * while exiting Disconnecting state
                 */
                Pos_SensorDataInit();
            break;

            case app_state_fast_advertising:
            case app_state_slow_advertising:
                /* Common things to do whenever application exits
                 * APP_*_ADVERTISING state.
                 */
                appAdvertisingExit();
            break;

            case app_state_connected:
                /* Nothing to do */
            break;

            case app_state_idle:
            {
#ifdef ENABLE_DORMANT_MODE_FUNCTIONALITY
                /* Delete the no HR input idle timer */
                TimerDelete(g_pos_sensor_data.idle_dormant_tid);
                g_pos_sensor_data.idle_dormant_tid = TIMER_INVALID;
#endif /* ENABLE_DORMANT_MODE_FUNCTIONALITY */
            }
            break;

            default:
                /* Nothing to do */
            break;
        }

        /* Set new state */
        g_pos_sensor_data.state = new_state;

        /* Handle entering new state */
        switch (new_state)
        {
            case app_state_fast_advertising:
            {
                GattTriggerFastAdverts();

                /* Indicate advertising mode by sounding two short beeps */
                /*SoundBuzzer(buzzer_beep_twice);*/
            }
            break;

            case app_state_slow_advertising:
            {
                GattStartAdverts(FALSE);
            }
            break;

            case app_state_idle:
            {
                /* Sound long beep to indicate non connectable mode*/
                /*SoundBuzzer(buzzer_beep_long);*/

#ifdef ENABLE_DORMANT_MODE_FUNCTIONALITY
                /* The application shall start a no HR input idle timer here. 
                 * On expiry of this timer, the application should enter the 
                 * Dormant mode.
                 */
                TimerDelete(g_pos_sensor_data.idle_dormant_tid);
                g_pos_sensor_data.idle_dormant_tid = TimerCreate(
                                                 IDLE_DORMANT_TIMER_VALUE,
                                                 TRUE, 
                                                 idleDormantTimerExpiryHandler);
#endif /* ENABLE_DORMANT_MODE_FUNCTIONALITY */
            }
            break;

            case app_state_connected:
            {
                /* Common things to do whenever application enters
                 * app_state_connected state.
                 */

                /* Update battery status at every connection instance. It may 
                 * not be worth updating timer more often, but again it will 
                 * primarily depend upon application requirements 
                 */
                /*BatteryUpdateLevel(g_hr_data.st_ucid);*/
                

#ifndef NO_ACTUAL_MEASUREMENT
                /* Reset the idle timer. The application shall disconnect the 
                 * link on idle timer expiry.
                 */
                ResetIdleTimerPos();
#endif /* ! NO_ACTUAL_MEASUREMENT */

                /*StartSendingHRMeasurements();*/
                /*JDP add in pos_sensor timer updates*/
                StartSendingPosMeasurements();

            }
            break;

            case app_state_disconnecting:
                GattDisconnectReq(g_pos_sensor_data.st_ucid);
            break;

            default:
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      hrSensorIdleTimerHandler
 *
 *  DESCRIPTION
 *      This function is used to handle IDLE timer.expiry in connected states.
 *      At the expiry of this timer, application shall disconnect with the 
 *      host and shall move to 'app_state_disconnecting' state.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void posSensorIdleTimerHandler(timer_id tid)
{

    /*Handling signal as per current state */
    switch(g_pos_sensor_data.state)
    {
        case app_state_connected:
        {
            if(tid == g_pos_sensor_data.app_tid)
            {
                /* Trigger Disconnect and move to app_state_disconnecting 
                 * state */

                g_pos_sensor_data.app_tid = TIMER_INVALID;

                /* Delete HR Measurement timer */
                TimerDelete(g_pos_sensor_data.pos_meas_tid);
                g_pos_sensor_data.pos_meas_tid = TIMER_INVALID;

                AppSetStatePos(app_state_disconnecting);

            } /* Else ignore the timer */
        }
        break;

        default:
            /* Ignore timer in any other state */
        break;
    }

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      ResetIdleTimer
 *
 *  DESCRIPTION
 *      This function is used to reset idle timer run by application in
 *      app_state_connected state
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void ResetIdleTimerPos(void)
{
    /* Reset Idle timer */
    TimerDelete(g_pos_sensor_data.app_tid);

    g_pos_sensor_data.app_tid = TimerCreate(CONNECTED_IDLE_TIMEOUT_VALUE, 
                                    TRUE, posSensorIdleTimerHandler);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      sendPosMeasurement
 *
 *  DESCRIPTION
 *      This function sends out one Pos Measurement.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/


static void sendPosMeasurement(void)
{
    /* If notifications are configured on the Pos Sensor 
     * Measurement characteristic, the application should take 
     * following actions:
     * 1. configure Radio event on Tx data,
     * 2. send a notification.
     * 3. wait for the radio event.
     */
     /* Send the Position Measurement notification. */
     Pos_SensorUpdateLevelALWAYS(g_pos_sensor_data.st_ucid,MAX_SENSOR_LENGTH);        
     /*Pos_SensorUpdateLevel(g_pos_sensor_data.st_ucid,0x6);*/
     /* Reset Idle timer only if we have received some 
      * measurements to transmit
      */
     
     ResetIdleTimerPos();
     

        /* Start a timer to schedule the next Pos Measurement transmission 
         * after timer POS_MEAS_TIME. If the application gets a radio event
         * for the notification sent, this timer will be deleted 
         * and a new timer with value (POS_MEAS_TIME-3ms)shall be started. 
         * And if the application does not receives any radio event, the 
         * application will send the next Measurement on this timer expiry.
         */
     TimerDelete(g_pos_sensor_data.pos_meas_tid);
     g_pos_sensor_data.pos_meas_tid = TimerCreate(POS_MEAS_TIME,
                                            TRUE, 
                                            posMeasTimerHandler);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      hrMeasTimerHandler
 *
 *  DESCRIPTION
 *      Called repeatedly via a timer to transmit HR measurements
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void posMeasTimerHandler(timer_id tid)
{
    /*Handling signal as per current state */
    switch(g_pos_sensor_data.state)
    {
        case app_state_connected:
        {
            if(tid == g_pos_sensor_data.pos_meas_tid)
            {
                g_pos_sensor_data.pos_meas_tid = TIMER_INVALID;
                sendPosMeasurement();
            }
        }
        break;

        case app_state_disconnecting:
        {
            /* Do nothing in this state as the device has triggered 
             * disconnect 
             */
            g_pos_sensor_data.pos_meas_tid = TIMER_INVALID;
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsHeartRateNotifyEnabled
 *
 *  DESCRIPTION
 *      This function returns whether notifications are enabled for heart rate 
 *      measurement characteristic.
 *
 *  RETURNS
 *      TRUE/FALSE: Notification configured or not
 *
 *---------------------------------------------------------------------------*/

extern bool IsPosSensorNotifyEnabled(void)
{
    return (g_pos_sensor_data.level_client_config & 
                    gatt_client_config_notification);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      StartSendingPosMeasurements
 *
 *  DESCRIPTION
 *      This function starts sending the HR measurements.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void StartSendingPosMeasurements(void)
{
    /* If notifications are enabled and there is no HR Measurement timer 
     * running, enable the Radio First TX events and start sending HR 
     * Measurements. 
     */
    if(IsPosSensorNotifyEnabled() && 
        (g_pos_sensor_data.pos_meas_tid == TIMER_INVALID))
    {
        /* Enable Radio Tx events. */
        LsRadioEventNotification(g_pos_sensor_data.st_ucid,radio_event_first_tx);

        sendPosMeasurement();
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryHandleAccessRead
 *
 *  DESCRIPTION
 *      This function handles read operation on battery service attributes
 *      maintained by the application and responds with the GATT_ACCESS_RSP 
 *      message.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void Pos_SensorHandleAccessRead(GATT_ACCESS_IND_T *p_ind)
{
    uint16 length = 0;
    uint16 sensor_right_val,sensor_left_val,sensor_temp,resp_left_val;
    uint8 value[MAX_SENSOR_LENGTH];
    uint8 *p_val = NULL;
    sys_status rc = sys_status_success;
    
    switch(p_ind->handle)
    {
        case HANDLE_POS_LOC_LEVEL:
        {
            /*get Analog value of potentiometers here*/
            /*Not sure if this gets called with the newer method
              of doing things, put a debug statement but see if
              this is needed, JDP*/
            
            sensor_left_val=AioRead(LEFT_SENSOR);
            g_pos_sensor_data.value[0]=sensor_left_val;
            g_pos_sensor_data.value[1]=sensor_left_val>>8;
            
            sensor_right_val=AioRead(RIGHT_SENSOR);
            g_pos_sensor_data.value[2]=sensor_right_val;
            g_pos_sensor_data.value[3]=sensor_right_val>>8;
            
            sensor_temp=ThermometerReadTemperature();
            g_pos_sensor_data.value[4]=sensor_temp;
            g_pos_sensor_data.value[5]=sensor_temp>>8;
            
            /*force sensor testing*/
            
            resp_left_val = loop(AIO0);
            g_pos_sensor_data.value[6]=resp_left_val;
            g_pos_sensor_data.value[7]=resp_left_val>>8;
            
            length = MAX_SENSOR_LENGTH; /*was 2, now 8*/
            p_val = value;
        }
        break;

        case HANDLE_POS_LOC_LEVEL_C_CFG:
        {
            length = 2;
            p_val = value;
            BufWriteUint16((uint8 **)&p_val, 
                           g_pos_sensor_data.level_client_config);
        }
        break;
        
        default:
            /* No more IRQ characteristics */
            rc = gatt_status_read_not_permitted;
        break;

    }

    /* Send Access response */
    GattAccessRsp(p_ind->cid, p_ind->handle, rc,
                  length, value);
}
        
/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryHandleAccessWrite
 *
 *  DESCRIPTION
 *      This function handles write operation on battery service attributes 
 *      maintained by the application.and responds with the GATT_ACCESS_RSP 
 *      message.
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/

extern void Pos_SensorHandleAccessWrite(GATT_ACCESS_IND_T *p_ind)
{
    uint8 *p_value = p_ind->value;
    uint8 i=0;    
    uint16 client_config;
    sys_status rc = sys_status_success;

    switch(p_ind->handle)
    {
        
        case HANDLE_POS_LOC_LEVEL_C_CFG:
        {
            client_config = BufReadUint16(&p_value);


            if((client_config == gatt_client_config_notification) ||
               (client_config == gatt_client_config_none))
            {
                g_pos_sensor_data.level_client_config = client_config;

                if(AppIsDeviceBonded())
                {
                     Nvm_Write(&client_config,
                              sizeof(client_config),
                              g_pos_sensor_data.nvm_offset + 
                              POS_SENSOR_NVM_LEVEL_CLIENT_CONFIG_OFFSET);
                }
                /*StartSendingPosMeasurements();*/ /*JDP uncomment later*/
            }
            else
            {
                
                rc = gatt_status_desc_improper_config;
            }

        }
        break;
        
        default:
            rc = gatt_status_write_not_permitted;
            break;

    }

    /* Send ACCESS RESPONSE */
    GattAccessRsp(p_ind->cid, p_ind->handle, rc, 0, NULL);

    
    /* Send an update as soon as notifications are configured */
    /* Send in the CID and the length (depends on # of sensors)*/
    
    if(g_pos_sensor_data.level_client_config & gatt_client_config_notification)
    {
        for(i=0;i<MAX_SENSOR_LENGTH;i++)
        {
            g_pos_sensor_data.value[i] = 0xF;
        }
        
        Pos_SensorUpdateLevel(p_ind->cid,MAX_SENSOR_LENGTH);
    }
    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      Pos_SensorUpdateLevel
 *
 *  DESCRIPTION
 *      This function is to monitor the battery level and trigger notifications
 *      (if configured) to the connected host.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void Pos_SensorUpdateLevel(uint16 ucid, uint16 length)
{
    uint16 cur_pos_value_right,cur_pos_checker_right,
            cur_pos_value_left,cur_pos_checker_left,
            cur_resp_value_left,cur_resp_checker_left,
            current_temp, cur_temp_checker;
        
    uint8 *p_val;
    uint8 value[MAX_SENSOR_LENGTH];

    
    /* Read the sensor levels */
    current_temp=ThermometerReadTemperature();
    
    cur_pos_value_right = loop(RIGHT_SENSOR);/*AioRead(RIGHT_SENSOR);*/
    cur_pos_value_left = loop(LEFT_SENSOR);/*AioRead(LEFT_SENSOR);*/

    cur_resp_value_left = loop(AIO0);/*)AioRead(AIO0);*/
    
    cur_pos_checker_left = ((uint16)g_pos_sensor_data.value[1] << 8) | 
                        g_pos_sensor_data.value[0];
    cur_pos_checker_right = ((uint16)g_pos_sensor_data.value[3] << 8) | 
                        g_pos_sensor_data.value[2];
    cur_temp_checker = ((uint16)g_pos_sensor_data.value[5] << 8) | 
                        g_pos_sensor_data.value[4];
    cur_resp_checker_left = ((uint16)g_pos_sensor_data.value[7] << 8) | 
                        g_pos_sensor_data.value[6];
    
    /* If the current and old position sensor levels are not same, update the 
     * connected host if notifications are configured.
     */
    if((cur_pos_checker_right != cur_pos_value_right) || 
       (cur_pos_checker_left != cur_pos_value_left) ||
       (cur_resp_checker_left != cur_resp_value_left) ||
        (cur_temp_checker != current_temp) )
    {

        if(ucid != GATT_INVALID_UCID)
        {
            if((g_pos_sensor_data.level_client_config &
                                            gatt_client_config_notification))
        {
            p_val = value;
            
            p_val[0]=cur_pos_value_left;
            p_val[1]=cur_pos_value_left>>8;
            
            p_val[2]=cur_pos_value_right;
            p_val[3]=cur_pos_value_right>>8;
            
            p_val[4]=current_temp;
            p_val[5]=current_temp>>8;
            
            p_val[6]=cur_resp_value_left;
            p_val[7]=cur_resp_value_left>>8;
            
            
            GattCharValueNotification(ucid, 
                                      HANDLE_POS_LOC_LEVEL, 
                                      length, p_val);

            /* Update Battery Level characteristic in database */
            g_pos_sensor_data.value[0]=cur_pos_value_left;
            g_pos_sensor_data.value[1]=cur_pos_value_left>>8;
            
            g_pos_sensor_data.value[2]=cur_pos_value_right;
            g_pos_sensor_data.value[3]=cur_pos_value_right>>8;
            
            g_pos_sensor_data.value[4]=current_temp;
            g_pos_sensor_data.value[5]=current_temp>>8;
            
            g_pos_sensor_data.value[6]=cur_resp_value_left;
            g_pos_sensor_data.value[7]=cur_resp_value_left>>8;
            
        }
        }
    }
    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      Pos_SensorUpdateLevelALWAYS
 *
 *  DESCRIPTION
 *      This function is to monitor the battery level and trigger notifications
 *      (if configured) to the connected host.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void Pos_SensorUpdateLevelALWAYS(uint16 ucid, uint16 length)
{
    uint16 cur_pos_value_right,cur_pos_checker_right,
            cur_pos_value_left,cur_pos_checker_left,
            cur_resp_value_left,cur_resp_checker_left,
            current_temp, cur_temp_checker;
        
    uint8 *p_val;
    uint8 value[MAX_SENSOR_LENGTH];

    
    /* Read the sensor levels */
    current_temp=ThermometerReadTemperature();
    
    cur_pos_value_right = AioRead(RIGHT_SENSOR);
    cur_pos_value_left = AioRead(LEFT_SENSOR);
    cur_resp_value_left = loop(AIO0);/*)AioRead(AIO0);*/
    
    cur_pos_checker_left = ((uint16)g_pos_sensor_data.value[1] << 8) | 
                        g_pos_sensor_data.value[0];
    cur_pos_checker_right = ((uint16)g_pos_sensor_data.value[3] << 8) | 
                        g_pos_sensor_data.value[2];
    cur_temp_checker = ((uint16)g_pos_sensor_data.value[5] << 8) | 
                        g_pos_sensor_data.value[4];
    cur_resp_checker_left = ((uint16)g_pos_sensor_data.value[7] << 8) | 
                        g_pos_sensor_data.value[6];
    /* If the current and old position sensor levels are not same, update the 
     * connected host if notifications are configured.
     */
    if((cur_pos_checker_right != cur_pos_value_right) || 
       (cur_pos_checker_left != cur_pos_value_left) ||
       (cur_resp_checker_left != cur_resp_value_left) ||
        (cur_temp_checker != current_temp) )
    {

        
            p_val = value;
            
            p_val[0]=cur_pos_value_left;
            p_val[1]=cur_pos_value_left>>8;
            
            p_val[2]=cur_pos_value_right;
            p_val[3]=cur_pos_value_right>>8;
            
            p_val[4]=current_temp;
            p_val[5]=current_temp>>8;
            
            p_val[6]=cur_resp_value_left;
            p_val[7]=cur_resp_value_left>>8;
            
            
            GattCharValueNotification(/*ucid*/0, 
                                      HANDLE_POS_LOC_LEVEL, 
                                      length, p_val);

            /* Update Battery Level characteristic in database */
            g_pos_sensor_data.value[0]=cur_pos_value_left;
            g_pos_sensor_data.value[1]=cur_pos_value_left>>8;
            
            g_pos_sensor_data.value[2]=cur_pos_value_right;
            g_pos_sensor_data.value[3]=cur_pos_value_right>>8;
            
            g_pos_sensor_data.value[4]=current_temp;
            g_pos_sensor_data.value[5]=current_temp>>8;
            
            g_pos_sensor_data.value[6]=cur_resp_value_left;
            g_pos_sensor_data.value[7]=cur_resp_value_left>>8;
            
            
    }
    
}

void setup()
{
  /* Fill the array of samples with zeros*/
  int i=0;
  for (; i<NUM_SAMPLES; i++) potSamplesLeft[i] = 0;
  for (i=0; i<NUM_SAMPLES; i++) potSamplesRight[i] = 0;
  for (i=0; i<NUM_SAMPLES; i++) potRespSamplesLeft[i] = 0;
}

uint16 loop(int side)
{
  /* potSamples[potIndex] is the oldest data in the array
     We are about to override it with a new value, but first
     we need to subtract it from our running total of pot values  
   */
  int i=0;
  for(;i<20;i++)
  {
      if(side==RIGHT_SENSOR)
      {
          totalPotValueRight -= potSamplesRight[potIndexRight];
  
          /* Get the current pot value*/
          uint16 currentPotValue = AioRead(RIGHT_SENSOR);
  
          /* Update our running total and add the new value to the array*/
          totalPotValueRight += currentPotValue;
          potSamplesRight[potIndexRight] = currentPotValue;
      
          /* Increment our index*/
          potIndexRight++;
          if (potIndexRight>NUM_SAMPLES-1) potIndexRight = 0;
      
          /* Calculate the average pot value*/
          int averagePotValue = totalPotValueRight / NUM_SAMPLES;
      
          /* Detect whether the new average pot value has changed enough
           from our last saved average pot value */
          int change = abs(averagePotValue-lastAveragePotValueRight);
          if (change >= CHANGE_THRESHOLD){
            lastAveragePotValueRight = averagePotValue;
          }
      }
      else if(side==LEFT_SENSOR)
      {
          totalPotValueLeft -= potSamplesLeft[potIndexLeft];
  
          /* Get the current pot value*/
          uint16 currentPotValue = AioRead(LEFT_SENSOR);
  
          /* Update our running total and add the new value to the array*/
          totalPotValueLeft += currentPotValue;
          potSamplesLeft[potIndexLeft] = currentPotValue;
      
          /* Increment our index*/
          potIndexLeft++;
          if (potIndexLeft>NUM_SAMPLES-1) potIndexLeft = 0;
      
          /* Calculate the average pot value*/
          int averagePotValue = totalPotValueLeft / NUM_SAMPLES;
      
          /* Detect whether the new average pot value has changed enough
           from our last saved average pot value */
          int change = abs(averagePotValue-lastAveragePotValueLeft);
          if (change >= CHANGE_THRESHOLD){
            lastAveragePotValueLeft = averagePotValue;
          }
      }
      else /*Respiration*/
      {
          totalPotRespValueLeft -= potRespSamplesLeft[potRespIndexLeft];
  
          /* Get the current pot value*/
          uint16 currentPotValue = AioRead(AIO0);
  
          /* Update our running total and add the new value to the array*/
          totalPotRespValueLeft += currentPotValue;
          potRespSamplesLeft[potRespIndexLeft] = currentPotValue;
      
          /* Increment our index*/
          potRespIndexLeft++;
          if (potRespIndexLeft>NUM_SAMPLES-1) potRespIndexLeft = 0;
      
          /* Calculate the average pot value*/
          int averagePotValue = totalPotRespValueLeft / NUM_SAMPLES;
      
          /* Detect whether the new average pot value has changed enough
           from our last saved average pot value */
          int change = abs(averagePotValue-lastAveragePotRespValueLeft);
          if (change >= CHANGE_THRESHOLD){
            lastAveragePotRespValueLeft = averagePotValue;
          }
      }  
  }
  /* Output the last saved average pot value,
     the last one that exceeeded the change threshold */
  /*Serial.println(lastAveragePotValue);*/
  if(side==RIGHT_SENSOR)
    return lastAveragePotValueRight;  
  else if(side==LEFT_SENSOR)
    return lastAveragePotValueLeft;
  else
    return lastAveragePotRespValueLeft;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryReadDataFromNVM
 *
 *  DESCRIPTION
 *      This function is used to read battery service specific data stored in 
 *      NVM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
/*JDP not as big a deal for initial configuration*/
extern void Pos_SensorReadDataFromNVM(uint16 *p_offset)
{
    g_pos_sensor_data.nvm_offset = *p_offset;

    if(AppIsDeviceBonded())
    {
        Nvm_Read((uint16*)&g_pos_sensor_data.level_client_config,
                sizeof(g_pos_sensor_data.level_client_config),
                *p_offset + 
                POS_SENSOR_NVM_LEVEL_CLIENT_CONFIG_OFFSET);
        /*JDP fix BATTERY here later*/
    }

    *p_offset += POS_SERVICE_NVM_MEMORY_WORDS;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryCheckHandleRange
 *
 *  DESCRIPTION
 *      This function is used to check if the handle belongs to the Battery 
 *      service
 *
 *  RETURNS
 *      Boolean - Indicating whether handle falls in range or not.
 *
 *---------------------------------------------------------------------------*/
/*JDP called from HandleAccessRead and AppProcessLmEvent, need for each of
  the sensors to gauge if the range is met*/
extern bool Pos_SensorCheckHandleRange(uint16 handle)
{
    return ((handle >= HANDLE_POS_SENSOR_SERVICE) &&
            (handle <= HANDLE_POS_SENSOR_SERVICE_END))
            ? TRUE : FALSE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryBondingNotify
 *
 *  DESCRIPTION
 *      This function is used by application to notify bonding status to 
 *      battery service
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void Pos_SensorBondingNotify(void)
{

    /* Write data to NVM if bond is established */
    if(AppIsDeviceBonded())
    {
        /* Write to NVM the client configuration value of battery level 
         * that was configured prior to bonding 
         */
        Nvm_Write((uint16*)&g_pos_sensor_data.level_client_config, 
                  sizeof(g_pos_sensor_data.level_client_config), 
                  g_pos_sensor_data.nvm_offset + 
                  POS_SENSOR_NVM_LEVEL_CLIENT_CONFIG_OFFSET);
    }

}

#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteBatteryServiceDataInNvm
 *
 *  DESCRIPTION
 *      This function writes Battery service data in NVM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void WritePos_SensorServiceDataInNvm(void)
{
    /* Write to NVM the client configuration value of battery level 
     * that was configured prior to bonding 
     */
    Nvm_Write((uint16*)&g_pos_sensor_data.level_client_config, 
                  sizeof(g_pos_sensor_data.level_client_config), 
                  g_pos_sensor_data.nvm_offset + 
                  POS_SENSOR_NVM_LEVEL_CLIENT_CONFIG_OFFSET);
}
#endif /* NVM_TYPE_FLASH */
