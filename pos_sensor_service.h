/******************************************************************************
 *
 *  FILE
 *      pos_sensor_service.h
 *
 *  DESCRIPTION
 *      Header definitions for Position Sensor service
 *
 *****************************************************************************/

#ifndef __POS_SENSOR_SERVICE_H__
#define __POS_SENSOR_SERVICE_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <bt_event_types.h>
#include <debug.h>
#include <aio.h>
#include "hr_sensor.h"

#define LEFT_SENSOR                     (AIO1)    /*AIO1*/
#define RIGHT_SENSOR                    (AIO2)    /*AIO2*/

#define MAX_SENSOR_LENGTH 8 /* 2 sensors that are typically uint16 
                                                        or uint8[2] */
#define POS_MEAS_TIME                   (1 * SECOND)


/* Position sensor service data type */
typedef struct
{
    /* Sensor Value Level in mV */
    uint8   value[MAX_SENSOR_LENGTH];
    
    /* Value for which advertisement timer needs to be started. 
     *
     * For bonded devices, the timer is initially started for 10 seconds to 
     * enable fast connection by bonded device to the sensor. If bonded device 
     * doesn't connect within this time, another 20 seconds timer is started 
     * to enable fast connections from any collector device in the vicinity. 
     * This is then followed by reduced power advertisements.
     *
     * For non-bonded devices, the timer is initially started for 30 seconds 
     * to enable fast connections from any collector device in the vicinity.
     * This is then followed by reduced power advertisements.
     */
    uint32                         advert_timer_value;

    /* Store timer id while doing 'UNDIRECTED ADVERTS' and for Idle timer 
     * in CONNECTED' states.
     */
    timer_id                       app_tid;

    /* Current state of application */
    app_state                      state;
    
    /* Store timer id for Pos Sensor transmission timer in 'CONNECTED' 
     * state 
     */
    timer_id                       pos_meas_tid;
    
    /* Track the UCID as Clients connect and disconnect */
    uint16                         st_ucid;    
    
    timer_id                       idle_dormant_tid;
    
    /* Client configurate for Position Sensor Level characteristic */
    /*JDP revisit this*/
    gatt_client_config level_client_config;

    /* NVM Offset at which Position Sensor data is stored */
    uint16 nvm_offset;

} POS_SENSOR_DATA_T;

/*============================================================================*
 *  Private Data
 *===========================================================================*/

/* Position sensor service data instance */
POS_SENSOR_DATA_T g_pos_sensor_data;


/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is used to initialise position sensor service data structure.*/
extern void Pos_SensorDataInit(void);

/* This function is used to initialise position sensor service data structure at 
 * chip reset
 */
extern void Pos_SensorInitChipReset(void);


/* This function returns whether notifications are enabled for heart rate 
 * measurement characteristic
 */
extern bool IsPosSensorNotifyEnabled(void);

/* This function is used to set the state of the application */
extern void AppSetStatePos(app_state new_state);

/* This function handles read operation on position sensor service attributes
 * maintained by the application
 */
extern void Pos_SensorHandleAccessRead(GATT_ACCESS_IND_T *p_ind);

/* This function handles write operation on position sensor service attributes 
 * maintained by the application
 */
extern void Pos_SensorHandleAccessWrite(GATT_ACCESS_IND_T *p_ind);

/* This function is to monitor the position sensor level and trigger notifications
 * (if configured) to the connected host
 */
extern void Pos_SensorUpdateLevel(uint16 ucid,uint16 length);

extern void Pos_SensorUpdateLevelALWAYS(uint16 ucid,uint16 length);

/* This function is used to read position sensor service specific data stored in 
 * NVM
 */
extern void Pos_SensorReadDataFromNVM(uint16 *p_offset);

/* This function is used to check if the handle belongs to the Position Sensor 
 * service
 */
extern bool Pos_SensorCheckHandleRange(uint16 handle);

/* This function is used to reset idle timer run by application in
 * app_state_connected state
 */
extern void ResetIdleTimerPos(void);

extern void StartSendingPosMeasurements(void);

extern void posMeasTimerHandler(timer_id tid);

/* This function is used by application to notify bonding status to 
 * position sensor service
 */
extern void Pos_SensorBondingNotify(void);

uint16 loop(int side);
void setup(void);


#ifdef NVM_TYPE_FLASH
/* This function writes Position Sensor service data in NVM */
extern void WritePos_SensorServiceDataInNvm(void);
#endif /* NVM_TYPE_FLASH */

#endif
