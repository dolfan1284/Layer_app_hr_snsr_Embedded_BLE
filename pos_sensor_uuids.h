/******************************************************************************
 *  
 *  FILE
 *      pos_sensor_uuids.h
 *
 * DESCRIPTION
 *      UUID MACROs for Position Sensor service
 *
 *****************************************************************************/

#ifndef __POS_SENSOR_UUIDS_H__
#define __POS_SENSOR_UUIDS_H__

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser which 
 * creates .c and .h files from .db file doesn't understand brackets and will
 * raise syntax errors. 
 */

/* For UUID values, refer http://developer.bluetooth.org/gatt/services/
 * Pages/ServiceViewer.aspx?u=org.bluetooth.service.battery_service.xml
 */

#define UUID_POS_SENSOR_SERVICE                           0x1f0f /*JDP made up*/

#define UUID_POS_LOCATION_LEVEL                           0x2f19 /*JDP made up*/

#endif