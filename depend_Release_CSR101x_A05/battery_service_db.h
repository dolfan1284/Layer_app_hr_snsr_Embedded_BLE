/*
 * THIS FILE IS AUTOGENERATED, DO NOT EDIT!
 *
 * generated by gattdbgen from depend_Release_CSR101x_A05/battery_service_db.db_
 */
#ifndef __BATTERY_SERVICE_DB_H
#define __BATTERY_SERVICE_DB_H

#include <main.h>

#define HANDLE_BATTERY_SERVICE          (0x0001)
#define HANDLE_BATTERY_SERVICE_END      (0xffff)
#define ATTR_LEN_BATTERY_SERVICE        (2)
#define HANDLE_BATT_LEVEL               (0x0003)
#define ATTR_LEN_BATT_LEVEL             (1)
#define HANDLE_BATT_LEVEL_C_CFG         (0x0004)
#define ATTR_LEN_BATT_LEVEL_C_CFG       (0)

extern uint16 *GattGetDatabase(uint16 *len);

#endif

/* End-of-File */
