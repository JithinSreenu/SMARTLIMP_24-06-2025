/**
 * @file    bluetooth.h
 * @brief   Bluetooth telemetry task.
 *
 * Sends a 50 Hz telemetry frame over USART1 every 20 ms.
 * Also handles inbound calibration / config commands.
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "project_types.h"

/* The actual task entry point — created from app_tasks.c. */
void vBluetoothTask(void *pv);

#endif /* BLUETOOTH_H */
