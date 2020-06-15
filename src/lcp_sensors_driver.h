/****************************************************************************/
/*                                                                          */
/* This Source Code Form is subject to the terms of the Mozilla Public      */
/* License, v. 2.0. If a copy of the MPL was not distributed with this      */
/* file, You can obtain one at http://mozilla.org/MPL/2.0/.                 */
/*                                                                          */
/****************************************************************************/

/*
 * This File is part of the LCP project
 */

/** 
 * @file lcp_sensors_driver.h
 * 
 * @author Bruno Agusto Casu
 *
 * @brief Driver for the attached sensros in the LCP
 */


#ifndef LCP_SENSORS_DRIVER_H
#define LCP_SENSORS_DRIVER_H
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

void analog_sensors_read (uint8_t *data);
void bmp280_read (uint8_t *data);

#endif
