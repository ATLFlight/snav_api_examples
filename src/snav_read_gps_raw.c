/****************************************************************************
 *   Copyright (c) 2017 Michael Shomin. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file. 
 ****************************************************************************/

/* Standard libraries */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* Snapdragon Navigator API */
#include "snapdragon_navigator.h"


/* Main entry point -- continually loop and retrieve the most
 * recent raw gps data from Snapdragon Navigator API
 */
int main(int argc, char* argv[])
{
  // Attempt to initialize pointer to snav data structure
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("Failed to receive data pointer -- aborting.\n");
    return -1;
  }

  int64_t gps0_last_update_time = 0;

  while( 1 )
  {
    // Update the cached data in Snapdragon Navigator. This should be done frequently enough
    // to capture changes for the desired application -- e.g. once per control loop
    int update_ret = sn_update_data();
    if(update_ret != 0)
    {
      printf("Data failed to update; ensure Snapdragon Navigator is running.\n");
      continue;
    }

    //check if the data got updated
    if (snav_data->gps_0_raw.time != gps0_last_update_time)
    {
      gps0_last_update_time = snav_data->gps_0_raw.time;

      Gps0Raw *gps0_raw = &snav_data->gps_0_raw;

      printf("----------------------------\r\n");
      printf("GPS: Week = %d, TOW = %.3fs, update counter = %d, snav time = %lldus\r\n",
             gps0_raw->gps_week, (float)(gps0_raw->gps_time_sec) + (gps0_raw->gps_time_nsec*1e-9), gps0_raw->cntr, gps0_raw->time);

      //multiply lat and lon by 1e-7 because values are integers (degrees * 10^7)
      printf("GPS: Latitude (deg) = %.9f, Longitude (deg) = %.9f\r\n",
             (gps0_raw->latitude*0.0000001), (gps0_raw->longitude*0.0000001));

      printf("GPS: Altitude (m) = %fm, num. satellites = %d, horizontal accuracy (m) = %f\r\n",gps0_raw->altitude, gps0_raw->num_satellites, gps0_raw->horizontal_acc);
      printf("GPS: Velocity NEU (m/s) = %+3.3f %+3.3f %+3.3f\r\n",gps0_raw->lin_vel[0], gps0_raw->lin_vel[1], gps0_raw->lin_vel[2]);

      int ii;
      printf("GPS: SV_IDS: ");
      for (ii=0; ii<sizeof(gps0_raw->sv_ids); ii++)
      {
        printf("%3d ",gps0_raw->sv_ids[ii]);
      }

      printf("\r\nGPS: SV_CN0: ");
      for (ii=0; ii<sizeof(gps0_raw->sv_ids); ii++)
      {
        printf("%3d ",gps0_raw->sv_cn0[ii]);
      }
      printf("\r\n");
    }

    //sleep and check data again
    usleep(100000);
  }

  return 0;
}
