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
 * recent sonar data from Snapdragon Navigator API
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
  
  uint32_t sonar_last_cntr = 0;

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

    // Get the status of the attitude estimator
    SnDataStatus sonar_status = (SnDataStatus) snav_data->data_status.sonar_0_status;

    // If the sonar state is valid, check if the data is new and print it
    if (sonar_status == SN_DATA_VALID)
    {
      if (sonar_last_cntr != snav_data->sonar_0_raw.cntr)
      {
        sonar_last_cntr = snav_data->sonar_0_raw.cntr;
        printf("[%lld] sonar range = %.3f\n", snav_data->sonar_0_raw.time, snav_data->sonar_0_raw.range);
      }
    }
    else
    {
      printf("sonar data timeout!\n");
    }

    //Sleep for a while
    usleep(20000);
  }

  //
  return 0;
}
