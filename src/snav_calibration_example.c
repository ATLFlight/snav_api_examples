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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "snapdragon_navigator.h"


int run_cal(SnMode cal_mode)
{

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return 1;
  }

  int cal_started = 0;
  int attempt_ctr = 0;

  while (true)
  {

    if(sn_update_data() != 0)
    {
      printf("\nDetected likely failure in SN, ensure it is running.\n\n");
      return 1;
    }

    if (snav_data->general_status.current_mode == cal_mode)
    {
      if (cal_started == 0)
      {
        printf("Calibration is in progress.\n");
      }
      cal_started = 1;
    }

    else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && cal_started)
    {
      printf("Calibration Succeeded.\n");
      return 0;
    }

    else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && cal_started)
    {
      printf("Calibration Failed.\n");
      return 0;
    }

    else
    {
      if (attempt_ctr < 10)
      {
        printf("Starting calibration, attempt %d.\n", attempt_ctr);

        if (cal_mode == SN_STATIC_ACCEL_CALIBRATION_MODE)
        {
          sn_start_static_accel_calibration();
        }

        if (cal_mode == SN_MAGNETOMETER_CALIBRATION_MODE)
        {
          sn_start_magnetometer_calibration();
        }

        ++attempt_ctr;
      }
      else
      {
        printf("Unable to start calibration.\n");
        return 1;
      }
    }

    usleep(1000000);
  }

  // Call sync to make sure any calibration files get written to disk
  system("sync");

  return 0;
}

int main(int argc, char* argv[])
{
  printf("\nUSAGE\n");
  printf("------------------------------\n");
  printf("-s  Attempt to run the static accel offset calibration\n");
  printf("-m  Attempt to run the magnetometer calibration\n");
  printf("\n");

  int c;

  while ((c = getopt(argc, argv, "sm")) != -1)
  {
    switch(c)
    {
      case 's':

        printf("\n\nSTATIC ACCEL CALIBRATION\n");
        printf("------------------------------\n");
        return run_cal(SN_STATIC_ACCEL_CALIBRATION_MODE);

      case 'm':
        printf("\n\nMAGNETOMETER CALIBRATION\n");
        printf("------------------------------\n");
        return run_cal(SN_MAGNETOMETER_CALIBRATION_MODE);
    }
  }

  return 0;
}

