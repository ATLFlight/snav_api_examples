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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* Snapdragon Navigator API */
#include "snapdragon_navigator.h"

void print_usage()
{
  printf("-r  Send RPM commands to ESCs, default\n");
  printf("-p  Send PWM commands to ESCs\n");
  printf("-h  Print this message\n");
}

int main(int argc, char* argv[])
{
  int c;
  bool send_rpms = true;

  // get user input
  while ((c = getopt(argc, argv, "rph")) != -1)
  {
    switch (c)
    {
      case 'r':
        send_rpms = true;
        break;
      case 'p':
        send_rpms = false;
        break;
      case 'h':
        print_usage();
        return -1;
      default:
        print_usage();
        return -1;
    }
  }

  // Attempt to initialize pointer to snav data structure
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  bool keep_going = true;
  while (keep_going)
  {
    // update cached data within Snapdragon Navigator
    int update_ret = sn_update_data();
    if (update_ret != 0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running\n");
      keep_going = false;
    }
    else
    {
      static unsigned int cntr = 0;

      // Cycle through ESC IDs 0 to 3 for requesting feedback
      static int fb_id = 0;

      if (send_rpms)
      {
        // Initialize RPM commands to all zeros
        int rpms[4] = {0, 0, 0, 0};

        // alternate motors every 100 counts
        rpms[cntr/100] = 2000; // 2000 RPM

        // Send the RPM commands and request feedback from ESC ID = fb_id
        sn_send_esc_rpm(rpms, 4, fb_id);
        printf("Sending RPMs = %d %d %d %d | fb_id = %d\n",
            rpms[0], rpms[1], rpms[2], rpms[3], fb_id);
      }
      else // send pwms
      {
        // Initialize PWM commands to all zeros
        int pwms[4] = {0, 0, 0, 0};

        // alternate motors every 100 counts
        pwms[cntr/100] = 10; // 10 % power

        // Send the PWM commands and request feedback from ESC ID = fb_id
        sn_send_esc_pwm(pwms, 4, fb_id);
        printf("Sending PWMs = %d %d %d %d | fb_id = %d\n",
            pwms[0], pwms[1], pwms[2], pwms[3], fb_id);
      }

      // Verify that the flight controller is acknowledging the commands
      static unsigned int mode_not_correct_cntr = 0;
      if ((send_rpms && snav_data->general_status.current_mode != SN_ESC_RPM_MODE)
          || (!send_rpms && snav_data->general_status.current_mode != SN_ESC_PWM_MODE))
      {
        if (++mode_not_correct_cntr > 1)
        {
          printf("Flight control is not responding to commands, exiting.\n");
          keep_going = false;
        }
      }

      if (++cntr == 400) cntr = 0;
      if (++fb_id == 4) fb_id = 0;
    }

    // sleep between iterations
    usleep(10000);
  }

  return 0;
}

