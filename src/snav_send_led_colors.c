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

int main(int argc, char* argv[])
{
  // loop counter
  unsigned int cntr = 0;

  // RGB led colors
  uint8_t led_colors[3] = {0,0,0};

  // timeout for flight controller to take over LED control after API commands stop
  int32_t timeout_us = 1e6;

  // Attempt to initialize pointer to snav data structure
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  // update LED colors in infinite loop
  bool keep_going = true;
  while ( keep_going )
  {
    // update cached data within Snapdragon Navigator
    int update_ret = sn_update_data();
    if (update_ret != 0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running\n");
      break;
    }

    // loop through different colors
    int cntr2 = cntr % 400;
    // -- OFF ---
    if      (cntr2 < 100) { led_colors[0] = 0;   led_colors[1] = 0;   led_colors[2] = 0;   }
    // -- RED ---
    else if (cntr2 < 200) { led_colors[0] = 255; led_colors[1] = 0;   led_colors[2] = 0;   }
    // -- GREEN ---
    else if (cntr2 < 300) { led_colors[0] = 0;   led_colors[1] = 255; led_colors[2] = 0;   }
    // -- BLUE ---
    else if (cntr2 < 400) { led_colors[0] = 0;   led_colors[1] = 0;   led_colors[2] = 255; }

    int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout_us);
    if (ret != 0)
    {
      printf("Flight control is not responding to commands, exiting.\n");
      keep_going = false;
    }

    // note that commands should only be sent as often as needed (minimize message traffic)
    usleep(10000);
    cntr++;
  }

  return 0;
}

