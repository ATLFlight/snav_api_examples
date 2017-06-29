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
#include <string.h>
#include <unistd.h>

/* Snapdragon Navigator API */
#include "snapdragon_navigator.h"

/* Main entry point -- continually loop and retrieve the most
 * recent data from Snapdragon Navigator API
 */
int main(int argc, char* argv[])
{
  // iterators to be reused
  int i, loop_counter;

  printf("\nRetrieving Snapdragon Navigator data pointer...\n");

  // Attempt to initialize pointer to snav data structure
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("  > Failed -- aborting.\n");
    return -1;
  }
  else
  {
    printf("  > Success");
  }

  /*
   * RETRIEVING INFORMATION ABOUT SUPPORTED RC COMMANDS
   */
  printf("\nRetrieving supported RC commands\n");

  // update cached data within Snapdragon Navigator
  int update_ret = sn_update_data();
  if(update_ret != 0)
  {
    printf("  > Data failed to update; ensure Snapdragon Navigator is running.\n");
  }

  const char* cmd_name[SN_RC_NUM_CMD_TYPES];
  SnRcCommandType rc_cmd_type[SN_RC_NUM_CMD_TYPES]
    = {SN_RC_RATES_CMD,
      SN_RC_THRUST_ANGLE_CMD,
      SN_RC_ALT_HOLD_CMD,
      SN_RC_THRUST_ANGLE_GPS_HOVER_CMD,
      SN_RC_GPS_POS_HOLD_CMD,
      SN_RC_OPTIC_FLOW_POS_HOLD_CMD,
      SN_RC_VIO_POS_HOLD_CMD,
      SN_RC_ALT_HOLD_LOW_ANGLE_CMD,
      SN_RC_POS_HOLD_CMD};

  for (i=0; i < SN_RC_NUM_CMD_TYPES; ++i)
  {
    cmd_name[i] = sn_get_cmd_name(rc_cmd_type[i]);
    printf("  > Command Type: %s\n",cmd_name[i]);

    const char* units[4];
    float min[4];
    float max[4];
    unsigned int j = 0;
    for (j = 0; j < 4; ++j)
    {
      units[j] = sn_get_dimensioned_units(rc_cmd_type[i], j);
      min[j] = sn_get_min_value(rc_cmd_type[i], j);
      max[j] = sn_get_max_value(rc_cmd_type[i], j);
      printf("    cmd%d has range [%f %s, %f %s]\n", j, min[j], units[j],
          max[j], units[j]);
    }
  }
  // wait a while before next example so prints remains visible
  usleep(1e6*2);

  /*
   * CONTINUAL LOOP, COLLECTING MOST RECENT STATUS AND MEASUREMENTS FROM API
   */
  printf("\nBeginning continuous data stream...\n");
  for(loop_counter=0; ; loop_counter++)
  {
    // Update the cached data in Snapdragon Navigator. This should be done frequently enough
    // to capture changes for the desired application -- e.g. once per control loop
    int update_ret = sn_update_data();
    if(update_ret != 0)
    {
      printf("  > Data failed to update; ensure Snapdragon Navigator is running.\n");
      continue;
    }

    printf("\n------------------ SNAPDRAGON NAVIGATOR DATA [%d] ------------------\n", loop_counter);
    // Read in current mode
    SnMode mode = (SnMode) snav_data->general_status.current_mode;
    printf("Mode: %s\n", sn_get_enum_string("SnMode", mode) );

    // Read in current state of propellers
    SnPropsState props_state = (SnPropsState) snav_data->general_status.props_state;
    printf("Prop state: %s\n", sn_get_enum_string( "SnPropsState", props_state ) );

    // Get the on ground flag
    if (snav_data->general_status.on_ground == 0)
      printf("flight control thinks vehicle is NOT ON GROUND\n");
    else
      printf("flight control thinks vehicle is ON GROUND\n");

    // Get the battery voltage
    printf("voltage = %.2f V\n", snav_data->general_status.voltage);

    // Get the status of the IMU
    SnDataStatus imu_status = (SnDataStatus) snav_data->data_status.imu_0_status;
    printf("imu_status = %s\n", sn_get_enum_string( "SnDataStatus", imu_status ) );

    // If the IMU state is valid, print inertial measurements
    if (imu_status == SN_DATA_VALID)
    {
      // Get the temp of the IMU
      printf("imu_temp = %.2ff\n", snav_data->imu_0_raw.temp);

      // Get the lin acc from IMU
      printf("lin_acc = %.2f %.2f %.2f\n",
          snav_data->imu_0_compensated.lin_acc[0],
          snav_data->imu_0_compensated.lin_acc[1],
          snav_data->imu_0_compensated.lin_acc[2]);

      // Get the ang vel from IMU
      printf("ang_vel = %.2f %.2f %.2f\n",
          snav_data->imu_0_compensated.ang_vel[0],
          snav_data->imu_0_compensated.ang_vel[1],
          snav_data->imu_0_compensated.ang_vel[2]);
    }

    // Get the barometer status
    SnDataStatus baro_status = (SnDataStatus) snav_data->data_status.baro_0_status;
    printf("baro_status = %s\n", sn_get_enum_string( "SnDataStatus", baro_status ) );

    // If tkhe barometer state is valid, print pressure measurement & temp
    if (baro_status == SN_DATA_VALID)
    {
      // Get the baro data
      printf("pressure = %.3f, baro temp = %.2f\n",
          snav_data->barometer_0_raw.pressure,
          snav_data->barometer_0_raw.temp);
    }

    // Get the RC status
    SnDataStatus rc_status = (SnDataStatus) snav_data->data_status.spektrum_rc_0_status;
    printf("rc_status = %s\n", sn_get_enum_string( "SnDataStatus", rc_status ) );

    // If RC state is valid, print raw stick commands
    if (rc_status == SN_DATA_VALID)
    {
      // Get the RC data
      for (i = 0; i < snav_data->spektrum_rc_0_raw.num_channels; ++i)
      {
        printf("%u ",snav_data->spektrum_rc_0_raw.vals[i]);
      }
      printf("\n");
    }

    // Get the mag status
    SnDataStatus mag_status = (SnDataStatus) snav_data->data_status.mag_0_status;
    printf("mag_status = %s\n", sn_get_enum_string( "SnDataStatus", mag_status ) );

    // Get the GPS status
    SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
    printf("gps_status = %s\n", sn_get_enum_string( "SnDataStatus", gps_status ) );

    // Get the sonar status
    SnDataStatus sonar_status = (SnDataStatus) snav_data->data_status.sonar_0_status;
    printf("sonar_status = %s\n", sn_get_enum_string( "SnDataStatus", sonar_status ) );

    // If sonar is valid, print range
    if (sonar_status == SN_DATA_VALID)
    {
      printf("sonar range = %.2f\n", snav_data->sonar_0_raw.range);
    }

    // Get the optic flow status
    SnDataStatus optic_flow_status = (SnDataStatus) snav_data->data_status.optic_flow_0_status;
    printf("optic_flow_status = %s\n", sn_get_enum_string( "SnDataStatus", optic_flow_status ) );

    // If optic flow is valid, print sample size
    if (optic_flow_status == SN_DATA_VALID)
    {
      printf("optic_flow_sample_size = %d\n", snav_data->optic_flow_0_raw.sample_size);
    }

    // Get control loop frequency
    printf("control frequency = %f\n", snav_data->update_rates.control_loop_freq);

    // Read in current rotation estimate
    printf("rot_est = %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
        snav_data->attitude_estimate.rotation_matrix[0],
        snav_data->attitude_estimate.rotation_matrix[1],
        snav_data->attitude_estimate.rotation_matrix[2],
        snav_data->attitude_estimate.rotation_matrix[3],
        snav_data->attitude_estimate.rotation_matrix[4],
        snav_data->attitude_estimate.rotation_matrix[5],
        snav_data->attitude_estimate.rotation_matrix[6],
        snav_data->attitude_estimate.rotation_matrix[7],
        snav_data->attitude_estimate.rotation_matrix[8]);

    printf("rpy = %.2f %.2f %.2f\n",
        snav_data->attitude_estimate.roll,
        snav_data->attitude_estimate.pitch,
        snav_data->attitude_estimate.yaw);

    // Get ESC feedback
    int number_of_escs = 4;
    printf("esc_sw_versions = ");
    for (i = 0; i < number_of_escs; ++i)
    {
      printf("%d ", snav_data->version_info.esc_sw_version[i]);
    }
    printf("\n");

    printf("esc_hw_versions = ");
    for (i = 0; i < number_of_escs; ++i)
    {
      printf("%d ", snav_data->version_info.esc_hw_version[i]);
    }
    printf("\n");

    printf("esc_packet_cntr_fb = ");
    for (i = 0; i < number_of_escs; ++i)
    {
      printf("%d ", (int) snav_data->esc_raw.packet_cntr[i]);
    }
    printf("\n");

    printf("esc_rpm_fb = ");
    for (i = 0; i < number_of_escs; ++i)
    {
      printf("%d ", snav_data->esc_raw.rpm[i]);
    }
    printf("\n");

    printf("esc_power_fb = ");
    for (i = 0; i < number_of_escs; ++i)
    {
      printf("%d ", (int) snav_data->esc_raw.power[i]);
    }
    printf("\n");

    printf("esc_voltage_fb = ");
    for (i = 0; i < number_of_escs; ++i)
    {
      printf("%.2f ", snav_data->esc_raw.voltage[i]);
    }
    printf("\n");

    // Get static accel calib status
    SnCalibStatus static_accel_calib_status;
    sn_get_static_accel_calibration_status(&static_accel_calib_status);
    printf("static_accel_calib_status = %s\n", sn_get_enum_string( "SnCalibStatus", static_accel_calib_status ) );

    // Get dynamic accel calib status
    SnCalibStatus dynamic_accel_calib_status;
    sn_get_dynamic_accel_calibration_status(&dynamic_accel_calib_status);
    printf("dynamic_accel_calib_status = %s\n", sn_get_enum_string( "SnCalibStatus", dynamic_accel_calib_status ) );

    // Get thermal imu calib status
    SnCalibStatus thermal_imu_calib_status;
    sn_get_imu_thermal_calibration_status(&thermal_imu_calib_status);
    printf("thermal_imu_calib_status = %s\n", sn_get_enum_string( "SnCalibStatus", thermal_imu_calib_status ) );

    // Get optic flow cam yaw calib status
    SnCalibStatus optic_flow_cam_yaw_calib_status;
    sn_get_optic_flow_camera_yaw_calibration_status(&optic_flow_cam_yaw_calib_status);
    printf("optic_flow_cam_yaw_calib_status = %s\n", sn_get_enum_string( "SnCalibStatus", optic_flow_cam_yaw_calib_status ) );

    // Get magnetometer calib status
    SnCalibStatus mag_calib_status;
    sn_get_magnetometer_calibration_status(&mag_calib_status);
    printf("mag_calib_status = %s\n", sn_get_enum_string( "SnCalibStatus", mag_calib_status ) );

    //Sleep until next loop
    usleep(100*1e3);
  }

  return 0;
}
