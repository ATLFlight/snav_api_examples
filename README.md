# Snapdragon Navigator<sup>TM</sup> Example API Programs

This repo contains a number of example programs for interfacing with the Qualcomm Snapdragon Navigator<sup>TM</sup> API. This assumes you have access to the appropriate [Qualcomm Snapdragon Flight](https://shop.intrinsyc.com/collections/product-development-kits/products/qualcomm-snapdragon-flight-sbc) development hardware as well as the latest version of Snapdragon Navigator<sup>TM</sup>.

## Installation
1. Obtain the most recent version of Snapdragon Navigator<sup>TM</sup> from the [Qualcomm Developer Network site](https://developer.qualcomm.com/hardware/snapdragon-flight/sd-navigator) and install the package on your device using the instructions in the [User Guide](https://developer.qualcomm.com/download/snapdragon-flight/navigator-user-guide.pdf).
2.  Clone this repo: `git clone https://github.com/ATLFlight/snav_api_examples.git`
3.  If appropriate, copy the repo to your device. If you cloned the repo on your device itself, you can skip this step. If you cloned the repo on a host machine, copy the files over using a USB cable and `adb`.  For example, from this repo's root directory run:
  1. `adb shell "mkdir -p /home/linaro/examples"`
  2. `adb push . /home/linaro/examples`
4.  Login to your device either through `adb` (`adb shell`) or via the network (e.g. `ssh username@deviceip`) and navigate to your examples directory. In our case, this is `/home/linaro/examples`
5.  To compile the example programs, run `make`. You should see a series of compilation lines, e.g. `gcc -Wall -I/usr/include/snav ...` for each example program.
6.  The example binaries are placed in the `build` subdirectory. Each is described in more detail below.


## Running the Examples
To run an example, navigate to the `build` directory. Here, the binaries can be executed directly--e.g. `./snav_read_attitude`. Each example is described in detail below:

* `snav_read_attitude`:
  * Usage: `./snav_read_attitude`
  * Result: this prints the roll, pitch, and yaw angles (in radians) as estimated by Snapdragon Navigator<sup>TM</sup>.
* `snav_read_gps_raw`:
  * Usage: `./snav_read_gps_raw`
  * Result: When a GPS unit is attached and a fix has been acquired, this prints timing, location, altitude, and velocity measurements.
* `snav_read_sonar`:
  * Usage: `./snav_read_sonar`
  * Result: This prints the raw sonar range measurement in meters.
* `snav_send_esc_commands`
  * **Warning**: This will spin the motors, if attached. Please use with caution.
  * Usage: `./snav_send_esc_commands`
  * Result: This will set the ESC LED colors and spin each motor to a low RPM in round-robin fashion.
* `snav_send_esc_commands_keybaord`
  * Usage: `./snav_send_esc_commands_keybaord [mode]`
    * `[mode]` is one of `-r` for RPM mode or `-p` for PWM mode.
    * Once running, the keybindings are:
      * `=` : increase PWM/RPM
      * `-` : decrease PWM/RPM
      * `]` : increase PWM by 5
      * `[` : decrease PWM by 5
      * `Q` : quit
* `snav_send_led_colors`:
  * Usage: `./snav_send_led_colors`
  * Result: sets the ESC LEDs to a few different colors
* `snav_test_receive_data`:
  * Usage: `./snav_test_receive_data`
  * Result: Initially prints command min and max values, then loops continuously printing a number of different params reported by Snapdragon Navigator<sup>TM</sup>.


## FAQ

### Why isn't the attitude estimate (roll, pitch, yaw angles) printing out?

Attitude estimation requires a calibrated IMU. You need to run
the on-ground accelerometer calibration before attempting to read the attitude
for the first time. Refer to the Snapdragon Navigator<sup>TM</sup> [User Guide](https://developer.qualcomm.com/download/snapdragon-flight/navigator-user-guide.pdf)
for more information on sensor calibration procedures.

## Contributing
Contributing is encouraged! In order to contribute, we recommend the following procedure:
1.  Fork this repo
2.  Create a new branch, e.g. `git checkout -b feature/new_feature`
3.  Commit and push changes to your forked repo:
  1. `git add` each changed file / folder
  1. `git commit -m "commit message"`
  2. `git push origin feature/new_feature`
4. Submit a pull request to initiate code review.
