# airsat-obc
University of Surrey AirSat On Board Computer (OBC)

# Requirements
- Raspberry Pi 3+ with SPI and I2C enabled
- Wiring Pi
- CMake

# Installation
To install on a Raspberry Pi:

- If it's not already, enable the I2C and SPI interfaces on the Raspberry Pi. See the internet for instructions on this for your version of Raspbian.
  - Probably: `sudo raspi-config`
- Pick a location on the Raspberry Pi to run it from
- `git clone https://github.com/rcjcooke/airsat-obc`
- Install WiringPi if it's not already:
  - Get the latest version link from here: 
  - Download it on the Raspberry Pi:https://github.com/WiringPi/WiringPi/releases
    - `wget https://github.com/WiringPi/WiringPi/releases/download/3.18/wiringpi_3.18_arm64.deb`
  - Install it
    - `sudo apt install ./wiringpi_3.18_arm64.deb`
      - Note: To avoid _apt user access errors, install this from /tmp
- Install CMake if it's not there already
  - `sudo apt-get install cmake`
- Change to the Software directory within the OBC directory you put it in
  - e.g. `cd ~/obc/Software`
- Build the OBC
  - `mkdir -p build && cd build && cmake .. && make -j$(nproc)`

## Optional
To set the OBC to start automatically when the Raspberry Pi starts up (highly recommended!):

- Copy the `obc.service` file to `/lib/systemd/system/`. 
  - Note: You'll need to change the `ExecStart` path to reflect the location you've installed the obc.
- Run `sudo systemctl daemon-reload` to get the system to find it
- Run `sudo systemctl enable obc.service` to enable it.
- Run `sudo reboot` to restart the system and test that it did in fact start up on boot!

# Usage
Run it from the `Software/build/` directory.

To run normally, just run `./OBC`.

Command line options:

- `--skip-cal` - Skips the calibration phase during startup

If you've set it up to run as a service, then you can view the logs at `Software/obc.log`. e.g. `tail -f ~/obc/Software/obc.log`

