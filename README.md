# XJRC_Dog
## Before start up
1. Use command `cat /proc/bus/input/devices` to check the keyboard input event
2. Use `sudo chmod 777 /dev/input -R` to grant privilledges to our program

## Run a simulation
1. Make sure pipeline file parameters is already filled in `main.cpp`
2. Run program with the simlator,located at `./simulator/main.py`
## Run
1. Check the simulation result (IMPORTANT)
2. Calibrate motors
3. Make sure the safety code is added (in `./LegMotors.cpp`)
4. Run
