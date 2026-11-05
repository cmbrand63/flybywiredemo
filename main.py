# Put the various CircuitPython import statements here
import board
import busio
import adafruit_bno055
import adafruit_bmp3xx
from digitalio import DigitalInOut
from digitalio import Direction
import pwmio
import pulseio
import time

# Configuration of the simulator (will have to remove these lines
# when running on real hardware) 
import dynamic_model
dynamic_model.enable_wind=True  # Set this to True to add a moment from the wind
dynamic_model.enable_vertical_motion=True #Set this to True to add vertical control system capability to simulation

# ***NOTE*** Do not get the various files which are part of the simulator
# (adafruit_bno055.py, busio.py, board.py, etc.) confused with similarly
# named files used for actual CircuitPython hardware. Do NOT copy the
# simulator versions onto your Feather microcontroller! 
import board

# Define and initalize pins

i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055_I2C(i2c)

rotation = pulseio.PulseIn(board.D5, maxlen = 10, idle_state = False)
forward = pulseio.PulseIn(board.D12, maxlen = 10, idle_state = False)
liftIn = pulseio.PulseIn(board.D6, maxlen = 10, idle_state = False)
left = pwmio.PWMOut(board.D9, frequency = 40, duty_cycle = 3931.5)
right = pwmio.PWMOut(board.D10, frequency = 40, duty_cycle = 3931.5)
led = pwmio.PWMOut(board.D13, frequency = 5, duty_cycle = 0)
liftOut = pwmio.PWMOut(board.D11, frequency=40, duty_cycle = 3931.5)

#Define and initialize PWM terms
integral = 0.0
previous_time = time.monotonic()        # Keep track of time for integral term

rot_pro = 0.01     
rot_der = 0.1
rot_int = 0.01

alt_pro = 0
alt_der = 0
alt_int = 0

hover_ms = 1.647
# Control Loop:
while True:
    #Rotation Control Loop:
    if len(rotation) > 0:
        rvalue = rotation.popleft()
        if rvalue < 2500:               
            cal = imu.calibrated        
            if cal == True:
                led.duty_cycle = 65535
            if cal == False:
                led.duty_cycle = 0
    
            heading = imu.euler[0]      # Extract the euler angles (Heading, Roll, Pitch)
            roll = imu.euler[1]         # in degrees from the IMU
            pitch = imu.euler[2]
            xrot = imu.gyro[0]
            yrot = imu.gyro[1]
            zrot = imu.gyro[2]
    
            hcommand = .36 * (rvalue - 1000) - 180      # Commanded orientation based on from your rotation pulse input      
    
            herror = ((hcommand - heading + 360 - 180) % 360 - 180) # Difference between commanded orientation and current orientation from sensor
        
            current_time = time.monotonic()
            dt = current_time - previous_time
            previous_time = current_time
            integral += rot_int * herror * dt
            integral = max(-0.1, min(0.1, integral))
                

            
    
            hout = rot_pro * herror + rot_der * zrot + integral
            hout = max(-0.5, min(0.5, hout)) # Bound commanded output rotation: cannot exceed 0.5 or be less than -0.5
            left.duty_cycle=(hout+1.5)*2621 # Set new duty cycle based on PID loop 
            right.duty_cycle=(hout-1.5)*-2621


    pass # Done with loop

    # Altitude Control Loop
    if len(liftIn) > 0:
        liftValue = liftIn.popleft()
        if (liftValue <= 2000):
                altitude = dynamic_model.dynamic_instance.pos[2]
                vertical_velocity = dynamic_model.dynamic_instance.vel[2]
                desired_vertical_velocity = (liftValue - 1500) * 0.002
                speed_error = desired_vertical_velocity - vertical_velocity
                lift_command = hover_ms + alt_pro * speed_error + alt_int
                
        

        



        

# Once you have the above working, you can test it out (making sure it is saved
# as "main.py" by running it with the simulator "python simulator.py"

# You can control the commanded heading by clicking in the lower-right quadrant.
# The x-y arrow in the upper-left quadrant should show you the orientation of
# your craft. You can add debugging prints as needed. Start out with a small
# proportional coefficient and increase it so that it makes the craft rotate
# at a reasonable rate. 

# You will find that the craft is extremely under-damped. This is because there is
# little air resistance to rotation.

# To get it to rapidly stabilize at the desired orientation you will need to add
# a derivative term. You can get the z axis rotation rate from the .gyro[2]
# property of the BNO055.
#
# Then you will need to define a derivative coefficient (which may have to be
# negative) represented as command_ms / (rad/sec). You can create the derivative
# by multiplying your derivative coefficient by the gyro rate and include it in the
# output rotation command.

# Once you have that working consider the case when some breeze tends to
# make the craft spontaneously rotate (set dynamic_model.enable_wind=True,
# above). To still equilibriate to the correct orientation you will need
# to add an integral term.
#
# Each time through the loop the integral gets added to it
# the integral coefficient * error * dt
# where dt is the difference in time (as measured with time.monotonic())
# from the previous loop iteration to this loop iteration. 
#  * Remember to use the TIP above when calculating the error.
#
# Every time you update the integral term you also have to bound it,
# lest it get too big and cause instability. This limit would probably
# have units of command_ms and would be a reasonably small fraction of
# the maximum command of 0.5 ms. Remember to bound it on both
# positive and negative sides. 
