# System imports
import time
import sys

# Core Logic Imports
from Core_Logic import MazeGrid, MouseState, PIDController

# Hardware Drivers Imports
from Hardware_Drivers import encoders, motors, sensors, imu
from gpiozero.pins.lgpio import LGPIOFactory #NEW

Device.pin_factory=LGPIOFactory() #new 



# NAVIGATION
def main():

    #Test Mode
    test_mode  = False 

    # Controller initialization
    drive_pid = PIDController(kp=1.8, ki=0.01, kd=0.1)
    turn_pid = PIDController(kp=2.5, ki=0.0, kd=0.5) # needs tuning for turning in place
    
    # Maze initialization
    maze = MazeGrid()
    
    # cardinal directions initialization
    directions = ['N','E','S','W']
    
    # position initialization
    x,y  = 0, 0 
    
    # heading initialization # 0:N, 1:E, 2:S, 3:W
    heading = 0     
    
    # Target angle initialization
    target_angle = 0.0    
    
    # traveled cell initialization
    cells_traveled = 0
    
    # loop duration counter initialization
    last_time = time.perf_counter() # perf_counter() returns a high-resolution timer value
    
    # Sensor Initializaton
    sensorlist = sensors.map_and_initialize_vl53() # returns a list
    print('Sensors initializing... ') 
    if sensorlist is None: # verifies sensor list.
        print("Aborting due to incomplete Vl53 sensor initialization.")
        safe_shutdown()
        sys.exit(1)

    # state initialization
    current_state = MouseState.wait_signal_standby
    previous_state = None
    
    # signal duration counter initialization
    signal_start_time = None # keep track of signal duration
    
    try:
            # CONTROL LOOP
        while True: 
            
            maze.visited[(x,y)] = True
            
            # MAP SENSORS - 0:N, 1:E, 2:S, 3:W
            f_dir, r_dir, l_dir = sensors.sensor_mapping(directions, heading)

            now = time.perf_counter()
            dt = now - last_time
            
            gyro_z = imu.gyro[2]
            current_angle += gyro_z * dt
            error = target_angle - current_angle

            # Emergency Fault Check
            #  checking if motors are being told to move but encoder reading is not changing (no wheel spin)
            if current_state in [MouseState.moving_forward, MouseState.turning]:
                if encoders.check_stall(motors.pwmA.value *100): 
                    current_state = MouseState.fault

            # READ SENSORS
            distances = []
            for idx, s in enumerate(sensorlist):
                try:
                    d = s.range
                    distances.append(d)
                except Exception as e:
                    distances.append(None)
                    print(f"Read error S{idx+1}: {e}")
            
            # FSM
            match current_state:
                    case MouseState.wait_signal_standby: #wait_signal_standby = 1
                        # Keep motors off
                        motors.stop_motors()

                        # Check front sensor ( index 1) for hand signal less than 30mm 
                        front_dist = distances[1]
                        
                        if front_dist is not None and front_dist < 30:
                            if signal_start_time is None: 
                                signal_start_time = now
                                print("Hand detected! Hold for 3 seconds...")
                            elif now - signal_start_time >= 3.0: # if elapsed time is 3.0 seconds
                                print("Start signal confirmed! Beginning maze search...")
                                
                                # State transition
                                current_state = MouseState.search_goal
                                #previous_state = MouseState.wait_signal_standby
                                # Reset for later use
                                signal_start_time = None 
                        else: 
                            if signal_start_time is not None: 
                                print("Hand removed early. Start aborted.")
                                signal_start_time = None                            
                    
                    case MouseState.search_goal: 
                        #1-stop motors at center of current cell
                        motors.stop_motors()
                        #2-read and translate sensor readings into true or false (binarization)
                        walls = sensors.binarize_sensors(distances)

                        #3-update the virtual maze
                        maze.update_wall(x,y,f_dir,walls['F'])
                        maze.update_wall(x,y,r_dir,walls['R'])
                        maze.update_wall(x,y,l_dir,walls['L'])

                        #4-check if goal is reached!
                        if (x,y) in maze.goals:
                            print("Goal reached! Returning to start or exiting")
                            
                            # Test Mode 
                            if test_mode:
                                exit(1)
                            
                            # re-define the goal to return to the start cell if the goal is reached.
                            maze.goals = [(0,0)]

                            # then you transition states
                            current_state = MouseState.return_to_start
                            
                            continue
                        
                        #5-call floodfill
                        maze.flood_fill(speed_run_mode=False)
                        
                        #6-decide the best cell for next move
                        best_dir = maze.get_best_direction(x,y, heading)
                        
                        #7-execute movement
                        if best_dir == heading:
                            # go straight. Reset encoder to measure the next cell. 
                            encoders.enc_l.steps = 0
                            
                            # Reset angles to eliminate drift
                            current_angle =0.0
                            target_angle = 0.0

                            # Save Context 
                            previous_state = MouseState.search_goal
                            
                            # transition
                            current_state = MouseState.moving_forward
                        else: 
                            #calculate turn, set target, and reset gyro angle reference
                            turn_degrees = maze.calculate_turn_angle(heading, best_dir)
                            
                            # if current angle is 0, we need to turn right, target is 90
                            target_angle = current_angle + turn_degrees

                            #update logical heading
                            heading = best_dir

                            # Save Context 
                            previous_state = MouseState.search_goal
                            current_state = MouseState.turning
                    
                    case MouseState.moving_forward:
                        # drive straight using PID until encoder reaches 1 cell distance of 180 mm.
                        distance_traveled_mm = encoders.calculate_mm_from_encoder(encoders.enc_l.steps) 
                        
                        if distance_traveled_mm >= 180: # 180mm = 1 cell
                            motors.stop_motors()
                            x, y = maze.update_xy(x,y, heading) # update x,y coordinates
                            cells_traveled += 1
                            if previous_state == MouseState.search_goal:
                                current_state = MouseState.search_goal

                            elif previous_state == MouseState.return_to_start:
                                current_state = MouseState.return_to_start
                        else:
                            correction = drive_pid.update(error, dt)
                            motors.set_speeds(30-correction, 30+correction)
                    
                    case MouseState.turning:
                        #if withing 2 degress of the target, stop!
                        if abs(error) < 2.0:
                            motors.stop_motors()

                            # Turn complete. Now drive forward into the new cell.
                            encoders.enc_l.steps =0
                            current_state = MouseState.moving_forward
                        else: 
                            # spin in place using the turn pid
                            spin_power = turn_pid.update(error, dt)

                            # opposing power to spin. right turns = positive power.
                            motors.set_motor_direction(spin_power, -spin_power)

                    case MouseState.return_to_start:  

                        #1-stop motors at center of current cell
                        motors.stop_motors()    

                        #2-read and translate sensor readings into true or false (binarization)
                        walls = sensors.binarize_sensors(distances)
                        #3-update the virtual maze
                        maze.update_wall(x,y,f_dir,walls['F'])
                        maze.update_wall(x,y,r_dir,walls['R'])
                        maze.update_wall(x,y,l_dir,walls['L'])
                        #4-check if goal is reached!
                        if (x,y) in maze.goals:
                            print("Goal reached!")
                            # re-define the goal to return to the goal cells once back to start.
                            maze.goals = [(7, 7), (7, 8), (8, 7), (8, 8)]
                            # transition states
                            current_state = MouseState.speed_run_moving
                            # flood fill
                            maze.flood_fill(speed_run_mode= True)
                            continue
                        #5-call floodfill
                        maze.flood_fill(speed_run_mode= False)
                        #6-decide the best cell for next move
                        best_dir = maze.get_best_direction(x,y, heading)
                        #6-execute movement
                        if best_dir == heading:
                            # go straight. Reset encoder to measure the next cell. 
                            encoders.enc_l.steps = 0
                            # Reset angles to eliminate drift
                            current_angle =0.0
                            target_angle = 0.0
                            current_state = MouseState.moving_forward
                        else: 
                            #calculate turn, set target, and reset gyro angle reference
                            turn_degrees = maze.calculate_turn_angle(heading, best_dir)
                            
                            # if current angle is 0, we need to turn right, target is 90
                            target_angle = current_angle + turn_degrees

                            #update logical heading
                            heading = best_dir
                            # state transition
                            current_state = MouseState.turning 

                    case MouseState.speed_run_moving: 
                        # Path Generation
                        path = maze.get_shortest_path()



                    #case MouseState.speed_run_turning:
                        

                    case MouseState.fault:     # Fault = 5
                        print("Critical Fault: Motor Stall Detected. Manual Reset Required.")
                        time.sleep(1) # Loop infiniely until PI is turned off.
                    #case 6: back_align = 6 must be hardware compatible.   
 
            last_time = now
            time.sleep(0.01)

    except KeyboardInterrupt:
        motors.stop_motors()
        motors.STBY.off()

def safe_shutdown(): #Stop motors, stop PWM (guarded), cleanup GPIO.
    try:
        motors.stop_motors()
    except Exception:
        pass
    try:
        motors.pwmA.stop()
    except Exception:
        pass
    try:
        motors.pwmB.stop()
    except Exception:
        pass
if __name__ == "__main__":
    main()
