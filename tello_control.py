from djitellopy import Tello
from controllers import PID, PDController
import time
import math

# Constants
MAX_VELOCITY = 100   # Tello's maximum velocity (cm/s)
WAYPOINT_THRESHOLD = 5  # cm
CONTROL_LOOP_INTERVAL = 0.1  # seconds

# Instantiate controllers with tuned parameters
pid_x = PID(kp=0.5, ki=0.01, kd=0.2, dt=CONTROL_LOOP_INTERVAL)
pid_y = PID(kp=0.5, ki=0.01, kd=0.2, dt=CONTROL_LOOP_INTERVAL)
pid_z = PID(kp=0.8, ki=0.02, kd=0.3, dt=CONTROL_LOOP_INTERVAL)  # More aggressive for altitude
pd_yaw = PDController(kp=1.2, kd=0.15)

# Relative waypoints (Δx, Δy, Δz, Δyaw) in cm and degrees
relative_waypoints = [
    (0, 0, 100, 0),      # Takeoff and ascend 100cm
    (100, 100, 0, 90),   # Move 100cm right and 100cm forward, rotate 90°
    (100, -50, 50, 90),  # From current position: move 100cm right, -50cm forward, ascend 50cm
    (0, 0, -100, 0)      # Descend 100cm to land
]

def get_position_from_optitrack():
    """Replace with actual OptiTrack position reading"""
    # Should return (x, y, z) in cm
    return (0, 0, 0)  # dummy values

def get_current_yaw():
    """Replace with actual OptiTrack yaw reading"""
    # Should return yaw in degrees
    return 0  # dummy value

# Initialize Tello
tello = Tello()
try:
    tello.connect()
    tello.streamoff()  # Disable video stream
    print(f"Battery: {tello.get_battery()}%")
    
    # Takeoff
    tello.takeoff()
    time.sleep(2)  # Stabilize

    # Get initial position from OptiTrack after takeoff
    initial_position = get_position_from_optitrack()
    initial_yaw = get_current_yaw()
    current_absolute_target = list(initial_position) + [initial_yaw]  # [x, y, z, yaw]

    for relative_waypoint in relative_waypoints:
        # Calculate absolute target from relative waypoint
        current_absolute_target[0] += relative_waypoint[0]  # x
        current_absolute_target[1] += relative_waypoint[1]  # y
        current_absolute_target[2] += relative_waypoint[2]  # z
        current_absolute_target[3] += relative_waypoint[3]  # yaw
        
        desired_position = current_absolute_target[:3]
        desired_yaw = current_absolute_target[3]
        
        print(f"Navigating to relative waypoint: {relative_waypoint}")
        print(f"Absolute target: {desired_position} with yaw: {desired_yaw}°")

        while True:
            # Get current state from OptiTrack
            current_position = get_position_from_optitrack()
            current_yaw = get_current_yaw()

            # Calculate distance to target for waypoint checking
            error_x = desired_position[0] - current_position[0]
            error_y = desired_position[1] - current_position[1]
            error_z = desired_position[2] - current_position[2]
            distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

            # Check if waypoint reached
            if distance < WAYPOINT_THRESHOLD:
                print(f"Reached waypoint: {desired_position}")
                time.sleep(1)  # Pause briefly at waypoint
                break

            # PID Position Control
            left_right_velocity = pid_x.update(desired_position[0], current_position[0])
            forward_backward_velocity = pid_y.update(desired_position[1], current_position[1])
            up_down_velocity = pid_z.update(desired_position[2], current_position[2])
            
            # PD Yaw Control
            yaw_velocity = pd_yaw.update(desired_yaw, current_yaw)

            # Clamp Tello velocities
            left_right_velocity = max(min(left_right_velocity, MAX_VELOCITY), -MAX_VELOCITY)
            forward_backward_velocity = max(min(forward_backward_velocity, MAX_VELOCITY), -MAX_VELOCITY)
            up_down_velocity = max(min(up_down_velocity, MAX_VELOCITY), -MAX_VELOCITY)
            yaw_velocity = max(min(yaw_velocity, MAX_VELOCITY), -MAX_VELOCITY)

            # Send RC command
            tello.send_rc_control(
                int(left_right_velocity),
                int(forward_backward_velocity),
                int(up_down_velocity),
                int(yaw_velocity)
            )

            time.sleep(CONTROL_LOOP_INTERVAL)

    # Land
    tello.land()

except Exception as e:
    print(f"Error occurred: {str(e)}")
    tello.emergency()  # Trigger emergency stop if needed

finally:
    tello.end()