from pymavlink import mavutil
import time

# Function to send ATTITUDE command
def send_attitude_command(roll, pitch, yaw, thrust):
    # Create MAVLink connection (replace with your connection parameters)
    master = mavutil.mavlink_connection('127.0.0.1:5761', baud=57600)

    # Send ATTITUDE command
    msg = master.mav.set_attitude_target_send(
        0,  # Timestamp in milliseconds since system boot
        master.target_system,  # Target system
        master.target_component,  # Target component
        0b00000000,  # Type mask: bit 1 is LSB (0: body rates, 1: angular rates)
        [roll, pitch, yaw, thrust],  # Attitude quaternion (w, x, y, z) and thrust
        0,  # Body roll rate (rad/s)
        0,  # Body pitch rate (rad/s)
        0,  # Body yaw rate (rad/s)
        0)  # Target rate for those in rad/s

    print("ATTITUDE command sent.")

# Main function
def main():
    # Desired attitude (replace with your desired values)
    roll = 0.0
    pitch = -0.93
    yaw = 0.0
    thrust = 0.5  # 50% thrust

    
    # Send ATTITUDE command
    send_attitude_command(roll, pitch, yaw, thrust)
    

# Entry point of the script
if __name__ == "__main__":
    main()
