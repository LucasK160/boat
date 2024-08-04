from pymavlink import mavutil
import time

class Boat:
    def __init__(self, connection_string):
        self.vehicle = mavutil.mavlink_connection(connection_string)
        self.vehicle.wait_heartbeat()
        print(f"Heartbeat from system (system {self.vehicle.target_system} component {self.vehicle.target_component})")
        self.vehicleConnected = True

    def move(self, throttle, roll):
        self.vehicle.mav.rc_channels_override_send(
            self.vehicle.target_system,    # target_system
            self.vehicle.target_component, # target_component
            roll,                          # chan1_raw
            210,                           # chan2_raw
            throttle,                      # chan3_raw
            4100,                          # chan4_raw
            510,                           # chan5_raw
            610,                           # chan6_raw
            710,                           # chan7_raw
            810                            # chan8_raw
        )

    def get_channel_value(self, channel_number):
        """
        Get the current value of the specified RC channel using pymavlink.
        
        :param channel_number: The RC channel number (1-8).
        :return: The current value of the specified RC channel.
        """
        if 1 <= channel_number <= 8:
            # Wait for the RC_CHANNELS message
            msg = self.vehicle.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
            if msg:
                return getattr(msg, f'chan{channel_number}_raw', None)
            else:
                print("No RC_CHANNELS message received.")
        else:
            raise ValueError("Channel number must be between 1 and 8")

# Create an instance of the Boat class
Titanic2 = Boat('/dev/ttyACM0')

# Move the boat
Titanic2.move(1600, 1500)

# Continuously print the value of channel 6
while True:
    try:
        channel_value = Titanic2.get_channel_value(6)
        if channel_value is not None:
            print(f"Channel 6 value: {channel_value}")
        else:
            print("Channel 6 value not received yet.")
    except Exception as e:
        print(f"Error: {e}")
    time.sleep(1)
