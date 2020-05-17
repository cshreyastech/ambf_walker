# Import the Client from ambf_comm package
# You might have to do: pip install gym
from ambf_client import Client
import time


if __name__ == "__main__":
    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bidrectional communication 
    _client.connect()

    # # You can print the names of objects found
    print(_client.get_obj_names())

    # Lets say from the list of printed names, we want to get the 
    exoskeleton_hip_obj = _client.get_obj_handle('Hiprob')
    exoskeleton_leftshankrob_obj = _client.get_obj_handle('Leftshankrob')
    exoskeleton_rightshankrob_obj = _client.get_obj_handle('Rightshankrob')


    exoskeleton_hip_obj.set_rpy(0, 0, 0)
    exoskeleton_hip_obj.set_pos(0, 0, 0.5)

    time.sleep(10) # Sleep for a while to see the effect of the command before moving on

    neuton_m = 50

    for i in range(0, 1000):
        print(neuton_m)
        for i in range(0, 100):
            
            exoskeleton_leftshankrob_obj.set_torque(neuton_m, 0, 0)
            exoskeleton_rightshankrob_obj.set_torque(-neuton_m, 0, 0)


            
            time.sleep(0.01) # Run the loop for 10 seconds
        neuton_m = -neuton_m
        time.sleep(1)

    # Lastly to cleanup
    _client.clean_up()