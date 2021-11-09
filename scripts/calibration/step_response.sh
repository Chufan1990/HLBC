python3 step_response.py &

rosbag record /ctrl_cmd /vehicle_feedback &

rostopic echo /ctrl_cmd