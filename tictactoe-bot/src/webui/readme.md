ros2 launch brain simulation_real.launch.py (launch all ros2 pkg)
ros2 run rosbridge_server rosbridge_websocket
python3 -m http.server 8000

http://localhost:8000
