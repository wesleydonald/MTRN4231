Hello this is my current best attempt at getting moveit working, to launch the file use the following steps.

In one terminal navigate to ```4231/4231_scripts``` and setup the robot
```bash
./setupFakeur5e.sh
```
Then when rviz opens up you need to do a couple of things. First in the MotionPlanning window (bottom left panel) open the ```Context``` 
panel and change the ```<unspecified>``` planner to ```RRTConnectkConfigDefault```. Apparently you can just change this in the code using
```move_group_interface->setPlannerId("RRTConnect");``` but I didn't really see that change anything. Next go back to planning set the 
```Goal State``` to ```test_configuration``` and hit plan and execute. This ensures the robot is within the ceiling plane constraint I have added.

Then, in another terminal navigate to ```MTRN4231/src/arm``` and run
```bash
colcon build
source install/setup.bash
ros2 launch movement arm.launch.py
```
The main code to focus on is all in ```src/arm.cpp``` where I have tried to implement orientation constraints as well as a joint constraint.
You can comment out the lines where the constraints are added in the ```set_constraint()``` function to try out different things.

