# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. I was provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Files
`main.cpp` - connects code to simulator, path generator, etc.
`vehicle.cpp` - Vehicle class, stores info about vehicles, adaptive cruise control, check lanes
`cost.cpp` - computes cost of potential next states and jerk minimized trajectories (JMT)
`constants.h` - contains constants defined by requirements
`spline.h` - creates spline for path  

---

## Writeup (Rubric Points)

1. The car is able to drive at least 4.32 miles without incident.

The car is able to drive at least 4.32 miles without an incident. However, there are edge cases where the vehicle is subjected to other vehicles with unsafe maneuvers which could potentially result in a crash. For example, I have seen aggressive lane changes from other vehicles merging into the ego vehicles lane imediately in front of ego vehicle. This can be mitigated if the lanes next to the ego vehicle are available for a lane change but that might not be the case everytime. If the lanes are not available for an ego vehicle lane change, I implemented an emergency braking scenario to apply the maximum deceleration allowed if this event does occur.

2. The car drives according to the speed limit.

The maximum speed is set to 49 mph to ensure that the vehicle will not exceed 50 mph. The 1 mph "cushion" is there for the vehicle to be able to accelerate at the maximum allowed limit while going 49 mph.

3. Max acceleration and jerk are not exceeded.

The maximum acceleration and jerk is not exceeded as I have set the acceleration slightly below the actual maximum value and a Jerk Minimizing Trajectory function was implemented to not exceed the jerk limits while changing lanes.

4. Car does not have collisions.

The car does not collide with other vehicles in resonable situations. Edge cases such as the one mentioned above in (1) may force a collision with the ego vehicle at the fault of the other car.

5. The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

I implemented the JMT function to handle lane changes which requires the vehicle to get into the next lane in under 3 seconds while also satisfying the jerk requirements. If the vehicle realizes that the cost of staying in the new lane is higher than changing to another lane immediately after completing a lane change, then the vehicle may look as if it has been outside of a lane for more than 3 seconds. However, it is simply moving to the lane with the lowest 'cost'.

6. The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

To ensure an overall smooth ride, I wanted to limit the amount of lane changes performed. So, instead of constantly changing lanes to the one with the lowest cost at any given time, the vehicle will only look to change lanes once it has encountered a vehicle within 35m in front of the ego vehicle. Then, it checks the available lanes to see if there are any vehicles coming from behind, in front, and calculates the speed of each vehicle to help compute the cost of changing into that lane. Once the vehicle decides to change lanes, it will compute the trajectory of this maneuver using the JMT function.

## Reflection: Generating the path

In order to generate a path using the data from the vehicle, simulator, and sensors, I found the spline function to be quite useful. To utililize the spline function I first had to get at least three points to create a spline. These points were generated as a list of waypoints from the map data that were evenly spaced 30m apart in addition to the vehicles current position. This gave me enough points to create a spline.

In order to make this spline align with the vehicle, I calculated a line tangent to the vehicles previous path using the paths 2 previous points. If the vehicle had no previous points I simply calculated points that would create a line tangent to the car.

Using the points from the previous path as well as the waypoints, these points were set in the spline function to generate the path. However, I needed to break up this spline into data that would be understood by the simulator. To do this, I used all the points from the previous path and appended the points from the spline. I broke up the spline into a specific number of points generated by using the vehicles speed and horizon value (30m). I then divided the spline into many points which were evenly spaced with respect to the vehicles velocity. These x and y-values were sent to the simulator to move the vehicle along these points on the road.

## Future Improvements

* Use the JMT function for velocity/acceleration control during lane changes
* Incorporate robust prediction method to 'track' other vehicles to predict their potential paths, acceleration, etc.
* Add a 'buffer' state which prepares for lane changes and a waiting period to finish a lance change before starting another
* More robust/elegant adaptive cruise control method 

