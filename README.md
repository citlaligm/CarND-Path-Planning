# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./


## Reflection
The car was able to drive in the simulatori without any incident with other cars, it drove at the specified speed limit 50 MPH and when it needed to change lane it did it fastly and safely. 

The first step I took was to make the car moving from the start position and try to achieve the limit speed. In order to do that, I used the previous points the simulator gives of the car in the its previous position. However at the beggining there is not enough values so I added 2 points using the current state of the car (position in x and y and the car current angle) to calculate the x and y position in the correct orientation at the last step. In the other case where there are already more previous points (the car is already moving) then I took the last 2 previous points and calculate the x and y value in the correct orientation. In either case I added this points to a list of point that I used to create the next path. Then using the car position in frenet coordinates I generated 3 more points using the "s" value at 30 mt, at 60mts and 90mts ahead and with "d" value at the current lane (converted to meters, considering the width of the lane). I converted this 3 values to XY system using the function provided from Udacity.

Then I transformed these points from the map coordinates to local car coodinates, so that the last point of the previous path is at the origin and the angle is at zero degrees. I fed these new points to the function "set_points" from the spline library to creates the line that fits all the generated points. 

Then to create the future trajectory that the car is going to follow I used all the previous points from the simulator that are saved in the variable "previous_path_x" and "previous_path_y", these points are the points that the car didn't traveled in the last iteration. These points are added to the variable  "next_x_vals" and "next_y_vals" respectively. I do this to avoid create the each time also this help to have continuity. Then I generate any number of points missing to always have 50 points in my trajectory. 

The missing points are calculated as follows:
We  want to calculate in how many points we want to split the trajectory so that car goes at the desired speed. If we linearize our system we can draw it as in figure 1.


Where:
N is the number of pieces we want to split the line.
Distance is the target distance in this case is equal to 30m
Velocity is the desired velocity.
NOTE: the car will visit a point every 0.02 seconds

So we have the following expression:


So if 








In order to make smooth transtions I used the spline library, which functionality is transform a set of points into a smooth line that passes for all the points, which is what we want for the car to move, follow a smooth line.


The approach used to solve this project is as follow. First I created two classes one for vehicles and one for the road. I did this so that all the relevant information for the cars and roads was easy to accesses. From the simulator data I initialized the AV and from the data sensor fusion data I initialized all the other cars which I called boring_car, then I collected in a map where the key is the Id of the car. 

Once all the data was collected I checked if there was a car in front and also checked if the distance in between was less than 30m which in a real situation is can of close. If this condition was met I raised I flag to acknowledge that there is some danger ahead and that I should evaluate the possible next move. I also keep the current speed of the car ahead just in case I need to stay there and I can adjust my speed in order not to hit it.

The next step was to evaluate the possible lanes that I could move, this means if the AV is in the middle lane it could turn to the left or to the right but in the case that the AV is in most left lane then it is not possible to turn left again since there is only 3 lanes in the correct direction. In a similar way, if the AV is the most right lane, it cannot turn right since it would be out of the road. This check takes care of driving within the legality.

Once we have stablish the possible lanes, it is time to ranked them, according to the cost that each possible move implies. I consider 4 different cost functions:
   - Cost of change lane: For comfort reason is preferable to stay in the current lane if possible, this is desirable to make the ride more enjoyable for passengers. So I penalized changing lane.
   - Cost of speeding: For safety reasons, it is better to drive at the speed of the current traffic, even if the traffic speed is less that the desired speed.
   - Cost of collision: One of the most important things in driving is avoid collision with other objects. For that reason if one state is probable to end up in collision then this state is heavily penalized.
   - Cost of buffer distance: For safety reasons it is desired to have a buffer distance which will try to keep the AV far from other vehicles.   
   
Once I calculated the costs of each possible next state, I took the one is minimal since means is the safest, most enjoyable and legal action to take.

If the next best state is to stay in the same lane the AV is, then I just adjust the current speed to the speed of the current trafic or the speed of the car infront. If for the contrary the AV needs to change lane, I set the lane to the lane choose in the step before.








## Video
https://youtu.be/E296pVzWB-k
