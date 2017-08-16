[//]: # (Image References)

[image1]: ./Images/points.png
[image2]: ./Images/equation.png

## Reflection
The car was able to drive in the simulator without any incident with other cars, it drove at the specified speed limit 50 MPH and when it needed to change lane it did it fastly and safely. 

The first step I took was to make the car moving from the start position and try to achieve the limit speed. In order to do that, I used the previous points the simulator gives of the car in the its previous position. However at the beggining there is not enough values so I added 2 points using the current state of the car (position in x and y and the car current angle) to calculate the x and y position in the correct orientation at the last step. In the other case where there are already more previous points (the car is already moving) then I took the last 2 previous points and calculate the x and y value in the correct orientation. In either case I added this points to a list of point that I used to create the next path. Then using the car position in frenet coordinates I generated 3 more points using the "s" value at 30 mt, at 60mts and 90mts ahead and with "d" value at the current lane (converted to meters, considering the width of the lane). I converted this 3 values to XY system using the function provided from Udacity.


Then I used the spline library, which functionality is transform a set of points into a smooth line that passes for all the points, which is what we want for the car to move, follow a smooth line. Using this library we create a line with the 5 o or more points created in the previous step.


Then I transformed these points from the map coordinates to local car coodinates, so that the last point of the previous path is at the origin and the angle is at zero degrees. I fed these new points to the function "set_points" from the spline library to creates the line that fits all the generated points. 

Then to create the future trajectory that the car is going to follow I used all the previous points from the simulator that are saved in the variable "previous_path_x" and "previous_path_y", these points are the points that the car didn't traveled in the last iteration. These points are added to the variable  "next_x_vals" and "next_y_vals" respectively. I do this to avoid create the each time also this help to have continuity. Then I generate any number of points missing to always have 50 points in my trajectory. 

The missing points are calculated as follows:
We  want to calculate in how many points we want to split the trajectory so that car goes at the desired speed. If we linearize our system we can draw it as in figure 1.

![alt text][image1]

Where:
N is the number of pieces we want to split the line.
Velocity is the desired velocity in metes per hour.
NOTE: the car will visit a point every 0.02 seconds

So we have the following expressions:

![alt text][image2]


Using the spline library if we give it a value in x, it will return the corresponding y value. Then using Pythagoras we can obtain the hypothenouse which in this case is the "Distance value" and substituting  in the equation above we can obtain the N.

Now that we have N, we can generate the missing points:
For x is the starting x point which is zero plus the target_x_distance divided by N
For y is the corresponding y value when our spline line is evaluated at the new x.

Finally we convert these points to the map coordinates and push them in the the variable "next_x_vals" and "next_y_vals" respectively. 


Next step, I created two classes one for vehicles and one for the road. I did this so that all the relevant information for the cars and roads was easy to accesses. From the simulator data I initialized the AV and from the data sensor fusion data I initialized all the other cars which I called boring_car, then I collected in a map where the key is the Id of the car. 

Once all the data was collected I checked if there was a car in front and also checked if the distance in between was less than 30m which in a real situation is can of close. If this condition was met I raised I flag to acknowledge that there is some danger ahead and that I should evaluate the possible next move. I also keep the current speed of the car ahead just in case I need to stay there and I can adjust my speed in order not to hit it.

The next step was to evaluate the possible lanes that I could move, this means if the AV is in the middle lane it could turn to the left or to the right but in the case that the AV is in most left lane then it is not possible to turn left again since there is only 3 lanes in the correct direction. In a similar way, if the AV is the most right lane, it cannot turn right since it would be out of the road. This check takes care of driving within the legality.

Once we have stablish the possible lanes, it is time to ranked them, according to the cost that each possible move implies. I consider 4 different cost functions:
   - Cost of change lane: For comfort reason is preferable to stay in the current lane if possible, this is desirable to make the ride more enjoyable for passengers. So I penalized changing lane.
   - Cost of speeding: For safety reasons, it is better to drive at the speed of the current traffic, even if the traffic speed is less that the desired speed.
   - Cost of collision: One of the most important things in driving is avoid collision with other objects. For that reason if one state is probable to end up in collision then this state is heavily penalized.
   - Cost of buffer distance: For safety reasons it is desired to have a buffer distance which will try to keep the AV far from other vehicles.   
   
Once I calculated the costs of each possible next state, I took the one is minimal since means is the safest, most enjoyable and legal action to take.

If the next best state is to stay in the same lane the AV is, then I just adjust the current speed to the speed of the current trafic or the speed of the car infront. If for the contrary the AV needs to change lane, I set the lane to the lane choose in the step before.


Finally, in the case the car was not too close to any car, I increase the speed as long is less than the speed limit.




## Video

You can find the video here:

https://youtu.be/E296pVzWB-k
