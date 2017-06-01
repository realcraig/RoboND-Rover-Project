## Project: Search and Sample Return
### Craig Robinson

---


[//]: # (Image References)

[image1]: ./output/warped_threshed_navigable.jpg
[image2]: ./output/warped_threshed_obstacle.jpg
[image3]: ./output/warped_threshed_rock.jpg


### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I modified the color_thresh function to include an upper and lower bound, like this:

~~~~
def color_thresh(img, rgb_low=(160, 160, 160), rgb_high=(255,255,255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where thresholds were met
    within_thresh = (img[:,:,0] >= rgb_low[0]) \
        & (img[:,:,1] >= rgb_low[1]) \
        & (img[:,:,2] >= rgb_low[2]) \
        & (img[:,:,0] <= rgb_high[0]) \
        & (img[:,:,1] <= rgb_high[1]) \
        & (img[:,:,2] <= rgb_high[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[within_thresh] = 1

    # Return the binary image

    return color_select    
~~~~

To detect navigable terrain, I call it with these values:

~~~~
    # find the navigable terrain
    nav_low=(160,160,160)
    nav_high=(255,255,255)
~~~~


![Navigable Terrain][image1]

To detect obstacles, I call it with these values:

~~~~
    # find obstacles
    obstacle_low=(1, 0, 0)
    obstacle_high=(160,160,160)
~~~~

![Obstacles][image2]

Note the "1" in the red channel for obstacle_low. I had to add that to avoid detecting the background as an obstacle.

To detect rock samples, I call it with these values:

~~~~
    # find rocks
    sample_low=(120, 100, 0)
    sample_high=(200,180,50)
~~~~
    
![Rock Samples][image3]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

Please see output/test_mapping.m4a

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### Changes to perception.py

The perception step first defines source and destination points for the perspective transform. It them calls perspective_transform to warp the the camera view to a top-down map.

I defined a function called extract_features which takes the warped image, a set of low and high thresholds, a channel (to designate navigable terrain, rock samples or obstacles) and the Rover. This method thresholds the image with the given values, updates the Rover vison image, converts the thresholded pixels to rover-centric coordinates, converts the rover-centric coordinates to world coordinates and updates the world map. If rover pitch and roll are larger than a threshold, the data is marked as invalid and the worldmap is not updated. The rover-centric and world-centric thersholded pixel coordinates are returned.

The extract_features method is called with different threshold values for obstacles, rock samples, and navigable terrain. The rover-centric coordinates are retained from the navigable terrain call, converted to polar coordinates and stored in the Rover for use in the decision step. I also stored weights (how much blue a pixel had), but didn't end up using these.

##### Changes to decision.py

I implemented a simple wall following algorithm for the rover. I also modified the stop condition by checking a narrow band in front of the rover for navigable pixels with a average distance greater than a threshold. Finally, I modified the rover's velocity by scaling it based on the proximity of upcoming obstacles. I also added another mode to handle spinning until a path is clear. I will explain in piece.

Wall Follower

The wall follower works by updating the Rover.steer value. It is implemented in the follow\_wall() function. This function defines a point in rover coordinates out in front and slightly to the right of the rover. It converts this point to Rover.vision\_image coordinates and samples the pixel value in the Rover.vision\_image. If the pixel is blue, the function sets the steering direction to -1 (towards the right wall), if the pixel value is red, the function sets the steering direction to 1 (away from the right wall). I then set a new steering angle using 8 * the new direction (a value I arrived at empirically). Finally, I dampen the new steering value by only contributing 40% of it to the new value (and 60% from the previous value). This helps to prevent oversteering and reacting too dramatically to small ripples in the wall.

Stop Condition

I changed the stop condition to stop when an obstacle came within a certain range of the rover. I defined a function called obstacle\_in\_range(), which in turn calls dist_front(). This function looks at a narrow band of pixels directly in front of the rover (I previously sorted the pixels by angle in the perception step). I walk through the angles until I find the minium, add up the dists and count until I find the maximum, and compute the average. This could be made more efficient with a better search (rather than walk through all the angles). Note that the average distance in this narrow band is approximately 1/2 the distance to the obstacle.

Velocity Scaling

I scale the velocity of the rover based on the distance to obstacles in front of it (using the dist\_front() function described above). It seems to help a bit to slow the rover down if it is coming to a place where it will need to stop. I tried, but was unable to completely eliminate the jerkiness when it stops completely.

Spinning

I added a new rover mode, 'spinning', which handles the state where the rover is blocked by an obstacle and needs to turn in place until the path is clear. I experimented with changing the direction of the spin, but once I implemented the right wall follower, a left spin seemed to always work best. The forward condition (stop spinning) uses the same obstacles\_in\_range() function as the stop condition, but with a larger distance threshold.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

As previously mentioned, I implemented a wall following algorith to guide the rover around the environment. I initially implemented this using the world image, but thought I would get better results by using the threshold camera image instead. Having tried both, I can't say one gave better results than the other, but using the thresolded camera image was much simpler to implement.

If I get lucky with starting position (and avoid some tricky spots along the way), I can map the entire environment with 65-80% fidelity and identify all rock sample locations. However, there are a few areas I have trouble. 

The biggest of these are the small rocks in the middle of the map. I have tried a number of schemes to avoid these, but thus far have been unable to consistently navigate around them. Sometimes I get caught under an overhang, other times I smack right into them. My obstacle detection doesn't seem to work very well for these.

As mentioned ealier, stopping is still jerkier than I would like. Spending more time adjusting the breaking algorithm could improve that.

There are also certain areas where the turns are too tight to follow the wall. My spinning mechanism handles these, but sometimes it isn't very smooth.

I would like to continue attempt to improve my results in this project. Some ideas for things I want to try:

 * Improve small obstaclle avoidance by analyzing the pattern of pixel angles & distances to see if I can easily determine when a small obstacle is right in the middle. The image looks like a 'V'. I should be able to recognize this by looking at how the vector angles change.

 * Implement a reverse mode for when the rover gets really stuck

 * Implement return to start position using A\* or other path finding algorithm

 * Improve time by speeding up rover in long, straight areas

 * Improve breaking algorithm so stopping is less jerky

 * Implement sample collection


