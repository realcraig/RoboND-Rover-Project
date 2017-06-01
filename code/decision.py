import numpy as np

def near_zero(val, delta):
    return val < delta and val > -delta


def dist_front(Rover):
    min_angle = -10 * np.pi / 180
    max_angle = 10 * np.pi / 180
    avg_dist = 0
    count = 0

    for r in Rover.nav:
        if r['angle'] >= min_angle and r['angle'] <= max_angle:
            avg_dist += r['dist']
            count += 1

    if count > 0:
        avg_dist /= count

    return avg_dist

def obstacles_in_range(Rover, range):

    return dist_front(Rover) < range

def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return x_pixel, y_pixel

def follow_wall(Rover):

    prx = 20
    pry = -10

    vrx = int(Rover.vision_image.shape[1]/2 - pry)
    vry = int(Rover.vision_image.shape[0] - prx)

    d = 0
    if Rover.vision_image[vry, vrx, 2] > 0:
        d = -1

    if Rover.vision_image[vry, vrx, 0] > 0:
        d = 1

    angle = 8 * d

    return Rover.steer * 0.6 + angle * 0.4


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    stop_dist = 20
    coast_dist = 30

    if not Rover.data_valid:
        return Rover
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    Rover.obstacle_dist = dist_front(Rover)
    velocity_scale = np.clip(Rover.obstacle_dist, 0, 100) / 100


    print("mode", Rover.mode, "Rover.steer", Rover.steer, "obstacle_dist", Rover.obstacle_dist, "velocity_scale", velocity_scale)


    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':

            # Check the extent of navigable terrain
            if not obstacles_in_range(Rover, stop_dist):

                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel * velocity_scale:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = follow_wall(Rover)

                if Rover.vel > 0.1 and Rover.obstacle_dist < coast_dist:
                    Rover.throttle = 0

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
                print ("STOPPING")
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                if Rover.brake < Rover.brake_set:
                    Rover.brake += 0.01
                Rover.steer = 0
                Rover.mode = 'stop'
                print ("STOPPING")

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                if Rover.brake < Rover.brake_set:
                    Rover.brake += 0.01
                Rover.steer = 0
                Rover.turn_dir = 1

                Rover.turn_dir = 1
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if obstacles_in_range(Rover, stop_dist):

                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = Rover.turn_dir * 15
                    Rover.mode = 'spinning'
                    print('SPINNING')
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = follow_wall(Rover)
                    Rover.mode = 'forward'
                    print ('FORWARD')

        elif Rover.mode == 'spinning':
            if not obstacles_in_range(Rover, coast_dist):
                Rover.mode = 'stop'
                print ("STOPPING")
            else:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = Rover.turn_dir * 15

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0
        Rover.steer = -15
        Rover.brake = 0

    return Rover

