import numpy as np

def near_zero(val, delta):
    return val < delta and val > -delta


def dist_front(Rover):
    min_angle = -5 * np.pi / 180
    max_angle = 5 * np.pi / 180
    avg_dist = 0
    count = 0

    for r in Rover.nav:
        if r['angle'] >= min_angle and r['angle'] <= max_angle:
            avg_dist += r['dist']
            count += 1

    if count > 0:
        avg_dist /= count

    return avg_dist

def compute_steer_angle(Rover):

    if dist_front(Rover) < 20:
        return np.clip(np.mean(Rover.nav['angle'] * 180/np.pi), -15, 15)

    # find grid cell 2m to left and 2m in front of rover
    px = 25
    py = -15

    yaw_rad = Rover.yaw * np.pi / 180
    xpix_rot = px * np.cos(yaw_rad) - py * np.sin(yaw_rad)
    ypix_rot = px * np.sin(yaw_rad) + py * np.cos(yaw_rad)

    scale = 10
    lx = np.int_(Rover.pos[0] + (xpix_rot / scale))
    ly = np.int_(Rover.pos[1] + (ypix_rot / scale))

    color = 'none'
    steer = 0
    if lx >= 0 and lx < Rover.worldmap.shape[1] and ly >= 0 and ly < Rover.worldmap.shape[0]:


        redval = int(Rover.worldmap[ly, lx, 0])
        blueval = int(Rover.worldmap[ly, lx, 2])

        if redval > 0 or blueval > 0:
            steer = (redval - blueval) / max(blueval, redval)

    if steer > 0:
        color = 'red'
    elif steer < 0:
        color = 'blue'


    # Rover.worldmap[ly, lx, 2] = 100
    # Rover.worldmap[ly, lx, 1] = 100
    # Rover.worldmap[ly-1, lx, 2] = 100
    # Rover.worldmap[ly-1, lx, 1] = 100


    angle = np.clip((8 * steer), -15, 15)

    print ("Rover.pos", Rover.pos, "lx, ly", lx, ly, "pixel [", redval, 0, blueval, "]", "color", color, "steer", steer, "angle", angle)

    return Rover.steer * 0.7 + angle * 0.3


def compute_steer_angleXXX(Rover):
    # rock in the path?
    if Rover.obstacle_dist < 20:
        angle = np.clip(np.mean(Rover.nav['angle'][int(len(Rover.nav['angle'])/2):] * 180/np.pi), -15, 15)
    else:
        angle = np.clip(np.mean(Rover.nav['angle'] * 180/np.pi), -15, 15)

    angle -= 0.5

    return Rover.steer * 0.8 + angle * 0.2


def compute_steer_angleXX(Rover):

    sort = Rover.nav

    third = int(len(sort) / 3)

    right = sort[:third]
    mid = sort[third+1:2*third]
    left = sort[2*third+1:]

    avg_weight_left  = np.mean(left['weight'])
    avg_dist_left    = np.mean(left['dist'])
    avg_weight_mid   = np.mean(mid['weight'])
    avg_dist_mid     = np.mean(mid['dist'])
    avg_weight_right = np.mean(right['weight'])
    avg_dist_right   = np.mean(right['dist'])

    print("weights", avg_weight_left, avg_weight_mid, avg_weight_right)


    min_weight_group = mid['angle']

    if avg_weight_left < avg_weight_mid:
        if avg_weight_left < avg_weight_right:
            print('LEFT')
            min_weight_group = left['angle']
        else:
            print('RIGHT')
            min_weight_group = right['angle']
    else:
        if avg_weight_mid < avg_weight_right:
            print('MID')
            min_weight_group = mid['angle']
        else:
            print('RIGHT')
            min_weight_group = right['angle']

    avg_angle = np.mean(mid['angle'] * 180 / np.pi)
    clipped =  np.clip(avg_angle, -15, 15)

    print("avg_angle", avg_angle)

    return Rover.steer * 0.8 + clipped * 0.2

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    stop_dist = 10

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
            if Rover.obstacle_dist > stop_dist:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel * velocity_scale:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav['angle'] * 180/np.pi), -15, 15)
                Rover.steer = compute_steer_angle(Rover)

                if Rover.vel > 0.1 and Rover.obstacle_dist < coast_dist:
                    Rover.throttle = 0

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
                print ("STOPPING")
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                if Rover.brake < Rover.brake_set:
                    Rover.brake += 0.05
                Rover.steer = 0
                Rover.mode = 'stop'
                print ("STOPPING")

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                if Rover.brake < Rover.brake_set:
                    Rover.brake += 0.05
                Rover.steer = 0
                # if np.mean(Rover.nav_angles) < 0:
                #     Rover.turn_dir = -1
                # else:
                #     Rover.turn_dir = 1

                Rover.turn_dir = 1
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if Rover.obstacle_dist <= stop_dist:

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
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    #Rover.steer = np.clip(np.mean(Rover.nav['angle'] * 180/np.pi), -15, 15)
                    Rover.steer = compute_steer_angle(Rover)
                    Rover.mode = 'forward'
                    print ('FORWARD')

        elif Rover.mode == 'spinning':
            if Rover.obstacle_dist > coast_dist:
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

