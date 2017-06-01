import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_low=(160, 160, 160), rgb_high=(255,255,255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
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

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    # Apply a rotation

    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    # Assume a scale factor of 10 between world space pixels and rover space pixels
    scale = 10
    # Perform translation and convert to integer since pixel values can't be float
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))

    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size[0] - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size[1] - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


def extract_features(img, low, high, channel, Rover):
    scale = 10

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(img, low, high)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, channel] = threshed*255.0

    # 5) Convert map image pixel values to rover-centric coords
    rx, ry = rover_coords(threshed)

    # 6) Convert rover-centric pixel values to world coordinates
    wx, wy = pix_to_world(rx, ry, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape, scale)


    # 7) Update Rover worldmap (to be displayed on right side of screen)
    max_roll = 1
    max_pitch = 1

    if Rover.roll > 180:
        Rover.roll = 360 - Rover.roll

    if Rover.pitch > 180:
        Rover.pitch = 360 - Rover.pitch

    if abs(Rover.roll) <= max_roll and abs(Rover.pitch) <= max_pitch:

        Rover.data_valid = True

        if channel == 0:
            Rover.worldmap[wy, wx, 0] += 1

        if channel == 1:
            Rover.worldmap[wy, wx, 1] += 1

        if channel == 2:
            Rover.worldmap[wy, wx, 2] += 1
            Rover.worldmap[wy, wx, 0] *= 0.9
            #Rover.worldmap[wy, wx, (channel + 2) % 3] = 0
    else:
        Rover.data_valid = False
        print ("roll/pitch threshold exceeded!", "roll", Rover.roll, "pitch", Rover.pitch)

    return rx, ry, wx, wy


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()


    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 5
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # find obstacles
    obstacle_low=(1, 0, 0)
    obstacle_high=(160,160,160)
    extract_features(warped, obstacle_low, obstacle_high, 0, Rover)

    # find the navigable terrain
    nav_low=(160,160,160)
    nav_high=(255,255,255)
    navigable_rover_x, navigable_rover_y, wx, wy = extract_features(warped, nav_low, nav_high, 2, Rover)

    # find rocks
    sample_low=(120, 100, 0)
    sample_high=(200,180,50)
    extract_features(warped, sample_low, sample_high, 1, Rover)

    # 8) Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_rover_x, navigable_rover_y)

    Rover.nav = np.zeros(len(Rover.nav_angles), dtype=Rover.nav_dtype)
    Rover.nav['angle'] = Rover.nav_angles
    Rover.nav['dist'] = Rover.nav_dists
    Rover.nav['weight'] = Rover.worldmap[wy, wx, 2] / 255

    Rover.nav = np.sort(Rover.nav, axis=-1, kind='quicksort', order='angle')


    return Rover