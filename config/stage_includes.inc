# http://answers.ros.org/question/34661/prolbems-with-stage-pioneer-and-laserrangers-on-fuerte/
# https://player-stage-manual.readthedocs.io/en/latest/WORLDFILES/
define pioneer_base position
(
  color "red"      # Default color.
  drive "diff"       # Differential steering model.
  gui_nose 1                    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1             # Can hit things.
  ranger_return 0.500              # reflects sonar beams
  blob_return 1                 # Seen by blobfinders
  fiducial_return 1             # Seen as "1" fiducial finders

  localization "gps"
  localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"             # Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]     # Odometry error or slip in X, Y and Theta
                                     # (Uniform random distribution)

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)

define pioneer2dx_base pioneer_base
(
  # actual size
  size [0.440 0.380 0.220] # sizes from MobileRobots' web site

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.040 0.000 0.000 0.000]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 23.0
  # differential steering model
  drive "diff"
 )

# as above, but with front sonar only
define pioneer2dx pioneer2dx_base
(
  # simplified Body shape:
  block(
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )
)

define sickcommon ranger
(
  sensor (
    # factory settings for LMS200
    range [ 0.0 8.0 ]
    pose [ 0 0 0.1 0 ]
    fov 180
    samples 180
    #samples 90 # still useful but much faster to compute
    color_rgba [ 0 0 1 0.15 ]
  )
  color "blue"
  size [ 0.156 0.155 0.190 ] # dimensions from LMS200 data sheet
)

# extends sicklaser to add nice-looking but relatively expensive geometry
define fancysick sickcommon
(
  # bottom
  block(
    points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0 0.02 ]
  )

  # back
  block(
    points 4
    point[0] [ -0.078 -0.077 ]
    point[1] [ -0.02  -0.077 ]
    point[2] [ -0.02   0.077 ]
    point[3] [ -0.078  0.077 ]
    z [0 0.21 ]
  )

  # top
  block( points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0.12 0.21 ]
  )

  # laser bit
  block( points 4
    point[0] [ -0.02 -0.05 ]
   point[1] [  0.06 -0.05 ]
   point[2] [  0.06  0.05 ]
   point[3] [ -0.02  0.05 ]
   z [0.02 0.12 ]
   color "gray10"
  )
)

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # no bounding box
  boundary 0

  gui_nose 0
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  #~ laser_return 1
)

