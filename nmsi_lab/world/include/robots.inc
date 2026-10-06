include "include/pioneer2dx.inc"

define hokuyolaser ranger
(
  sensor(       
    # laser-specific properties
    # factory settings for LMS200  
    pose [ 0 0 0 0 ]
    size [ 0.07 0.07 0.05 ]
    range [ 0.0  20.0 ]
    fov 360.0
    samples 1000
    color_rgba [ 0 0 1 0.15 ]
  )
  model
  (
    # generic model properties
    pose [ 0.0 0.0 -0.2 0.0 ]
    size [ 0.07 0.07 0.05 ]  
    color "blue"
  )
)

define qrcodereader fiducial
(
  fiducial_key 1 # fiducial_key values of model and fiducial detector must match

  # fiducial properties
  range_min 0.0
  range_max 8.0
  range_max_id 5.0
  fov 270.0
  ignore_zloc 1  #  When set to 1, the fiducial finder ignores the z component when checking a fiducial

  model
  (
    # model properties
    size [ 0.1 0.1 0.1 ]
    color "green"
  )
)

define pioneer2dx_with_laser pioneer2dx_no_sonar 
(
  # can refer to the robot by this name
  hokuyolaser(pose [ 0.15 0 0 0 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0.0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )
)

define pioneer2dx_with_laser_and_fiducial pioneer2dx_no_sonar 
(
  # can refer to the robot by this name
  hokuyolaser(pose [ 0.15 0 0 0 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0.0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )

  fiducial
  (
    # fiducial properties
    range_min 0.0
    range_max 8.0
    range_max_id 5.0
    fov 270
    ignore_zloc 1

    pose [ -0.1 0.0 0.1 0.0 ]
    model
    (
      # generic model properties
      pose [ 0.0 0.0 0 -0.05 ]
      size [ 0.05 0.05 0.1]  
      color "RoyalBlue"
    )
    alwayson 1 
  )
)

define pioneer2dx_with_laser_and_camera pioneer2dx_with_laser 
(
  # can refer to the robot by this name
  mount( pose [ 0 -0.15 0 0.000 ] color "gray")
  mount( pose [ 0  0.15 0 0.000 ] color "gray")
)


define pioneer2dx_with_two_laser pioneer2dx_no_sonar 
(
  # can refer to the robot by this name
  hokuyolaser(pose [ 0.15 0.15 0 45 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )  
  hokuyolaser(pose [ -0.20 -0.15 0 -135 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0.0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )
)
