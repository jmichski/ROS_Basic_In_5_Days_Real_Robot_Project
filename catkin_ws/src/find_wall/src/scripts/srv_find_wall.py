#!/usr/bin/env python3

import rospy
import math
from find_wall.srv import FindWall, FindWallResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, UInt32

FORWARD_INDEX             = 360 
RIGHT_INDEX               = 180
FORWARD_DISTANCE          = 0.3
FORWARD_SPEED             = 0.15
ROTATION_SPEED            = 0.15
ROTATION_INDEX_TOLERANCE  = 10
SCAN_AVERAGE_WINDOW       = 50
SCAN_STD_WINDOW_REJECTION = .025
TRACK_WINDOW              = 30   # index search window around locked wall
LIDAR_RANGE_MIN           = 0.12 # LDS-01 minimum reliable range — filters zeros and floor noise
CMD_VEL_PUBLISH_RATE      = 50.0 # Hz — continuous publish rate for real robot

wall_found     = False
searching      = False
rotating       = False
moving         = False
wall_angle_pub = None
latest_scan    = None
latest_cmd     = Twist()   # latest cmd stored for 50 Hz timer
cmd_timer      = None      # timer handle — shutdown when wall found to hand off cmd_vel

def average_std_range(ranges, index, range_min, window=SCAN_AVERAGE_WINDOW):
    """Average and std dev of 'window' samples on each side of index.
    Filters zeros, floor noise, inf and NaN — real LDS-01 returns 0.0 for invalid readings.
    Returns (inf, inf) if 4 or more samples are inf or no valid samples remain."""
    samples   = []
    inf_count = 0
    for i in range(index - window, index + window + 1):
        val = ranges[i % len(ranges)]
        if math.isinf(val):
            samples.append(range_min)  # replace inf with range_min (SIMULATION_ONLY)Lidar should never escape the box
        elif math.isnan(val):
            pass                       # should never happen
        elif val < LIDAR_RANGE_MIN:    # filter zeros and below-floor readings (real LDS-01 noise)
            samples.append(range_min)  # replace 0.0 with range_min
        else:
            samples.append(val)
    if inf_count >= 4:
        return float('inf'), float('inf')
    if not samples:
        return float('inf'), float('inf')
    mean    = sum(samples) / len(samples)
    std_dev = math.sqrt(sum((x - mean) ** 2 for x in samples) / len(samples))
    return mean, std_dev

def find_wall_angle(laserscan_data):
    """Find the closest wall angle and index."""
    rospy.loginfo(f"Find Wall: Finding Closest wall ...")
    min_range  = float('inf')
    min_std    = float('inf')
    min_index  = 0
    num_ranges = len(laserscan_data.ranges)
    for i in range(num_ranges):
        avg, std = average_std_range(laserscan_data.ranges, i, laserscan_data.range_min)
        if not math.isinf(avg) and avg < min_range and std < SCAN_STD_WINDOW_REJECTION:
            min_range = avg
            min_std   = std
            min_index = i
    angle = laserscan_data.angle_min + (min_index * laserscan_data.angle_increment)
    rospy.loginfo(f"Find Wall: Closest wall found — index: {min_index}  range: {min_range:.3f} range std: {min_std:.3f}  m  angle: {math.degrees(angle):.1f} deg")
    return angle, min_index

def find_wall_angle_windowed(laserscan_data, center_index):
    """ADDED — search only near center_index to track a locked wall.
    Prevents jumping to a different wall in symmetric environments."""
    min_range  = float('inf')
    best_index = center_index
    num_ranges = len(laserscan_data.ranges)

    for i in range(center_index - TRACK_WINDOW, center_index + TRACK_WINDOW + 1):
        idx = i % num_ranges
        avg, std = average_std_range(laserscan_data.ranges, idx, laserscan_data.range_min)
        if not math.isinf(avg) and avg < min_range and std < SCAN_STD_WINDOW_REJECTION:
            min_range  = avg
            best_index = idx
    return best_index

def rotate_to_index(target_index, tolerance=ROTATION_INDEX_TOLERANCE):
    """
    Rotate robot until nearest wall index is within target_index +/- tolerance.
    Publishes cmd_vel via 50 Hz timer — not reliant on scan rate for motor commands.

    Args:
        target_index — scan index to rotate nearest wall to (e.g. 360=forward, 180=right)
        tolerance    — acceptable index window either side of target
    """
    global rotating, latest_cmd

    target_min = target_index - tolerance
    target_max = target_index + tolerance

    rospy.loginfo("Find Wall: Rotating nearest wall to index %d (window: %d-%d)",
                  target_index, target_min, target_max)
    rate = rospy.Rate(10)

    # wait for first valid scan
    while latest_scan is None and not rospy.is_shutdown():
        rate.sleep()

    # determine and lock direction — set latest_cmd ONCE before loop
    _, initial_index = find_wall_angle(latest_scan)
    locked_index     = initial_index   # lock wall index, never search globally again

    rot_cmd = Twist()
    rot_cmd.linear.x = 0
    if initial_index > target_max:
        rot_cmd.angular.z = ROTATION_SPEED
        rospy.loginfo("Find Wall: Rotation direction locked: left")
    elif initial_index < target_min:
        rot_cmd.angular.z = -ROTATION_SPEED
        rospy.loginfo("Find Wall: Rotation direction locked: right")
    else:
        rospy.loginfo("Find Wall: Already at target index %d — no rotation needed", target_index)
        return

    latest_cmd = rot_cmd   # timer picks this up at 50 Hz
    cmd_pub.publish(latest_cmd)

    # loop only checks stop condition — direction locked in latest_cmd
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        angle_index  = find_wall_angle_windowed(latest_scan, locked_index)  # limited search tracks locked wall only
        locked_index = angle_index                                           # update for next iteration

        wall_angle_pub.publish(Float32(math.degrees(
            latest_scan.angle_min + (angle_index * latest_scan.angle_increment))))
        wall_angle_index_pub.publish(UInt32(angle_index))

        if target_min <= angle_index <= target_max:
            break
        rate.sleep()

    latest_cmd = Twist()   # zero — timer publishes stop at 50 Hz
    cmd_pub.publish(latest_cmd)
    rospy.loginfo("Find Wall: Rotation complete — wall at index %d", angle_index)

def move_to_wall():
    """Drive forward until wall is within FORWARD_DISTANCE.
    Publishes cmd_vel via 50 Hz timer — not reliant on scan rate for motor commands."""
    global moving, latest_cmd
    rospy.loginfo("Find Wall: Moving to Nearest Wall ...")

    move_cmd = Twist()
    move_cmd.linear.x  = FORWARD_SPEED
    move_cmd.angular.z = 0
    latest_cmd = move_cmd   # timer picks this up at 50 Hz
    cmd_pub.publish(latest_cmd)
 
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        fwd, std = average_std_range(latest_scan.ranges, FORWARD_INDEX, latest_scan.range_min)

        if not math.isinf(fwd) and fwd <= FORWARD_DISTANCE:
            break

        rate.sleep()

    latest_cmd = Twist()   # zero — timer publishes stop at 50 Hz
    cmd_pub.publish(latest_cmd)
  
    rospy.loginfo("Find Wall: Nearest Wall reached")

def cmd_vel_timer_callback(event):
    """Publish latest cmd_vel at CMD_VEL_PUBLISH_RATE Hz — keeps real robot moving between scan updates."""
    global cmd_pub, latest_cmd
    if cmd_pub is not None:
        cmd_pub.publish(latest_cmd)

def scan_callback(laserscan_data):
    """Store latest scan only — motion logic runs in service thread."""
    global latest_scan 
    latest_scan = laserscan_data

def handle_find_wall(req):
    global wall_found, searching, moving, rotating, cmd_timer

    rate = rospy.Rate(10)
    while rospy.Time.now() == rospy.Time(0):
        rospy.loginfo(f"Find Wall: Waiting for simulated clock...")
        rate.sleep()

    rospy.loginfo(f"Find Wall: Service Started ...")
    wall_found = False
    searching  = True
    rotating   = True

    rotate_to_index(FORWARD_INDEX)   # face wall forward
    rotating = False

    moving = True
    move_to_wall() # drive to wall
    moving = False

    rotating = True
    rotate_to_index(RIGHT_INDEX)     # rotate so wall is on right side
    rotating = False

    wall_found = True
    searching  = False
    rospy.set_param('/wall_aligned', True)

    cmd_timer.shutdown()   # stop 50 Hz timer — hand cmd_vel control to lap server 
    rospy.loginfo("Find Wall: cmd_vel timer stopped — handing control to lap server")
    return FindWallResponse(wallfound=wall_found)
    

def find_wall_server():
    global wall_angle_pub, wall_angle_index_pub, cmd_pub, latest_cmd, cmd_timer

    rospy.init_node('find_wall_svc')

    wall_angle_pub       = rospy.Publisher('/find_wall/wall_angle',       Float32, queue_size=1)
    wall_angle_index_pub = rospy.Publisher('/find_wall/wall_angle_index', UInt32,  queue_size=1)
    cmd_pub              = rospy.Publisher('/cmd_vel', Twist,              queue_size=1)

    rospy.Service('find_wall', FindWall, handle_find_wall)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    latest_cmd = Twist()
    cmd_timer = rospy.Timer(rospy.Duration(1.0 / CMD_VEL_PUBLISH_RATE), cmd_vel_timer_callback)

    rospy.Rate(1).sleep()   # interruptible 1s startup delay — won't block on ROS shutdown
    rospy.loginfo("Find_Wall: Service Ready ...")

    rospy.spin()

if __name__ == '__main__':
    find_wall_server()