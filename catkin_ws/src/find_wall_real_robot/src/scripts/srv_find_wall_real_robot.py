#!/usr/bin/env python3

import rospy
import math
from find_wall_real_robot.srv import FindWallReal, FindWallRealResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, UInt32

# ══════════════════════════════════════════════════════
# SIMULATION FLAG  —  set 1 for sim, 0 for real robot
# ══════════════════════════════════════════════════════
SIMULATION = 1

# ── Scan indexes — real robot: angle_min=0, 450 pts, CW ──
# ── Simulation:   angle_min=-π, 720 pts, CCW             ──
if SIMULATION:
    FORWARD_INDEX  = 360   # sim —   0 deg forward   (angle_min=-π, 720 pts)
    DIAGONAL_INDEX = 450   # sim —  45 deg diagonal  (right-forward)
    RIGHT_INDEX    = 540   # sim —  90 deg right
else:
    FORWARD_INDEX  = 0     # real —   0 deg forward  (angle_min=0, 450 pts)
    DIAGONAL_INDEX = 56    # real —  45 deg diagonal  (right-forward)
    RIGHT_INDEX    = 112   # real —  90 deg right

# ── Forward approach ──────────────────────────────────
if SIMULATION:
    DIAGONAL_STOP_DIST    = 0.40  # sim — robot stops cleanly, tighter threshold
    FORWARD_STOP_DIST     = 0.30  # sim — no inertia overshoot
    FORWARD_SPEED         = 0.15  # sim — full approach speed safe in sim
else:
    DIAGONAL_STOP_DIST    = 0.55  # real — extra margin for inertia
    FORWARD_STOP_DIST     = 0.45  # real — forward safety stop
    FORWARD_SPEED         = 0.12  # real — slower to control inertia

# ── Rotation tuning ───────────────────────────────────
if SIMULATION:
    ROTATION_SPEED_START     = 0.15  # sim — slower, no friction to overcome
    ROTATION_SPEED_SLOW      = 0.10  # sim — fine approach speed
    ROTATION_SLOW_ZONE       = 30    # sim — tighter slow zone, cleaner response
    ROTATION_INDEX_TOLERANCE = 10    # sim — robot lands more precisely
    ROTATION_SETTLE_RATE     = 5     # sim — shorter settle (1/5 s)
    ROTATION_MAX_ATTEMPTS    = 3
    ROTATION_MIN_DWELL       = 1     # sim — single scan sufficient, no noise
else:
    ROTATION_SPEED_START     = 0.35  # real — overcome static friction
    ROTATION_SPEED_SLOW      = 0.08  # real — minimum creep speed
    ROTATION_SLOW_ZONE       = 60    # real — wide zone for inertia
    ROTATION_INDEX_TOLERANCE = 20    # real — wider window, robot can't land precisely
    ROTATION_SETTLE_RATE     = 3     # real — longer settle (1/3 s)
    ROTATION_MAX_ATTEMPTS    = 3
    ROTATION_MIN_DWELL       = 2     # real — require 2 consecutive scans, filters noise

# ── Scan filtering ────────────────────────────────────
if SIMULATION:
    SCAN_AVERAGE_WINDOW       = 50    # sim — wider average, clean sensor
    SCAN_STD_WINDOW_REJECTION = 0.025
else:
    SCAN_AVERAGE_WINDOW       = 10    # real — narrower, LDS-01 noise
    SCAN_STD_WINDOW_REJECTION = 0.025

TRACK_WINDOW              = 60        # index search window around locked wall
CMD_VEL_PUBLISH_RATE      = 50.0      # Hz — continuous publish rate for real robot
LIDAR_RANGE_MIN           = 0.12      # LDS-01 minimum reliable range — filters zeros and floor noise

wall_found           = False
searching            = False
rotating             = False
moving               = False
wall_angle_pub       = None
wall_angle_index_pub = None
cmd_pub              = None
latest_scan          = None
latest_cmd           = Twist()   # latest cmd stored for 50 Hz timer
cmd_timer            = None      # timer handle — shutdown when wall found to hand off cmd_vel

def average_std_range(ranges, index, range_min, window=SCAN_AVERAGE_WINDOW):
    """Average and std dev of 'window' samples on each side of index.
    Filters zeros, floor noise, inf and NaN — real LDS-01 returns 0.0 for invalid readings.
    Returns (inf, inf) if 4 or more samples are inf or no valid samples remain."""
    samples   = []
    inf_count = 0
    for i in range(index - window, index + window + 1):
        val = ranges[i % len(ranges)]
        if math.isinf(val):
            inf_count += 1
            samples.append(range_min)  # replace inf with range_min
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
    rospy.loginfo("Find Wall (Real): Finding Closest wall ...")
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
    rospy.loginfo("Find Wall (Real): Closest wall found — index: %d  range: %.3f  std: %.3f  angle: %.1f deg",
                  min_index, min_range, min_std, math.degrees(angle))
    return angle, min_index

def find_wall_angle_windowed(laserscan_data, center_index):
    """Search only near center_index to track a locked wall.
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
    rospy.logdebug("Find Wall (Real): Windowed search — center: %d  best: %d  range: %.3f",
                   center_index, best_index, min_range)
    return best_index

def rotate_to_index(target_index, tolerance=ROTATION_INDEX_TOLERANCE, attempt=1):
    """
    Rotate robot until nearest wall index is within target_index +/- tolerance.
    Two-speed: fast to overcome friction, slow near target to reduce overshoot.
    On overshoot: stops, settles, restarts from current position (max ROTATION_MAX_ATTEMPTS).

    Args:
        target_index — scan index to rotate nearest wall to
        tolerance    — acceptable index window either side of target
        attempt      — current attempt number (incremented on overshoot restart)
    """
    global rotating, latest_cmd

    if attempt > ROTATION_MAX_ATTEMPTS:
        rospy.logerr("Find Wall (Real): Rotation failed after %d attempts — giving up", ROTATION_MAX_ATTEMPTS)
        latest_cmd = Twist()
        cmd_pub.publish(latest_cmd)
        return

    target_min = target_index - tolerance
    target_max = target_index + tolerance

    rospy.loginfo("Find Wall (Real): Rotating to index %d (window: %d-%d) attempt %d/%d",
                  target_index, target_min, target_max, attempt, ROTATION_MAX_ATTEMPTS)
    rate = rospy.Rate(10)

    # wait for first valid scan
    while latest_scan is None and not rospy.is_shutdown():
        rate.sleep()

    # determine and lock direction — set latest_cmd ONCE before loop
    _, initial_index = find_wall_angle(latest_scan)
    locked_index     = initial_index

    # record which side of target we start on — used for overshoot detection
    started_above = initial_index > target_max
    started_below = initial_index < target_min

    rot_cmd = Twist()
    rot_cmd.linear.x = 0
    # Sim  (CCW scan, angle_min=-π): CCW rotation (+angular.z) DECREASES index
    # Real (CW scan,  angle_min=0):  CCW rotation (-angular.z) DECREASES index
    if initial_index > target_max:
        rot_cmd.angular.z = ROTATION_SPEED_START if SIMULATION else -ROTATION_SPEED_START
        rospy.loginfo("Find Wall (Real): Rotation direction locked: index above target  initial: %d  angular.z: %.3f",
                      initial_index, rot_cmd.angular.z)
    elif initial_index < target_min:
        rot_cmd.angular.z = -ROTATION_SPEED_START if SIMULATION else ROTATION_SPEED_START
        rospy.loginfo("Find Wall (Real): Rotation direction locked: index below target  initial: %d  angular.z: %.3f",
                      initial_index, rot_cmd.angular.z)
    else:
        rospy.loginfo("Find Wall (Real): Already at target index %d — no rotation needed", target_index)
        return

    latest_cmd = rot_cmd
    cmd_pub.publish(latest_cmd)
    rospy.logdebug("Find Wall (Real): cmd_vel published — angular.z: %.3f", latest_cmd.angular.z)

    scan_count = 0
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        angle_index  = find_wall_angle_windowed(latest_scan, locked_index)
        locked_index = angle_index
        scan_count  += 1

        wall_angle_deg = math.degrees(latest_scan.angle_min + (angle_index * latest_scan.angle_increment))
        wall_angle_pub.publish(Float32(wall_angle_deg))
        wall_angle_index_pub.publish(UInt32(angle_index))

        # wrap-aware remaining — handles 0/max seam crossing in sim
        num_ranges  = len(latest_scan.ranges)
        raw_remain  = abs(angle_index - target_index)
        remaining   = min(raw_remain, num_ranges - raw_remain)
        rospy.loginfo("Find Wall (Real): scan: %d  wall_index: %d  target: %d-%d  remaining: %d  speed: %.3f rad/s  angle: %.1f deg",
                      scan_count, angle_index, target_min, target_max, remaining,
                      latest_cmd.angular.z, wall_angle_deg)

        # success — landed in window
        if target_min <= angle_index <= target_max:
            rospy.loginfo("Find Wall (Real): Target window reached after %d scans", scan_count)
            break

        # overshoot detection — wrap-aware: ignore crossings near the 0/max seam
        # threshold: only flag as overshoot if index jumped > half the scan array
        crossed_above = started_above and angle_index < target_min
        crossed_below = started_below and angle_index > target_max
        seam_crossing = raw_remain > num_ranges // 2   # large jump = wrap, not overshoot
        overshot = (crossed_above or crossed_below) and not seam_crossing

        if overshot:
            rospy.logwarn("Find Wall (Real): Overshoot detected at index %d — stopping and restarting (attempt %d)",
                          angle_index, attempt)
            latest_cmd = Twist()
            cmd_pub.publish(latest_cmd)
            rospy.Rate(ROTATION_SETTLE_RATE).sleep()
            rotate_to_index(target_index, tolerance, attempt + 1)
            return

        # slow zone — reduce speed as approaching target to reduce overshoot
        if remaining <= ROTATION_SLOW_ZONE:
            slow_cmd = Twist()
            slow_cmd.angular.z = math.copysign(ROTATION_SPEED_SLOW, rot_cmd.angular.z)
            if latest_cmd.angular.z != slow_cmd.angular.z:
                latest_cmd = slow_cmd
                cmd_pub.publish(latest_cmd)
                rospy.loginfo("Find Wall (Real): Entering slow zone — speed reduced to %.3f rad/s", ROTATION_SPEED_SLOW)
        else:
            if latest_cmd.angular.z != rot_cmd.angular.z:
                latest_cmd = rot_cmd
                cmd_pub.publish(latest_cmd)
                rospy.loginfo("Find Wall (Real): Back to full speed — %.3f rad/s", ROTATION_SPEED_START)

        rate.sleep()

    latest_cmd = Twist()
    cmd_pub.publish(latest_cmd)
    rospy.loginfo("Find Wall (Real): Stop published — settling for %.0f ms", 1000.0 / ROTATION_SETTLE_RATE)
    rospy.Rate(ROTATION_SETTLE_RATE).sleep()
    rospy.loginfo("Find Wall (Real): Rotation complete — wall at index %d  angle: %.1f deg",
                  angle_index, wall_angle_deg)

def move_to_wall_diagonal():
    """Approach wall at 45 deg diagonal — checks DIAGONAL_INDEX (405) and FORWARD_INDEX (360).
    Diagonal approach gives more stopping room than 90 deg head-on.
    Skips if already within stop distance. Publishes via 50 Hz timer."""
    global moving, latest_cmd
    rospy.loginfo("Find Wall (Real): Moving to wall at 45 deg diagonal ...")

    # check if already close enough — skip move
    if latest_scan is not None:
        diag, _ = average_std_range(latest_scan.ranges, DIAGONAL_INDEX, latest_scan.range_min)
        fwd,  _ = average_std_range(latest_scan.ranges, FORWARD_INDEX,  latest_scan.range_min)
        if (not math.isinf(diag) and diag <= DIAGONAL_STOP_DIST) or \
           (not math.isinf(fwd)  and fwd  <= FORWARD_STOP_DIST):
            rospy.loginfo("Find Wall (Real): Already within stop distance (diag: %.3f  fwd: %.3f) — skipping",
                          diag, fwd)
            return

    move_cmd = Twist()
    move_cmd.linear.x  = FORWARD_SPEED
    move_cmd.angular.z = 0
    latest_cmd = move_cmd
    cmd_pub.publish(latest_cmd)
    rospy.loginfo("Find Wall (Real): Speed: %.2f m/s  diag stop: %.2f m  fwd stop: %.2f m",
                  FORWARD_SPEED, DIAGONAL_STOP_DIST, FORWARD_STOP_DIST)

    scan_count = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        diag, _ = average_std_range(latest_scan.ranges, DIAGONAL_INDEX, latest_scan.range_min)
        fwd,  _ = average_std_range(latest_scan.ranges, FORWARD_INDEX,  latest_scan.range_min)
        scan_count += 1

        rospy.loginfo("Find Wall (Real): scan: %d  fwd: %.3f m  diag(45): %.3f m  diag_stop: %.3f  fwd_stop: %.3f",
                      scan_count, fwd, diag, DIAGONAL_STOP_DIST, FORWARD_STOP_DIST)

        if not math.isinf(diag) and diag <= DIAGONAL_STOP_DIST:
            rospy.loginfo("Find Wall (Real): Diagonal stop triggered at %.3f m after %d scans", diag, scan_count)
            break
        if not math.isinf(fwd) and fwd <= FORWARD_STOP_DIST:
            rospy.loginfo("Find Wall (Real): Forward safety stop at %.3f m after %d scans", fwd, scan_count)
            break

        rate.sleep()

    latest_cmd = Twist()
    cmd_pub.publish(latest_cmd)
    rospy.loginfo("Find Wall (Real): Stop published — settling for %.0f ms", 1000.0 / ROTATION_SETTLE_RATE)
    rospy.Rate(ROTATION_SETTLE_RATE).sleep()
    rospy.loginfo("Find Wall (Real): Wall approach complete")

def cmd_vel_timer_callback(event):
    """Publish latest cmd_vel at CMD_VEL_PUBLISH_RATE Hz — keeps real robot moving between scan updates."""
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
        rospy.loginfo("Find Wall (Real): Waiting for clock...")
        rate.sleep()

    rospy.loginfo("Find Wall (Real): Service Started ...")
    wall_found = False
    searching  = True
    rotating   = True

    rotate_to_index(FORWARD_INDEX)   # rotate nearest wall to directly ahead
    rotating = False

    moving = True
    move_to_wall_diagonal()          # drive forward toward wall — stops at DIAGONAL/FORWARD stop dist
    moving = False

    # rotating = True
    # rotate_to_index(RIGHT_INDEX)   # align wall to right side for lap — disabled
    # rotating = False               # robot starts lap at 45 deg diagonal angle

    wall_found = True
    searching  = False
    rospy.set_param('/real_wall_aligned', True)

    cmd_timer.shutdown()   # stop 50 Hz timer — hand cmd_vel control to lap server
    rospy.loginfo("Find Wall (Real): cmd_vel timer stopped — handing control to lap server")
    return FindWallRealResponse(wallfound=wall_found)

def find_wall_real_server():
    global wall_angle_pub, wall_angle_index_pub, cmd_pub, cmd_timer

    rospy.init_node('find_wall_real_robot_svc', log_level=rospy.DEBUG)

    wall_angle_pub       = rospy.Publisher('/find_wall_real_robot/wall_angle',       Float32, queue_size=1)
    wall_angle_index_pub = rospy.Publisher('/find_wall_real_robot/wall_angle_index', UInt32,  queue_size=1)
    cmd_pub              = rospy.Publisher('/cmd_vel', Twist,                         queue_size=1)

    rospy.Service('find_wall_real_robot', FindWallReal, handle_find_wall)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    cmd_timer = rospy.Timer(rospy.Duration(1.0 / CMD_VEL_PUBLISH_RATE), cmd_vel_timer_callback)

    rospy.Rate(1).sleep()
    rospy.loginfo("Find_Wall (Real): Service Ready ...")
    rospy.spin()

if __name__ == '__main__':
    find_wall_real_server()