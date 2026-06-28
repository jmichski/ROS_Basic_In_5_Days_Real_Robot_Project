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

LOG_TUNING_CONSTANTS = 0
LOG_FUNCTION_CALLS = 0
LOG_FUNCTION_RETURNS = 0
LOG_TWISTS = 1
LOG_SCAN_DISTANCES = 1
LOG_LOGIC_STATES = 1
LOG_AI_DEBUGINFO = 0
LOG_DEV_DEBUGINFO = 0

# ── NODE / CLASS constants — used in every structured log line so the
# pipe-delimited output imports into Excel/CSV with consistent columns,
# matching the log_real_robot_lap_server convention (NODE | CLASS | FUNC | TAG).
# This module is function-based, not OOP — CLASS is a fixed placeholder so
# the column position lines up across both modules' logs. ─────────────────
LOG_NODE  = "find_wall_real_robot_svc"
LOG_CLASS = "Find Wall"

# ── Scan indexes — real robot: angle_min=0, 450 pts, CW ──
# ── Simulation:   angle_min=-π, 720 pts, CCW             ──
if SIMULATION:
    FORWARD_INDEX        = 360   # sim —   0 deg forward   (angle_min=-π, 720 pts)
    LEFT_INDEX           = 180   # sim — -90 deg left side  (CCW rotation decreases index)
    FINAL_ALIGN_TARGET   = 90    # sim —  stop alignment short to account for inertia
else:
    FORWARD_INDEX        = 0     # real —   0 deg forward  (angle_min=0, 450 pts)
    RIGHT_INDEX          = 112   # real —  90 deg right
    LEFT_INDEX           = 56    # real —  indices short of RIGHT_INDEX to stop early
    FINAL_ALIGN_TARGET   = 28    # real —  stop alignment short to account for inertia

# ── Forward approach ──────────────────────────────────
if SIMULATION:
    FORWARD_STOP_DIST    = 0.30  # sim — no inertia overshoot
    FORWARD_SPEED        = 0.15  # sim — full approach speed safe in sim
    YAW_SPEED            = 0.0   # sim — approach wall straight forward
else:
    FORWARD_STOP_DIST        = 0.40  # real — safety stop on forward wall
    RIGHT_ALIGN_DIST         = 0.40  # real — stop when right wall is aligned
    ALIGNMENT_TURN_DISTANCE  = 0.30  # real — when right <= this, apply YAW_SPEED
                                     # to arc robot into right-side alignment
    FORWARD_SPEED            = 0.10  # real — slower to control inertia
    YAW_SPEED                = 0.15  # real — yaw applied once ALIGNMENT_TURN_DISTANCE reached

# ── Final alignment (sim only) ───────────────────────
# Real robot uses yaw-during-approach instead of a final pivot.
if SIMULATION:
    FINAL_ALIGN_LINEAR_VEL = 0.0   # sim — pure pivot
    FINAL_ALIGN_TOLERANCE  = 10    # sim — landing window around FINAL_ALIGN_TARGET

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

# ── Parallel wall detection ───────────────────────────
PARALLEL_WALL_THRESHOLD  = 0.50   # m — both walls must be within this range to flag parallel
PARALLEL_ANGLE_MIN_DEG   = 150.0  # deg — minimum angular separation between the two walls
PARALLEL_ANGLE_MAX_DEG   = 210.0  # deg — maximum angular separation between the two walls
PARALLEL_SEARCH_HALF_DEG = 90.0   # deg — hemisphere to search for the opposite wall

# ── Inner wall rejection ──────────────────────────────
if SIMULATION:
    MIN_WALL_CLUSTER_WIDTH = 20
else:
    MIN_WALL_CLUSTER_WIDTH = 12

TRACK_WINDOW              = 60
CMD_VEL_PUBLISH_RATE      = 50.0
LIDAR_RANGE_MIN           = 0.12
CMD_HANDOFF_SETTLE_SECS   = 5.0


def log_constants():
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CALL", LOG_NODE, LOG_CLASS)

    if not LOG_TUNING_CONSTANTS:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | RETURN: (disabled)", LOG_NODE, LOG_CLASS)
        return

    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: SIMULATION; %d", LOG_NODE, LOG_CLASS, SIMULATION)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: FORWARD_INDEX; %d", LOG_NODE, LOG_CLASS, FORWARD_INDEX)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: FORWARD_STOP_DIST; %.2f", LOG_NODE, LOG_CLASS, FORWARD_STOP_DIST)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: FORWARD_SPEED; %.2f", LOG_NODE, LOG_CLASS, FORWARD_SPEED)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: SCAN_AVERAGE_WINDOW; %d", LOG_NODE, LOG_CLASS, SCAN_AVERAGE_WINDOW)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: SCAN_STD_WINDOW_REJECTION; %.3f", LOG_NODE, LOG_CLASS, SCAN_STD_WINDOW_REJECTION)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: MIN_WALL_CLUSTER_WIDTH; %d", LOG_NODE, LOG_CLASS, MIN_WALL_CLUSTER_WIDTH)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ROTATION_SPEED_START; %.2f", LOG_NODE, LOG_CLASS, ROTATION_SPEED_START)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ROTATION_SPEED_SLOW; %.2f", LOG_NODE, LOG_CLASS, ROTATION_SPEED_SLOW)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ROTATION_SLOW_ZONE; %d", LOG_NODE, LOG_CLASS, ROTATION_SLOW_ZONE)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ROTATION_INDEX_TOLERANCE; %d", LOG_NODE, LOG_CLASS, ROTATION_INDEX_TOLERANCE)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ROTATION_MAX_ATTEMPTS; %d", LOG_NODE, LOG_CLASS, ROTATION_MAX_ATTEMPTS)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ROTATION_SETTLE_RATE; %d", LOG_NODE, LOG_CLASS, ROTATION_SETTLE_RATE)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: TRACK_WINDOW; %d", LOG_NODE, LOG_CLASS, TRACK_WINDOW)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: CMD_VEL_PUBLISH_RATE; %.1f", LOG_NODE, LOG_CLASS, CMD_VEL_PUBLISH_RATE)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: CMD_HANDOFF_SETTLE_SECS; %.1f", LOG_NODE, LOG_CLASS, CMD_HANDOFF_SETTLE_SECS)
    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: LIDAR_RANGE_MIN; %.2f", LOG_NODE, LOG_CLASS, LIDAR_RANGE_MIN)
    if not SIMULATION:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: RIGHT_INDEX; %d", LOG_NODE, LOG_CLASS, RIGHT_INDEX)
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: RIGHT_ALIGN_DIST; %.2f", LOG_NODE, LOG_CLASS, RIGHT_ALIGN_DIST)
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: ALIGNMENT_TURN_DISTANCE; %.2f", LOG_NODE, LOG_CLASS, ALIGNMENT_TURN_DISTANCE)
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: YAW_SPEED; %.2f", LOG_NODE, LOG_CLASS, YAW_SPEED)
    else:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: FINAL_ALIGN_TARGET; %d", LOG_NODE, LOG_CLASS, FINAL_ALIGN_TARGET)
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: FINAL_ALIGN_TOLERANCE; %d", LOG_NODE, LOG_CLASS, FINAL_ALIGN_TOLERANCE)
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | CONST: FINAL_ALIGN_LINEAR_VEL; %.2f", LOG_NODE, LOG_CLASS, FINAL_ALIGN_LINEAR_VEL)

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: log_constants() | RETURN: (dump complete)", LOG_NODE, LOG_CLASS)


wall_found           = False
searching            = False
rotating             = False
moving               = False
wall_angle_pub       = None
wall_angle_index_pub = None
cmd_pub              = None
latest_scan          = None
latest_cmd           = Twist()
cmd_timer            = None
_prev_logic_state    = None   # last announced LOG_LOGIC_STATES label — logs only on change


def _log_state(new_state):
    """Emit a LOG_LOGIC_STATES transition line only when the state actually
    changes — mirrors log_real_robot_lap_server's STATE: prev -> curr pattern.
    Module-level (not per-object) since this file has no class instance."""
    global _prev_logic_state
    if LOG_LOGIC_STATES and new_state != _prev_logic_state:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _log_state() | STATE: %s -> %s",
                      LOG_NODE, LOG_CLASS, _prev_logic_state, new_state)
    _prev_logic_state = new_state


def average_std_range(ranges, index, range_min, window=SCAN_AVERAGE_WINDOW):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: average_std_range() | CALL", LOG_NODE, LOG_CLASS)

    samples   = []
    inf_count = 0
    for i in range(index - window, index + window + 1):
        val = ranges[i % len(ranges)]
        if math.isinf(val):
            inf_count += 1
            samples.append(range_min)
        elif math.isnan(val):
            pass
        elif val < LIDAR_RANGE_MIN:
            samples.append(range_min)
        else:
            samples.append(val)
    if inf_count >= 4:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: average_std_range() | RETURN: mean; inf std_dev; inf", LOG_NODE, LOG_CLASS)
        return float('inf'), float('inf')
    if not samples:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: average_std_range() | RETURN: mean; inf std_dev; inf", LOG_NODE, LOG_CLASS)
        return float('inf'), float('inf')
    mean    = sum(samples) / len(samples)
    std_dev = math.sqrt(sum((x - mean) ** 2 for x in samples) / len(samples))

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: average_std_range() | RETURN: mean; %.3f std_dev; %.3f", LOG_NODE, LOG_CLASS, mean, std_dev)

    return mean, std_dev


def find_wall_angle(laserscan_data):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle() | CALL", LOG_NODE, LOG_CLASS)

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

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle() | RETURN: angle; %.1f deg index; %d range; %.3f",
                      LOG_NODE, LOG_CLASS, math.degrees(angle), min_index, min_range)

    return angle, min_index


def _measure_cluster_width_cached(avg_cache, std_cache, center_index, num_ranges):
    """Count consecutive scan indices around center_index that pass the std filter,
    using pre-computed avg/std arrays instead of recalling average_std_range().
    Walks outward in both directions from center_index and stops when a reading
    fails the std filter. Returns total width in indices.
    Used by find_wall_angle_filtered to discriminate short walls from long ones."""
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _measure_cluster_width_cached() | CALL", LOG_NODE, LOG_CLASS)

    width = 1   # center index itself counts as 1

    # walk forward (increasing index)
    offset = 1
    while offset < num_ranges // 2:
        idx = (center_index + offset) % num_ranges
        if math.isinf(avg_cache[idx]) or std_cache[idx] >= SCAN_STD_WINDOW_REJECTION:
            break
        width += 1
        offset += 1

    # walk backward (decreasing index)
    offset = 1
    while offset < num_ranges // 2:
        idx = (center_index - offset) % num_ranges
        if math.isinf(avg_cache[idx]) or std_cache[idx] >= SCAN_STD_WINDOW_REJECTION:
            break
        width += 1
        offset += 1

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _measure_cluster_width_cached() | RETURN: width; %d", LOG_NODE, LOG_CLASS, width)

    return width


def find_wall_angle_filtered(laserscan_data):
    """Find the closest wall, rejecting clusters narrower than MIN_WALL_CLUSTER_WIDTH
    (e.g. the short inner track wall).

    PERFORMANCE NOTE: avg/std for every scan index is computed exactly ONCE up front
    and cached in avg_cache/std_cache. The original implementation called
    average_std_range() repeatedly inside both the outer scan loop AND inside every
    cluster-width walk — for a clean scan where most indices pass the std filter,
    that meant tens of thousands of redundant average_std_range() calls (each itself
    iterating a SCAN_AVERAGE_WINDOW-sized window), hanging this function for multiple
    seconds with no error and no visible symptom other than rotate_to_index() never
    being reached. Caching reduces this to one pass over num_ranges plus a cheap
    array-lookup walk per cluster — roughly 100x fewer calls. The outer loop also
    skips past a cluster immediately after measuring it, since every index inside
    that cluster has already been accounted for."""
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | CALL", LOG_NODE, LOG_CLASS)

    num_ranges = len(laserscan_data.ranges)

    # Pre-compute avg/std for every index ONCE — avoids recomputation in
    # the outer loop and inside every cluster-width walk.
    avg_cache = [None] * num_ranges
    std_cache = [None] * num_ranges
    for i in range(num_ranges):
        avg_cache[i], std_cache[i] = average_std_range(laserscan_data.ranges, i, laserscan_data.range_min)

    candidates = []
    i = 0
    while i < num_ranges:
        avg, std = avg_cache[i], std_cache[i]
        if not math.isinf(avg) and std < SCAN_STD_WINDOW_REJECTION:
            width = _measure_cluster_width_cached(avg_cache, std_cache, i, num_ranges)
            candidates.append((avg, i, width))
            i += max(1, width)   # skip past the cluster just measured
        else:
            i += 1

    if not candidates:
        rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | WARN: no valid candidates - returning index 0", LOG_NODE, LOG_CLASS)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | RETURN: angle; %.3f best_idx; 0", LOG_NODE, LOG_CLASS, laserscan_data.angle_min)
        return laserscan_data.angle_min, 0
    accepted = [(avg, idx, w) for avg, idx, w in candidates if w >= MIN_WALL_CLUSTER_WIDTH]
    rejected = [(avg, idx, w) for avg, idx, w in candidates if w <  MIN_WALL_CLUSTER_WIDTH]
    logged_rejected = set()
    for avg, idx, w in sorted(rejected, key=lambda x: x[0]):
        if not any(abs(idx - seen) < MIN_WALL_CLUSTER_WIDTH for seen in logged_rejected):
            angle_deg = math.degrees(laserscan_data.angle_min + idx * laserscan_data.angle_increment)
            if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | INFO: Cluster REJECTED - index; %d range; %.3f m width; %d (< %d) angle; %.1f deg - likely inner wall",
                              LOG_NODE, LOG_CLASS, idx, avg, w, MIN_WALL_CLUSTER_WIDTH, angle_deg)
            logged_rejected.add(idx)
    if accepted:
        best_avg, best_idx, best_width = min(accepted, key=lambda x: x[0])
        source = "accepted"
    else:
        best_avg, best_idx, best_width = max(candidates, key=lambda x: x[2])
        source = "FALLBACK-widest"
        if LOG_AI_DEBUGINFO:
            rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | WARN: All clusters below threshold - fallback to widest cluster (index; %d width; %d) likely corner position",
                          LOG_NODE, LOG_CLASS, best_idx, best_width)

    angle = laserscan_data.angle_min + (best_idx * laserscan_data.angle_increment)

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | INFO: Closest wall (%s) - index; %d range; %.3f m cluster_width; %d angle; %.1f deg",
                      LOG_NODE, LOG_CLASS, source, best_idx, best_avg, best_width, math.degrees(angle))

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_filtered() | RETURN: angle; %.3f best_idx; %d", LOG_NODE, LOG_CLASS, angle, best_idx)

    return angle, best_idx


def find_wall_angle_windowed(laserscan_data, center_index):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_windowed() | CALL", LOG_NODE, LOG_CLASS)

    min_range  = float('inf')
    best_index = center_index
    num_ranges = len(laserscan_data.ranges)
    for i in range(center_index - TRACK_WINDOW, center_index + TRACK_WINDOW + 1):
        idx = i % num_ranges
        avg, std = average_std_range(laserscan_data.ranges, idx, laserscan_data.range_min)
        if not math.isinf(avg) and avg < min_range and std < SCAN_STD_WINDOW_REJECTION:
            min_range  = avg
            best_index = idx

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_windowed() | INFO: center; %d best; %d range; %.3f",
                       LOG_NODE, LOG_CLASS, center_index, best_index, min_range)

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_angle_windowed() | RETURN: best_index; %d", LOG_NODE, LOG_CLASS, best_index)

    return best_index


def rotate_to_index(target_index, tolerance=ROTATION_INDEX_TOLERANCE, attempt=1,
                    linear_x=0.0, full_scan=False, initial_index=None):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | CALL", LOG_NODE, LOG_CLASS)

    global rotating, latest_cmd

    if attempt == 1 and target_index == FORWARD_INDEX and latest_scan is not None:
        if rotate_to_index_close_parallel_walls(latest_scan):
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | RETURN: (handled by parallel-wall close rotation)", LOG_NODE, LOG_CLASS)
            return

    if attempt > ROTATION_MAX_ATTEMPTS:
        rospy.logerr(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | ERROR: Rotation failed after %d attempts", LOG_NODE, LOG_CLASS, ROTATION_MAX_ATTEMPTS)
        latest_cmd = Twist()
        if LOG_TWISTS:
            rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
        cmd_pub.publish(latest_cmd)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | RETURN: (aborted, max attempts exceeded)", LOG_NODE, LOG_CLASS)
        return

    target_min = target_index - tolerance
    target_max = target_index + tolerance

    if LOG_AI_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Rotating to index %d (window; %d-%d) attempt %d/%d linear_x; %.2f full_scan; %s",
                      LOG_NODE, LOG_CLASS, target_index, target_min, target_max, attempt, ROTATION_MAX_ATTEMPTS, linear_x,
                      'yes' if full_scan else 'no')
    if LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Rotating to index %d (window; %d-%d)",
                      LOG_NODE, LOG_CLASS, target_index, target_min, target_max)

    rate = rospy.Rate(10)

    while latest_scan is None and not rospy.is_shutdown():
        rate.sleep()

    if initial_index is not None and attempt == 1:
        if LOG_AI_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Using pre-filtered initial index %d", LOG_NODE, LOG_CLASS, initial_index)
        _initial_index = initial_index
    else:
        _, _initial_index = find_wall_angle(latest_scan)
    locked_index  = _initial_index
    initial_index = _initial_index
    started_above = initial_index > target_max
    started_below = initial_index < target_min

    rot_cmd = Twist()
    rot_cmd.linear.x = linear_x
    if initial_index > target_max:
        rot_cmd.angular.z = ROTATION_SPEED_START
        if LOG_AI_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Rotation direction locked; index above target (decrease) initial; %d angular.z; %.3f",
                          LOG_NODE, LOG_CLASS, initial_index, rot_cmd.angular.z)
        if LOG_DEV_DEBUGINFO:
            rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, rot_cmd.linear.x, rot_cmd.angular.z)
    elif initial_index < target_min:
        rot_cmd.angular.z = -ROTATION_SPEED_START
        if LOG_AI_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Rotation direction locked; index below target (increase) initial; %d angular.z; %.3f",
                          LOG_NODE, LOG_CLASS, initial_index, rot_cmd.angular.z)
    else:
        if LOG_AI_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Already at target index %d - no rotation needed", LOG_NODE, LOG_CLASS, target_index)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | RETURN: (already at target, no rotation needed)", LOG_NODE, LOG_CLASS)
        return

    latest_cmd = rot_cmd
    cmd_pub.publish(latest_cmd)
    if LOG_TWISTS:
        rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)

    scan_count = 0
    MAX_ROTATION_SCANS = 500
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        if scan_count >= MAX_ROTATION_SCANS:
            rospy.logerr(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | ERROR: Rotation timeout after %d scans - wall never found, check SIMULATION flag and LiDAR",
                         LOG_NODE, LOG_CLASS, scan_count)
            latest_cmd = Twist()
            if LOG_TWISTS:
                rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
            cmd_pub.publish(latest_cmd)
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | RETURN: (aborted, rotation timeout)", LOG_NODE, LOG_CLASS)
            return

        if full_scan:
            _, angle_index = find_wall_angle(latest_scan)
        else:
            angle_index = find_wall_angle_windowed(latest_scan, locked_index)
        locked_index = angle_index
        scan_count  += 1

        wall_angle_deg = math.degrees(latest_scan.angle_min + (angle_index * latest_scan.angle_increment))
        wall_angle_pub.publish(Float32(wall_angle_deg))
        wall_angle_index_pub.publish(UInt32(angle_index))

        num_ranges  = len(latest_scan.ranges)
        raw_remain  = abs(angle_index - target_index)
        remaining   = min(raw_remain, num_ranges - raw_remain)
        if LOG_AI_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: scan; %d wall_index; %d target; %d-%d remaining; %d speed; %.3f rad/s angle; %.1f deg",
                          LOG_NODE, LOG_CLASS, scan_count, angle_index, target_min, target_max, remaining,
                          latest_cmd.angular.z, wall_angle_deg)

        if target_min <= angle_index <= target_max:
            if LOG_AI_DEBUGINFO:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Target window reached after %d scans", LOG_NODE, LOG_CLASS, scan_count)
            break

        crossed_above = started_above and angle_index < target_min
        crossed_below = started_below and angle_index > target_max
        seam_crossing = raw_remain > num_ranges // 2
        overshot = (crossed_above or crossed_below) and not seam_crossing

        if overshot:
            if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
                rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | WARN: Overshoot detected at index %d - stopping and restarting (attempt %d)",
                          LOG_NODE, LOG_CLASS, angle_index, attempt)
            latest_cmd = Twist()
            if LOG_TWISTS:
                rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
            cmd_pub.publish(latest_cmd)
            rospy.Rate(ROTATION_SETTLE_RATE).sleep()
            rotate_to_index(target_index, tolerance, attempt + 1, linear_x, full_scan)
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | RETURN: (overshoot, retried as attempt %d)", LOG_NODE, LOG_CLASS, attempt + 1)
            return

        if remaining <= ROTATION_SLOW_ZONE:
            slow_cmd = Twist()
            slow_cmd.angular.z = math.copysign(ROTATION_SPEED_SLOW, rot_cmd.angular.z)
            if latest_cmd.angular.z != slow_cmd.angular.z:
                latest_cmd = slow_cmd
                if LOG_TWISTS:
                    rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
                cmd_pub.publish(latest_cmd)
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Entering slow zone - speed reduced to %.3f rad/s", LOG_NODE, LOG_CLASS, ROTATION_SPEED_SLOW)
        else:
            if latest_cmd.angular.z != rot_cmd.angular.z:
                latest_cmd = rot_cmd
                if LOG_TWISTS:
                    rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
                cmd_pub.publish(latest_cmd)
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Back to full speed - %.3f rad/s", LOG_NODE, LOG_CLASS, ROTATION_SPEED_START)

        rate.sleep()

    latest_cmd = Twist()
    if LOG_TWISTS:
        rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
    cmd_pub.publish(latest_cmd)
    if LOG_AI_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Stop published - settling for %.0f ms", LOG_NODE, LOG_CLASS, 1000.0 / ROTATION_SETTLE_RATE)
    rospy.Rate(ROTATION_SETTLE_RATE).sleep()

    if LOG_AI_DEBUGINFO or LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | INFO: Rotation complete - wall at index %d angle; %.1f deg",
                      LOG_NODE, LOG_CLASS, angle_index, wall_angle_deg)

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index() | RETURN: (settled in target window)", LOG_NODE, LOG_CLASS)


def detect_parallel_walls(laserscan_data):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | CALL", LOG_NODE, LOG_CLASS)

    num_ranges    = len(laserscan_data.ranges)
    angle_inc_deg = math.degrees(laserscan_data.angle_increment)
    _, wall_a_index = find_wall_angle(laserscan_data)
    range_a, _ = average_std_range(laserscan_data.ranges, wall_a_index, laserscan_data.range_min)
    if math.isinf(range_a):
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | RETURN: False (range_a inf)", LOG_NODE, LOG_CLASS)
        return False, wall_a_index, None
    if range_a > PARALLEL_WALL_THRESHOLD:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | RETURN: False (range_a > threshold)", LOG_NODE, LOG_CLASS)
        return False, wall_a_index, None
    opposite_index = (wall_a_index + num_ranges // 2) % num_ranges
    search_indices = int(PARALLEL_SEARCH_HALF_DEG / angle_inc_deg)
    best_b_index = opposite_index
    best_b_range = float('inf')
    for offset in range(-search_indices, search_indices + 1):
        idx      = (opposite_index + offset) % num_ranges
        avg, std = average_std_range(laserscan_data.ranges, idx, laserscan_data.range_min)
        if not math.isinf(avg) and avg < best_b_range and std < SCAN_STD_WINDOW_REJECTION:
            best_b_range = avg
            best_b_index = idx
    if math.isinf(best_b_range):
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | RETURN: False (best_b_range inf)", LOG_NODE, LOG_CLASS)
        return False, wall_a_index, None
    if best_b_range > PARALLEL_WALL_THRESHOLD:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | RETURN: False (best_b_range > threshold)", LOG_NODE, LOG_CLASS)
        return False, wall_a_index, None
    raw_sep   = abs(best_b_index - wall_a_index)
    sep_index = min(raw_sep, num_ranges - raw_sep)
    sep_deg   = sep_index * angle_inc_deg
    if not (PARALLEL_ANGLE_MIN_DEG <= sep_deg <= PARALLEL_ANGLE_MAX_DEG):
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | RETURN: False (separation %.1f deg out of range)", LOG_NODE, LOG_CLASS, sep_deg)
        return False, wall_a_index, None

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: detect_parallel_walls() | RETURN: True wall_a_index; %d best_b_index; %d",
                      LOG_NODE, LOG_CLASS, wall_a_index, best_b_index)

    return True, wall_a_index, best_b_index


def rotate_to_index_close_parallel_walls(laserscan_data):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index_close_parallel_walls() | CALL", LOG_NODE, LOG_CLASS)

    parallel, wall_a_idx, wall_b_idx = detect_parallel_walls(laserscan_data)
    if not parallel:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index_close_parallel_walls() | RETURN: False", LOG_NODE, LOG_CLASS)
        return False
    num_ranges = len(laserscan_data.ranges)
    def shortest_rotation(target_idx):
        raw = abs(target_idx - FORWARD_INDEX)
        return min(raw, num_ranges - raw)
    rot_a = shortest_rotation(wall_a_idx)
    rot_b = shortest_rotation(wall_b_idx)
    if rot_a <= rot_b:
        target_idx  = wall_a_idx
        chosen_wall = 'A'
        delta       = rot_b - rot_a
    else:
        target_idx  = wall_b_idx
        chosen_wall = 'B'
        delta       = rot_a - rot_b
    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index_close_parallel_walls() | INFO: Parallel walls detected - choosing wall %s (target_idx; %d delta; %d)",
                      LOG_NODE, LOG_CLASS, chosen_wall, target_idx, delta)
    rotate_to_index(target_idx)

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: rotate_to_index_close_parallel_walls() | RETURN: True", LOG_NODE, LOG_CLASS)

    return True


def move_to_wall():
    """Drive toward wall.
    Sim:  straight forward until fwd <= FORWARD_STOP_DIST.
    Real: straight forward until right <= ALIGNMENT_TURN_DISTANCE, then apply
          YAW_SPEED to arc robot into right-side alignment while continuing forward.
          Stops when right <= RIGHT_ALIGN_DIST or fwd <= FORWARD_STOP_DIST.
    No separate final pivot — yaw during approach handles alignment."""

    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | CALL", LOG_NODE, LOG_CLASS)

    global moving, latest_cmd

    _log_state("MOVING_TO_WALL")

    if latest_scan is not None:
        fwd, _ = average_std_range(latest_scan.ranges, FORWARD_INDEX, latest_scan.range_min)
        if LOG_SCAN_DISTANCES:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | SCAN: phase; pre-check forward; %.3f stop_dist; %.3f",
                          LOG_NODE, LOG_CLASS, fwd, FORWARD_STOP_DIST)
        if not math.isinf(fwd) and fwd <= FORWARD_STOP_DIST:
            if LOG_AI_DEBUGINFO:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Already within forward stop distance (fwd; %.3f) - skipping approach",
                              LOG_NODE, LOG_CLASS, fwd)
            _log_state("WALL_APPROACH_SKIPPED")
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | RETURN: (already within stop distance)", LOG_NODE, LOG_CLASS)
            return

    if LOG_AI_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Driving to wall speed; %.2f m/s fwd_stop; %.2f m",
                      LOG_NODE, LOG_CLASS, FORWARD_SPEED, FORWARD_STOP_DIST)

    move_cmd = Twist()
    move_cmd.linear.x  = FORWARD_SPEED
    move_cmd.angular.z = 0.0
    latest_cmd = move_cmd
    if LOG_TWISTS:
        rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
    cmd_pub.publish(latest_cmd)

    ARC_LIMIT_INDICES = 112 if not SIMULATION else 0

    scan_count         = 0
    turning_phase      = False
    arc_start_index    = None
    arc_degrees_turned = 0.0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        fwd, _ = average_std_range(latest_scan.ranges, FORWARD_INDEX, latest_scan.range_min)
        scan_count += 1

        if SIMULATION:
            if LOG_SCAN_DISTANCES:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | SCAN: phase; approach scan; %d forward; %.3f stop_dist; %.3f",
                              LOG_NODE, LOG_CLASS, scan_count, fwd, FORWARD_STOP_DIST)
            if LOG_AI_DEBUGINFO:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: fwd approach scan; %d fwd; %.3f m stop; %.3f m",
                              LOG_NODE, LOG_CLASS, scan_count, fwd, FORWARD_STOP_DIST)
            if not math.isinf(fwd) and fwd <= FORWARD_STOP_DIST:
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Forward stop triggered at %.3f m after %d scans",
                                  LOG_NODE, LOG_CLASS, fwd, scan_count)
                _log_state("FORWARD_STOP_REACHED")
                break
        else:
            right, _ = average_std_range(latest_scan.ranges, RIGHT_INDEX, latest_scan.range_min)

            if LOG_SCAN_DISTANCES:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | SCAN: phase; %s scan; %d forward; %.3f right; %.3f fwd_stop; %.3f right_stop; %.3f",
                              LOG_NODE, LOG_CLASS, "turning" if turning_phase else "approach",
                              scan_count, fwd, right, FORWARD_STOP_DIST, RIGHT_ALIGN_DIST)

            if not turning_phase and not math.isinf(right) and right <= ALIGNMENT_TURN_DISTANCE:
                turning_phase      = True
                arc_start_index    = RIGHT_INDEX
                move_cmd           = Twist()
                move_cmd.linear.x  = FORWARD_SPEED
                move_cmd.angular.z = YAW_SPEED
                latest_cmd         = move_cmd
                cmd_pub.publish(latest_cmd)
                if LOG_TWISTS:
                    rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
                _log_state("ALIGNMENT_TURN_STARTED")
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Alignment turn started - right; %.3f m yaw; %.2f rad/s",
                                  LOG_NODE, LOG_CLASS, right, YAW_SPEED)

            if turning_phase:
                arc_degrees_turned = scan_count * (1.0 / 10.0) * math.degrees(YAW_SPEED)

            if LOG_AI_DEBUGINFO:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: approach scan; %d fwd; %.3f m right; %.3f m turning; %s arc_deg; %.1f right_stop; %.3f",
                              LOG_NODE, LOG_CLASS, scan_count, fwd, right, turning_phase, arc_degrees_turned, RIGHT_ALIGN_DIST)

            if not math.isinf(fwd) and fwd <= FORWARD_STOP_DIST:
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Forward safety stop at %.3f m after %d scans",
                                  LOG_NODE, LOG_CLASS, fwd, scan_count)
                _log_state("FORWARD_SAFETY_STOP")
                break
            if not math.isinf(right) and right <= RIGHT_ALIGN_DIST:
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Right alignment stop at %.3f m after %d scans",
                                  LOG_NODE, LOG_CLASS, right, scan_count)
                _log_state("RIGHT_ALIGNMENT_STOP")
                break
            if turning_phase and arc_degrees_turned >= 90.0:
                if LOG_AI_DEBUGINFO:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Arc limit reached (%.1f deg) after %d scans - stopping",
                                  LOG_NODE, LOG_CLASS, arc_degrees_turned, scan_count)
                _log_state("ARC_LIMIT_REACHED")
                break

        rate.sleep()

    latest_cmd = Twist()
    if LOG_TWISTS:
        rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
    cmd_pub.publish(latest_cmd)
    if LOG_AI_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | INFO: Stop published - settling for %.0f ms", LOG_NODE, LOG_CLASS, 1000.0 / ROTATION_SETTLE_RATE)
    rospy.Rate(ROTATION_SETTLE_RATE).sleep()

    _log_state("WALL_APPROACH_COMPLETE")

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: move_to_wall() | RETURN: (approach complete)", LOG_NODE, LOG_CLASS)


def cmd_vel_timer_callback(event):
    if cmd_pub is not None:
        cmd_pub.publish(latest_cmd)


def scan_callback(laserscan_data):
    global latest_scan
    latest_scan = laserscan_data


def handle_find_wall(req):
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | CALL", LOG_NODE, LOG_CLASS)

    global wall_found, searching, moving, rotating, cmd_timer, latest_cmd, cmd_pub

    rate = rospy.Rate(10)
    while rospy.Time.now() == rospy.Time(0):
        if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: Waiting for clock...", LOG_NODE, LOG_CLASS)
        rate.sleep()

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: Service Started ...", LOG_NODE, LOG_CLASS)
    wall_found = False
    searching  = True
    rotating   = True
    _log_state("SEARCHING")

    while latest_scan is None and not rospy.is_shutdown():
        if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: Waiting for first scan...", LOG_NODE, LOG_CLASS)
        rospy.Rate(10).sleep()

    scan_wait_count = 0
    while not rospy.is_shutdown():
        if latest_scan is not None and any(
            not math.isinf(r) and not math.isnan(r) and r > LIDAR_RANGE_MIN
            for r in latest_scan.ranges
        ):
            break
        scan_wait_count += 1
        if scan_wait_count % 10 == 0:
            if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
                rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | WARN: Waiting for valid scan data - %d s elapsed (check SIMULATION flag and LiDAR)",
                              LOG_NODE, LOG_CLASS, scan_wait_count // 10)
        rospy.Rate(10).sleep()

    # Step 1 — rotate closest wall to directly ahead
    _log_state("ROTATING_TO_FORWARD")
    _, filtered_index = find_wall_angle_filtered(latest_scan)
    rotate_to_index(FORWARD_INDEX, initial_index=filtered_index)
    rotating = False

    # Step 2 — drive toward wall
    moving = True
    move_to_wall()
    moving = False

    # Step 3 — final pivot (sim only)
    # Real robot aligns via yaw during move_to_wall() — no pivot needed.
    #
    # BUGFIX 1: previously called with full_scan=True, which makes rotate_to_index()
    # call find_wall_angle() (a fresh closest-wall search over all 720 indices)
    # EVERY scan, with no memory of which wall it was just tracking. At a 45 deg
    # diagonal near a corner, the forward wall and the side wall can be near-equal
    # distance, so the full scan flips between them scan to scan (e.g. landing on
    # index 529 instead of anywhere near FINAL_ALIGN_TARGET) and the rotation
    # never settles into the target window — the robot spins indefinitely.
    # Fix: do ONE full scan to (re)locate the wall robustly, then track it with
    # the windowed search (full_scan=False) for the rest of the pivot, exactly
    # like Step 1 seeds rotate_to_index() with initial_index=filtered_index.
    #
    # BUGFIX 2: FINAL_ALIGN_TARGET=90 is an ABSOLUTE index, calibrated for a robot
    # that approached the wall straight-on (wall starting near FORWARD_INDEX=360).
    # move_to_wall() now approaches at a 45 deg diagonal, so the wall is already
    # sitting near index ~300 (45 deg off FORWARD_INDEX) when the pivot starts.
    # Rotating from ~300 all the way to the absolute index 90 is a ~105 deg turn,
    # not the 90 deg turn needed to bring the right side parallel to the wall —
    # that's why the robot kept spinning well past where it should have stopped.
    # Fix: compute the pivot target RELATIVE to the seed index. Sim is 720 pts /
    # 360 deg = 2 indices/deg. Confirmed rotation direction here is "index
    # decreases as the robot turns CCW", so a 90 deg CCW pivot is seed - 180.
    #
    # BUGFIX 2b: the first attempt at this fix used seed - 90, which at
    # 2 indices/deg is only a 45 deg pivot — half the rotation needed. The
    # comment claimed "45 deg CCW pivot" but the actual requirement (right
    # side parallel to the wall ahead of the robot) is a 90 deg turn, so the
    # offset must be -180 indices, not -90. This is why the right side
    # consistently landed at ~45 deg off-parallel instead of parallel.
    if SIMULATION:
        rotating = True
        _log_state("FINAL_ALIGNMENT_PIVOT")
        _, final_pivot_seed_index = find_wall_angle(latest_scan)
        num_ranges_pivot = len(latest_scan.ranges)
        relative_pivot_target = (final_pivot_seed_index - 180) % num_ranges_pivot
        if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: Final alignment (sim) - seed_index=%d relative_target=%d tolerance=%d",
                          LOG_NODE, LOG_CLASS, final_pivot_seed_index, relative_pivot_target, FINAL_ALIGN_TOLERANCE)
        rotate_to_index(relative_pivot_target, tolerance=FINAL_ALIGN_TOLERANCE,
                        linear_x=FINAL_ALIGN_LINEAR_VEL, full_scan=False,
                        initial_index=final_pivot_seed_index)
        rotating = False

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: Wall approach complete - handing off to lap server", LOG_NODE, LOG_CLASS)

    wall_found = True
    searching  = False
    _log_state("WALL_FOUND")
    rospy.set_param('/real_wall_aligned', True)

    cmd_timer.shutdown()
    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: cmd_vel timer stopped", LOG_NODE, LOG_CLASS)

    latest_cmd = Twist()
    if LOG_TWISTS:
        rospy.logdebug(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | TWIST: lin.x; %.3f rot.z; %.3f", LOG_NODE, LOG_CLASS, latest_cmd.linear.x, latest_cmd.angular.z)
    cmd_pub.publish(Twist())

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: cmd_vel zeroed - waiting %.0f s before handing off to lap server",
                      LOG_NODE, LOG_CLASS, CMD_HANDOFF_SETTLE_SECS)

    rospy.sleep(CMD_HANDOFF_SETTLE_SECS)
    cmd_pub.publish(Twist())

    cmd_pub.unregister()
    cmd_pub = None
    _log_state("HANDOFF_COMPLETE")
    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: cmd_pub unregistered - find_wall removed from /cmd_vel publishers", LOG_NODE, LOG_CLASS)
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | INFO: Handoff complete - lap server may now start cmd_vel timer", LOG_NODE, LOG_CLASS)

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: handle_find_wall() | RETURN: wallfound; %s", LOG_NODE, LOG_CLASS, wall_found)

    return FindWallRealResponse(wallfound=wall_found)


def find_wall_real_server():
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_real_server() | CALL", LOG_NODE, LOG_CLASS)

    global wall_angle_pub, wall_angle_index_pub, cmd_pub, cmd_timer

    rospy.init_node('find_wall_real_robot_svc', log_level=rospy.DEBUG)

    if LOG_TUNING_CONSTANTS:
        log_constants()

    wall_angle_pub       = rospy.Publisher('/find_wall_real_robot/wall_angle',       Float32, queue_size=1)
    wall_angle_index_pub = rospy.Publisher('/find_wall_real_robot/wall_angle_index', UInt32,  queue_size=1)
    cmd_pub              = rospy.Publisher('/cmd_vel', Twist,                         queue_size=1)

    rospy.Service('find_wall_real_robot', FindWallReal, handle_find_wall)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    cmd_timer = rospy.Timer(rospy.Duration(1.0 / CMD_VEL_PUBLISH_RATE), cmd_vel_timer_callback)

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_real_server() | INFO: cmd_timer started", LOG_NODE, LOG_CLASS)

    rospy.Rate(1).sleep()

    if LOG_AI_DEBUGINFO or LOG_DEV_DEBUGINFO:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_real_server() | INFO: Service Ready", LOG_NODE, LOG_CLASS)

    rospy.spin()

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: find_wall_real_server() | RETURN", LOG_NODE, LOG_CLASS)


if __name__ == '__main__':
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: main() | CALL", LOG_NODE, LOG_CLASS)

    find_wall_real_server()

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: main() | RETURN", LOG_NODE, LOG_CLASS)