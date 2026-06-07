def rotate_to_index(target_index, tolerance=ROTATION_INDEX_TOLERANCE):
    """
    Rotate robot until nearest wall index is within target_index +/- tolerance.
    
    Args:
        target_index  — scan index to rotate nearest wall to (e.g. 360=forward, 270=right)
        tolerance    — acceptable index window either side of target ROTATION_INDEX_TOLERANCE)
    """
    global rotating
 
    target_min = target_index - tolerance
    target_max = target_index + tolerance
 
    rospy.loginfo(f"Find Wall: Rotating nearest wall to index {target_index} (window: {target_min}-{target_max})")
    rate = rospy.Rate(10)
 

    # wait for first valid scan
    while latest_scan is None and not rospy.is_shutdown():
        rate.sleep()

     
    # determine and lock direction — publish rotation command ONCE before loop
    _, initial_index = find_wall_angle(latest_scan)

    locked_index = initial_index   # ADDED — lock wall index, never search globally again

    rot_cmd = Twist()
    rot_cmd.linear.x  = 0
    if initial_index > target_max:
        rot_cmd.angular.z = ROTATION_SPEED
        rospy.loginfo(f"Find Wall: Rotation direction locked: left")
    elif initial_index < target_min:
        rot_cmd.angular.z = -ROTATION_SPEED
        rospy.loginfo(f"Find Wall: Rotation direction locked: right")
    else:
        rospy.loginfo(f"Find Wall: lready at target index {target_index} — no rotation needed")
        return
    cmd_pub.publish(rot_cmd)    # start rotating — one publish, locked direction
 
    # loop only checks stop condition — never touches direction
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue
#        _, angle_index = find_wall_angle(latest_scan)
        angle_index  = find_wall_angle_windowed(latest_scan, locked_index)  # CHANGED — tracks locked wall only
        locked_index = angle_index                                           # ADDED — update for next iteration

        wall_angle_pub.publish(Float32(math.degrees(latest_scan.angle_min + (angle_index * latest_scan.angle_increment))))
    
        wall_angle_index_pub.publish(UInt32(angle_index))
        # rospy.loginfo(f"Rotating: angle_index: {angle_index}  target: {target_min}-{target_max}")
        if target_min <= angle_index <= target_max:
            break
        rate.sleep()
 
    cmd_pub.publish(Twist())    # stop
    rospy.loginfo(f"Find Wall: Rotation complete — wall at index {angle_index}")
