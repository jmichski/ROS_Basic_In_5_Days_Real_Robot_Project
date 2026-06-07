def move_to_wall():
    """Drive forward until wall is within FORWARD_DISTANCE."""
    global moving
    rospy.loginfo(f"Find Wall: Moving to Nearest Wall ...")

    move_cmd = Twist()
    move_cmd.linear.x  = FORWARD_SPEED
    move_cmd.angular.z = 0
    cmd_pub.publish(move_cmd)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_scan is None:
            rate.sleep()
            continue

        fwd,std = average_std_range(latest_scan.ranges, FORWARD_INDEX, latest_scan.range_min)
        # rospy.loginfo(f"Forward range: {fwd:.3f}")

        if not math.isinf(fwd) and fwd <= FORWARD_DISTANCE:
            break

        rate.sleep()

    cmd_pub.publish(Twist())   # stop
    rospy.loginfo(f"Find Wall: Nearest Wall reached")
