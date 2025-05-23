!start.

/* General */

convert_to_radian(Angle, Result) :- Result = Angle * math.pi / 180.0.

+!start: true
    <-
    !wall_follow.

/* Wall Following */

calculate_wall_distance_error(Error) :- 
    wall_follow_speed(SPEED) &
    wall_follow_aim_angle(AIM_ANGLE) &
    convert_to_radian(AIM_ANGLE, AimAngleRadian) &
    Error = SPEED * math.sin(AimAngleRadian) / 2.

too_far_from_wall(Distance) :-
    wall_follow_distance_setpoint(SETPOINT) &
    calculate_wall_distance_error(Error) &
    Distance > SETPOINT + Error.

too_close_to_wall(Distance) :-
    wall_follow_distance_setpoint(SETPOINT) &
    calculate_wall_distance_error(Error) &
    Distance < SETPOINT - Error.

about_to_do_big_angle_wall_follow_turn(Angle) :-
    wall_follow_angle_change_tolerance(TOLERANCE) &
    math.abs(Angle) > TOLERANCE.

+!wall_follow: facing_wall(Distance, Angle) & not too_far_from_wall(Distance) & not too_close_to_wall(Distance)
    <-
    .print("Appropriate distance from wall; wall following");
    !align_with_wall(Angle);
    !start.

+!wall_follow: facing_wall(Distance, Angle) & too_far_from_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !align_with_wall(-1 * AIM_ANGLE + Angle);
    !start.

+!wall_follow: facing_wall(Distance, Angle) & too_close_to_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("Too close to the wall!");
    !align_with_wall(AIM_ANGLE + Angle);
    !start.

+!wall_follow: true
    <-
    .print("Cannot wall follow! Need perceptions.");
    !start.

+!align_with_wall(Angle): wall_follow_speed(SPEED) & about_to_do_big_angle_wall_follow_turn(Angle) & convert_to_radian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, RadianAngle).

+!align_with_wall(Angle): wall_follow_speed(SPEED) & not about_to_do_big_angle_wall_follow_turn(Angle) & convert_to_radian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, RadianAngle).