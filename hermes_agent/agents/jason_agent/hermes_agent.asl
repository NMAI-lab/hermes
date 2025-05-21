!start.

/* Rules */

convert_to_radian(Angle, Result) :- Result = Angle * math.pi / 180.0.

calculate_wall_distance_error(Error) :- 
    wall_follow_speed(SPEED) &
    wall_follow_aim_angle(AIM_ANGLE) &
    convert_to_radian(AIM_ANGLE, AimAngleRadian) &
    Error = SPEED * math.sin(AimAngleRadian).

too_far_from_wall(Distance) :-
    wall_follow_distance_setpoint(SETPOINT) &
    calculate_wall_distance_error(Error) &
    Distance > SETPOINT + Error.

too_close_to_wall(Distance) :-
    wall_follow_distance_setpoint(SETPOINT) &
    calculate_wall_distance_error(Error) &
    Distance < SETPOINT - Error.

about_to_do_big_angle_change(Angle) :-
    wall_follow_angle_change_tolerance(TOLERANCE) &
    math.abs(Angle) > TOLERANCE.

/* Plans */

+!start: facing_wall(Distance, Angle) & not too_far_from_wall(Distance) & not too_close_to_wall(Distance)
    <-
    .print("Appropriate distance from wall; wall following");
    !wall_follow(Angle).

+!start: facing_wall(Distance, Angle) & too_far_from_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !wall_follow(-1 * AIM_ANGLE + Angle).

+!start: facing_wall(Distance, Angle) & too_close_to_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("Too close to the wall!");
    !wall_follow(AIM_ANGLE + Angle).

+!start: true
    <-
    //.print("Cannot wall follow! Need perceptions.");
    !start.

+!wall_follow(Angle): wall_follow_speed(SPEED) & about_to_do_big_angle_change(Angle) & convert_to_radian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, RadianAngle);
    !start.

+!wall_follow(Angle): wall_follow_speed(SPEED) & not about_to_do_big_angle_change(Angle) & convert_to_radian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, RadianAngle);
    !start.