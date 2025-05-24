!start.

/* Main Behaviour */

+!start: facing_wall(Distance, Angle)
    <-
    !wall_follow(Distance, Angle);
    !start.

+!start: true
    <-
    .print("Cannot do anything. Need perceptions.");
    !start.

/* Wall Following */

calculate_wall_distance_error(Error) :- 
    speed(SPEED) &
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

+!wall_follow(Distance, Angle): not too_far_from_wall(Distance) & not too_close_to_wall(Distance)
    <-
    .print("Appropriate distance from wall; wall following");
    !turn(Angle);
    !start.

+!wall_follow(Distance, Angle): too_far_from_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !turn(-1 * AIM_ANGLE + Angle);
    !start.

+!wall_follow(Distance, Angle): too_close_to_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("Too close to the wall!");
    !turn(AIM_ANGLE + Angle);
    !start.

/* Helpers */

about_to_do_big_turn(Angle) :-
    wall_follow_angle_change_tolerance(TOLERANCE) &
    math.abs(Angle) > TOLERANCE.

convert_to_radian(Angle, Result) :- Result = Angle * math.pi / 180.0.

+!turn(Angle): speed(SPEED) & about_to_do_big_turn(Angle) & convert_to_radian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, RadianAngle).

+!turn(Angle): speed(SPEED) & not about_to_do_big_turn(Angle) & convert_to_radian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, RadianAngle).