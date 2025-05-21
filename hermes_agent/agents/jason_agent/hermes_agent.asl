!start.

/* Rules */

too_far_from_wall(Distance) :-
    wall_follow_distance_setpoint(SETPOINT) &
    wall_follow_distance_tolerance(ERROR) &
    Distance > SETPOINT + ERROR.

too_close_to_wall(Distance) :-
    wall_follow_distance_setpoint(SETPOINT) &
    wall_follow_distance_tolerance(ERROR) &
    Distance < SETPOINT - ERROR.

about_to_do_big_angle_change(Angle) :-
    wall_follow_angle_change_tolerance(TOLERANCE) &
    math.abs(Angle) > TOLERANCE.

to_radian(Angle, Result) :- Result = Angle * math.pi / 180.0.

/* Plans */

+!start: facing_wall(Distance, Angle) & not too_far_from_wall(Distance) & not too_close_to_wall(Distance)
    <-
    .print("appropriate distance from wall; wall following");
    !wall_follow(Angle).

+!start: facing_wall(Distance, Angle) & too_far_from_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("too far from the wall!");
    !wall_follow(-1 * AIM_ANGLE + Angle).

+!start: facing_wall(Distance, Angle) & too_close_to_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("too close to the wall!");
    !wall_follow(AIM_ANGLE + Angle).

+!start: true
    <-
    .print("Cannot wall follow! Need perceptions.");
    !start.

+!wall_follow(Angle): wall_follow_speed(SPEED) & about_to_do_big_angle_change(Angle) & to_radian(Angle, Radian_angle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, Radian_angle);
    !start.

+!wall_follow(Angle): wall_follow_speed(SPEED) & not about_to_do_big_angle_change(Angle) & to_radian(Angle, Radian_angle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, Radian_angle);
    !start.