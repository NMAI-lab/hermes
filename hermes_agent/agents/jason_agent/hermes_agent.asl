!start.

/* Rules */
too_far_from_wall(DISTANCE) :-
    wall_follow_distance_setpoint(SETPOINT) &
    wall_follow_distance_tolerance(ERROR) &
    DISTANCE > SETPOINT + ERROR.

too_close_to_wall(DISTANCE) :-
    wall_follow_distance_setpoint(SETPOINT) &
    wall_follow_distance_tolerance(ERROR) &
    DISTANCE < SETPOINT - ERROR.

appropriate_distance_from_wall(DISTANCE) :-
    wall_follow_distance_setpoint(SETPOINT) &
    wall_follow_distance_tolerance(ERROR) &
    DISTANCE <= SETPOINT + ERROR &
    DISTANCE >= SETPOINT - ERROR.

big_angle_change(ANGLE) :-
    wall_follow_angle_change_tolerance(TOLERANCE) &
    math.abs(ANGLE) > TOLERANCE.

/* Plans */

+!start: facing_wall(Distance,Angle) & appropriate_distance_from_wall(Distance)
    <-
    .print("appropriate distance from wall; wall following");
    !wall_follow(Angle * math.pi / 180.0).

+!start: facing_wall(Distance,Angle) & too_far_from_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("too far from the wall!");
    !wall_follow((-1 * AIM_ANGLE + Angle) * math.pi / 180.0).

+!start: facing_wall(Distance,Angle) & too_close_to_wall(Distance) & wall_follow_aim_angle(AIM_ANGLE)
    <-
    .print("too close to the wall!");
    !wall_follow((AIM_ANGLE + Angle) * math.pi / 180.0).

+!start: true
    <-
    .print("STUCK");
    !start.

+!wall_follow(Angle): wall_follow_speed(SPEED) & big_angle_change(Angle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, Angle);
    !start.

+!wall_follow(Angle): wall_follow_speed(SPEED) & not big_angle_change(Angle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, Angle);
    !start.