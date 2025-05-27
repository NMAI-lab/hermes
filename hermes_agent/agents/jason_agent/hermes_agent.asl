!start.

/* Main Behaviour */

+!start: facingWall(Distance, Angle)
    <-
    !wallFollow(Distance, Angle);
    !start.

+!start: true
    <-
    .print("Cannot do anything. Need perceptions.");
    !start.

/* Wall Following */

calculateWallDistanceError(Error) :- 
    speed(SPEED) &
    wallFollowAimAngle(AIM_ANGLE) &
    convertToRadian(AIM_ANGLE, AimAngleRadian) &
    Error = SPEED * math.sin(AimAngleRadian) / 4.

tooFarFromWall(Distance) :-
    wallFollowDistanceSetpoint(SETPOINT) &
    calculateWallDistanceError(Error) &
    Distance > SETPOINT + Error.

tooCloseToWall(Distance) :-
    wallFollowDistanceSetpoint(SETPOINT) &
    calculateWallDistanceError(Error) &
    Distance < SETPOINT - Error.

+!wallFollow(Distance, Angle): not tooFarFromWall(Distance) & not tooCloseToWall(Distance)
    <-
    .print("Appropriate distance from wall; wall following");
    !turn(Angle).

+!wallFollow(Distance, Angle): tooFarFromWall(Distance) & wallFollowAimAngle(AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !turn(-1 * AIM_ANGLE + Angle).

+!wallFollow(Distance, Angle): tooCloseToWall(Distance) & wallFollowAimAngle(AIM_ANGLE)
    <-
    .print("Too close to the wall!");
    !turn(AIM_ANGLE + Angle).

/* Helpers */

about_to_do_big_turn(Angle) :-
    wallFollowAngleChangeTolerance(TOLERANCE) &
    math.abs(Angle) > TOLERANCE.

convertToRadian(Angle, Result) :- Result = Angle * math.pi / 180.0.

+!turn(Angle): speed(SPEED) & aboutToDoBigTurn(Angle) & convertToRadian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, RadianAngle).

+!turn(Angle): speed(SPEED) & not aboutToDoBigTurn(Angle) & convertToRadian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, RadianAngle).