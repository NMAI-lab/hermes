!start.

/* Main Behaviour */

+!start: navigationInstruction(NavInstruction)
    <-
    .print("Got navigation instruction:", NavInstruction);
    +navigation(NavInstruction);
    -navigationInstruction(NavInstruction);
    !start.

+!start: navigation(NavInstruction) & intersection(ForwardDistance, LTurnDistance, UTurnDistance)
    <-
    .print("Reached an intersection!");
    !handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance).
    //!start.

+!start: facingWall(Distance, Angle)
    <-
    !wallFollow(Distance, Angle);
    !start.

+!start: true
    <-
    .print("Cannot do anything. Need perceptions.");
    !start.

/* intersection Handling */

calculateNumForwardMoves(Distance, NumForwardMoves) :-
    speed(SPEED) &
    NumForwardMoves = math.ceil(Distance / SPEED).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(WALL_FOLLOW) & facingWall(Distance, Angle)
    <-
    .print("At the intersection performing a right turn action...");
    !wallFollow(Distance, Angle);
    -navigation(WALL_FOLLOW).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(FORWARD) & calculateNumForwardMoves(ForwardDistance, NumForwardMoves)
    <-
    .print("At the intersection performing a forward action...");
    !goThroughIntersection(NumForwardMoves).
    -navigation(FORWARD).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(L_TURN) & lTurnAimAngle(L_TURN_AIM_ANGLE) & calculateNumForwardMoves(LTurnDistance, NumForwardMoves)
    <-
    .print("At the intersection performing a left turn action...");
    !turn(L_TURN_AIM_ANGLE);
    !goThroughIntersection(NumForwardMoves);
    -navigation(L_TURN).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(U_TURN) & uTurnAimAngle(U_TURN_AIM_ANGLE) & calculateNumForwardMoves(UTurnDistance, NumForwardMoves)
    <-
    .print("At the intersection performing a U-turn action...")
    !turn(U_TURN_AIM_ANGLE);
    !goThroughIntersection(NumForwardMoves);
    -navigation(U_TURN).

+!goThroughIntersection(NumForwardMoves): NumForwardMoves > 0
    <-
    .print("Passing through the intersection...");
    cmd_vel(SPEED, 0, 0, 0, 0, 0);
    !goThroughIntersection(NumForwardMoves - 1).

+!goThroughIntersection(NumForwardMoves): NumForwardMoves <= 0
    <-
    .print("Completed the intersection!").

/* Wall Following */

calculateWallDistanceError(Error) :- 
    speed(SPEED) &
    wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE) &
    convertToRadian(WALL_FOLLOW_AIM_ANGLE, AimAngleRadian) &
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

+!wallFollow(Distance, Angle): tooFarFromWall(Distance) & wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !turn(-1 * WALL_FOLLOW_AIM_ANGLE + Angle).

+!wallFollow(Distance, Angle): tooCloseToWall(Distance) & wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE)
    <-
    .print("Too close to the wall!");
    !turn(WALL_FOLLOW_AIM_ANGLE + Angle).

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