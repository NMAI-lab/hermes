!start.

/* Main Behaviour */

+!start: navigation(dock) & dockVisible
    <-
    .print("Reached the destination!");
    dock;
    -navigation(dock);
    !start.

+!start: navigation(NavInstruction) & intersection(ForwardDistance, LTurnDistance, UTurnDistance) & not navigation(dock)
    <-
    .print("Reached an intersection!");
    !handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance);
    -navigation(NavInstruction);
    !start.

+!start: facingWall(WallDistance, WallAngle)
    <-
    !wallFollow(WallDistance, WallAngle);
    !start.

+!start: true
    <-
    .print("Cannot do anything. Need perceptions.");
    !start.

/* navigation */

+navigationInstruction(NavInstruction)
    <-
    .print("Got navigation instruction:", NavInstruction);
    -+navigation(NavInstruction);
    -navigationInstruction(NavInstruction).

/* intersection Handling */

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(wall_follow)
    <-
    .print("At the intersection performing a right turn action...").

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(forward) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection performing a forward action...");
    !performRepeatedForwards(math.ceil(ForwardDistance / SPEED / ACTION_EXECUTION_DURATION)).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(l_turn) & lTurnAimAngle(L_TURN_AIM_ANGLE) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection performing a left turn action...");
    !performRepeatedTurns(L_TURN_AIM_ANGLE, 1 / ACTION_EXECUTION_DURATION);
    !performRepeatedForwards(math.ceil(LTurnDistance / SPEED / ACTION_EXECUTION_DURATION)).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(u_turn) & uTurnAimAngle(U_TURN_AIM_ANGLE) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection performing a U-turn action...");
    !performRepeatedTurns(U_TURN_AIM_ANGLE, 1 / ACTION_EXECUTION_DURATION);
    !performRepeatedForwards(math.ceil(UTurnDistance / SPEED / ACTION_EXECUTION_DURATION)).

/* Wall Following */

calculateWallDistanceError(WallDistanceError) :- 
    speed(SPEED) &
    wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE) &
    convertToRadian(WALL_FOLLOW_AIM_ANGLE, AimAngleRadian) &
    WallDistanceError = SPEED * math.sin(AimAngleRadian) / 4.

tooFarFromWall(WallDistance) :-
    wallFollowDistanceSetpoint(SETPOINT) &
    calculateWallDistanceError(WallDistanceError) &
    WallDistance > SETPOINT + WallDistanceError.

tooCloseToWall(WallDistance) :-
    wallFollowDistanceSetpoint(SETPOINT) &
    calculateWallDistanceError(WallDistanceError) &
    WallDistance < SETPOINT - WallDistanceError.

+!wallFollow(WallDistance, WallAngle): not tooFarFromWall(WallDistance) & not tooCloseToWall(WallDistance)
    <-
    .print("Appropriate distance from wall; wall following");
    !turn(WallAngle).

+!wallFollow(WallDistance, WallAngle): tooFarFromWall(WallDistance) & wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !turn(-1 * WALL_FOLLOW_AIM_ANGLE + WallAngle).

+!wallFollow(WallDistance, WallAngle): tooCloseToWall(WallDistance) & wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE)
    <-
    .print("Too close to the wall!");
    !turn(WALL_FOLLOW_AIM_ANGLE + WallAngle).

/* Helpers */

about_to_do_big_turn(Angle) :-
    wallFollowAngleChangeTolerance(TOLERANCE) &
    math.abs(Angle) > TOLERANCE.

convertToRadian(Angle, Result) :- Result = Angle * math.pi / 180.0.

+!performRepeatedForwards(NumForwardMoves): NumForwardMoves > 1 & speed(SPEED)
    <-
    .print("Going forward...");
    cmd_vel(SPEED, 0, 0, 0, 0, 0);
    !performRepeatedForwards(NumForwardMoves - 1).

+!performRepeatedForwards(NumForwardMoves): NumForwardMoves == 1 & speed(SPEED)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, 0).

+!performRepeatedTurns(Angle, NumTurns): NumTurns > 1
    <-
    .print("Turning...");
    !turn(Angle);
    !performRepeatedTurns(Angle, NumTurns - 1).

+!performRepeatedTurns(Angle, NumTurns): NumTurns == 1
    <-
    !turn(Angle).

+!turn(Angle): speed(SPEED) & aboutToDoBigTurn(Angle) & convertToRadian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, RadianAngle).

+!turn(Angle): speed(SPEED) & not aboutToDoBigTurn(Angle) & convertToRadian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED / 2, 0, 0, 0, 0, RadianAngle).