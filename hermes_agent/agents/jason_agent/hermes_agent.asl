/* Initial Goals */

!requestTrip.

/* Main Behaviour */

// Navigation
+navigationInstruction(start): true
    <-
    .print("Got a new trip!");
    +hasTrip.

+navigationInstruction(NavInstruction): true
    <-
    .print("Got navigation instruction:", NavInstruction);
    -+navigation(NavInstruction).

// Collision Detection
+bumperPressed: true
    <-
    .print("Bumper was pressed! Backing up...");
    .drop_all_intentions;
    !handleCollision.

// Dock Station Detection
+dockVisible: navigation(dock) & not(.intend(handleDocking)) & not(.intend(handleCollision))
    <-
    .print("Reached the destination!");
    .drop_all_intentions;
    !handleDocking.

// Intersection Detection
+intersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(NavInstruction) & facingWall(WallDistance, WallAngle) & not(navigation(dock)) & not(.intend(handleIntersection(_,_,_,_))) & not(.intend(handleDocking)) & not(.intend(handleCollision))
    <-
    .print("Reached an intersection!");
    .drop_all_intentions;
    !handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance, WallAngle).

// Wall Detection
+facingWall(WallDistance, WallAngle): hasTrip & not(.intend(wallFollow(_,_))) & not(.intend(handleIntersection(_,_,_,_))) & not(.intend(handleDocking)) & not(.intend(handleCollision))
    <-
    .print("Wall following...");
    !wallFollow(WallDistance, WallAngle).

// Requesting trips
-hasTrip: true
    <-
    .print("Completed a trip. Will try to request another one!");
    !requestTrip.

+!requestTrip: not(hasTrip)
    <-
    .print("Requeting a new trip!!");
    request_trip;
    !requestTrip.

/* Collision Handling */

+!handleCollision: actionExecutionDuration(ACTION_EXECUTION_DURATION) & speed(SPEED) & wallFollowDistanceSetpoint(SETPOINT)
    <-
    !performRepeatedBackwards(math.ceil(SETPOINT / SPEED / ACTION_EXECUTION_DURATION)).

/* Docking */

+!handleDocking: true
    <-
    dock;
    -hasTrip;
    -navigation(dock).

/* Intersection Handling */

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance, WallAngle): navigation(wall_follow)
    <-
    .print("At the intersection; will continue wall following...");
    -navigation(wall_follow).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance, WallAngle): navigation(forward) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection; performing a forward action...");
    !performRepeatedTurns(-1 * WallAngle, math.ceil(1 / ACTION_EXECUTION_DURATION));
    !performRepeatedForwards(math.ceil(ForwardDistance / SPEED / ACTION_EXECUTION_DURATION));
    -navigation(forward).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance, WallAngle): navigation(l_turn) & lTurnAimAngle(L_TURN_AIM_ANGLE) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection; performing a left turn action...");
    !performRepeatedTurns(L_TURN_AIM_ANGLE - WallAngle, math.ceil(1 / ACTION_EXECUTION_DURATION));
    !performRepeatedForwards(math.ceil(LTurnDistance / SPEED / ACTION_EXECUTION_DURATION));
    -navigation(l_turn).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance, WallAngle): navigation(u_turn) & uTurnAimAngle(U_TURN_AIM_ANGLE) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection; performing a U-turn action...");
    !performRepeatedTurns(U_TURN_AIM_ANGLE - WallAngle, math.ceil(1 / ACTION_EXECUTION_DURATION));
    !performRepeatedForwards(math.ceil(UTurnDistance / SPEED / ACTION_EXECUTION_DURATION));
    -navigation(u_turn).

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

+!wallFollow(WallDistance, WallAngle): not(tooFarFromWall(WallDistance)) & not(tooCloseToWall(WallDistance))
    <-
    .print("Appropriate distance from wall.");
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

aboutToDoBigTurn(Angle) :-
    angleChangeTolerance(TOLERANCE) &
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

+!performRepeatedBackwards(NumBackwardMoves): NumBackwardMoves > 1 & speed(SPEED)
    <-
    .print("Going backwards...");
    cmd_vel(-1 * SPEED, 0, 0, 0, 0, 0);
    !performRepeatedBackwards(NumBackwardMoves - 1).

+!performRepeatedBackwards(NumBackwardMoves): NumBackwardMoves == 1 & speed(SPEED)
    <-
    cmd_vel(-1 * SPEED, 0, 0, 0, 0, 0).

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
    cmd_vel(SPEED / 2, 0, 0, 0, 0, RadianAngle).

+!turn(Angle): speed(SPEED) & not(aboutToDoBigTurn(Angle)) & convertToRadian(Angle, RadianAngle)
    <-
    cmd_vel(SPEED, 0, 0, 0, 0, RadianAngle).