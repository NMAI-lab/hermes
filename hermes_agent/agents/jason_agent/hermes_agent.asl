!wallFollow.

/* Main Behaviour */

// Navigation
+navigationInstruction(NavInstruction): true
    <-
    .print("Got navigation instruction:", NavInstruction);
    -+navigation(NavInstruction);
    -navigationInstruction(NavInstruction).

// Collision Handling
+bumperPressed: actionExecutionDuration(ACTION_EXECUTION_DURATION) & not(.desire(performRepeatedBackwards(_)))
    <-
    .print("Bumper was pressed! Backing up...");
    .drop_all_desires;
    !performRepeatedBackwards(math.ceil(1 / ACTION_EXECUTION_DURATION));
    !wallFollow.

// Docking
+dockVisible: navigation(dock)
    <-
    .print("Reached the destination!");
    .drop_all_desires;
    dock;
    -navigation(dock);
    !wallFollow.

// Intersection Detection
+intersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(NavInstruction) & not(navigation(dock)) & not(.desire(handleIntersection(_,_,_)))
    <-
    .print("Reached an intersection!");
    .drop_all_desires;
    !handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance);
    -navigation(NavInstruction);
    !wallFollow.

// Wall Detection
+!wallFollow: facingWall(WallDistance, WallAngle)
    <-
    .print("Wall following...");
    !alignWithWall(WallDistance, WallAngle);
    !wallFollow.

+!wallFollow: true
    <-
    .print("Cannot wall follow. Need perceptions.");
    !wallFollow.

/* Intersection Handling */

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(wall_follow)
    <-
    .print("At the intersection; will continue wall following...").

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(forward) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection; performing a forward action...");
    !performRepeatedForwards(math.ceil(ForwardDistance / SPEED / ACTION_EXECUTION_DURATION)).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(l_turn) & lTurnAimAngle(L_TURN_AIM_ANGLE) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection; performing a left turn action...");
    !performRepeatedTurns(L_TURN_AIM_ANGLE, 1 / ACTION_EXECUTION_DURATION);
    !performRepeatedForwards(math.ceil(LTurnDistance / SPEED / ACTION_EXECUTION_DURATION)).

+!handleIntersection(ForwardDistance, LTurnDistance, UTurnDistance): navigation(u_turn) & uTurnAimAngle(U_TURN_AIM_ANGLE) & speed(SPEED) & actionExecutionDuration(ACTION_EXECUTION_DURATION)
    <-
    .print("At the intersection; performing a U-turn action...");
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

+!alignWithWall(WallDistance, WallAngle): not(tooFarFromWall(WallDistance)) & not(tooCloseToWall(WallDistance))
    <-
    .print("Appropriate distance from wall.");
    !turn(WallAngle).

+!alignWithWall(WallDistance, WallAngle): tooFarFromWall(WallDistance) & wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE)
    <-
    .print("Too far from the wall!");
    !turn(-1 * WALL_FOLLOW_AIM_ANGLE + WallAngle).

+!alignWithWall(WallDistance, WallAngle): tooCloseToWall(WallDistance) & wallFollowAimAngle(WALL_FOLLOW_AIM_ANGLE)
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