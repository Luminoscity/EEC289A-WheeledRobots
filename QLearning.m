function [R, values] = QLearning(stateActionValues, robot, env)
   stepSize = env.C.ALPHA;   
   R = 0.0;
   [goalDir, ~] = robot.GoalBearing(env);
   [closeIdx, farIdx] = robot.ReadCloseFarAngles(env);
   currentState = [closeIdx, farIdx, goalDir];
   currentPos = robot.position(:)';
   goalPos = env.goal(:)';
   while currentPos(1) ~= goalPos(1) || currentPos(2) ~= goalPos(2)
      currentAction = ChooseAction(currentState, stateActionValues, ...
         robot, env);
      [reward, closeIdx, farIdx, goalDir] = robot.Move(currentAction, env);
      R = R + reward;
      newState = [closeIdx, farIdx, goalDir];
      val = stateActionValues(currentState(1), currentState(2), ...
         currentState(3), currentAction);
      stateActionValues(currentState(1), currentState(2), ...
         currentState(3), currentAction) = val + stepSize * (reward + ...
         env.C.GAMMA * max(stateActionValues(newState(1), newState(2), ...
         newState(3), :)) - val);
      currentState = newState(:)';
      currentPos = robot.position(:)';
   end
   values = stateActionValues(:,:,:,:);
end