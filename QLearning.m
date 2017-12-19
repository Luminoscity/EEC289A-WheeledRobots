function [R, values] = QLearning(stateActionValues, robot, env, ...
        stateDefinition, worldString, MOVE_DRAW)
   stepSize = env.C.ALPHA;   
   R = 0.0;
   [goalDir, ~] = robot.GoalBearing(env);
   [closeIdx, farIdx, close, far] = robot.ReadCloseFarAngles(env);
   if stateDefinition == 1
      currentState = [closeIdx, farIdx, goalDir];
   elseif stateDefinition == 2
      currentState = [closeIdx, farIdx, goalDir, close, far];
   else
      currentState = [robot.position(1), robot.position(2)];
   end
   currentPos = robot.position(:)';
   goalPos = env.goal(:)';
   
   moves = 0;
   while currentPos(1) ~= goalPos(1) || currentPos(2) ~= goalPos(2)
      currentAction = ChooseAction(currentState, stateActionValues, ...
         robot, env, stateDefinition);
      [reward, closeIdx, farIdx, goalDir, close, far] = robot.Move(...
         currentAction, env, stateDefinition);
      moves = moves + 1;
      if mod(moves, MOVE_DRAW) == 0
         robot.ShowWorld(env, worldString);
      end
      
      R = R + reward;
      if stateDefinition == 1
         newState = [closeIdx, farIdx, goalDir];
         val = stateActionValues(currentState(1), currentState(2), ...
            currentState(3), currentAction);
         stateActionValues(currentState(1), currentState(2), ...
            currentState(3), currentAction) = val + stepSize * (reward + ...
            env.C.GAMMA * max(stateActionValues(newState(1), newState(2), ...
            newState(3), :)) - val);
      elseif stateDefinition == 2
        newState = [closeIdx, farIdx, goalDir, close, far];
        val = stateActionValues(currentState(1), currentState(2), ...
          currentState(3), currentState(4), currentState(5), ...
          currentAction);
        stateActionValues(currentState(1), currentState(2), ...
          currentState(3), currentState(4), currentState(5), ...
          currentAction) = val + stepSize * (reward + ...
          env.C.GAMMA * max(stateActionValues(newState(1), newState(2), ...
          newState(3), newState(4), newState(5), :)) - val);
      else
         newState = [robot.position(1), robot.position(2)];
         val = stateActionValues(currentState(1), currentState(2), ...
            currentAction);
         stateActionValues(currentState(1), currentState(2), ...
            currentAction) = val + stepSize * (reward + ...
            env.C.GAMMA * max(stateActionValues(newState(1), ...
            newState(2), :)) - val);
      end
      currentState = newState(:)';
      currentPos = robot.position(:)';
   end
   if stateDefinition == 1
      values = stateActionValues(:,:,:,:);
   elseif stateDefinition == 2
      values = stateActionValues(:,:,:,:,:,:);
   else
      values = stateActionValues(:,:,:);
   end
end