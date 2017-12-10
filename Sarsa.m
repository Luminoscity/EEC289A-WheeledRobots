function [R, values] = Sarsa(stateActionValues, expected, robot, env)
   stepSize = env.C.ALPHA;
   R = 0.0;
   [goalDir, ~] = robot.GoalBearing(env);
   [closeIdx, farIdx] = robot.ReadCloseFarAngles(env);
   currentState = [closeIdx, farIdx, goalDir];
   currentPos = robot.position(:)';
   goalPos = env.goal(:)';
   currentAction = ChooseAction(currentState, stateActionValues, robot, ...
      env);
   
   while currentPos(1) ~= goalPos(1) || currentPos(2) ~= goalPos(2)
      [reward, closeIdx, farIdx, goalDir] = robot.Move(currentAction, env);
      newState = [closeIdx, farIdx, goalDir];
      newAction = ChooseAction(newState, stateActionValues, robot, env);
      R = R + reward;
      if ~expected
         valueTarget = stateActionValues(newState(1), newState(2), ...
            newState(3), newAction);
      else
         valueTarget = 0.0;
         actionValues = zeros(1, env.C.ACTIONS);
         actionValues(:) = stateActionValues(newState(1), newState(2), ...
            newState(3), :);
         bestActions = find(actionValues == max(actionValues));
         for a = 1:env.C.ACTIONS
            if ismember(a, bestActions)
               valueTarget = valueTarget + ((1.0 - env.C.EPSILON) / ...
                  size(bestActions, 2) + env.C.EPSILON / env.C.ACTIONS) ...
                  * stateActionValues(newState(1), newState(2), ...
                  newState(3), a);
            else
               valueTarget = valueTarget + env.C.EPSILON / env.C.ACTIONS...
                  * stateActionValues(newState(1), newState(2), ...
                  newState(3), a);
            end
         end
      end
      
      valueTarget = valueTarget * env.C.GAMMA;
      val = stateActionValues(currentState(1), currentState(2), ...
         currentState(3), currentAction);
      stateActionValues(currentState(1), currentState(2), ...
         currentState(3), currentAction) = val + stepSize * (reward + ...
         valueTarget - val);
      currentState = newState(:)';
      currentAction = newAction;
      currentPos = robot.position(:)';
   end
   values = stateActionValues(:,:,:,:);
end