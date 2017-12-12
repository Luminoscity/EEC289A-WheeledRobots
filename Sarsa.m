function [R, values] = Sarsa(stateActionValues, expected, robot, ...
        env, stateDefinition, worldString, MOVE_DRAW)
   stepSize = env.C.ALPHA;
   R = 0.0;
   [goalDir, ~] = robot.GoalBearing(env);
   [closeIdx, farIdx, close, far] = robot.ReadCloseFarAngles(env);
   if stateDefinition == 1
      currentState = [closeIdx, farIdx, goalDir];
   else
      currentState = [closeIdx, farIdx, goalDir, close, far];
   end
   currentPos = robot.position(:)';
   goalPos = env.goal(:)';
   currentAction = ChooseAction(currentState, stateActionValues, robot, ...
      env, stateDefinition);
   
   moves = 0;
   while currentPos(1) ~= goalPos(1) || currentPos(2) ~= goalPos(2)
      [reward, closeIdx, farIdx, goalDir, close, far] = robot.Move(...
         currentAction, env);
      moves = moves + 1;
      if mod(moves, MOVE_DRAW) == 0
         robot.ShowWorld(env, worldString);
      end
         
      if stateDefinition == 1
         newState = [closeIdx, farIdx, goalDir];
      else
         newState = [closeIdx, farIdx, goalDir, close, far];
      end
      newAction = ChooseAction(newState, stateActionValues, robot, env, ...
          stateDefinition);
      R = R + reward;
      if ~expected
         if stateDefinition == 1
            valueTarget = stateActionValues(newState(1), newState(2), ...
               newState(3), newAction);
         else
            valueTarget = stateActionValues(newState(1), newState(2), ...
               newState(3), newState(4), newState(5), newAction);
         end
      else
         valueTarget = 0.0;
         actionValues = zeros(1, env.C.ACTIONS);
         if stateDefinition == 1
           actionValues(:) = stateActionValues(newState(1), newState(2), ...
              newState(3), :);
         else
           actionValues(:) = stateActionValues(newState(1), newState(2), ...
              newState(3), newState(4), newState(5), :);
         end
         bestActions = find(actionValues == max(actionValues));
         for a = 1:env.C.ACTIONS
           if ismember(a, bestActions)
             if stateDefinition == 1
               valueTarget = valueTarget + ((1.0 - env.C.EPSILON) / ...
                  size(bestActions, 2) + env.C.EPSILON / env.C.ACTIONS) ...
                  * stateActionValues(newState(1), newState(2), newState(3), a);
             else
               valueTarget = valueTarget + ((1.0 - env.C.EPSILON) / ...
                  size(bestActions, 2) + env.C.EPSILON / env.C.ACTIONS) ...
                  * stateActionValues(newState(1), newState(2), ...
                  newState(3), newState(4), newState(5), a);
             end
           else
              if stateDefinition == 1
              valueTarget = valueTarget + env.C.EPSILON / env.C.ACTIONS...
                 * stateActionValues(newState(1), newState(2), ...
                 newState(3), a);
              else
              valueTarget = valueTarget + env.C.EPSILON / env.C.ACTIONS...
                 * stateActionValues(newState(1), newState(2), ...
                 newState(3), newState(4), newState(5), a);
              end
           end
         end
      end
      
      valueTarget = valueTarget * env.C.GAMMA;
      if stateDefinition == 1
         val = stateActionValues(currentState(1), currentState(2), ...
            currentState(3), currentAction);
         stateActionValues(currentState(1), currentState(2), ...
            currentState(3), currentAction) = val + stepSize * (reward + ...
            valueTarget - val);
      else
         val = stateActionValues(currentState(1), currentState(2), ...
            currentState(3), currentState(4), currentState(5), ...
            currentAction);
         stateActionValues(currentState(1), currentState(2), ...
            currentState(3), currentState(4), currentState(5), ...
            currentAction) = val + stepSize * (reward + ...
            valueTarget - val);
      end
      currentState = newState(:)';
      currentAction = newAction;
      currentPos = robot.position(:)';
   end
   if stateDefinition == 1
      values = stateActionValues(:,:,:,:);
   else
      values = stateActionValues(:,:,:,:,:,:);
   end
end