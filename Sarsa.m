function [R, values] = Sarsa(stateActionValues, expected, stepSize, startState, goalState, C)
   global actionDestination;
   global actionRewards;
   R = 0.0;
   currentState = startState(:)';
   currentAction = ChooseAction(currentState, stateActionValues, C);
   
   while currentState(1) ~= goalState(1) || currentState(2) ~= goalState(2)
      newState = [0, 0];
      newState(1:2) = actionDestination(currentState(1), currentState(2), currentAction, :);
      newAction = ChooseAction(newState, stateActionValues, C);
      reward = actionRewards(currentState(1), currentState(2), currentAction);
      R = R + reward;
      if ~expected
         valueTarget = stateActionValues(newState(1), newState(2), newAction);
      else
         valueTarget = 0.0;
         actionValues = zeros(1, C.ACTIONS);
         actionValues(:) = stateActionValues(newState(1), newState(2), :);
         bestActions = find(actionValues == max(ActionValues));
         for a = 1:C.ACTIONS
            if ismember(a, bestActions)
               valueTarget = valueTarget + ((1.0 - C.EPSILON) / size(bestActions, 2) +...
                  C.EPSILON / C.ACTIONS) * stateActionValues(newState(1), newState(2), a);
            else
               valueTarget = valueTarget + C.EPSILON / C.ACTIONS *...
                  stateActionValues(newState(1), newState(2), a);
            end
         end
      end
      
      valueTarget = valueTarget * C.GAMMA;
      val = stateActionValues(currentState(1), currentState(2), currentAction);
      stateActionValues(currentState(1), currentState(2), currentAction) =...
         val + stepSize * (reward + valueTarget - val);
      currentState = newState(:)';
      currentAction = newAction;
   end
   values = stateActionValues(:,:,:);
end