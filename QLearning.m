function [R, values] = QLearning(stateActionValues, stepSize, startState, goalState, C)
   global actionDestination;
   global actionRewards;
   R = 0.0;
   currentState = startState(:)';
   while currentState(1) ~= goalState(1) || currentState(2) ~= goalState(2)
      currentAction = ChooseAction(currentState, stateActionValues, C);
      reward = actionRewards(currentState(1), currentState(2), currentAction);
      R = R + reward;
      newState = [0, 0];
      newState(1:2) = actionDestination(currentState(1), currentState(2), currentAction, :);
      val = stateActionValues(currentState(1), currentState(2), currentAction);
      stateActionValues(currentState(1), currentState(2), currentAction) =...
         val + stepSize * (reward + C.GAMMA * max(stateActionValues(...
         newState(1), newState(2), :)) - val);
      currentState = newState(:)';
   end
   values = stateActionValues(:,:,:);
end