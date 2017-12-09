% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

function A = ChooseAction(state, stateActionValues, robot, env)
   destinations = zeros(env.C.ACTIONS, 2);
   M = env.C.ROUND_MAGNITUDE;
   %determine possible next positions
   for i = 1:env.C.ACTIONS
      if env.C.actions(i,1) < 0
         destinations(i,:) = robot.position;
      else
         angle = env.C.actions(i,1) + robot.orientation;
         destinations(i,:) = robot.position + double([int32(M * ...
            cos(angle)), int32(M * sin(angle))]);
      end
   end
   %determine actions that don't fall off the world or move towards an
   %object 0 cm away
   numPossible = 0;
   possibleActions = zeros(1, env.C.ACTIONS);
   for i = 1:env.C.ACTIONS
      if destinations(i,1) > 0 && destinations(i,1) <= env.C.WORLD_WIDTH...
         && destinations(i,2) > 0 && destinations(i,2) <= ...
         env.C.WORLD_HEIGHT && ~env.obstacles(destinations(i,2),...
         destinations(i,1))
      
         numPossible = numPossible + 1;
         possibleActions(numPossible) = i;
      end
   end
   possibleActions = possibleActions(1:numPossible);
   
   %choose action based on epsilon-greedy
   if binornd(1, env.C.EPSILON) == 1
      A = datasample(possibleActions, 1);
   else
      
      sz = size(stateActionValues);
      if length(sz) < 4 || state(1) > sz(1) || state(2) > sz(2) || state(3)...
            > sz(3) || max(possibleActions) > sz(4)
         fprintf('')
      end
      
      values = stateActionValues(state(1), state(2), state(3), ...
         possibleActions);
      maxValue = max(values);
      maxActions = find(values == maxValue);
      if isempty(maxActions)
         fprintf('')
      end
      A = possibleActions(datasample(maxActions, 1));
   end
end