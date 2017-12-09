% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

function A = ChooseAction(state, stateActionValues, robot, env)
   destinations = zeros(env.C.ACTIONS, 2);
   for i = 1:env.C.ACTIONS
      if env.C.actions(i,1) < 0
         destinations(i,:) = robot.position;
      else
         angle = env.C.actions(i,1) + robot.orientation;
         destinations(i,:) = robot.position + [floor(1.5*cos(angle)),...
            floor(1.5*sin(angle))];
      end
   end
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
   if binornd(1, env.C.EPSILON) == 1
      A = randi(env.C.ACTIONS);
   else
      values = stateActionValues(state(1), state(2), state(3), ...
         possibleActions);
      maxValue = max(values);
      maxActions = find(values == maxValue);
      A = datasample(maxActions, 1);
   end
end