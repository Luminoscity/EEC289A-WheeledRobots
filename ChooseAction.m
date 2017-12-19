% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

function A = ChooseAction(state, stateActionValues, robot, env, ...
   stateDefinition)
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
      if destinations(i,1) > 0 && destinations(i,1) <= ...
         env.C.WORLD_WIDTH && destinations(i,2) > 0 && ...
         destinations(i,2) <= env.C.WORLD_HEIGHT && ...
         ~env.obstacles(destinations(i,2), destinations(i,1))

         numPossible = numPossible + 1;
         possibleActions(numPossible) = i;
      end
   end
   possibleActions = possibleActions(1:numPossible);
      
   [~, goalAngle] = robot.GoalBearing(env);
   goalDist = robot.distanceToGoal;
   if robot.ReadDistance(env, goalAngle) > goalDist + 2 * env.C.DIST_RES
      possibleDists = zeros(1, length(possibleActions));
      for i = 1:length(possibleActions)
         possibleDists(i) = sqrt((env.goal(1) - destinations(...
            possibleActions(i),1))^2 + (env.goal(2) - destinations(...
            possibleActions(i),2))^2);
      end
      [~, A] = min(possibleDists);
      A = possibleActions(A);
   else
      %choose action based on epsilon-greedy
      if binornd(1, env.C.EPSILON) == 1
         A = datasample(possibleActions, 1);
      else

         if stateDefinition == 1
            values = stateActionValues(state(1), state(2), state(3), ...
               possibleActions);
         elseif stateDefinition == 2
            values = stateActionValues(state(1), state(2), state(3), ...
               state(4), state(5), possibleActions);
         else
            values = stateActionValues(state(1), state(2), possibleActions);
         end
         maxValue = max(values);
         maxActions = find(values == maxValue);
         A = possibleActions(datasample(maxActions, 1));
      end
   end
end