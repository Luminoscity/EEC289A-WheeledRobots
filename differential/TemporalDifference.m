function values = TemporalDifference(stateValues, n, alpha, C)
   currentState = C.START_STATE;
   states = zeros(1, 100);
   rewards = zeros(1, 100);
   states(1) = currentState;
   numStates = 1;
   
   time = 0;
   T = Inf;
   
   while true
      time = time + 1;
      if time < T
         if binornd(1, 0.5) == 1
            newState = currentState + 1;
         else
            newState = currentState - 1;
         end
         
         if newState == 1
            reward = -1;
         elseif newState == 21
            reward = 1;
         else
            reward = 0;
         end
         
         numStates = numStates + 1;
         if numStates > size(states, 2)
            states = [states, zeros(1, size(states, 2))];
            rewards = [rewards, zeros(1, size(rewards, 2))];
         end
         states(numStates) = newState;
         rewards(numStates) = reward;
         
         if ismember(newState, C.END_STATES)
            T = time;
         end
      end
      
      updateTime = time - n;
      if updateTime >= 0
         returns = 0.0;
         for t = (updateTime + 1):min(T, updateTime + n)
            returns = returns + C.GAMMA ^ (t - updateTime - 1) * rewards(t+1);
         end
         if updateTime + n <= T
            returns = returns + C.GAMMA ^ n * stateValues(states(updateTime + n + 1));
         end
         stateToUpdate = states(updateTime + 1);
         if ~ismember(stateToUpdate, C.END_STATES)
            stateValues(stateToUpdate) = stateValues(stateToUpdate) + alpha...
               * (returns - stateValues(stateToUpdate));
         end
      end
      if updateTime == T - 1
         break;
      end
      currentState = newState;
   end
   values = stateValues(:)';
end