% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

function A = ChooseAction(state, stateActionValues, C)
   if binornd(1, C.EPSILON) == 1
      A = randi(4);
   else
      values = stateActionValues(state(1), state(2), :);
      maxValue = max(values);
      maxActions = find(values == maxValue);
      A = datasample(maxActions, 1);
   end
end