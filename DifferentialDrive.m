% Tim Ambrose
% 9 November 2017
% EEC289A - HW4 - Q2

% Adapted from supplied python code
%######################################################################
% Copyright (C)                                                       #
% 2016 Shangtong Zhang(zhangshangtong.cpp@gmail.com)                  #
% 2016 Kenta Shimada(hyperkentakun@gmail.com)                         #
% Permission given to modify the code as long as you keep this        #
% declaration at the top                                              #
%######################################################################

%----------------------CONSTANTS---------------------------
clear;
STATES = 19;
GAMMA = 1;
START_STATE = 11;
END_STATES = [1, STATES + 2];
C = struct('END_STATES', END_STATES,...
           'GAMMA', GAMMA,...
           'START_STATE', START_STATE);

%------------------------MAIN------------------------------
stateValues = zeros(1, STATES + 2);
states = 2:(STATES + 1);
realStateValues = (-20:2:20) ./ 20.0;
realStateValues(1) = 0;
realStateValues(end) = 0;

truncateValue = 0.55;
steps = 2 .^ (0:9);
alphas = 0 : 0.02 : 1.1;  % 0 : 0.03 : 1.1    for smoother curves
episodes = 10;
runs = 200;
errors = zeros(size(steps, 2), size(alphas, 2));

for run = 1:runs
   fprintf('\nRun: %d\tStep: ', run);
   for stepIdx = 1:size(steps, 2)
      fprintf('%d ', steps(stepIdx));
      for alphaIdx = 1:size(alphas, 2)
         currentStateValues = stateValues(:)';
         for ep = 1:episodes
            currentStateValues = TemporalDifference(currentStateValues,...
               steps(stepIdx), alphas(alphaIdx), C);
            errors(stepIdx, alphaIdx) = errors(stepIdx, alphaIdx) +...
               sqrt(sum((currentStateValues - realStateValues) .^ 2) / STATES);
         end
      end
   end
end

errors = errors ./ (episodes * runs);
errors(errors > truncateValue) = truncateValue;

hold on;
for i = 1:size(steps, 2)
   plot(alphas, errors(i, :));
end
xlabel('Alpha');
ylabel('RMS Error');
labels = strings(1, size(steps, 2));
for i = 1:size(labels, 2)
   labels(i) = sprintf('n = %d', steps(i));
end
legend(labels);
xlim([0 1.1]);
title('Random Walk, n-Step TD');