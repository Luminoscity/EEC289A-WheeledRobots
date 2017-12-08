% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

% Adapted from python code from Sutton's website
%###################################################################
% Copyright (C)                                                    #
% 2016 Shangtong Zhang(zhangshangtong.cpp@gmail.com)               #
% 2016 Kenta Shimada(hyperkentakun@gmail.com)                      #
% Permission given to modify the code as long as you keep this     #
% declaration at the top                                           #
%###################################################################

%----------------------CONSTANTS---------------------------
clear;
DIST_QUANTIZE = 5;           % number of quantized distances
DIST_QUANT_STEP = 5;         % centimeters between quantized distances
DIST_ANGLE_QUANTIZE = 60;    % number of measured distance angles
TURN_ANGLE = pi / 4;         % 45 degree turns
WORLD_WIDTH = 30;
WORLD_HEIGHT = 30;
OBSTACLE_SEPARATION = 2;
MAX_OBSTACLES = 12;
MAX_OBSTACLE_AREA = 40;
DISTANCE_SENSOR_RESOLUTION = 0.1;

EPSILON = 0.1;
ALPHA = 0.5;
GAMMA = 1;

ACTION_NORTH = [0, 1, 0];
ACTION_LEFT = [0, 0, -pi/4];   %turn left slightly
ACTION_RIGHT = [0, 0, pi/4];  %turn right slightly

DIR_NORTH = pi / 2;
DIR_NORTHWEST = pi / 4;
DIR_WEST = 0;
DIR_SOUTHWEST = 7 * pi / 4;
DIR_SOUTH = 3 * pi / 2;
DIR_SOUTHEAST = 5 * pi / 4;
DIR_EAST = pi;
DIR_NORTHEAST = 3 * pi / 4;

actions = [ACTION_NORTH, ACTION_LEFT, ACTION_RIGHT];
directions = [DIR_NORTH, DIR_NORTHWEST, DIR_WEST, DIR_SOUTHWEST,...
              DIR_SOUTH, DIR_SOUTHEAST, DIR_EAST, DIR_NORTHEAST];
angleStep = 2 * pi / DIST_ANGLE_QUANTIZE;
angles = 0:angleStep:(2 * pi - angleStep);

C = struct('EPSILON', EPSILON,...
           'ALPHA', ALPHA,...
           'GAMMA', GAMMA,...
           'ACTIONS', length(actions),...
           'actions', actions,...
           'DIRS', length(directions),...
           'directions', directions,...
           'ANGLES', length(angles),...
           'angles', angles,...
           'DISTS', DIST_QUANTIZE,...
           'MAX_OBSTACLE_AREA', MAX_OBSTACLE_AREA,...
           'MAX_OBSTACLES', MAX_OBSTACLES,...
           'OBS_SPACE', OBSTACLE_SEPARATION,...
           'WORLD_WIDTH', WORLD_WIDTH,...
           'WORLD_HEIGHT', WORLD_HEIGHT,...
           'DIST_RES', DISTANCE_SENSOR_RESOLUTION);

%------------------------MAIN---------------------------
env = Environment(C);
env.GenerateObstacles();

stateValues = zeros(1, STATES + 2);
%{
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
%}