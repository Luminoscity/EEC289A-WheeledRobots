% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - 

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
DIST_ANGLE_QUANTIZE = 30;    % number of measured distance angles
TURN_ANGLE = pi / 4;         % 45 degree turns
WORLD_WIDTH = 20;
WORLD_HEIGHT = 20;
OBSTACLE_SEPARATION = 1;
MAX_OBSTACLES = 5;
MAX_OBSTACLE_AREA = 10;

EPSILON = 0.1;
ALPHA = 0.5;
GAMMA = 1;

ACTION_NORTH = 1;
ACTION_LEFT = 2;   %turn left slightly
ACTION_RIGHT = 3;  %turn right slightly
ACTION_NORTHWEST = 4;
ACTION_WEST = 5;
ACTION_SOUTHWEST = 6;
ACTION_SOUTH = 7;
ACTION_SOUTHEAST = 8;
ACTION_EAST = 9;
ACTION_NORTHEAST = 10;
DIR_NORTH = pi / 2;
DIR_NORTHWEST = pi / 4;
DIR_WEST = 0;
DIR_SOUTHWEST = 7 * pi / 4;
DIR_SOUTH = 3 * pi / 2;
DIR_SOUTHEAST = 5 * pi / 4;
DIR_EAST = pi;
DIR_NORTHEAST = 3 * pi / 4;

actions = [ACTION_NORTH, ACTION_LEFT, ACTION_RIGHT, ACTION_NORTHWEST,...
           ACTION_WEST, ACTION_SOUTHWEST, ACTION_SOUTH,...
           ACTION_SOUTHEAST, ACTION_EAST, ACTION_NORTHEAST];
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
           'OBSTACLE_SEPARATION', OBSTACLE_SEPARATION,...
           'WORLD_WIDTH', WORLD_WIDTH,...
           'WORLD_HEIGHT', WORLD_HEIGHT);

%------------------------MAIN---------------------------

stateActionValues = zeros(C.ANGLES, C.DISTS, , C.ACTIONS);
%{
startState = [WORLD_HEIGHT, 1];
goalState = [WORLD_HEIGHT, WORLD_WIDTH];

global actionRewards;
actionRewards = zeros(WORLD_HEIGHT, WORLD_WIDTH, ACTIONS) - 1.0;
actionRewards(WORLD_HEIGHT - 1, 2:(WORLD_WIDTH - 1), ACTION_DOWN) = CLIFF;
actionRewards(WORLD_HEIGHT, 1, ACTION_RIGHT) = CLIFF;

global actionDestination;
actionDestination = zeros(WORLD_HEIGHT, WORLD_WIDTH, ACTIONS, 2);

for i = 1:WORLD_HEIGHT
   for j = 1:WORLD_WIDTH
      actionDestination(i, j, ACTION_UP, :) = [max(i-1, 1), j];
      actionDestination(i, j, ACTION_LEFT, :) = [i, max(j-1, 1)];
      actionDestination(i, j, ACTION_RIGHT, :) = [i, min(j+1, WORLD_WIDTH)];
      if i == WORLD_HEIGHT - 1 && j > 1 && j < WORLD_WIDTH
         actionDestination(i, j, ACTION_DOWN, :) = startState(:);
      else
         actionDestination(i, j, ACTION_DOWN, :) = [min(i+1, WORLD_HEIGHT), j];
      end
   end
end
actionDestination(WORLD_HEIGHT, 1, ACTION_RIGHT, :) = startState(:);

averageRange = 10;
EPISODES = 500;
RUNS = 20;

rewardsSarsa = zeros(1, EPISODES);
rewardsQLearning = zeros(1, EPISODES);
fprintf('Runs remaining: ');
for run = 1:RUNS
   fprintf('%d ', RUNS - run + 1);
   stateActionValuesSarsa = stateActionValues(:,:,:);
   stateActionValuesQLearning = stateActionValues(:,:,:);
   for i = 1:EPISODES
      [Value, stateActionValuesSarsa] = Sarsa(stateActionValuesSarsa,...
         false, ALPHA, startState, goalState, C);
      rewardsSarsa(i) = rewardsSarsa(i) + max(Value, CLIFF);
      [Value, stateActionValuesQLearning] = QLearning(stateActionValuesQLearning,...
         ALPHA, startState, goalState, C);
      rewardsQLearning(i) = rewardsQLearning(i) + max(Value, CLIFF);
   end
end
fprintf('\n');

rewardsSarsa = rewardsSarsa ./ RUNS;
rewardsQLearning = rewardsQLearning ./ RUNS;
sarsaResults = rewardsSarsa(:)';
qLearningResults = rewardsQLearning(:)';
for i = (averageRange + 1):EPISODES
   sarsaResults(i) = mean(rewardsSarsa((i - averageRange):i));
   qLearningResults(i) = mean(rewardsQLearning((i - averageRange):i));
end

%--------------------PLOT RESULTS--------------------------------
plot(sarsaResults);
hold on;
plot(qLearningResults);
legend('Sarsa', 'Q-Learning');
xlabel('Episodes');
ylabel('Sum of Rewards During Episode');
title('Cliff Walking');
%}
