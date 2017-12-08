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
WORLD_WIDTH = 30;%60;
WORLD_HEIGHT = 30;%60
OBSTACLE_SEPARATION = 2;
MIN_OBSTACLES = 2;
MAX_OBSTACLES = 15;%35
MAX_OBSTACLE_AREA = 40;%80
DISTANCE_SENSOR_RESOLUTION = 0.1;

EPSILON = 0.1;
ALPHA = 0.5;
GAMMA = 1;

ACTION_NORTH = [0, 1, 0];
ACTION_LEFT = [0, 0, -pi/4];   %turn left slightly
ACTION_RIGHT = [0, 0, pi/4];  %turn right slightly
ACTION_NORTHWEST = [1, -1, 0];
ACTION_WEST = [1, 0, 0];
ACTION_SOUTHWEST = [1, 1, 0];
ACTION_SOUTH = [0, 1, 0];
ACTION_SOUTHEAST = [-1, 1, 0];
ACTION_EAST = [-1, 0, 0];
ACTION_NORTHEAST = [-1, -1, 0];
DIR_NORTH = pi / 2;
DIR_NORTHWEST = pi / 4;
DIR_WEST = 0;
DIR_SOUTHWEST = 7 * pi / 4;
DIR_SOUTH = 3 * pi / 2;
DIR_SOUTHEAST = 5 * pi / 4;
DIR_EAST = pi;
DIR_NORTHEAST = 3 * pi / 4;

actions = [ACTION_NORTH; ACTION_LEFT; ACTION_RIGHT; ACTION_NORTHWEST;...
           ACTION_WEST; ACTION_SOUTHWEST; ACTION_SOUTH;...
           ACTION_SOUTHEAST; ACTION_EAST; ACTION_NORTHEAST];
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
           'MIN_OBSTACLES', MIN_OBSTACLES,...
           'MAX_OBSTACLES', MAX_OBSTACLES,...
           'OBS_SPACE', OBSTACLE_SEPARATION,...
           'WORLD_WIDTH', WORLD_WIDTH,...
           'WORLD_HEIGHT', WORLD_HEIGHT,...
           'DIST_RES', DISTANCE_SENSOR_RESOLUTION);

%------------------------MAIN---------------------------
env = Environment(C);
env.GenerateObstacles();
imagesc(env.whichObstacles)
colormap Jet;
colorbar;

robot = WheeledRobot(env);
robot.StartAt(env.start, pi/2);
distances = robot.ReadDistances(env);
figure
for i = 1:C.ANGLES
   polarplot([C.angles(i), (C.angles(i)+0.001)], [0, distances(i)])
   hold on
end
hold off

%{
env.GenerateObstacles();
figure
imagesc(env.whichObstacles)
colormap Jet
colorbar;

env.GenerateObstacles();
figure
imagesc(env.whichObstacles)
colormap Jet
colorbar;
%}
stateActionValues = zeros(C.ANGLES, C.DISTS, C.DIRS, C.ACTIONS);

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
EPISODES = 100;
RUNS = 10;

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
%{
plot(sarsaResults);
hold on
plot(qLearningResults);
legend('Sarsa', 'Q-Learning');
xlabel('Episodes');
ylabel('Sum of Rewards During Episode');
title('Cliff Walking');
%}
hold off
%}