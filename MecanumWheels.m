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
clear
close all
PLOT_WORLD = true;
PLOT_INITIAL_DISTANCES = false;
PROGRESS_STEP = 5;
DIST_QUANTIZE = 5;           % number of quantized distances
DIST_QUANT_STEP = 5;         % centimeters between quantized distances
DIST_ANGLE_MEAS = 60;        % number of angles to measure distances at
DIST_ANGLE_QUANTIZE = 16;    % number of quantized distance angles to
                             % record as part of the state
TURN_ANGLE = pi / 4;         % 45 degree turns
WORLD_WIDTH = 15;%40;%60;
WORLD_HEIGHT = 15;%40;%60;
OBSTACLE_SEPARATION = 2;
MIN_OBSTACLES = 2;
MAX_OBSTACLES = 10;%15;%35;
MAX_OBSTACLE_AREA = 20;%40;%80;
DISTANCE_SENSOR_RESOLUTION = 0.1;
ROUND_MAGNITUDE = 1.49;
REWARD_FLOOR = -200;

EPSILON = 0.1;
ALPHA = 0.5;
GAMMA = 1;

ACTION_NORTH = [2 * TURN_ANGLE, 0];
ACTION_LEFT = [-1, -TURN_ANGLE];   %turn left slightly
ACTION_RIGHT = [-1, TURN_ANGLE];   %turn right slightly
ACTION_NORTHWEST = [TURN_ANGLE, 0];
ACTION_WEST = [0, 0];
ACTION_SOUTHWEST = [7 * TURN_ANGLE, 0];
ACTION_SOUTH = [6 * TURN_ANGLE, 0];
ACTION_SOUTHEAST = [5 * pi/4, 0];
ACTION_EAST = [4 * TURN_ANGLE, 0];
ACTION_NORTHEAST = [3 * pi/4, 0];
DIR_WEST = 0;
DIR_NORTHWEST = pi / 4;
DIR_NORTH = pi / 2;
DIR_NORTHEAST = 3 * pi / 4;
DIR_EAST = pi;
DIR_SOUTHEAST = 5 * pi / 4;
DIR_SOUTH = 3 * pi / 2;
DIR_SOUTHWEST = 7 * pi / 4;

actions = [ACTION_NORTH; ACTION_LEFT; ACTION_RIGHT; ACTION_NORTHWEST;...
           ACTION_WEST; ACTION_SOUTHWEST; ACTION_SOUTH;...
           ACTION_SOUTHEAST; ACTION_EAST; ACTION_NORTHEAST];
directions = [DIR_WEST, DIR_NORTHWEST, DIR_NORTH, DIR_NORTHEAST,...
              DIR_EAST, DIR_SOUTHEAST, DIR_SOUTH, DIR_SOUTHWEST];
angleStep = 2 * pi / DIST_ANGLE_MEAS;
angles = 0:angleStep:(2 * pi - angleStep);
angleStep = 2 * pi / DIST_ANGLE_QUANTIZE;
qAngles = 0:angleStep:(2 * pi - angleStep);

C = struct('EPSILON', EPSILON,...
           'ALPHA', ALPHA,...
           'GAMMA', GAMMA,...
           'ACTIONS', length(actions),...
           'actions', actions,...
           'DIRS', length(directions),...
           'directions', directions,...
           'ANGLES', length(angles),...
           'angles', angles,...
           'QUANT_ANGLES', length(qAngles),...
           'qAngles', qAngles,...
           'DISTS', DIST_QUANTIZE,...
           'DIST_STEP', DIST_QUANT_STEP,...
           'MAX_OBSTACLE_AREA', MAX_OBSTACLE_AREA,...
           'MIN_OBSTACLES', MIN_OBSTACLES,...
           'MAX_OBSTACLES', MAX_OBSTACLES,...
           'OBS_SPACE', OBSTACLE_SEPARATION,...
           'WORLD_WIDTH', WORLD_WIDTH,...
           'WORLD_HEIGHT', WORLD_HEIGHT,...
           'DIST_RES', DISTANCE_SENSOR_RESOLUTION,...
           'ROUND_MAGNITUDE', ROUND_MAGNITUDE);

%------------------------MAIN---------------------------
env = Environment(C);
error = env.GenerateObstacles();
if error > 0
   fprintf('\nWorld Creation failed, please run again\n')
   return
end

if PLOT_WORLD
   imagesc(env.whichObstacles)
   set(gca, 'YDir', 'Normal')
   colormap Jet;
   colorbar;
end

robot = WheeledRobot(env);
robot.StartAt(env.start, 0);

if PLOT_INITIAL_DISTANCES
   distances = robot.ReadDistances(env);
   figure
   for i = 1:C.ANGLES
      polarplot([C.angles(i), (C.angles(i)+0.001)], [0, distances(i)])
      hold on
   end
   [dirToGoal, angle] = robot.GoalBearing(env);
   polarplot([angle, (angle+0.001)], [0, robot.distanceToGoal], '--k')
   polarplot([directions(dirToGoal), (directions(dirToGoal)+0.001)], ...
      [0, robot.distanceToGoal], 'k', 'LineWidth', 2)
   hold off


   qDistances = robot.QuantizedDistReadings(env);
   figure
   for i = 1:C.ANGLES
      polarplot([C.angles(i), (C.angles(i)+0.001)], [0, qDistances(i)])
      hold on
   end
   hold off
end


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
stateActionValues = zeros(C.QUANT_ANGLES, C.QUANT_ANGLES, C.DIRS, ...
   C.ACTIONS);

averageRange = 10;
EPISODES = 50;
RUNS = 5;

tstart = tic;
rewardsSarsa = zeros(1, EPISODES);
rewardsQLearning = zeros(1, EPISODES);
fprintf('Runs remaining: ');
for run = 1:RUNS
   fprintf('%d:', RUNS - run + 1);
   stateActionValuesSarsa = stateActionValues(:,:,:,:);
   stateActionValuesQLearning = stateActionValues(:,:,:,:);
   for i = 1:EPISODES
      if mod(i - 1, PROGRESS_STEP) == 0
         fprintf('%d ', EPISODES - i + 1);
      end
      [Value, stateActionValuesSarsa] = Sarsa(stateActionValuesSarsa,...
         false, robot, env);
      robot.StartAt(env.start, 0);
      rewardsSarsa(i) = rewardsSarsa(i) + max(Value, REWARD_FLOOR);
      
      [Value, stateActionValuesQLearning] = QLearning(...
         stateActionValuesQLearning, robot, env);
      robot.StartAt(env.start, 0);
      rewardsQLearning(i) = rewardsQLearning(i) + max(Value, REWARD_FLOOR);
   end
end

rewardsSarsa = rewardsSarsa ./ RUNS;
rewardsQLearning = rewardsQLearning ./ RUNS;
sarsaResults = rewardsSarsa(:)';
qLearningResults = rewardsQLearning(:)';
for i = (averageRange + 1):EPISODES
   sarsaResults(i) = mean(rewardsSarsa((i - averageRange):i));
   qLearningResults(i) = mean(rewardsQLearning((i - averageRange):i));
end
tstop = toc(tstart);
fprintf('\nTime for %d runs: %f seconds\n', runs, tstop)

%--------------------PLOT RESULTS--------------------------------
figure
plot(sarsaResults)
hold on
plot(qLearningResults)
legend('Sarsa', 'Q-Learning');
xlabel('Episodes');
ylabel('Sum of Rewards During Episode');
title('Cliff Walking');
hold off
