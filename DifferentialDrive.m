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
WORLD_IDX_START = 1;
DIST_QUANTIZE = 5;           % number of quantized distances
DIST_QUANT_STEP = 5;         % centimeters between quantized distances
DIST_ANGLE_MEAS = 60;        % number of angles to measure distances at
DIST_ANGLE_QUANTIZE = 16;    % number of quantized distance angles to
                             % record as part of the state
TURN_ANGLE = pi / 4;         % 45 degree turns
WORLD_WIDTH = [10,10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,18,19,20,21,22,23,24,25];
WORLD_HEIGHT = [10,10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,18,19,20,21,22,23,24,25];
OBSTACLE_SEPARATION = 2;
MIN_OBSTACLES = [1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2];
MAX_OBSTACLES = [1,1,1,2,2,2,2,2,2,3,3,3,4,4,4,5,5,5,6,6,6,7,8,9,10,11,12,13,14,15];%15;%35;
MAX_OBSTACLE_AREA = [10,10,10,12,12,12,14,14,14,16,16,16,18,18,18,20,20,20,22,22,22,24,26,28,30,32,34,36,38,40];
DISTANCE_SENSOR_RESOLUTION = 0.1;
ROUND_MAGNITUDE = 1.49;
REWARD_FLOOR = -200;



EPSILON = 0.1;
ALPHA = 0.5;
GAMMA = 1;

ACTION_NORTH = [2 * TURN_ANGLE, 0];
ACTION_LEFT = [-1, -TURN_ANGLE];   %turn left slightly
ACTION_RIGHT = [-1, TURN_ANGLE];   %turn right slightly
%{
ACTION_NORTHWEST = [TURN_ANGLE, 0];
ACTION_WEST = [0, 0];
ACTION_SOUTHWEST = [7 * TURN_ANGLE, 0];
ACTION_SOUTH = [6 * TURN_ANGLE, 0];
ACTION_SOUTHEAST = [5 * pi/4, 0];
ACTION_EAST = [4 * TURN_ANGLE, 0];
ACTION_NORTHEAST = [3 * pi/4, 0];
%}
DIR_WEST = 0;
DIR_NORTHWEST = pi / 4;
DIR_NORTH = pi / 2;
DIR_NORTHEAST = 3 * pi / 4;
DIR_EAST = pi;
DIR_SOUTHEAST = 5 * pi / 4;
DIR_SOUTH = 3 * pi / 2;
DIR_SOUTHWEST = 7 * pi / 4;

directions = [DIR_WEST, DIR_NORTHWEST, DIR_NORTH, DIR_NORTHEAST,...
              DIR_EAST, DIR_SOUTHEAST, DIR_SOUTH, DIR_SOUTHWEST];
           
actions = [ACTION_NORTH; ACTION_LEFT; ACTION_RIGHT];
%{
; ACTION_NORTHWEST;...
           ACTION_WEST; ACTION_SOUTHWEST; ACTION_SOUTH;...
           ACTION_SOUTHEAST; ACTION_EAST; ACTION_NORTHEAST];
%}

stateActionValues = zeros(DIST_ANGLE_QUANTIZE, DIST_ANGLE_QUANTIZE, ...
   length(directions), length(actions));

stateActionValuesSarsa = stateActionValues(:,:,:,:);
stateActionValuesQLearning = stateActionValues(:,:,:,:);
stateActionValuesESarsa = stateActionValues(:,:,:,:);

wIdx = 1;
for world = WORLD_IDX_START:(WORLD_IDX_START + length(WORLD_WIDTH) - 1)

fileString = sprintf('World%d-Mec.mat', world);
worldString = sprintf('World%d', world);
load(fileString)

EPISODES = 100;
RUNS = 1;

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
           'MAX_OBSTACLE_AREA', MAX_OBSTACLE_AREA(wIdx),...
           'MIN_OBSTACLES', MIN_OBSTACLES(wIdx),...
           'MAX_OBSTACLES', MAX_OBSTACLES(wIdx),...
           'OBS_SPACE', OBSTACLE_SEPARATION,...
           'WORLD_WIDTH', WORLD_WIDTH(wIdx),...
           'WORLD_HEIGHT', WORLD_HEIGHT(wIdx),...
           'DIST_RES', DISTANCE_SENSOR_RESOLUTION,...
           'ROUND_MAGNITUDE', ROUND_MAGNITUDE);

%------------------------MAIN---------------------------
env.C.ACTIONS = C.ACTIONS;
env.C.actions = C.actions;
%{
env = Environment(C);
error = 1;
while error > 0
   error = env.GenerateObstacles();
end
%}
if PLOT_WORLD
   subplot(1, 2, 1)
   imagesc(env.whichObstacles)
   title(worldString)
   set(gca, 'YDir', 'Normal')
   colormap Jet
   colorbar
   drawnow()
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


averageRange = 10;

tstart = tic;

rewardsSarsa = zeros(1, EPISODES);
rewardsQLearning = zeros(1, EPISODES);
rewardsESarsa = zeros(1, EPISODES);
fprintf('World %d Runs remaining: ', world);
for run = 1:RUNS
   fprintf('%d:', RUNS - run + 1);
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
      
      [Value, stateActionValuesESarsa] = Sarsa(stateActionValuesESarsa,...
         true, robot, env);
      robot.StartAt(env.start, 0);
      rewardsESarsa(i) = rewardsESarsa(i) + max(Value, REWARD_FLOOR);
   end
end

rewardsSarsa = rewardsSarsa ./ RUNS;
rewardsQLearning = rewardsQLearning ./ RUNS;
sarsaResults = rewardsSarsa(:)';
qLearningResults = rewardsQLearning(:)';
rewardsESarsa = rewardsESarsa ./ RUNS;
eSarsaResults = rewardsESarsa(:)';
for i = (averageRange + 1):EPISODES
   sarsaResults(i) = mean(rewardsSarsa((i - averageRange):i));
   qLearningResults(i) = mean(rewardsQLearning((i - averageRange):i));
   eSarsaResults(i) = mean(rewardsESarsa((i - averageRange):i));
end

tstopSQE = toc(tstart);
fprintf('\nTime for World %d (%d runs, %d episodes): %f seconds\n', ...
   world, RUNS, EPISODES, tstopSQE)

%--------------------SAVE RESULTS--------------------------------
saveString = sprintf('SQE-Diff/DiffWorld%d.mat', world);

save(saveString, 'env', 'sarsaResults', 'qLearningResults', 'eSarsaResults', ...
   'EPISODES', 'RUNS', 'tstopSQE')
wIdx = wIdx + 1;

%--------------------PLOT RESULTS--------------------------------
subplot(1, 2, 2)
plot(sarsaResults)
hold on
plot(qLearningResults)
plot(eSarsaResults)
legend('Sarsa', 'Q-Learning', 'Expected Sarsa');
xlabel('Episodes');
ylabel('Sum of Rewards During Episode');
str = strcat('Mecanum Wheels - ', worldString);
title(str);
hold off
drawnow()
end
