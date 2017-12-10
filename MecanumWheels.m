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
MECANUM = [true,false];              %false for differential wheels
PROGRESS_STEP = 10;
WORLD_IDX_START = 1;
%MAX_PARAM = 15;
DIST_QUANTIZE = 5;           % number of quantized distances
DIST_QUANT_STEP = 5;         % centimeters between quantized distances
DIST_ANGLE_MEAS = 60;        % number of angles to measure distances at
DIST_ANGLE_QUANTIZE = 16;    % number of quantized distance angles to
                             % record as part of the state
TURN_ANGLE = pi / 4;         % 45 degree turns
OBSTACLE_SEPARATION = 2;
MIN_OBSTACLES = 2;
DISTANCE_SENSOR_RESOLUTION = 0.1;
ROUND_MAGNITUDE = 1.49;
REWARD_FLOOR = -200;

EPISODES = 200;  %max is 200
RUNS = 3;

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

directions = [DIR_WEST, DIR_NORTHWEST, DIR_NORTH, DIR_NORTHEAST,...
              DIR_EAST, DIR_SOUTHEAST, DIR_SOUTH, DIR_SOUTHWEST];
          
for typeIndex = 1:2
if MECANUM(typeIndex)
   actions = [ACTION_NORTH; ACTION_LEFT; ACTION_RIGHT; ACTION_NORTHWEST;...
              ACTION_WEST; ACTION_SOUTHWEST; ACTION_SOUTH;...
              ACTION_SOUTHEAST; ACTION_EAST; ACTION_NORTHEAST];
   wheelType = 'Mecanum';
else
   actions = [ACTION_NORTH; ACTION_LEFT; ACTION_RIGHT];
   wheelType = 'Differential';
end

stateActionValues = zeros(DIST_ANGLE_QUANTIZE, DIST_ANGLE_QUANTIZE, ...
   length(directions), length(actions));



angleStep = 2 * pi / DIST_ANGLE_MEAS;
angles = 0:angleStep:(2 * pi - angleStep);
angleStep = 2 * pi / DIST_ANGLE_QUANTIZE;
qAngles = 0:angleStep:(2 * pi - angleStep);
%{
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
%}
%--------------------------MAIN-----------------------------
%{
env = Environment(C);
error = 1;
while error > 0
   error = env.GenerateObstacles();
end
%}

averageRange = 10;

tstart = tic;

rewardsSarsa = zeros(1, EPISODES);
rewardsQLearning = zeros(1, EPISODES);
rewardsESarsa = zeros(1, EPISODES);
fprintf('Runs:Episodes Remaining ');
figure
for run = 1:RUNS
   fprintf(' %d:', RUNS - run + 1);
   stateActionValuesSarsa = stateActionValues(:,:,:,:);
   stateActionValuesQLearning = stateActionValues(:,:,:,:);
   stateActionValuesESarsa = stateActionValues(:,:,:,:);
   for i = 1:EPISODES
      if mod(i - 1, PROGRESS_STEP) == 0
         fprintf('%d ', EPISODES - i + 1);
      end
      
      worldString = sprintf('NewWorld%d', i);
      load(strcat('NewWorlds/',worldString,'.mat'))
      env.C.actions = actions;
      env.C.ACTIONS = length(actions);
      robot = WheeledRobot(env);
      %robot.StartAt(env.start, 0);
      
      imagesc(env.whichObstacles)
      title(worldString)
      set(gca, 'YDir', 'Normal')
      colormap Jet
      colorbar
      drawnow()
      
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

tstop = toc(tstart);
fprintf('\nTime for %s Wheels (%d runs, %d episodes): %f seconds\n', ...
   wheelType, RUNS, EPISODES, tstop)

%--------------------SAVE RESULTS--------------------------------
if MECANUM(typeIndex)
   saveString = sprintf('MecResults%dRuns%dWorlds.mat', RUNS, EPISODES);
else
   saveString = sprintf('DiffResults%dRuns%dWorlds.mat', RUNS, EPISODES);
end
C = env.C;
save(saveString, 'C', 'sarsaResults', 'qLearningResults', 'eSarsaResults', ...
   'EPISODES', 'RUNS', 'tstop')

%--------------------PLOT RESULTS--------------------------------
figure
plot(sarsaResults)
hold on
plot(qLearningResults)
plot(eSarsaResults)
legend('Sarsa', 'Q-Learning', 'Expected Sarsa');
xlabel('Episodes');
ylabel('Sum of Rewards During Episode');
str = sprintf('%s Wheels - %d Runs, %d Episodes', wheelType, RUNS, ...
   EPISODES);
title(str);
hold off

end