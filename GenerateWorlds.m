clear
close all
PLOT_WORLD = true;
WORLDS = 200;
WORLD_IDX_START = 200;
SAVE_WORLDS = true;
MAX_COMPLEXITY = 40;
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

EPISODES = 200;  %max is 200
RUNS = 1;

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
          
actions = [ACTION_NORTH; ACTION_LEFT; ACTION_RIGHT; ACTION_NORTHWEST;...
           ACTION_WEST; ACTION_SOUTHWEST; ACTION_SOUTH;...
           ACTION_SOUTHEAST; ACTION_EAST; ACTION_NORTHEAST];



angleStep = 2 * pi / DIST_ANGLE_MEAS;
angles = 0:angleStep:(2 * pi - angleStep);
angleStep = 2 * pi / DIST_ANGLE_QUANTIZE;
qAngles = 0:angleStep:(2 * pi - angleStep);
for wIdx = WORLD_IDX_START:WORLDS
   complexity = randi(MAX_COMPLEXITY + 1) - 1;
   WORLD_WIDTH = 10 + complexity;
   WORLD_HEIGHT = WORLD_WIDTH;
   MAX_OBSTACLE_AREA = 10 + 2 * complexity;
   MAX_OBSTACLES = MIN_OBSTACLES + floor(complexity * 4 / 5);
   
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

   %--------------------------MAIN-----------------------------

   env = Environment(C);

   tryAgain = true;
   while tryAgain
      tryAgain = false;
      error = 1;
      while error > 0
         error = env.GenerateObstacles();
      end
      
      worldString = sprintf('World%d', wIdx);
      if PLOT_WORLD
         imagesc(env.whichObstacles)
         title(worldString)
         set(gca, 'YDir', 'Normal')
         colormap Jet
         colorbar
         drawnow()
         fprintf('')    %breakpoint this line
      end
   end
   if SAVE_WORLDS
      save(strcat('FunWorlds/', worldString, '.mat'), 'env')
   end
end