% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

classdef WheeledRobot < handle
   properties
      orientation     %0 is facing north
      position        %x and y in the world
      farthest        %current farthest obstacle distance
      closest         %current closest obstacle distance
      distanceToGoal  %distance to goal as the crow flies
   end
   properties (Access = private)
      directions
   end
   
   methods
      function obj = WheeledRobot(env)
         obj.position = env.start;
         obj.orientation = datasample(env.C.directions, 1);
         obj.directions = env.C.directions;
         obj.farthest = 0;
         obj.closest = 0;
      end
      
      function StartAt(obj, pos, orient)
         obj.position = pos;
         if nargin > 2
            obj.orientation = orient;
         else
            obj.orientation = datasample(obj.directions, 1);
         end
      end
      
      function [reward, closeIdx, farIdx, dirToGoal] = Move(obj, ...
                actionIdx, env)
         M = env.C.ROUND_MAGNITUDE;
         action = env.C.actions(actionIdx, :);
         reward = 0;
         %adjust position
         if action(1) >= 0
            angle = action(1) + obj.orientation;
            obj.position = obj.position + double([int32(M * cos(angle)),...
               int32(M * sin(angle))]);
         end
         %adjust orientation
         obj.orientation = mod(obj.orientation + action(2), 2*pi);
         %recalculate state
         close = obj.closest;
         [closeIdx, farIdx] = obj.ReadCloseFarAngles(env);
         dist = obj.distanceToGoal;
         [dirToGoal, ~] = obj.GoalBearing(env);
         
         %determine reward
         if obj.closest < close || obj.distanceToGoal > dist
            reward = -1;
         elseif isequal(obj.position, env.goal)
            reward = 1;
         end
      end
      
      function [dirToGoal, angleToGoal] = GoalBearing(obj, env)
         xToGoal = env.goal(1) - obj.position(1);
         yToGoal = env.goal(2) - obj.position(2);
         %avoid divide by zero
         if xToGoal == 0
            xToGoal = 0.00001;
         end
         %find direction to goal
         if xToGoal > 0 && yToGoal < 0
            angleToGoal = 2*pi + atan(yToGoal / xToGoal);
         elseif xToGoal < 0
            angleToGoal = pi + atan(yToGoal / xToGoal);
         else
            angleToGoal = atan(yToGoal / xToGoal);
         end
         %quantize direction to goal
         dirs = [obj.directions(:)', (2*pi)];
         [~, dirToGoal] = min(abs(dirs - angleToGoal));
         dirToGoal = max(mod(dirToGoal, env.C.DIRS + 1), 1);
         %find distance to goal
         obj.distanceToGoal = sqrt(xToGoal * xToGoal + yToGoal * yToGoal);
      end
      
      function distanceReadings = QuantizedDistReadings(obj, env)
         distanceReadings = obj.ReadDistances(env);
         divided = distanceReadings ./ env.C.DIST_STEP;
         distanceReadings = uint8(min(floor(divided) + 1, env.C.DISTS));
      end
      
      function [closeIdx, farIdx] = ReadCloseFarAngles(obj, env)
         distanceReadings = obj.ReadDistances(env);
         %pick out angles of min and max distances
         [obj.closest, closeI] = min(distanceReadings);
         [obj.farthest, farI] = max(distanceReadings);
         angles = [env.C.qAngles(:)', (2*pi)];
         
         %quantize angles
         [~, closeIdx] = min(abs(angles - env.C.angles(closeI)));
         closeIdx = max(mod(closeIdx, env.C.QUANT_ANGLES + 1), 1);
         [~, farIdx] = min(abs(angles - env.C.angles(farI)));
         farIdx = max(mod(farIdx, env.C.QUANT_ANGLES + 1), 1);
      end
      
      %MAYBE function [closeAngleIdx, close, farAngleIdx, far] = ReadCloseFarAnglesAndDist(obj, env)
      
      function distanceReadings = ReadDistances(obj, env)
         distanceReadings = zeros(1, env.C.ANGLES);
         
         for i = 1:env.C.ANGLES
            beam = obj.position + 0.5;
            distance = 0;
            beamAngle = env.C.angles(i) + obj.orientation;
            %find distance to closest wall or obstacle at this angle
            while beam(1) >= 1 && beam(1) < env.C.WORLD_WIDTH + 1 && ...
                  beam(2) >= 1 && beam(2) < env.C.WORLD_HEIGHT + 1
               %has the beam hit an obstacle
               if env.obstacles(floor(beam(2)), floor(beam(1)))
                  break
               end
               %increment beam length
               beam(1) = beam(1) + env.C.DIST_RES * cos(beamAngle);
               beam(2) = beam(2) + env.C.DIST_RES * sin(beamAngle);
               distance = distance + env.C.DIST_RES;
            end
            distanceReadings(i) = distance;
         end
      end
   end
end