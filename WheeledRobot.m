% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

classdef WheeledRobot < handle
   properties
      orientation
      position
   end
   properties (Access = private)
      directions
   end
   
   methods
      function obj = WheeledRobot(env)
         obj.position = env.start;
         obj.orientation = datasample(env.C.directions, 1);
         obj.directions = env.C.directions;
      end
      
      function StartAt(obj, pos, orient)
         obj.position = pos;
         if nargin > 2
            obj.orientation = orient;
         else
            obj.orientation = datasample(obj.directions, 1);
         end
      end
      
      function Move(obj, action)
         obj.position(1) = obj.position(1) + action(1);
         obj.position(2) = obj.position(2) + action(2);
         obj.orientation = mod(obj.orientation + action(3), 2*pi);
      end
      
      function [distToGoal, dirToGoal, angleToGoal] = GetBearing(obj, env)
         xToGoal = env.goal(1) - obj.position(1);
         yToGoal = env.goal(2) - obj.position(2);
         if xToGoal == 0         %avoid divide by zero
            xToGoal = 0.00001;
         end
         if xToGoal > 0 && yToGoal < 0
            angleToGoal = 2*pi + atan(yToGoal / xToGoal);
         elseif xToGoal < 0
            angleToGoal = pi + atan(yToGoal / xToGoal);
         else
            angleToGoal = atan(yToGoal / xToGoal);
         end
         dirs = [obj.directions(:)', (2*pi)];
         [~, dirToGoal] = min(abs(dirs - angleToGoal));
         dirToGoal = max(mod(dirToGoal, env.C.DIRS + 1), 1);
         distToGoal = sqrt(xToGoal * xToGoal + yToGoal * yToGoal);
      end
      
      function distanceReadings = QuantizedDistReadings(obj, env)
         distanceReadings = obj.ReadDistances(env);
         divided = distanceReadings ./ env.C.DIST_STEP;
         distanceReadings = uint8(min(floor(divided) + 1, env.C.DISTS));
      end
      
      function distanceReadings = ReadDistances(obj, env)
         distanceReadings = zeros(1, env.C.ANGLES);
         
         for i = 1:env.C.ANGLES
            beam = obj.position + 0.5;
            distance = 0;
            beamAngle = env.C.angles(i) + obj.orientation - pi/2;
            while beam(1) >= 1 && beam(1) < env.C.WORLD_WIDTH + 1 && ...
                  beam(2) >= 1 && beam(2) < env.C.WORLD_HEIGHT + 1
               
               if env.obstacles(floor(beam(2)), floor(beam(1)))
                  break
               end
               beam(1) = beam(1) + env.C.DIST_RES * sin(beamAngle);
               beam(2) = beam(2) + env.C.DIST_RES * cos(beamAngle);
               distance = distance + env.C.DIST_RES;
            end
            distanceReadings(i) = distance;
         end
      end
   end
end