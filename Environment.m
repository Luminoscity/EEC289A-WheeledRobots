% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

classdef Environment < handle
   properties
      C
      obstacles            %Is there an obstacle at each coordinate
      whichObstacles       %Which obstacle is at each coordinate
      start
      goal
   end
   %{
   properties (Access = private)
      whichObstacles    %Which obstacle is at each coordinate
   end
   %}
   methods
      function obj = Environment(constants)
         obj.C = constants;
      end
      
      function GenerateObstacles(obj)
         obj.obstacles = false(obj.C.WORLD_HEIGHT, obj.C.WORLD_WIDTH);
         obstCount = randi(obj.C.MAX_OBSTACLES);
         obj.whichObstacles = zeros(obj.C.WORLD_HEIGHT, obj.C.WORLD_WIDTH);
         
         for obs = 1:obstCount
            obstacleArea = randi(obj.C.MAX_OBSTACLE_AREA);
            %generate obstacles leaving some space between obstacles
            area = 0;
            count = 0;
            while count == 0
               [count, positions] = obj.OkayPositions(randi(...
                  obj.C.WORLD_WIDTH), randi(obj.C.WORLD_HEIGHT), obs);
            end
            
            while area < obstacleArea
               if count > 0
                  newPos = positions{randi(count)};
                  newX = newPos(1);
                  newY = newPos(2);
                  obj.whichObstacles(newY, newX) = obs;
                  obj.obstacles(newY, newX) = true;
                  area = area + 1;
               else         %No more possible positions
                  area = obstacleArea;
               end
               [count, positions] = obj.OkayPositions(newX, newY, obs);
            end
         end
         
         count = 0;
         while count == 0
            [count, positions] = obj.OkayPositions(randi(...
               obj.C.WORLD_WIDTH), randi(obj.C.WORLD_HEIGHT), obs+2);
         end
         obj.start = positions{randi(count)};
         obj.whichObstacles(obj.start(2), obj.start(1)) = obstCount + 2;
         
         count = 0;
         while count == 0
            [count, positions] = obj.OkayPositions(randi(...
               obj.C.WORLD_WIDTH), randi(obj.C.WORLD_HEIGHT), obs+4);
         end
         obj.goal = positions{randi(count)};
         obj.whichObstacles(obj.goal(2), obj.goal(1)) = obstCount + 4;
      end
      
      function [count, positions] = OkayPositions(obj, x, y, current)
         positions = {[0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0],...
                      [0,0]};
         count = 0;
         for i = -1:1
            for j = -1:1
               if ~(i == 0 && j == 0) && x + i > 0 &&...
                  x + i <= obj.C.WORLD_WIDTH && y + j > 0 &&...
                  y + j <= obj.C.WORLD_HEIGHT
               
                  if obj.whichObstacles(y + j, x + i) == 0
                     clean = true;
                     for k = -obj.C.OBS_SPACE : obj.C.OBS_SPACE
                        for l = -obj.C.OBS_SPACE : obj.C.OBS_SPACE
                           if x + i + k > 0 && x + i + k <=...
                              obj.C.WORLD_WIDTH && y + j + l > 0 &&...
                              y + j + l <= obj.C.WORLD_HEIGHT
                           
                              a = obj.whichObstacles(y + j + l, x + i + k);
                              if a ~= 0 && a ~= current
                                 clean = false;
                              end
                           end
                        end
                     end
                     
                     if clean
                        count = count + 1;
                        positions{count} = [x + i, y + j];
                     end
                  end
               end
            end
         end
      end
   end
end