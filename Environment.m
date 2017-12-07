classdef Environment
   properties
      C
      obstacles
   end
   methods
      function obj = Environment(constants)
         obj.C = constants;
         obstacles = false(obj.C.WORLD_HEIGHT, obj.C.WORLD_WIDTH);
      end
      function GenerateObstacles()
         obstacleCount = randi(C.MAX_OBSTACLES);
   
         for i = 1:obstacleCount
            obstacleArea = randi(C.MAX_OBSTACLE_AREA);
            %generate obstacle leaving some space between existing obstacles
         end
   
         obstacles = zeros();
      end
   end
end