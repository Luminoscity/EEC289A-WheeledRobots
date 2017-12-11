% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

WORLD_START = 23;
WorldsToPlot = 4;
Mecanum = true;
figure

for world = WORLD_START:(WORLD_START + WorldsToPlot - 1)
   if Mecanum
      loadString = sprintf('SQE-Mec/MecWorld%d.mat', world);
   else
      loadString = sprintf('SQE-Diff/DiffWorld%d.mat', world);
   end
   
   load(loadString)
   subplot(WorldsToPlot, 2, (((world-WORLD_START) * 2)+1))
   imagesc(env.whichObstacles)
   worldString = sprintf('World%d', world);
   axis square
   title(worldString)
   set(gca, 'YDir', 'Normal')
   colormap Jet
   colorbar

   subplot(WorldsToPlot, 2, (((world-WORLD_START) * 2)+2))
   plot(sarsaResults)
   hold on
   plot(qLearningResults)
   plot(eSarsaResults)
   legend('Sarsa', 'Q-Learning', 'Expected Sarsa');
   xlabel('Episodes')
   ylabel('Sum of Rewards During Episode')
   if Mecanum
      str = strcat('Mecanum Wheels - ', worldString);
   else
      str = strcat('Differential Wheels - ', worldString);
   end
   title(str)
   hold off
end