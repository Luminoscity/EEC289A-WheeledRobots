% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

WORLD_START = 23;
WorldsToPlot = 4;
Mecanum = true;
figure

for world = WORLD_START:(WORLD_START + WorldsToPlot - 1)
   loadString = sprintf('NewWorlds/NewWorld%d.mat', world);
   
   load(loadString)
   subplot(WorldsToPlot, 2, (((world-WORLD_START) * 2)+1))
   imagesc(env.whichObstacles)
   worldString = sprintf('World%d', world);
   axis square
   title(worldString)
   set(gca, 'YDir', 'Normal')
   colormap Jet
   colorbar

   load(loadString)
   subplot(WorldsToPlot, 2, (((world-WORLD_START) * 2)+2))
   worldString = sprintf('World%d', world);
   imagesc(env.whichObstacles)
   axis square
   title(worldString)
   set(gca, 'YDir', 'Normal')
   colormap Jet
   colorbar
end