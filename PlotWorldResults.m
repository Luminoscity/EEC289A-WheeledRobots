% Tim Ambrose and Karthika Pai
% 11 December 2017
% EEC289A - UC Davis

world = 20;
loadString = sprintf("World%d.mat", world);
load(loadString)

subplot(1, 2, 1)
imagesc(env.whichObstacles)
worldString = sprintf("World%d", world);
axis square
title(worldString)
set(gca, 'YDir', 'Normal')
colormap Jet
colorbar

subplot(1, 2, 2)
plot(sarsaResults)
hold on
plot(qLearningResults)
legend('Sarsa', 'Q-Learning')
xlabel('Episodes')
ylabel('Sum of Rewards During Episode')
str = "Mecanum Wheels - " + worldString;
title(str)
hold off