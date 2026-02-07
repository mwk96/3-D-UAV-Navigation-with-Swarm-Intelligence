%% Benchmark 3 : Ant Colony Optimization
% Ant colony optimization: a new meta-heuristic,
% Dorigo, M. & Di Caro, G. , 1999
% Cited in Report 

%% This Implementation is from : https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm
% This is NOT SUBMISSION but is benchmark used

clc; clear; close all;
rand('seed',5);
%% Parameter reading and setting
% spheric obstacle
obstacleMatrix = csvread("../FYP/environment_3/sphere.csv");
RobstacleMatrix = csvread("../FYP/environment_3/sphere_radius.csv")';

% cylinderic obstacle
cylinderMatrix  = csvread("../FYP/environment_3/cylinder.csv");   % cordinate which define centre of obstacle around which structure is built
cylinderRMatrix = csvread("../FYP/environment_3/cylinder_radius.csv")'; % radius
cylinderHMatrix = csvread("../FYP/environment_3/cylinder_height.csv")'; % height

% cone obstacle
coneMatrix      = csvread("../FYP/environment_3/cone.csv");  % cordinate which define centre of obstacle around which structure is built
coneRMatrix     = csvread("../FYP/environment_3/cone_radius.csv"); % radius
coneHMatrix     = csvread("../FYP/environment_3/cone_height.csv"); % height

% start and target
% change environment folder here too as the goal positions are 
% different for each environment
start = csvread("../FYP/environment_3/start.csv")';
goal = csvread("../FYP/environment_3/goal.csv")'; %goal1, goal11%goal1, goal11

[numberOfSphere, ~] = size(obstacleMatrix);
[numberOfCylinder, ~] = size(cylinderMatrix);
[numberOfCone,~] = size(coneMatrix);

popNumber = 15;  % population
rou = 0.5;        % Volatile factor
bestFitness = []; % The best fitness value storage list for each generation
bestfitness = inf;% Initialize the best fitness value (smaller is better in this case)
everyIterFitness = [];
deltaX = 0.4; deltaY = 0.4; deltaZ = 0.4;
gridXNumber = floor(abs(goal(1) - start(1)) / deltaX);
gridYNumber = 80; gridZNumber = 80;
ybegin = start(2) - 20*deltaY; zbegin = start(3) - 20*deltaZ;
pheromone = ones(gridXNumber, gridYNumber, gridZNumber);
ycMax = 3; zcMax = 3; % The maximum number of cells that change along the y-axis and the maximum number of cells that change along the z-axis
bestPath = []; 
iterMax = 100; 
%% Draw the obstacle environment
figure(1)
for i = 1:numberOfSphere   %Draw Static Ball Obstacles
    drawSphere(obstacleMatrix(i,:), RobstacleMatrix(i))
end

 for i = 1:numberOfCylinder   %Draw a cylindrical obstacle
     drawCylinder(cylinderMatrix(i,:), cylinderRMatrix(i), cylinderHMatrix(i));
 end
 
  for i = 1:numberOfCone       %Draw a cone of obstacles
      drawCone(coneMatrix(i,:), coneRMatrix(i), coneHMatrix(i));
  end



bar1 = scatter3(start(1),start(2),start(3),80,"cyan",'filled','o','MarkerEdgeColor','k');hold on
bar2 = scatter3(goal(1),goal(2),goal(3),80,"magenta",'filled',"o",'MarkerEdgeColor','k');
text(start(1),start(2),start(3),'  Start');
text(goal(1),goal(2),goal(3),'  Goal');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Ant Colony Optimization');
axis equal
% set(gcf,'unit','centimeters','position',[30 10 20 15]);
%% main loop
tic;
for iter = 1:iterMax
    fprintf("Computing Path：%.2f%%\n",iter/iterMax*100);
    % path search by ants 
    [path, pheromone] = searchPath(popNumber, pheromone, start, goal, ycMax, zcMax,...
                                   deltaX, deltaY, deltaZ, obstacleMatrix,RobstacleMatrix,...
                                   cylinderMatrix, cylinderRMatrix, cylinderHMatrix,...
                                   coneMatrix, coneRMatrix, coneHMatrix,...
                                   ybegin, zbegin, gridYNumber, gridZNumber);
    % Path fitness value calculation
    fitness = calFit(path, deltaX, start, goal);
    [newBestFitness, bestIndex] = min(fitness);
    everyIterFitness = [everyIterFitness, newBestFitness];
    if newBestFitness < bestfitness
        bestfitness = newBestFitness;
        bestPath = path(bestIndex, :, :);
    end
    bestFitness = [bestFitness, bestfitness];

    % update pheromone -- update equation
    cfit = 100 / bestfitness;
    iterNum = 0;
    for x = start(1) + deltaX : deltaX : goal(1) - 0.001
        iterNum = iterNum + 1;
        pheromone(iterNum, round((bestPath(:,iterNum+1,1)-ybegin)/deltaY), round((bestPath(:,iterNum+1,2)-zbegin)/deltaZ))...
        = (1 - rou) * pheromone(iterNum, round((bestPath(:,iterNum+1,1)-ybegin)/deltaY), round((bestPath(:,iterNum+1,2)-zbegin)/deltaZ)) + cfit;
    end
%     for ant = 1:popNumber %path for each ant
%         cfit = fitness(ant);
%         iterNum = 0;
%         for x = start(1) + deltaX : deltaX : goal(1) - 0.001
%             iterNum = iterNum + 1;
%             pheromone(iterNum, round((path(ant,iterNum+1,1)-ybegin)/deltaY), round((path(ant,iterNum+1,2)-zbegin)/deltaZ))...
%             = (1 - rou) * pheromone(iterNum, round((path(ant,iterNum+1,1)-ybegin)/deltaY), round((path(ant,iterNum+1,2)-zbegin)/deltaZ)) + 1/cfit;
%         end
%     end
end
%% Draw the best path and the graph of the fitness value change
% draw path
x = [start(1):deltaX:goal(1)-0.001,goal(1)];
[~,m] = size(x);
path_ = [];
for i = 1:m
    path_ = [path_;bestPath(:,i,1),bestPath(:,i,2)];
end
bar3 = plot3(x, path_(:,1), path_(:,2),'LineWidth',2,'MarkerSize',7,'Color','r');
filPathX = [start(1),MovingAverage(x(2:end-1),5),goal(1)];
filPathY = [start(2),MovingAverage(path_(2:end-1,1),5),goal(2)];
filPathZ = [start(3),MovingAverage(path_(2:end-1,2),5),goal(3)];
bar4 = plot3(filPathX, filPathY, filPathZ,'LineWidth',3,'color','g');
legend([bar1, bar2,bar3,bar4],["starting point","ending point","UAV track","UAV track with MA"],'Location','northwest');
% final path
xx = [start(1),x(2:end-1),goal(1)]; yy = [start(2),path_(2:end-1,1)',goal(2)]; zz = [start(3),path_(2:end-1,2)',goal(3)];
path = [xx',yy',zz'];
pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + getDist(path(i,1:3),path(i+1,1:3)); end
fprintf('calculating time：%fseconds\n path length:%f\n GS:%f°\n LS:%f°',toc,pathLength,calGs(path)/pi*180,calLs(path)/pi*180);
% 绘制适应值变化图
figure(2)
plot(bestFitness,'LineWidth',2,'Color','r'); hold on;
plot(everyIterFitness,'LineWidth',2,'Color','b')
legend('The best individual fitness in history','The best individual fitness for each generation')
title('fitness trend'); xlabel('iterations'); ylabel('fitness val'); grid on;

%% function
function [path, pheromone] = searchPath(popNumber, pheromone, start, goal, ycMax, zcMax,...
                                        deltaX, deltaY, deltaZ, obstacleMatrix, RobstacleMatrix,...
                                        cylinderMatrix, cylinderRMatrix, cylinderHMatrix,...
                                        coneMatrix, coneRMatrix, coneHMatrix,...
                                        ybegin, zbegin, gridYNumber, gridZNumber)
% Get the path function from the start point to the end point
path = []; % Used to record the paths of all ants
for ant = 1:popNumber % for every ant
    path(ant, 1, 1:2) = start(2:3); % Only record y and z axis coordinates，Add deltaX each time to the x-axis
    nowPoint = start(2:3);
    iterNum = 0;
    for x = start(1) + deltaX : deltaX : goal(1) - 0.001 % Subtract a decimal to avoid taking x directly to goal(1)
        iterNum = iterNum + 1;
        nextPoint = [];
        p = [];   
        for y = -ycMax * deltaY : deltaY : ycMax * deltaY
            for z = -zcMax * deltaZ : deltaZ : zcMax * deltaZ
                nextPoint = [nextPoint; nowPoint + [y, z]];
                if nextPoint(end,1) > ybegin+0.01 && nextPoint(end,1) < ybegin + gridYNumber*deltaY && ...
                   nextPoint(end,2) > zbegin+0.01 && nextPoint(end,2) < zbegin + gridZNumber*deltaZ  % Determine whether it is out of bounds (the size of the pheromone matrix has been determined, avoid exceeding it)
                    hValue = calHeuristicValue(nowPoint, nextPoint(end,:), goal, x, deltaX, obstacleMatrix,...
                                               RobstacleMatrix, cylinderMatrix, cylinderRMatrix, cylinderHMatrix,...
                                               coneMatrix, coneRMatrix, coneHMatrix);
%                     pher = pheromone(iterNum, round((nextPoint(end,1) - ybegin)/deltaY), round((nextPoint(end,2) - zbegin)/deltaZ));
                    try
                        pher = pheromone(iterNum, round((nextPoint(end,1) - ybegin)/deltaY), round((nextPoint(end,2) - zbegin)/deltaZ));
                    catch
                        round((nextPoint(end,1) - ybegin)/deltaY)
                    end
                    p = [p, pher * hValue];
                else
                    p = [p,0]; %Zero is impossible to be picked in roulette
                end
            end
        end
        % Roulette to choose the next coordinate point
        p1 = p / sum(p); % Normalized
        pc = cumsum(p1);
        targetIndex = find(pc >= rand);
        targetNextPoint = nextPoint(targetIndex(1),:);
        path(ant, iterNum + 1, 1:2) = targetNextPoint;
        nowPoint = targetNextPoint;
    end
    path(ant, iterNum + 2, 1:2) = goal(2:3);
end
end

function h = calHeuristicValue(now, next, goal, x, deltaX, obstacleMatrix, RobstacleMatrix,...
                               cylinderMatrix, cylinderRMatrix, cylinderHMatrix,...
                               coneMatrix, coneRMatrix, coneHMatrix)
% Determine whether the next coordinate point collides，If there is a collision, set the heuristic value to 0，It will be impossible to be selected in subsequent roulette point selections
nextXYZ = [x, next];
flag = checkCol(nextXYZ, obstacleMatrix, RobstacleMatrix,...
                cylinderMatrix, cylinderRMatrix, cylinderHMatrix,...
                coneMatrix, coneRMatrix, coneHMatrix);
% Calculate heuristic value
d1 = getDist([x - deltaX, now], [x, next]);
d2 = getDist([x, next], goal);
D = 50 / (d1 + d2);
h = flag * D;
end

function f = calFit(path, deltaX, start, goal)
% Calculate the fitness function
[n,m,~] = size(path);
x = [start(1) : deltaX : goal(1) - 0.001, goal(1)];
for i = 1:n
    f(i) = 0;
    for j = 1:m-1
        f(i) = f(i) + getDist([x(j), path(i,j,1), path(i,j,2)], [x(j+1), path(i,j+1,1), path(i,j+1,2)]);
    end
end
end

function flag = checkCol(pos, circleCenter,circleR, cylinderCenter,cylinderR, cylinderH,...
                         coneMatrix, coneRMatrix, coneHMatrix)
% Collision detection function
[numberOfSphere, ~] = size(circleCenter);
[numberOfCylinder, ~] = size(cylinderCenter);
[numberOfCone,~] = size(coneMatrix);
flag = true;
for i = 1:numberOfSphere
    if getDist(pos, circleCenter(i,:)) <= circleR(i)
        flag = false;
        break;
    end
end
for i = 1:numberOfCylinder
    if getDist(pos(1:2), cylinderCenter(i,:)) <= cylinderR(i) && pos(3) <= cylinderH(i)
        flag = false;
        break;
    end
end
for i = 1:numberOfCone
    if getDist(pos(1:2), coneMatrix(i,:)) <= coneRMatrix(i) - coneRMatrix(i)/coneHMatrix(i)*pos(3)...
            && pos(3) >= 0
        flag = false;
        break;
    end
end
if pos(3) <= 0, flag = false; end
end

function d = getDist(x,y)
d = sqrt(sum((x - y).^2));
end







































