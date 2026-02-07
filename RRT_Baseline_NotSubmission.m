%% Baseline 2 : Rapidly-Exploring Random Trees
% Randomly-Exploring Random Trees: A New Tool for Path-Planning
% Steven M. La Valle, 1998
% Cited in Report 

%% This Implementation is from : https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm
% This is NOT SUBMISSION but is benchmark used


% UAV path planning using RRT
clc; clear; close all;
%% Environment files 
%% Parameter reading and setting
% spheric obstacle
obstacleMatrix = csvread("../FYP/environment_3/sphere.csv");
RobstacleMatrix = csvread("../FYP/environment_3/sphere_radius.csv")';

% cylinderic obstacle
% cordinate which define centre of obstacle around which structure is built
cylinderMatrix  = csvread("../FYP/environment_3/cylinder.csv");   
cylinderRMatrix = csvread("../FYP/environment_3/cylinder_radius.csv")'; % radius
cylinderHMatrix = csvread("../FYP/environment_3/cylinder_height.csv")'; % height

% cone obstacle
% cordinate which define centre of obstacle around which structure is built
coneMatrix      = csvread("../FYP/environment_3/cone.csv");  
coneRMatrix     = csvread("../FYP/environment_3/cone_radius.csv"); % radius
coneHMatrix     = csvread("../FYP/environment_3/cone_height.csv"); % height

% start and target
start = csvread("../FYP/environment_3/start.csv")'; %Start coordinates
goal = csvread("../FYP/environment_3/goal.csv")'; %goal1, goal11%goal1, goal11



[numberOfSphere, ~] = size(obstacleMatrix);
[numberOfCylinder, ~] = size(cylinderMatrix);
[numberOfCone,~] = size(coneMatrix);
stepSize = 0.4; %
threshold = 0.2;
maxFailedAttempts = 10000; 
searchSize = 1.4*[goal(1) - start(1), goal(2) - start(2), goal(3) - start(3)]; 
RRTree = double([start, -1]);
failedAttempts = 0;
pathFound = false;
display = true;
%% Draw obstacle environment
figure(1)
 for i = 1:numberOfSphere   %Draw sphere obstacles
     drawSphere(obstacleMatrix(i,:), RobstacleMatrix(i))
 end
 
   for i = 1:numberOfCylinder   %Draw cylinder obstacles
       drawCylinder(cylinderMatrix(i,:), cylinderRMatrix(i), cylinderHMatrix(i));
   end
 
  for i = 1:numberOfCone       %Draw cone obstacles
      drawCone(coneMatrix(i,:), coneRMatrix(i), coneHMatrix(i));
  end

bar1 = scatter3(start(1),start(2),start(3),80,"cyan",'filled','o','MarkerEdgeColor','k');hold on
bar2 = scatter3(goal(1),goal(2),goal(3),80,"magenta",'filled',"o",'MarkerEdgeColor','k');
text(start(1),start(2),start(3),'  Start');
text(goal(1),goal(2),goal(3),'  end');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Rapidly Exploring Random-Tree');
axis equal


tic;
while failedAttempts <= maxFailedAttempts
    %% Selection of random node/point with atleast 50% chance of being goal (random expansion phase)
    if rand < 0.5
        sample = rand(1,3).*searchSize + start;
    else
        sample = goal;
    end
    %% Select closest node to sample to extend towards
    [A, I] = min( distanceCost(RRTree(:,1:3),sample) ,[],1);
    closestNode = RRTree(I(1),1:3);
    %% Extension of tree towards closest neighbour chosen above
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  
    newPoint = closestNode + stepSize * movingVec;
    %% Determine if extended new point is at a safe distance from obstacle (collision check)
    if ~checkPath(closestNode, newPoint, obstacleMatrix,RobstacleMatrix, cylinderMatrix,...
            cylinderRMatrix, cylinderHMatrix, stepSize, numberOfSphere, numberOfCylinder,...
            numberOfCone, coneMatrix, coneRMatrix, coneHMatrix) 
        failedAttempts = failedAttempts + 1;
        continue;
    end
    % Determine distance of new node/point from goal
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end
    % discard newpoint if distance between newPoint and obstacle is less
    % than threshold
    [A, I2] = min( distanceCost(RRTree(:,1:3),newPoint) ,[],1);
    if distanceCost(newPoint,RRTree(I2(1),1:3)) < threshold, failedAttempts = failedAttempts + 1; continue; end
    %% Add successful newpoints to RRT tree
    RRTree = [RRTree; newPoint I(1)]; % add node
    failedAttempts = 0;
    if display, plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',2); end
    pause(0.05);
end
if display && pathFound, plot3([closestNode(1);goal(1)],[closestNode(2);goal(2)],[closestNode(3);goal(3)],'LineWidth',2); end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% Backtracking mechanism for optimum node connections
path = goal;
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:3); path];
    prev = RRTree(prev,4);
end
bar3 = plot3(path(:,1),path(:,2),path(:,3),'LineWidth',3,'color','r');
filPathX = [start(1),MovingAverage(path(2:end-1,1),5),goal(1)];
filPathY = [start(2),MovingAverage(path(2:end-1,2),5),goal(2)];
filPathZ = [start(3),MovingAverage(path(2:end-1,3),5),goal(3)];
bar4 = plot3(filPathX,filPathY,filPathZ,'LineWidth',3,'color','b');
legend([bar1,bar2,bar3,bar4],["Start","Goal","Sampling-Tree","Path"],'Location','northwest');

%% Compute path length and computation time
pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost(path(i,1:3),path(i+1,1:3)); end % calculate path length
fprintf('Time：%d \nPath_Length=%d', toc, pathLength);

%% Function for collision detection (we compute minimal distance from obstacles must be less than radius of obstacle)
function flag = checkPath(n, newPos, circleCenter,circleR, cylinderCenter,...
    cylinderR, cylinderH, step, numberOfSphere, numberOfCylinder, numberOfCone,...
    coneMatrix, coneRMatrix, coneHMatrix)
flag = true;
movingVec = [newPos(1)-n(1),newPos(2)-n(2),newPos(3)-n(3)];
movingVec = movingVec/sqrt(sum(movingVec.^2)); 
for R = 0:step/5:distanceCost(n, newPos)
    posCheck = n + R.*movingVec;
    for i = 1:numberOfSphere
        if distanceCost(posCheck, circleCenter(i,:)) <= circleR(i)
            flag = false;
            break;
        end
    end
    for i = 1:numberOfCylinder
        if distanceCost(posCheck(1:2), cylinderCenter(i,:)) <= cylinderR(i) && posCheck(3) <= cylinderH(i)...
           && posCheck(3) >= 0
            flag = false;
            break;
        end
    end
    for i = 1:numberOfCone
        if distanceCost(posCheck(1:2), coneMatrix(i,:)) <= coneRMatrix(i) - coneRMatrix(i)/coneHMatrix(i)*posCheck(3)...
           && posCheck(3) >= 0
            flag = false;
            break;
        end
    end
end
end

function h=distanceCost(a,b)     
	h = sqrt(sum((a-b).^2, 2)); % Euclidean Distance
end

















