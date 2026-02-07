%% Baseline 1 : A* Algorithm
% A Formal Basis for the Heuristic Determination of Minimum Cost Paths
% Hart, Peter E. and Nilsson, Nils J. and Raphael, Bertram, 1968
% Cited in Report 

%% This Implementation is from : https://pythonmana.com/2021/01/20210129211032798U.html 
% This is NOT SUBMISSION but is benchmark used

clc; 
clear; 
close all;
%% Parameter reading and setting
% spheric obstacle
sphere = csvread("../FYP/environment_1/sphere.csv");
Rsphere = csvread("../FYP/environment_1/sphere_radius.csv")';

% cylinderic obstacle
cylinderMatrix  = csvread("../FYP/environment_2/cylinder.csv");   % cordinate which define centre of obstacle around which structure is built
cylinderRMatrix = csvread("../FYP/environment_2/cylinder_radius.csv")'; % radius
cylinderheight = csvread("../FYP/environment_2/cylinder_height.csv")'; % height

% cone obstacle
cone      = csvread("../FYP/environment_2/cone.csv");  % cordinate which define centre of obstacle around which structure is built
coneRMatrix     = csvread("../FYP/environment_2/cone_radius.csv"); % radius
coneHMatrix     = csvread("../FYP/environment_2/cone_height.csv"); % height

% start and target
start = csvread("../FYP/environment_1/start.csv")';
goal = csvread("../FYP/environment_1/goal.csv")'; %goal1, goal11%goal1, goal11
[total_sphere, ~] = size(sphere);
[total_cylinder, ~] = size(cylinderMatrix);
[total_cone,~] = size(cone);

% Define the degrees of freedom below
DegFreedom = [[1,0,0];[0,1,0];[0,0,1];[-1,0,0];[0,-1,0];[0,0,-1];...
            [1,1,0];[1,0,1];[0,1,1];[-1,-1,0];[-1,0,-1];[0,-1,-1];...
            [1,-1,0];[-1,1,0];[1,0,-1];[-1,0,1];[0,1,-1];[0,-1,1];...
            [1,1,1];[-1,-1,-1];[1,-1,-1];[-1,1,-1];[-1,-1,1];[1,1,-1];...
            [1,-1,1];[-1,1,1]];
step_thresh = 0.7;
stop = step_thresh*1.5;
g = [start, 0; goal, inf]; % First 3 numbers on each line is coordinates，fourth number is path dissipation
Path = [];
Parent = [];
Open = [start, g(findIndex(g,start),4) + getDist(start,goal)];

%% Draw the obstacle environment - make modifications in csv file for position and no. of obstacles changes
figure(1)
for i = 1:total_sphere   %Draw Static Ball Obstacles
    drawSphere(sphere(i,:), Rsphere(i))
end

%  for i = 1:total_cylinder   %Draw a cylindrical obstacle
%      drawCylinder(cylinderMatrix(i,:), cylinderRMatrix(i), cylinderheight(i));
%  end
%  
%   for i = 1:total_cone       %Draw a cone of obstacles
%       drawCone(cone(i,:), coneRMatrix(i), coneHMatrix(i));
%   end


% Information Visualization
bar1 = scatter3(start(1),start(2),start(3),80,"cyan",'filled','o','MarkerEdgeColor','k');hold on
bar2 = scatter3(goal(1),goal(2),goal(3),80,"magenta",'filled',"o",'MarkerEdgeColor','k');
text(start(1),start(2),start(3),'  Start');
text(goal(1),goal(2),goal(3),'  End');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('A* Algorithm');
axis equal





%% Main Loop
tic;
while ~isempty(Open) %openlist
    [xi, index] = findMin(Open); % find minimum from openlist
    Open(index,:) = [];
    if getDist(xi, goal) < stop
        break;
    end
    children = getChildren(xi, DegFreedom, step_thresh, sphere, Rsphere,...
                           cylinderMatrix, cylinderRMatrix, cylinderheight,...
                           cone, coneRMatrix, coneHMatrix,...
                           total_sphere, total_cylinder, total_cone);
    scatter3(children(:,1),children(:,2),children(:,3),10,'filled','o');
    drawnow;
    [n,~] = size(children);
    for i = 1:n
        child = children(i,:);
        if findIndex(g, child) == 0   % child is not present
            g = [g; child, inf];
        end
        a = g(findIndex(g, xi),4) + getDist(xi,child);
        if a < g(findIndex(g, child),4)
            g(findIndex(g, child),4) = a;
            Parent = setParent(Parent, child,xi);
            Open = open_list(Open, child, a, goal);
        end
    end  
end
lastPoint = xi;


%% Back-Tracking
x = lastPoint;
Path = x;
[n,~] = size(Parent);
while any(x ~= start)
    for i = 1:n
        if Parent(i,1:3) == x
            Path = [Parent(i,4:6); Path];
            break;
        end
    end
    x = Parent(i,4:6);
end


bar3 = plot3([Path(:,1);goal(1)],[Path(:,2);goal(2)],[Path(:,3);goal(3)],'LineWidth',3,'color','r');
filPathX = [start(1),MovingAverage(Path(2:end,1),5),goal(1)];
filPathY = [start(2),MovingAverage(Path(2:end,2),5),goal(2)];
filPathZ = [start(3),MovingAverage(Path(2:end,3),5),goal(3)];

legend([bar1, bar2,bar3],["Start","Goal","UAV Path"],'Location','northwest');



%% full path
path = [Path;goal];

%% Calculate track distance
pathLength = 0;
[n,~] = size(Path);
for i = 1:n-1
    pathLength = pathLength + getDist(Path(i,:),Path(i+1,:)); % f(n) = g(n) + h(n)
end

pathLength = pathLength + getDist(Path(end,:),goal);
fprintf('calculating time:%fseconds\nPath Length:%f',toc,pathLength);
%% Store track

%% function
function children = getChildren(pos, DegFreedom, step,circleCenter,circleR,...
                                cylinderCenter,cylinderR, cylinderH,...
                                cone, coneRMatrix, coneHMatrix,...
                                total_sphere, total_cylinder, total_cone)
allchild = [];
[n,~] = size(DegFreedom);
for i = 1:n
    direc = DegFreedom(i,:);
    child = pos + direc * step;
    if ~checkCol(child, circleCenter,circleR, cylinderCenter,cylinderR, cylinderH,...
                 cone, coneRMatrix, coneHMatrix,total_sphere, total_cylinder, total_cone)
        continue;
    end
    allchild = [allchild; child];
end
children = allchild;
end

function flag = checkCol(pos, circleCenter,circleR, cylinderCenter,cylinderR, cylinderH,...
                         cone, coneRMatrix, coneHMatrix,total_sphere, total_cylinder, total_cone)
flag = true;
for i = 1:total_sphere
    if getDist(pos, circleCenter(i,:)) <= circleR(i)
        flag = false;
        break;
    end
end
for i = 1:total_cylinder
    if getDist(pos(1:2), cylinderCenter(i,:)) <= cylinderR(i) && pos(3) <= cylinderH(i)
        flag = false;
        break;
    end
end
for i = 1:total_cone
    if getDist(pos(1:2), cone(i,:)) <= coneRMatrix(i) - coneRMatrix(i)/coneHMatrix(i)*pos(3)...
            && pos(3) >= 0
        flag = false;
        break;
    end
end
if pos(3) <= 0, flag = false; end
end

function Par = setParent(Parent, xj, xi)
[n,~] = size(Parent);
if n == 0
    Par = [xj, xi];
else
    for i = 1:n
        if Parent(i,1:3) == xj
            Parent(i,4:6) = xi;
            Par = Parent;
            break;
        end
        if i == n
            Par = [Parent; xj, xi];
        end
    end
end
end

function Ope = open_list(Open, child, a, goal)
[n,~] = size(Open);
if n == 0
    Ope = [child, a + getDist(child, goal)];
else
    for i = 1:n
        if Open(i,1:3) == child
            Open(i,4) = a + getDist(child, goal);
            Ope = Open;
        end
        if i == n
            Ope = [Open; child, a + getDist(child, goal)];
        end
    end
end
end

function h = heuristic(pos, goal)
h = max([abs(goal(1) - pos(1)),abs(goal(2) - pos(2)),abs(goal(3) - pos(3))]);
end

function index = findIndex(g, pos)
[n,~] = size(g);
index = 0;    % Indicates that the index was not found

for i = 1:n
    if g(i,1:3) == pos
        index = i;   % index is i
        break;
    end
end
end

function d = getDist(x,y) % Euclidean Distance 
d = sqrt(sum((x - y).^2));
end

function [pos, index] = findMin(Open) % Find minimal distance from open-list
[~,index] = min(Open(:,4));
pos = Open(index,1:3);
end