%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UAV Path Generation Using DFO (Dispersive Flies Optimisation)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dispersive Flies Optimization
%% Dr Mohammad Majid al_Rifaie (https://www.researchgate.net/publication/267514160_Dispersive_Flies_Optimisation)
%% 2014
% Cited in report
% This proposed methodology is indeed submission
% where external environment and adaptation are cited 


clc; clear;
tic;

%% Environment - Change digit at end of environment_x to denote environment
folder = '../FYP/environment_1';
[obstacles, start, goal] = read_environment(folder);
clear folder

%% Map Traversal part is adapted from 
% https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm
%% Due to the same environment used to benchmark performance versus baseline methods
%% Parameters for map traversal

 % step-size 
dx = 0.4; dy = 0.4; dz = 0.4;

%lower boundary
lb = [ 0 0 0]; 
%upper boundary
ub = [25 25 25]; 
X = lb(1):dx:ub(1); Nx = length(X); % X Axis  
Y = lb(2):dy:ub(2); Ny = length(Y); % Y Axis
Z = lb(3):dz:ub(3); Nz = length(Z); % Z Axis

% Initialize the collisionMap list for the 3 axis as false (as no navigation has occured yet)
% in matlab int/array start from 1, not 0
CollisionMap = false(Nx, Ny, Nz); 
for ix = 1:Nx % for origin to length of whole path in x-axis                
    for iy = 1:Ny % for origin to length of whole path in y-axis 
        for iz = 1:Nz % for origin to length of whole path in z-axis 
            pos = [X(ix), Y(iy), Z(iz)]; % Position matrix stored in list

             % Thus, this collision-map is composed of position of obstacles 
              % in the environment where collision will occur if UAV violates these areas.
            CollisionMap(ix, iy, iz) = collision(pos, obstacles);        
        end                                                       
    end
end

% Computing average length of path for the 3 axis
startid(1,1) = round(start(1) / dx) + 1;   
startid(1,2) = round(start(2) / dy) + 1;
startid(1,3) = round(start(3) / dz) + 1;
goalid(1,1) = round(goal(1) / dx) + 1;
goalid(1,2) = round(goal(2) / dy) + 1;
goalid(1,3) = round(goal(3) / dz) + 1;

% Constraint : Path length cannot exceed 100
PathLengthMax = 100;  

param.PathLengthMax = PathLengthMax;
param.start = start; param.goal = goal;
param.startid = startid; param.goalid = goalid;
param.X = X; param.Y = Y; param.Z = Z; 

clear ix iy iz pos X Y Z dx dy dz Nx Ny Nz startid goalid

%% Generate the initial paths
% Population of flies 
NP = 15; 
% Disperse flies in all 3 dimensions/axis for path formation
path_all = zeros(NP, 3*PathLengthMax); 

% for the number of population of flies
for i = 1:NP 
    while true
        while true
            path1 = getPath(CollisionMap, param); 
            % if cmap is empty i.e. no collision end/break algorithm || get the collision free path
            if ~isempty(path1), break; end 
        end
        % get the length of collision free path produced
        path2 = adjust_path(path1, PathLengthMax); 
        if ~isempty(path2), break; end 
    end
    % 3 x path length denotes the 3 axis x, y and z
    path_all(i,:) = reshape(path2', 1, 3*PathLengthMax); 
end
clear path1 path2 i NP CollisionMap % clear for next iteration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN DFO LOOP -- START
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% INITIALISE PARAMETERS
X = path_all(:,4:end-3); clear path_all;
[N, D] = size(X); 

%% Disturbance threshold for dispersal of flies when value of U(0,1) is less than 1
% Best are 0.0001 and DFO Decay (separate file)
delta = 0.0001; 
% 100 iter used as standard for DFO and compared ACO
maxIter = 100; 

% LOWER AND UPPER BOUNDS : https://uk.mathworks.com/help/matlab/ref/repmat.html
lowerB = repmat(lb, 1, PathLengthMax-2); clear lb;
upperB = repmat(ub, 1, PathLengthMax-2); clear ub;
clear PathLengthMax;

% ITERATION
best_record = zeros(maxIter,1);
for itr = 1:maxIter
    
	% EVALUATE FITNESS OF THE FLIES AND FIND THE BEST
	fitness = fitness_fcn(X, obstacles, start, goal); 
    % Get the minimal fitness score acquired to display
	[sFitness, s] = min(fitness); 
	disp(['#: ' ,         num2str(itr, '%04d'), ...
          '    Best fly: ', num2str(s, '%03d'), ...
          '    Fitness: ',  num2str(sFitness, '%.3f'), ] ) 
    % extablish best record as the ideal fitness
    best_record(itr) = sFitness; 

	% UPDATE EACH FLY INDIVIDUALLY
	for i = 1:N
		
        % ELITIST STRATEGY (DON'T UPDATE BEST FLY)
		if i == s, continue; end

		% FIND BEST NEIGHBOUR FOR EACH FLY
		left  = mod(i - 2, N) + 1; 
        right = mod(i, N) + 1; % INDICES: LEFT & RIGHT FLIES
        if fitness(right) < fitness(left)
            bNeighbour = right; 
        else
            bNeighbour = left;
        end

        for d = 1:D
			
			% DISTURBANCE MECHANISM
            if rand() < delta
                X(i,d) = lowerB(d) + rand() * (upperB(d) - lowerB(d));
                continue;
            end
            
			% UPDATE EQUATION
			X(i,d) = X(bNeighbour, d) + rand() * ( X(s,d) - X(i,d) );

			% OUT OF BOUND CONTROL
            X(i,d) = min(max(X(i,d), lowerB(d)), upperB(d));
            
        end % loop for d
        
	end % loop for i
    
end %  main loop

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN DFO LOOP -- END
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Fitness Function Evaluation to acquire minimum fitness
fitness = fitness_fcn(X, obstacles, start, goal);
[sFitness, s] = min(fitness);
calc_time = toc;

%% Result
path = reshape(X(s,:), 3, D / 3)'; path = [start; path; goal];

%%% Path-Smooting Technique using moving average : https://github.com/shorane/Path-smoothing-and-Optimization
pathx = path(:,1); pathy = path(:,2); pathz = path(:,3); 
pathx1 = [pathx(1), MovingAverage(pathx(2:end-1), 5), pathx(end)]';
pathy1 = [pathy(1), MovingAverage(pathy(2:end-1), 5), pathy(end)]';
pathz1 = [pathz(1), MovingAverage(pathz(2:end-1), 5), pathz(end)]';
path_averaged = [pathx1 pathy1 pathz1];


%%% Display Path/Navigation Metrics
disp('')
disp(['Calculation Time: ', num2str(calc_time, '%.3f'), 's'])
disp(['Path Length     : ', num2str(sFitness, '%.3f'), 'm'])


%% Convergence Curve
figure(1); clf;
draw_environment(obstacles, start, goal);
h1 = plot3(path(:,1), path(:,2), path(:,3), ...
    'LineWidth', 2, 'MarkerSize', 7, 'Color', 'r');
h2 = plot3(path_averaged(:,1), path_averaged(:,2), path_averaged(:,3), ...
    'LineWidth', 2, 'MarkerSize', 7, 'Color', 'b');
legend([h1; h2], ['Original'; 'Averaged']);

figure(2)
plot(best_record); title('History of Best Fitness')
xlabel('Iteration'); ylabel('Fitness'); grid on;

%% functions
% function to read coordinates of obstacles and start/goal points
function [obstacles, start, goal] = read_environment(folder) 

%% For changing environment, simply change the folder name to specified
%% environment name 
% Example : "../FYP/environment_1/sphere.csv" change to
% "../FYP/environment_2/sphere.csv"
% All other names remain the same. 

% spheric obstacle - for env 1 change to 1 env 2 change to 2 env 3 change
% to 3
obstacleMatrix = csvread("../FYP/environment_1/sphere.csv");
RobstacleMatrix = csvread("../FYP/environment_1/sphere_radius.csv")';

% cylinderic obstacle - for env 1 keep at 2 or 3
% cordinate which define centre of obstacle around which structure is built
cylinderMatrix  = csvread("../FYP/environment_2/cylinder.csv");   
cylinderRMatrix = csvread("../FYP/environment_2/cylinder_radius.csv")'; % radius
cylinderHMatrix = csvread("../FYP/environment_2/cylinder_height.csv")'; % height

% cone obstacle - for env 1 keep at 2 or 3
% cordinate which define centre of obstacle around which structure is built
coneMatrix      = csvread("../FYP/environment_2/cone.csv");  
coneRMatrix     = csvread("../FYP/environment_2/cone_radius.csv"); % radius
coneHMatrix     = csvread("../FYP/environment_2/cone_height.csv"); % height

% start and target
start = csvread("../FYP/environment_1/start.csv")'; %Start coordinates
goal = csvread("../FYP/environment_1/goal.csv")'; %Goal Coordinates

obstacles.sphere.center = obstacleMatrix;
obstacles.sphere.radius = RobstacleMatrix;
obstacles.cylinder.center = cylinderMatrix;
obstacles.cylinder.radius = cylinderRMatrix;
obstacles.cylinder.height = cylinderHMatrix;
obstacles.cone.center = coneMatrix;
obstacles.cone.radius = coneRMatrix;￼
obstacles.cone.height = coneHMatrix;

end

function draw_environment(obstacles, start, goal)

ob_sphere   = obstacles.sphere;
ob_cylinder = obstacles.cylinder;
ob_cone     = obstacles.cone;

%% Function to compute collision avoidance and draw obstacles are
%% obtained from https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm

[numberOfSphere, ~]   = size(ob_sphere.center);
[numberOfCylinder, ~] = size(ob_cylinder.center);
[numberOfCone,~]      = size(ob_cone.center);


figure(1)
 for i = 1:numberOfSphere
     drawSphere(ob_sphere.center(i,:), ob_sphere.radius(i))
 end

  %% For Environment 1 : Comment out lines 262-268
 %% For Environments 2 and 3 : Uncomment

   for i = 1:numberOfCylinder
       drawCylinder(ob_cylinder.center(i,:), ob_cylinder.radius(i), ob_cylinder.height(i));
   end
   
   for i = 1:numberOfCone
       drawCone(ob_cone.center(i,:), ob_cone.radius(i), ob_cone.height(i));
   end

scatter3(start(1), start(2), start(3), 80, ...
    'c', 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
scatter3(goal(1),  goal(2),  goal(3),  80, ...
    'm', 'filled', 'o', 'MarkerEdgeColor', 'k');

text(start(1),start(2),start(3),'  Start');
text(goal(1), goal(2), goal(3), '  Target');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Dispersive Flies Optimization');
axis equal

end

function path = getPath(cmap, param)
%Generate a path.
%
% cmap: collision map

X = param.X; Y = param.Y; Z = param.Z; 
[Nx, Ny, Nz] = size(cmap); % cmap size in 3 Dimensions
startid = param.startid; goalid = param.goalid;
PathLengthMax = param.PathLengthMax;
pathid = startid; nNodes = 1; posid = startid;
visited = false(Nx, Ny, Nz);
visited(posid(1), posid(2), posid(3)) = true;
ferror = false;
while any(posid ~= goalid) 
    [nid1, nid2, nid1nv, nid2nv] = getNeighbor(posid, goalid, cmap, visited);
    % select not-visited and valid neighbor first
    if ~isempty(nid2nv) 
        id = ceil(rand * size(nid2nv,1));
        posid = nid2nv(id,:);
        % if impossible, select not-visited neighbor next
    elseif ~isempty(nid1nv) 
        id = ceil(rand * size(nid1nv,1));
        posid = nid1nv(id,:);
        % if impossible, select valid neighbor next
    elseif ~isempty(nid2) 
        id = ceil(rand * size(nid2,1));
        posid = nid2(id,:);
        % if impossible, select general neighbor last
    elseif ~isempty(nid1) 
        id = ceil(rand * size(nid1,1));
        posid = nid1(id,:);
    else % no neighbor
        break;
    end
    visited(posid(1), posid(2), posid(3)) = true;
    pathid = [pathid; posid]; nNodes = nNodes + 1;
    % If there are no more unvisited nodes, break out of this loop because
    % all paths have been found or
    % No feasible path available
    if nNodes >= PathLengthMax 
        ferror = true; break;
    end
end
if ferror
    path = []; 
else
    % id -> position
    N = size(pathid,1);
    path = zeros(N,3);
    for i = 1:N
        id = pathid(i,:);
        path(i,:) = [X(id(1)), Y(id(2)), Z(id(3))];
    end
end

end

function [nid1, nid2, nid1nv, nid2nv] = getNeighbor(posid, goalid, cmap, visited)
%Find the next position based on the current position of UAV.
%   Check before and after (x axis), left and right (y axis), top and bottom
%   (z axis) of current position as the candidate of next position.
%   For the above 6 neighbors, 
%   - check if there is a collision
%   - check if the position was already visitid (contained already in path)
%   - for the x, y, and z axis, select a (valid) position nearer to goal.dx
%   (in other words, before or after in x axis, left or right in y axis, ...)
%
% cmap  : collision map
%
% nid1  :       neighbor nodes (without collision)
% nid2  : valid neighbor nodes (without collision + nearer to goal)
% nid1nv:       neirghbo nodes (not visited + without collision)
% nid2nv: valid neighbor nodes (not visited + without collision + nearer to goal)

[Nx, Ny, Nz] = size(cmap);
delta = eye(3); 
dir = goalid - posid;
nid1 = []; nid2 = []; nid1nv = []; nid2nv = []; % List for each heirarchy of nodes found
for i = 1:3 % for x, y, z
    
    % negative neighbor (before/left/bottom) 
    mpos = posid - delta(i,:); %
    if all(mpos > 0) && all([Nx Ny Nz] - mpos >= 0) % check range
        if cmap(mpos(1), mpos(2), mpos(3)) == false % check for presence of collision (breach of minimum range)
            not_visited = ~visited(mpos(1), mpos(2), mpos(3)); % check for presence of not_visited
            nid1 = [nid1; mpos];
            if not_visited
                nid1nv = [nid1nv; mpos]; 
            end
            if dir(i) < 0 % goal is in the negative direction
                nid2 = [nid2; mpos]; % hecks for a valid neighbor node (nid1) and then checks for not_visited.
                if not_visited
                    nid2nv = [nid2nv; mpos]; % If not_visited is true, nid2 will be checked to see if it's valid as well.
                end
            end
        end
    end
    
    % positive neighbor (after/right/top)
    ppos = posid + delta(i,:); 
    if all(ppos > 0) && all([Nx Ny Nz] - ppos >= 0)
        if cmap(ppos(1), ppos(2), ppos(3)) == false
            not_visited = ~visited(ppos(1), ppos(2), ppos(3));
            nid1 = [nid1; ppos];
            if not_visited
                nid1nv = [nid1nv; ppos];
            end
            if dir(i) > 0 % goal is in the positive direction
                nid2 = [nid2; ppos];
                if not_visited
                    nid2nv = [nid2nv; ppos];
                end
            end
        end
    end
    
end

end

function flag = collision(pos, obstacles)
%Detect collision at the position 'pos'
%   For example, for the shpere, when the distance between the UAV and the 
%   center of the sphere is smaller than the radius of the sphere, it means
%   collision.

ob_sphere   = obstacles.sphere;
ob_cylinder = obstacles.cylinder;
ob_cone     = obstacles.cone;
[numberOfSphere, ~]   = size(ob_sphere.center);
[numberOfCylinder, ~] = size(ob_cylinder.center);
[numberOfCone,~]      = size(ob_cone.center);
flag = false; dr = 0.1; % SAFETY MARGIN : minimum distance from obstacles  - 0.1
x = pos(1); y = pos(2); z = pos(3); 


%% Obstacle avoidance and penalty similar across all baseline methods for objective evaluation
% sphere
for i = 1:numberOfSphere
    o = ob_sphere.center(i,:); 
    r = ob_sphere.radius(i);
    % if position of UAV is less than (radius of object + safety margin) then collision has occured
    if norm(pos - o) <= r + dr 
        flag = true;
        break;
    end
end

% cylinder
for i = 1:numberOfCylinder
    o = ob_cylinder.center(i,:); 
    r = ob_cylinder.radius(i); 
    h = ob_cylinder.height(i);
    % if position of UAV is less than (radius of object + safety margin) then collision has occured
    if norm([x y] - o) <= r + dr && z >= 0 && z <= h 
        flag = true;
        break;
    end
end

% cone
for i = 1:numberOfCone
    o = ob_cone.center(i,:); 
    r = ob_cone.radius(i); 
    h = ob_cone.height(i);
    rz = r - r * z / h;
    % if position of UAV is less than (radius of object + safety margin) then collision has occured
    if z >= 0 && z <= h && norm([x y] - o) <= rz + dr 
        flag = true;
        break;
    end
end

end

%% Inspiration from Mutation operation of genetic algorithm where insertion occurs. I referred to ACO code for this need

%Adjust the path obtained by getPath.
%   The paths obtained by getPath have random length.
%   To make the paths(that belongs to a population) to have the same length,
%   their lengthes must be adjusted.
%   The method is to insert new nodes between two neighbor nodes of the short path.
%   The inserted node is the intermediate point of two nodes.
%   Insertion positions are selected randomly.
%   [1,  2,  3,  4,  5,  6]: length = 6. (Length must be 9.)
%   [1,7,2,8,3,  4,9,5,  6]: length = 9. (Inserted at 1, 2, and 4.)


function path2 = adjust_path(path1, MaxLen)
len = size(path1,1); lenleft = MaxLen - len;
if lenleft == 0
    path2 = path1;
else
    if len - 1 < lenleft % too short
        path2 = [];
    else
        id = sort(randperm(len-1, lenleft)); % insertion point
        path2 = zeros(MaxLen,3);
        n = 0; m = 1;
        for i = 1:len-1
            n = n + 1;
            path2(n,:) = path1(i,:);
            if m > lenleft, continue; end
            if i == id(m)
                n = n + 1;
                path2(n,:) = (path1(i,:) + path1(i+1,:)) / 2; % intermediate
                m = m + 1;
            end
        end
        n = n + 1;
        path2(n,:) = path1(len,:);
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fitness Function -- START
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fitness = fitness_fcn(X, obstacles, start, goal)
%Calculate the fitness of the path X
%   The fitness of the path is its total length.
%   If the path collides with obstacles, the fitness value is greatly increased.
%   In other words, penalties apply for such paths.

%matrix with size(X) rows and columns where each row corresponds to one dimension of traversal (the x-axis), 
%while each column corresponds to one node on that axis (the y-axis).
[N, D] = size(X); 
%define dsg as Euclidean(start - goal) because this helps determine how much penalty should be applied 
% when a collision occurs between two nodes on different dimensions.
NP = D / 3; dsg = norm(start - goal); 
% starts out at 0 for every node in our matrix representing every possible direction from start point towards goal point.
fitness = zeros(N,1); 
% The function iterates through each node in the graph (n) and calculates its fitness value using f = 0 for 
% start and prev = curr for all nodes until it reaches goal.
for n = 1:N 
    x = X(n,:);
    x = reshape(x, 3, NP)'; % 
    prev = start; f = 0; % 
    for i = 1:NP
        curr = x(i,:);
        if collision(curr, obstacles)
            % Apply penalty if collision occurs
            f = f + dsg; 
        else
            % penalty is added to Euclidean distance between current and previous point|add the remaining nodes
            f = f + norm(curr - prev); 
        end
        prev = curr;
    end
    % Euclidean Distance from start to goal
    f = f + norm(prev - goal); 
    % fitness is the previous (last best path) - goal
    fitness(n) = f; 
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fitness Function -- END
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
