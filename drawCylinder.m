%% This function is from : https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm

function drawCylinder(pos, r, h)

[x,y,z] = cylinder(r,40);
z(2,:) = h;
surfc(x + pos(1), y + pos(2), z, 'FaceColor', 'interp'); hold on;
theta = linspace(0, 2 * pi, 40);
X = r * cos(theta) + pos(1);
Y = r * sin(theta) + pos(2);
Z = ones(size(X)) * h;
fill3(X, Y, Z, [0 0.5 1]); 
fill3(X, Y, zeros(size(X)), [0 0.5 1]); 
end