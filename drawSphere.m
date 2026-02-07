%% This function is from : https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm

function drawSphere(pos, r)

[x,y,z] = sphere(60);
surfc(r * x + pos(1), r * y + pos(2), r * z + pos(3));
hold on;
end

