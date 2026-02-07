%% This Implementation is from : https://github.com/ZYunfeii/UAV_Obstacle_Avoiding_DRL/tree/master/Traditional_obstacle_avoidance_algorithm

function res = MovingAverage(input,N)


sz = max(size(input));
n = (N - 1) / 2;
res = []; % empty list called res, which will hold the values for each point in time.
for i = 1:length(input) % loops through all the points in input and calculates their sum, dividing that number by 2 to get the moving average value at that point in time.
    if i <= n
        %if there are any more points left to calculate or not, and if so it adds them together with a new value calculated from adding 1 to end-1 until there is only one point left (the last point).
        res(i) = sum(input(1:2*i-1)) / (2 * i - 1);
    elseif i < length(input) - n + 1
        res(i) = sum(input(i-n:i+n)) / (2 * n + 1);
    else
        % If there are no more points left to calculate, then it calculates the moving average using just one number: end-1.
        temp = length(input) - i + 1;
        % input array and divides it into two parts, one part is used to calculate the moving average of the first half of the input array and another part is used to calculate the moving average of the second half.
        res(i) = sum(input(end-(2*temp-1)+1:end)) / (2 * temp - 1);
    end
end
end


