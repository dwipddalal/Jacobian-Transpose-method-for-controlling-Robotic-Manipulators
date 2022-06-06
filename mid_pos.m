%The function calculates the postion of the midjoint with the inputs of theta
function [s_mid] = mid_pos (theta, origin)

len = [1 1];
% len(1) is the lenth of first arm and len(2) is the length of second arm
y = (len(1)*cos(theta(2)))*sin(theta(1)) - origin(1);
x = (len(1)*cos(theta(2)))*cos(theta(1)) - origin(2);
z = len(1)*sin(theta(2)) - origin(3);
s_mid = [x;y;z];

end