%The function calculates the postion of the end effector with the inputs of theta. 
function s = end_pos(theta, origin)

len = [1 1]; 
% len(1) is the lenth of first arm and len(2) is the length of second arm
x = (len(1)*cos(theta(2)) + len(2)*cos(theta(2)-theta(3)))*cos(theta(1)) - origin(1); 
y = (len(1)*cos(theta(2)) + len(2)*cos(theta(2)-theta(3)))*sin(theta(1)) - origin(2); 
z = (len(1)*sin(theta(2)) + len(2)*sin(theta(2)-theta(3))) - origin(3); 

s = [x;y;z];