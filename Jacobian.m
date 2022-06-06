%% Here we calculate the jacobian based on the joint angles

function J = Jacobian(theta)

len = [1 1];

dx1 = -(len(1)*cos(theta(2)) + len(2)*cos(theta(2)-theta(3)))*sin(theta(1));
dx2 = (-len(1)*sin(theta(2))-len(2)*sin(theta(2)-theta(3)))*cos(theta(1));
dx3 = (len(2)*sin(theta(2)-theta(3)))*cos(theta(1));

dy1 = (len(1)*cos(theta(2))+len(2)*cos(theta(2)-theta(3)))*cos(theta(1));
dy2 = (-len(1)*sin(theta(2))-len(2)*sin(theta(2)-theta(3)))*sin(theta(1));
dy3 = len(2)*sin(theta(2)-theta(3))*sin(theta(1));

dz1 = 0;
dz2 = len(1)*cos(theta(2))+len(2)*cos(theta(2)-theta(3));
dz3 = -len(2)*cos(theta(2)-theta(3));

J = [dx1,dx2,dx3;
     dy1,dy2,dy3;
	dz1,dz2,dz3];
