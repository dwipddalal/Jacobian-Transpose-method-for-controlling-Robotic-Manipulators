% the function uses Jacobian transpose method to calculate the joint angles theta that will minimize the error and bring it within the error distance of target

function[theta] = theta_Calculation(target, initial_theta, err)

theta = initial_theta;
s = end_pos(theta, [0;0;0]); % function gets the end effector
t = target;
e = t - s;
i = 0;

while(i < 250 && norm(e) > err) 
	i = i+1;
	% recalculation of the direction that the arm is supposed to move towards and gets the partial derivatives of end effector co-ordinates w.r.t the angles
	J = Jacobian(theta);
	J_T = transpose(J);
	cal = J*J_T*e;
    
	% moving towards the goal using the Jacobian
	a = (dot(e,cal))/(dot(cal,cal));
    % amount to move the joints towards the goal
	delta_theta = a*J_T*e;
	theta = theta + delta_theta;
    
	% calculates the new position     
	s = end_pos(theta, [0;0;0]);
    e = t - s;
end


