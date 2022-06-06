%% In this file we will visualize the movement of the end effector between each iteration of the jacobian transpose method. 


%% Initialization of variables
%Moves the end effector to the arm between each iteration of the jacobian tranpose method,
% results in fractured pathing, but it is interesting


more off; 
% This allows lots of output to the command window without having to press F

origin = [0;0;0];    
%reference point for the entire process, basically the origin.

theta = [0;20;70]; 
% Vector of the three angles 

n = 25;
% Vhe number of iterations for which the Jacobian Transpose method iterates

theta = theta*(pi/180); 
% We convert the values to radians

theta_mat = (theta);
% A matrix for values of theta, will be used later

s = end_pos(theta, [0;0;0]); 
% We use the above function to get the vector that represents the position
% vector

init = s;
% Used in plotting, to store the location of the arm initially

t = [-0.8;-1;0.8]; 
% the goal of where the end effector must be, basically the target
% location

s_mat = transpose(s);
% We convert the column vector of s to row vector of s. It is done to give cleaner output later while plotting. 

e = t - s;
% We define the error vector

%% We go ahead with the Jacobian Transpose method with n interations.

for i = 1:1:n 
    % We run the jacobian method n times

	e = t - s; 
    % We recalculate the direction that the arm is supposed to move towards

	J = Jacobian(theta); 
    % We get the partial derivatives of end effector co-ordinates with 
    % respect to the angles
	
    JT = transpose(J); 
    % We get the transpose of the Jacobian matrix.

	jjte = J*JT*e; 
    % Used to find the value of the arbitrary coefficient difined as a in
    % the next line

	a = (dot(e,jjte))/(dot(jjte,jjte)); 
    % We calculate the value of the coefficient as required.

	deltaQ = a*JT*e; 
    % We calculate the change in q given by the Jacobian Transpose method 
    % as per the approach given in the report.

	theta = theta + deltaQ; 
    % We modify the value of theta as required.
    
    theta_mat = [theta_mat,theta];
    %append the values of theta to theta_mat, will be used while plotting.

	s = end_pos(theta, [0;0;0]); 
    % We get the new position after the movement undergone upon the
    % transition to new values of angles.

	s_mat = [s_mat;transpose(s)];   
    % We convert the column vectors to row vectors for better utilization
    % later
end

%% Plotting the path traversed 

X = [init(1),t(1)];
Y = [init(2),t(2)];
Z = [init(3),t(3)];
% We set up the parameters of the ideal linear path

for a = 1:n
    
    Arm = [origin ,mid_pos(theta_mat(:,a), [0;0;0]) , end_pos(theta_mat(:,a),[0;0;0])];
    plot3(X,Y,Z, 'green','linewidth',3,'DisplayName',sprintf('Ideal Linear Path'),'marker','o' );
    %Here we plot the ideal path
    
    %We set up the parameters for plotting
    hold on;
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    view(20,50);
    
    %We give the labels
    xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
    ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
    zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
    
    
    %We plot the current linear path of the Jacobian method
    plot3(s_mat([1:a],1),s_mat([1:a],2),s_mat([1:a],3), 'cyan','linewidth',3,'DisplayName',sprintf('path of arm') );
    
    %We plot the current version of the arm
    plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'black', 'linewidth',2,'DisplayName',sprintf('arm'));
    
    lgd = legend('show', 'location', 'northwest');
    hold off;
    
    pause(0.2);
    % We are pausing for better visualization 
end
  


