%% Initialize Variables

theta = [0;-150;-50];
% the initial position in angles of deg

t = [1.8;0.9;-0.5];
% the target position

Jac_error = 0.001;    
% the end case precision on the Jacobian transpose method

Emax = 0.9;  
% absolute worst positional error

subsections = 4;
% number of breaks in path in-between  sections 

%% Computations

theta = theta*(pi/180);     
origin = [0;0;0];

s = end_pos(theta,[0;0;0]);
% computes all the starting points

e = t - s;      
%this is the length of your linear path in the form of a vector

n = ceil(norm(e)/Emax);

theta_mat = [theta,zeros(3,n)];  
% intialize list of all angles for each target point starting at the initial position

Points = make_smatrix(t,theta,n); 
% points is a 3 x n+1 matrix, it contains the values of n+1 points in coordinate points the path

for a = 1:n
   theta_mat(:,a+1) = theta_Calculation( Points(:,a+1), theta_mat(:,a),Jac_error); % ERROR of Jac_error
end
%CALCULATES THE ANGLES FOR EACH STEP POINT USING THE JACOBIAN TRANSPOSE METHOD

actualpath_end = [Points(:,1),zeros(3,subsections*n)]; 
% will hold the points for the end effector on the true path
actualpath_mid = [mid_pos(theta,[0;0;0]),zeros(3,subsections*n)]; 
% will hold the points for the midjoint on the true path
 
for a = 1:n 
    
    dtheta_mat = theta_mat(:,a+1) - theta_mat(:,a); 
    %takes the opposite of the sign of your dtheta_mat * 360, and adds your dtheta_mat to find your opposite rotation       
    if(abs(dtheta_mat(1))>pi) 
     dtheta_mat(1)= dtheta_mat(1)-2*pi*sign(dtheta_mat(1));  
    end
    if(abs(dtheta_mat(2))>pi) 
     dtheta_mat(2) = dtheta_mat(2)-2*pi*sign(dtheta_mat(2)); 
    end
    if(abs(dtheta_mat(3))>pi) 
     dtheta_mat(3) = dtheta_mat(3)-2*pi*sign(dtheta_mat(3));  
    end
    
    dtheta = dtheta_mat/subsections;                     
    % these are the incremental changes to theta to show that the end effector does not move linearly
   
    for b = 1:subsections                   
      actualpath_end(:, (a-1)*subsections+b+1) = end_pos( (theta_mat(:,a) + b*dtheta) , [0;0;0] );
      actualpath_mid(:, (a-1)*subsections+b+1) = mid_pos( ( theta_mat(:,a) + b*dtheta) , [0;0;0] );
    % calculates the value of points on each of the subsections in the interval of the path
    end
end

%% PLOTING
 
for c = 1:n*subsections+1  
    % the we added one here to print of the initial position also
    
    N = ceil(c/subsections);
    % N is the subsection we are surrently printing
        
    plot3(Points(1,[1:N]),Points(2,[1:N]),Points(3,[1:N]), 'g','linewidth',1,'DisplayName',sprintf('Ideal Path with Target Points'),'marker','o','markersize',4, 'MarkerEdgeColor',[0 0.7 0]);
    % print the ideal path up to each current target point with points and lines
    hold on;
    
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    view(-150,20);
    
    xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
    ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
    zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
    
    Arm = [origin ,actualpath_mid(:,c), actualpath_end(:,c)];
    % to plot the current arm
    plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'black', 'linewidth',2,'marker','o','DisplayName',sprintf('Arm'));
    
    plot3(actualpath_end(1,[1:c]),actualpath_end(2,[1:c]),actualpath_end(3,[1:c]),'cyan', 'linewidth',1,'DisplayName',sprintf('True Path')) 
    % plots the  path of the end effecor
    lgd = legend('show', 'location', 'northwest');
    
    pause(0.01); 
    
    hold off; 
    % clear the current frame    
    
end

