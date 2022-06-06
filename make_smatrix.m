
%%  This function gives back a matrix of positions in which we have divided the true linear path



function snew = make_smatrix(t, thetainit, n)

%n isthe number of steps
%snew is a list of new positions to be outputted
%t is the target position
%thetainit is the initial value of thetas of the arm.

s = end_pos( thetainit, [0;0;0]);
  e = t - s;
  eN = e / n;
  snew = [s,zeros(3, n)];         
  %We set the first value to the initial position 
  for a = 1:n
    snew(:,a+1) = snew(:,a) + eN; 
    %The size of snew is 3 x n+1
  end
    
end



