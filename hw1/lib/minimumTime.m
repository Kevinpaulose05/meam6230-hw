% Task 2a: Minimum time 
% This function minimizes the time scaling parameter Tf to minimize
% trajectory time

% The robot arm is modeled with the following state space representation:
% - X: state vector = 4 joint angles of the robot arm
% - U(1:4): input vector = 4 joint speed
% - U(5): final time of the trajectory (constant for all timesteps), at 
% which the robot arm should reach a specified target
function cost = minimumTime(X, U, e, data, robot, target)

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract the final time from the control vector
    Tf = U(5); 
    
    % Define the cost function as the final time
    cost = Tf;
    
    % Optional: Add a small regularization term to stabilize optimization
    % cost = Tf + 1e-6 * sum(U(1:4).^2);

end