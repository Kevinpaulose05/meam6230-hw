% Task 2c: Minimum distance in joint space 
% This function integrates dq to minimize joint trajectory length
% USE THE SQUARE OF THE NORM FOR NUMERICAL STABILITY

% The robot arm is modeled with the following state space representation:
% - X: state vector = 4 joint angles of the robot arm
% - U(1:4): input vector = 4 joint speed
% - U(5): final time of the trajectory (constant for all timesteps), at 
% which the robot arm should reach a specified target
function cost = minimumJointDistance(X, U, e, data, robot, target)
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize cost
    cost = 0;
    
    % Number of waypoints in the trajectory
    N = size(X, 1);

    % Accumulate the squared joint increments
    for i = 1:N-1
        dq = X(i+1,:)' - X(i,:)'; 
        cost = cost + norm(dq, 2)^2;
    end
end
