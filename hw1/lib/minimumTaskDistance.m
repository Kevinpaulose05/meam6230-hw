% Task 2b: Minimum distance in task space 
% This function integrates dx = J*dq to minimize Cartesian trajectory length
% You can obtain the Jacobian J at configuration q using
% J = robot.fastJacobian(q)
% USE THE SQUARE OF THE NORM FOR NUMERICAL STABILITY

% The robot arm is modeled with the following state space representation:
% - X: state vector = 4 joint angles of the robot arm
% - U(1:4): input vector = 4 joint speed
% - U(5): final time of the trajectory (constant for all timesteps), at 
% which the robot arm should reach a specified target
function cost = minimumTaskDistance(X, U, e, data, robot, target)
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize cost
    cost = 0;
    
    % Number of waypoints in the trajectory
    N = data.PredictionHorizon;
    
    % Loop through trajectory waypoints
    for i = 1:N
        % Current joint configuration
        q_current = X(i, :);

        % Extract the joint velocity vector from the control input
        dq = U(i, 1:4)'; 
        
        % Compute the Jacobian at the current configuration
        J = robot.fastJacobian(q_current);
        
        % Compute displacement in Cartesian space
        dx = J * dq;             % Cartesian displacement
        
        % Accumulate the square of the Euclidean norm of dx
        cost = cost + norm(dx)^2;
    end
end
