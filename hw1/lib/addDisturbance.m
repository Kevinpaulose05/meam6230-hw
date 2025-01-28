%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3a: Add disturbance to trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [disturbance_idx, q_mid] = addDisturbance(optimal_solution_full, q_disturbance, disturbance_val)
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % q_disturbance is the index of joint you want to disturb
    % Modify q_mid configuration to simulate a perturbation pushing the robot
    % arm in another configuration

    % Extract the trajectory and time vector
    Xopt = optimal_solution_full.Xopt; % Joint positions over the trajectory
    Topt = optimal_solution_full.Topt; % Time vector
    
    % Choose a disturbance index (mid-point of the trajectory)
    disturbance_idx = round(length(Topt) / 2); 
    
    % Extract the joint configuration at the disturbance point
    q_mid = Xopt(:, disturbance_idx);
    
    % Apply the disturbance to the specified joint
    disturbance_val = deg2rad(disturbance_val); % value in radians
    q_mid(q_disturbance) = q_mid(q_disturbance) + disturbance_val;
    
    % Ensure the joint values remain within feasible bounds
    joint_min = -pi; % Replace with actual joint limits
    joint_max = pi;  % Replace with actual joint limits
    q_mid(q_disturbance) = max(min(q_mid(q_disturbance), joint_max), joint_min);

end