%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 5: Feasible Trajectory Dataset Generation via Iterative Optimization  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function trajectory = generateIK(robot, lambda, q0, targetPosition, maxJointSpeed, toleranceDistance, dt)
    trajectory = []; % 8xN array with 4 joints position and 4 joints speed

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory = []; % 8xN array with 4 joints position and 4 joints speed

    q_current = q0;  % Initial joint configuration
    t = 0;           % Initialize time
    condition_numbers = []; % Store condition numbers for analysis
    success = false; % Flag for success

    % Select Jacobian inversion method
    inversion_method = 'moore-penrose'; % Change to 'damped' for damped pseudo-inverse

    % Start timing
    tic;

    % Iterative optimization loop
    while true
        % Step 1: Compute current Cartesian position
        x_current = robot.fastForwardKinematic(q_current);
        
        % Step 2: Check termination condition
        distance_to_target = norm(targetPosition - x_current);
        if distance_to_target < toleranceDistance
            success = true;
            break;
        end

        % Step 3: Compute Jacobian at the current configuration
        J = robot.fastJacobian(q_current);

        % Step 4: Compute the condition number of the Jacobian
        cond_number = cond(J); % Condition number of J
        condition_numbers = [condition_numbers, cond_number];

        % Step 5: Select Jacobian inversion method
        switch lower(inversion_method)
            case 'moore-penrose'
                % Moore-Penrose pseudo-inverse
                J_pseudo_inverse = pinv(J);
            case 'damped'
                % Damped pseudo-inverse
                J_pseudo_inverse = J' * (J * J' + lambda^2 * eye(size(J, 1)))^(-1);
            otherwise
                error('Invalid Jacobian inversion method.');
        end

        % Step 6: Compute desired Cartesian velocity towards the target
        v_task = (targetPosition - x_current) / distance_to_target;

        % Step 7: Compute joint velocities from Cartesian velocity
        q_dot = J_pseudo_inverse * v_task;

        % Step 8: Scale velocities to ensure they are within limits
        max_joint_velocity = max(abs(q_dot));
        if max_joint_velocity > maxJointSpeed
            q_dot = (q_dot / max_joint_velocity) * maxJointSpeed;
        end

        % Step 9: Compute the next joint configuration
        q_next = q_current + q_dot * dt;

        % Step 10: Store the joint positions, velocities, and condition numbers
        trajectory = [trajectory, [q_current; q_dot]];

        % Step 11: Update for the next iteration
        q_current = q_next;
        t = t + dt;
    end

    % Stop timing
    time_taken = toc;

    % Output metrics
    fprintf('Iterative Optimization:\n');
    fprintf('- Time Taken: %.4f seconds\n', time_taken);
    fprintf('- Final Distance to Target: %.4f\n', distance_to_target);
    fprintf('- Success: %d\n', success);
    fprintf('- Average Condition Number: %.2f\n', mean(condition_numbers));
    fprintf('- Maximum Condition Number: %.2f\n', max(condition_numbers));
end