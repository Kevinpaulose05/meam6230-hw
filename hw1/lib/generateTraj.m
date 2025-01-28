%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3b: Generate complete trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function optimal_solution_final = generateTraj(disturbance_idx, optimal_solution_full, optimal_solution_after_disturbance)
    optimal_solution_final = [];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract pre-disturbance trajectory components
    T_pre = optimal_solution_full.Topt(1:disturbance_idx)';
    X_pre = optimal_solution_full.Xopt(:, 1:disturbance_idx);
    Y_pre = optimal_solution_full.Yopt(:, 1:disturbance_idx);
    MV_pre = optimal_solution_full.MVopt(:, 1:disturbance_idx);
    
    % Extract post-disturbance trajectory components
    T_post = optimal_solution_after_disturbance.Topt';
    X_post = optimal_solution_after_disturbance.Xopt;
    Y_post = optimal_solution_after_disturbance.Yopt;
    MV_post = optimal_solution_after_disturbance.MVopt;
    
    % Combine pre- and post-disturbance trajectories
    optimal_solution_final.Topt = [T_pre; T_pre(end)' + T_post]; % concantenation
    optimal_solution_final.Xopt = [X_pre, X_post];
    optimal_solution_final.Yopt = [Y_pre, Y_post];
    optimal_solution_final.MVopt = [MV_pre, MV_post];
end