%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for  Chapter 1 of:                                      %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Alberic de Lajarte                                             %
% email:   alberic.lajarte@epfl.ch                                        %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 1: Initial trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('chp1_ex2.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'lib')));

% Create robot and optimal control
robot = RobotisWrapper();
optimal_control = MPC4DOF(robot);

% Set target position, initial configuration, and max time
initial_joint_configuration = [0; 0; 0; 0];
target_position = [0.1; -0.3; 0.1];
max_time = 3;

% Solve for the nominal trajectory
optimal_solution_full = optimal_control.solveOptimalTrajectory(target_position, ...
    initial_joint_configuration, max_time);

% Define disturbance points (e.g., 25%, 50%, and 75% of trajectory)
disturbance_points = [round(0.25 * length(optimal_solution_full.Topt)), ...
                      round(0.5 * length(optimal_solution_full.Topt)), ...
                      round(0.75 * length(optimal_solution_full.Topt))];
                  
% Define disturbance values (limited to 0 to 2)
disturbance_values = [0.5, 1.0, 1.5]; % Adjust values as needed
q_disturbance = 2; % Example: Apply disturbance to Joint 2

% Metrics to evaluate the effects of disturbances
results = struct();

for i = 1:length(disturbance_points)
    % Extract the disturbance point and value
    disturbance_idx = disturbance_points(i);
    disturbance_val = disturbance_values(i);
    
    % Add disturbance
    [disturbance_idx, q_mid] = addDisturbance(optimal_solution_full, q_disturbance, disturbance_val);
    
    % Recompute the trajectory
    time_left = max_time - optimal_solution_full.Topt(disturbance_idx);
    optimal_solution_after_disturbance = optimal_control.solveOptimalTrajectory(target_position, ...
        q_mid, time_left);
    
    % Generate the final trajectory
    optimal_solution_final = generateTraj(disturbance_idx, optimal_solution_full, ...
        optimal_solution_after_disturbance);
    
    % Calculate metrics
    nominal_Yopt = optimal_solution_full.Yopt;
    disturbed_Yopt = optimal_solution_final.Yopt;
    
    % Interpolate disturbed trajectory to match the size of the nominal trajectory
    disturbed_Yopt_resampled = interp1(1:size(disturbed_Yopt, 2), disturbed_Yopt', ...
                                       linspace(1, size(disturbed_Yopt, 2), size(nominal_Yopt, 2)))';
    
    % Calculate deviation
    deviation = mean(vecnorm(disturbed_Yopt_resampled - nominal_Yopt, 2, 1));
    
    % Recovery time: Time taken to recover after disturbance
    recovery_time = time_left;
    
    % Smoothness: Compute joint velocity variations
    joint_velocities = optimal_solution_final.MVopt(1:4, :);
    smoothness = sum(vecnorm(diff(joint_velocities, 1, 2), 2, 1));
    
    % Store results for analysis
    results(i).disturbance_idx = disturbance_idx;
    results(i).disturbance_val = disturbance_val;
    results(i).deviation = deviation;
    results(i).recovery_time = recovery_time;
    results(i).smoothness = smoothness;
    
    % Visualize the trajectory
    robot.animateTrajectory(optimal_solution_final.Xopt, ...
        optimal_solution_final.Yopt, target_position, ...
        sprintf('Disturbance at %.2f%%', (disturbance_idx / length(optimal_solution_full.Topt)) * 100));
    
    disp("Press space to continue...");
    pause();
end

% Display results
disp('Results of disturbances:');
for i = 1:length(results)
    fprintf('Disturbance at %.2f%% of trajectory:\n', (results(i).disturbance_idx / length(optimal_solution_full.Topt)) * 100);
    fprintf(' - Disturbance Value: %.2f\n', results(i).disturbance_val);
    fprintf(' - Deviation from Nominal: %.4f\n', results(i).deviation);
    fprintf(' - Recovery Time: %.4f seconds\n', results(i).recovery_time);
    fprintf(' - Smoothness (Velocity Changes): %.4f\n', results(i).smoothness);
    fprintf('\n');
end