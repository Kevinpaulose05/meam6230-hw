%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 6: Feasible Trajectory Dataset Generation via Optimal Control   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for  Chapter 2 of:                                      %
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
%%  Create solver
clear; close all; clc;
filepath = fileparts(which('chp2_algo1_optimal_control.m'));
addpath(genpath(fullfile(filepath, '..',  'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'lib')));

robot = RobotisWrapper();
optimalControl = MPC4DOF(robot);
% Uncomment one of the cost functions to test
optimalControl.nlSolver.Optimization.CustomCostFcn = @minimumTime;
%optimalControl.nlSolver.Optimization.CustomCostFcn = @minimumTaskDistance;
%optimalControl.nlSolver.Optimization.CustomCostFcn = @minimumJointDistance;

target_position = [0.25; 0; 0];
toleranceDistance = 10e-3;
maxTime = 5;

%% Generate batch of trajectories
nTraj = 10; % Number of trajectories
nPoints = optimalControl.nlSolver.PredictionHorizon + 1;
optimalTrajectories = nan(3, nPoints, nTraj);
metrics = []; % Store performance metrics

h = waitbar(0, 'Computing trajectories...');
for iTraj = 1:nTraj
    % Start timing
    tic;

    % Find solution starting at random configuration
    q0 = robot.robot.randomConfiguration;
    optimalSolution = optimalControl.solveOptimalTrajectory(target_position, q0, maxTime, true, true);

    % Stop timing
    time_taken = toc;

    % Compute final position and deviation
    final_position = optimalSolution.Yopt(:, end);
    deviation = norm(final_position - target_position);
    success = deviation < toleranceDistance;

    % Store successful trajectories
    if success
        optimalTrajectories(:, :, iTraj) = optimalSolution.Yopt;
    end

    % Log metrics
    metrics = [metrics; struct('trajectory_id', iTraj, ...
                               'time_taken', time_taken, ...
                               'deviation', deviation, ...
                               'success', success)];

    % Visualize progression
    waitbar(iTraj / nTraj, h, sprintf('Computing trajectories... (%d/%d)', iTraj, nTraj));
end
close(h);

%% Display metrics
fprintf('Optimal Control Metrics:\n');
for i = 1:length(metrics)
    fprintf('Trajectory %d:\n', metrics(i).trajectory_id);
    fprintf('- Time Taken: %.4f seconds\n', metrics(i).time_taken);
    fprintf('- Deviation: %.4f\n', metrics(i).deviation);
    fprintf('- Success: %d\n\n', metrics(i).success);
end

% Success rate summary
success_rate = mean([metrics.success]) * 100;
fprintf('Summary:\n');
fprintf('- Total Trajectories: %d\n', nTraj);
fprintf('- Success Rate: %.2f%%\n', success_rate);

% Display all successful trajectories
optimalControl.showTaskVolume(optimalTrajectories);