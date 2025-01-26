%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 4: Compute closed-form trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cartesianTrajectory = fillTrajectory(time, initial_position, waypoints, target_position)
    % Compute trajectory based on third order polynomial
    cartesianTrajectory = nan(3, length(time));
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute trajectory based on third order polynomial
    num_points = length(time);
    cartesianTrajectory = nan(3, num_points);

    % Combine all points: initial, waypoints, and target
    all_positions = [initial_position, waypoints, target_position];
    num_segments = size(all_positions, 2) - 1;

    % Time allocation for each segment
    segment_times = linspace(0, max(time), num_segments + 1);

    % Generate cubic polynomials for each segment
    for seg = 1:num_segments
        % Start and end positions for the segment
        p_start = all_positions(:, seg);
        p_end = all_positions(:, seg + 1);

        % Time for the segment
        t_start = segment_times(seg);
        t_end = segment_times(seg + 1);

        % Solve for polynomial coefficients for each dimension
        A = [1, t_start, t_start^2, t_start^3;
             0, 1, 2*t_start, 3*t_start^2;
             1, t_end, t_end^2, t_end^3;
             0, 1, 2*t_end, 3*t_end^2];

        for dim = 1:3
            % Values for start and end positions and their derivatives (velocities)
            b = [p_start(dim); 0; p_end(dim); 0];

            % Solve for cubic coefficients
            coeffs = A \ b;

            % Evaluate the polynomial for this segment's time
            segment_idx = find(time >= t_start & time <= t_end);
            t_segment = time(segment_idx);
            cartesianTrajectory(dim, segment_idx) = coeffs(1) + coeffs(2)*t_segment + ...
                                                    coeffs(3)*t_segment.^2 + coeffs(4)*t_segment.^3;
        end
    end
end