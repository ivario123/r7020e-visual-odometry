function ShowPoseAndLandmarks(all_poses, landmarks, end_index)
    % Plots the pose and landmarks in 3d in the same figure
    % all_poses: Nx3 matrix of poses
    % landmarks: Mx3 matrix of landmarks
    %
    % This is quite expensive to run, so only use it for debugging or
    % visualization at the end of the algorithm.

    % Plot the landmarks
    view(3);
    hold on
    % Show the landmarks as small red dots
    plot3(landmarks(:,1), landmarks(:,2), landmarks(:,3), 'r.', 'MarkerSize', 1);

    for i = 2:end_index
        % Plot the pose
        plot3([ all_poses(i).Translation(1) all_poses(i-1).Translation(1) ], ...
              [ all_poses(i).Translation(2) all_poses(i-1).Translation(2) ], ...
              [ all_poses(i).Translation(3) all_poses(i-1).Translation(3) ], 'b-','LineWidth', 5);
    end
    set(gca, 'Projection','perspective')
    axis equal
    % Show the plot from the perspective of the robot
    

    hold off
end
