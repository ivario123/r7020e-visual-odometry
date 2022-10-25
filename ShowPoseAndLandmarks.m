function ShowPoseAndLandmarks(all_poses, landmarks, end_index)

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
    axis equal
    hold off
end
