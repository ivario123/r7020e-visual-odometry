function ShowFeaturesOnFeed(old_features, current_features,non_tracked_features, img)
    % Displays lines between the features in the old and current images.
    % The lines are drawn on the current image.
    % Also computes the distance between the features. and displays it on the
    % image at the coordinates of the current features.
    imshow(img);
    hold on;
    title('Left camera frame')

    for i = 1:size(old_features.pos, 1)
        % Shows the current features as dark green crosses.
        scatter(non_tracked_features.Location(:,1),non_tracked_features.Location(:,2),"Marker","x","MarkerEdgeColor",[0 0.5 0],"MarkerFaceColor",[0 0.5 0]);
        plot([old_features.l_pos.Location(i, 1) current_features.l_pos.Location(i, 1)], ...
            [old_features.l_pos.Location(i, 2) current_features.l_pos.Location(i, 2)], 'r');
        % Shows the distance between the features in a light green color.
        text(current_features.l_pos.Location(i, 1), current_features.l_pos.Location(i, 2), ...
            sprintf("%.1f[m]",norm(old_features.pos(i, :) - current_features.pos(i, :))),"Color", [0.5 1 0.5]);
    end

    hold off;

end
