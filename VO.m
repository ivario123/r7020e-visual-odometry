clear
clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Load the relevant files                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Time stamp for each frame
frame_times = readmatrix("./kitti/00/times.txt");

% Get the camera feeds
cam0 = imageDatastore("./kitti/00/image_0/*", "FileExtensions", ".png");
cam1 = imageDatastore("./kitti/00/image_1/*", "FileExtensions", ".png");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Camera calibration data loading             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the calibration data
calib = readmatrix('./kitti/00/calib.txt');
% Collect the projection matricies
p11 = calib(1, 2:13);
p11 = reshape(p11, 4, 3);
p1 = permute(p11, [2, 1]); % Left projection matrix
p22 = calib(2, 2:13);
p22 = reshape(p22, 4, 3);
p2 = permute(p22, [2, 1]); % Right projection matrix

% Projection parameters
fu1 = p1(1, 1); % focal lengths
fv1 = p1(2, 2);
cu1 = p1(1, 3);
cv1 = p1(2, 3);
bx1 = -p1(1, 4) / fu1; % baseline

fu2 = p2(1, 1);
fv2 = p2(2, 2);
cu2 = p2(1, 3);
cv2 = p2(2, 3);
bx2 = -p2(1, 4) / fu2;

f = mean([fu1 fu2 fv1 fv2]);
intrinsics = cameraIntrinsics([fu1, fv1], [cu1 cv1], size(readimage(cam0, 1)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Prepare some state data                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
features = [];
pose = rigidtform3d(eye(3, 3), zeros(1, 3));
all_poses = zeros(length(cam0.Files), 3);

for i = 1:length(cam0.Files)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %               Load images and detect features               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    lf = readimage(cam0, i);
    rf = readimage(cam1, i);
    % Get the features
    f_l = detectSIFTFeatures(lf);
    f_r = detectSIFTFeatures(rf);
    best = 1000;
    % I think that the intrinsics were in mm not in meters
    intrinsics_l = cameraIntrinsics([fu1, fv1], [cu1 cv1], size(lf));
    intrinsics_r = cameraIntrinsics([fu1, fv1], [cu1 cv1], size(rf));
    % Extract descriptors
    lf_rect = undistortImage(lf, intrinsics_l);
    rf_rect = undistortImage(rf, intrinsics_r);
    [l_desc, l_pos] = extractFeatures(lf_rect, f_l);
    [r_desc, r_pos] = extractFeatures(rf_rect, f_r);

    % If it's not the first itteration we want to match features
    if ~isempty(features)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                      Match the features                     %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Prepare for last step
        inter_frame = matchFeatures(l_desc, r_desc);

        % Package the data together neatly
        current_features = struct( ...
        "l_desc", l_desc, ...
            "l_pos", l_pos, ...
            "r_desc", r_desc, ...
            "r_pos", r_pos ...
        );
        old_features = features;

        % Track the features that still exist
        [features_in_current_space, remaining_old_features, lm, rm] = ...
        find_remaining_points(old_features, current_features);

        % Triangulate 3d coords in both the old and the new coordinate
        % frame
        p = zeros([size(features_in_current_space.l_pos.Location, 1), 3]);
        remaining_old_features.pos = zeros( ...
            [size(features_in_current_space.l_pos.Location, 1), 3]);

        for itter = 1:size(features_in_current_space.l_pos.Location, 1)
            remaining_old_features.pos(itter, :) = triangulate(remaining_old_features.l_pos(itter).Location, remaining_old_features.r_pos(itter).Location, p1, p2);
            p(itter, :) = triangulate(features_in_current_space.l_pos(itter).Location, features_in_current_space.r_pos(itter).Location, p1, p2);

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                           Odometry                          %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Estimate current world pose
        est_pose = estworldpose( ...
        double(features_in_current_space.l_pos.Location), ...
            double(remaining_old_features.pos), ...
            intrinsics ...
        );

        % Convert coord system to the world coord system
        pose = rigidtform3d(pose.A * est_pose.A);

        % Append the pose to the world poses
        all_poses(i, :) = pose.Translation;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                      Add new landmarks                      %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for index = 1:length(inter_frame)

            if isempty(find(lm(:, 1) == inter_frame(index, 1), 1)) && isempty(find(rm(:, 2) == inter_frame(index, 2), 1))
                lm = [lm; inter_frame(index, 1), 0];
                rm = [rm; inter_frame(index, 2), 0];
            end

        end

        % possibly add features
        l_desc = l_desc(lm(:, 1), :);
        l_pos = l_pos(lm(:, 1));
        r_desc = r_desc(rm(:, 1), :);
        r_pos = r_pos(rm(:, 1));

        % Now we can display results
        pretty_print(pose, est_pose, frame_times, i);

        % Show the poses
        figure(2);
        plot(all_poses(1:i, 1), all_poses(1:i, 3));

        % Display the current frame
        figure(1);
        imshow(lf_rect);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Keep the features that exist in both the left and right view%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Match the descriptors
    matched = matchFeatures(l_desc, r_desc);

    %ptk = min([best, length(matched)]);

    % If result is strange, limit the number of points kept with ptk
    l_pos = l_pos(matched(:, 1));
    r_pos = r_pos(matched(:, 2));
    l_desc = l_desc(matched(:, 1), :);
    r_desc = r_desc(matched(:, 2), :);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       Store the features detected until next itteration     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Store the features untill the next itteration
    features = struct( ...
    "l_desc", l_desc, ... % Feature descriptors in left image
        "r_desc", r_desc, ... % -||- right image
        "l_pos", l_pos, ...
        "r_pos", r_pos, ...
        "matched", matched ...
    );

end

function pretty_print(pose, est_pose, frame_times, i)
    clc;
    fprintf("      Pose at frame %d\n",i)
    disp("---                       ---");
    fprintf("Estimated distance :%.2f\n", norm(est_pose.T))
    disp("---                       ---");
    fprintf("Estimated velocity :%.2f\n", 3.6 * norm(est_pose.T) ./ (frame_times(i) - frame_times(i - 1)))
    disp("---                       ---");
    disp("Pose")
    disp(pose.Translation);
    disp("---                       ---");
    disp("Rotation")
    disp(pose.R);
    disp("---                       ---");
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         FUNCTIONS                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper functions to remove clutter from the main function

function [current_features, old_features, lm, rm] = find_remaining_points(old_features, current_features)

    % Match the descriptors over time in the left image
    feed_formward_left_frame_matches = matchFeatures(current_features.l_desc, old_features.l_desc);
    lm = feed_formward_left_frame_matches;
    %p = p(lm(:, 2));

    old_features.l_desc = old_features.l_desc(feed_formward_left_frame_matches(:, 2), :);
    old_features.l_pos = old_features.l_pos(feed_formward_left_frame_matches(:, 2), :);
    old_features.r_pos = old_features.r_pos(feed_formward_left_frame_matches(:, 2), :);
    old_features.r_desc = old_features.r_desc(feed_formward_left_frame_matches(:, 2), :);

    % Match the descriptors over time in the right image
    feed_formward_right_frame_matches = matchFeatures(current_features.r_desc, old_features.r_desc);
    rm = feed_formward_right_frame_matches;
    % Now we have the old points
    %old_points = p(rm(:, 2));
    old_features.l_desc = old_features.l_desc(feed_formward_right_frame_matches(:, 2), :);
    old_features.l_pos = old_features.l_pos(feed_formward_right_frame_matches(:, 2), :);
    old_features.r_pos = old_features.r_pos(feed_formward_right_frame_matches(:, 2), :);
    old_features.r_desc = old_features.r_desc(feed_formward_right_frame_matches(:, 2), :);

    current_features.l_desc = current_features.l_desc(feed_formward_left_frame_matches(:, 1), :);
    current_features.l_pos = current_features.l_pos (feed_formward_left_frame_matches(:, 1));

    current_features.r_desc = current_features.r_desc(feed_formward_right_frame_matches(:, 1), :);
    current_features.r_pos = current_features.r_pos(feed_formward_right_frame_matches(:, 1));

    % Match the descriptors between the left and right image
    current_frame_matches = matchFeatures(current_features.l_desc, current_features.r_desc);

    % Get new valid points
    current_features.l_desc = current_features.l_desc(current_frame_matches(:, 1), :);
    current_features.r_desc = current_features.r_desc(current_frame_matches(:, 2), :);
    current_features.l_pos = current_features.l_pos(current_frame_matches(:, 1));
    current_features.r_pos = current_features.r_pos(current_frame_matches(:, 2));

    % Now that we know what features still exist between the two cameras we
    % can remove the ones that are out of view from atleast one of the images
    last_match = matchFeatures(current_features.l_desc, old_features.l_desc);
    old_features.l_desc = old_features.l_desc(last_match(:, 2), :);
    old_features.l_pos = old_features.l_pos(last_match(:, 2), :);
    old_features.r_pos = old_features.r_pos(last_match(:, 2), :);
    current_features.l_desc = current_features.l_desc(last_match(:, 1), :);
    current_features.r_desc = current_features.r_desc(last_match(:, 1), :);
    current_features.l_pos = current_features.l_pos(last_match(:, 1), :);
    current_features.r_pos = current_features.r_pos(last_match(:, 1), :);
end
