close all
clear
clc

% Toggle, set this to false if you don't want the 3D view
view_3D = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Load the relevant files                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Time stamp for each frame
frame_times = readmatrix("./kitti/00/times.txt");

% Get the camera feeds
cam0 = imageDatastore("./kitti/00/image_0/*", "FileExtensions", ".png");
cam1 = imageDatastore("./kitti/00/image_1/*", "FileExtensions", ".png");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Camera calibration data loading               %
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
fu1 = p1(1, 1); % Focal lengths
fv1 = p1(2, 2);
cu1 = p1(1, 3); % Camera centers
cv1 = p1(2, 3);
bx1 = -p1(1, 4) / fu1;

fu2 = p2(1, 1);
fv2 = p2(2, 2);
cu2 = p2(1, 3);
cv2 = p2(2, 3);
bx2 = -p2(1, 4) / fu2;

T = bx2 - bx1; % Baseline
f = mean([fu1 fu2 fv1 fv2]);

intrinsics_l = cameraIntrinsics([fu1, fv1], [cu1 cv1], size(readimage(cam0, 1)));
intrinsics_r = cameraIntrinsics([fu2, fv2], [cu2 cv2], size(readimage(cam1, 1)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Prepare some state data                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

features = [];
pose = rigidtform3d(eye(3, 3), zeros(1, 3));
all_poses = [];
n_frames = length(cam0.Files);
landmarks = [];
keepvars = {'landmarks', 'n_frames','all_poses','keepvars','pose','features','intrinsics_l','intrinsics_r','p1','p2','cam0','cam1','view_3D','frame_times'};

for i = 1:n_frames

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %               Load images and detect features               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Read images
    lf = readimage(cam0, i);
    rf = readimage(cam1, i);

    % Undistort images
    lf_rect = undistortImage(lf, intrinsics_l);
    rf_rect = undistortImage(rf, intrinsics_r);

    % Find features
    f_l = detectSIFTFeatures(lf_rect);
    f_r = detectSIFTFeatures(rf_rect);

    % Extract descriptors
    [l_desc, l_pos] = extractFeatures(lf_rect, f_l, "Method", "SIFT");
    [r_desc, r_pos] = extractFeatures(rf_rect, f_r, "Method", "SIFT");

    % Match descriptors
    matched = matchFeatures(l_desc, r_desc);

    % If it's not the first iteration we want to match features
    if ~isempty(features)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                      Match the features                     %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Package the data together neatly
        current_features = struct( ...
            "l_desc", l_desc, ...
            "l_pos", l_pos, ...
            "r_desc", r_desc, ...
            "r_pos", r_pos ...
        );
        old_features = features;

        % Track the features that matches between frames
        [features_in_current_space, remaining_old_features, lm, rm] = ...
        find_remaining_points(old_features, current_features);

        % Triangulate 3D coordinates in both the old and the new coordinate frame
        features_in_current_space.pos = zeros([size(features_in_current_space.l_pos.Location, 1), 3]);
        remaining_old_features.pos = zeros([size(features_in_current_space.l_pos.Location, 1), 3]);

        for it = 1:size(features_in_current_space.l_pos.Location, 1)
            remaining_old_features.pos(it, :) = triangulate(remaining_old_features.l_pos(it).Location, remaining_old_features.r_pos(it).Location, p1, p2);
            features_in_current_space.pos(it, :) = triangulate(features_in_current_space.l_pos(it).Location, features_in_current_space.r_pos(it).Location, p1, p2);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                           Odometry                          %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Estimate current worldpose
        rel_pose = estworldpose( ...
            double(features_in_current_space.l_pos.Location), ...
            double(remaining_old_features.pos), ...
            intrinsics_l ...
        );

        % Convert relative coordinate system to the world coordinate system
        pose = rigidtform3d(pose.A * rel_pose.A);

        % Append the pose to the worldposes
        all_poses = [all_poses
                    pose];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                      Add new landmarks                      %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % If result is strange, limit the number of points kept with ptk
        l_pos = l_pos(matched(:, 1));
        r_pos = r_pos(matched(:, 2));
        l_desc = l_desc(matched(:, 1), :);
        r_desc = r_desc(matched(:, 2), :);
        if view_3D
            % Find the landmarks that did not exist in the previous frame
            indices = [];
            for index = 1:size(l_pos.Location, 1)
                % This is terribly slow, but it works
                if isempty(find(remaining_old_features.l_pos.Location == l_pos.Location(index, :),1)) ...
                        && isempty(find(remaining_old_features.r_pos.Location == r_pos.Location(index, :),1))
                    indices = [indices, index];
                end
            end
    
            % Get the locations of the new landmarks in the left and right images
            new_landmarks_l = l_pos.Location(indices, :);
            new_landmarks_r = r_pos.Location(indices, :);
            % Add the new landmarks to the landmarks array
            landmarks = CreateLandmarksFromFeatures(new_landmarks_l, new_landmarks_r, p1, p2, pose, landmarks);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                        Visualization                        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Only log things to the screen every n'th frame
        if mod(i, 100) == 0
            % Display results
            pretty_print(pose, rel_pose, frame_times, i, n_frames, true);
            mkdir img
            mkdir(sprintf("./img/%d",i));
            % Display the current frame
            figure(1);
            ShowFeaturesOnFeed(features_in_current_space, remaining_old_features, l_pos, lf);
            saveas(gcf,sprintf('./img/%d/view.png',i));
            % Show the poses
            figure(2);
            error = PlotOnMap(all_poses, i);
            saveas(gcf,sprintf('./img/%d/map.png',i));
            % Plot error over time
            figure(3);
            plot(frame_times(1:size(error,2)), error);
            saveas(gcf,sprintf('./img/%d/error.png',i));
            title('Error in xz-plane over time')
            xlabel('Time[s]')
            ylabel('Error[m]')

            if view_3D
                % Plot the landmarks
                figure(4);
                ShowPoseAndLandmarks(all_poses, landmarks, i - 1);
                saveas(gcf,sprintf('./img/%d/3d_map.png',i))
                title('Error in xz-plane over time')
                xlabel('X [m]')
                ylabel('Y [m]')
                zlabel('Z [m]')
            end
            close all

        else
            % Only show the frame number every frame
            pretty_print(pose, rel_pose, frame_times, i, n_frames, false);
        end
    else
        % If result is strange, limit the number of points kept with ptk
        l_pos = l_pos(matched(:, 1));
        r_pos = r_pos(matched(:, 2));
        l_desc = l_desc(matched(:, 1), :);
        r_desc = r_desc(matched(:, 2), :);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Keep the features that exist in both the left and right view%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %best = 1000;
    %ptk = min([best, length(matched)]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       Store the features detected until next iteration     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Store the features until the next iteration
    features = struct( ...
        "l_desc", l_desc, ... % Feature descriptors in left image
        "r_desc", r_desc, ... %       -      ||      - right image
        "l_pos", l_pos, ...   % xy-position of descriptors in left image
        "r_pos", r_pos ...    %       -      ||      -        right image
    );
    clearvars('-except', keepvars{:});
end

% Show the final result
figure(1);
ShowFeaturesOnFeed(features_in_current_space, remaining_old_features, l_pos, lf);

% Show the poses
figure(2);
error = PlotOnMap(all_poses, i);

% Plot error over time
figure(3)
plot(frame_times(1:i), error)

% Save the poses
save("poses.mat", "all_poses");

% Save the error
save("error.mat", "error");

% Save the landmarks
save("landmarks.mat", "landmarks");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         FUNCTIONS                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper functions to remove clutter from the main function

% Prints useful information to the command window
function pretty_print(pose, rel_pose, frame_times, i, feed_length, full)
    clc;
    fprintf("        frame %d/%d\n", i, feed_length)
    if full
        disp("---                       ---");
        fprintf("Estimated distance :%.2f\n", norm(rel_pose.T))
        disp("---                       ---");
        fprintf("Estimated velocity :%.2f\n", 3.6 * norm(rel_pose.T) ./ (frame_times(i) - frame_times(i - 1)))
        disp("---                       ---");
        disp("Translation")
        disp(pose.Translation);
        disp("---                       ---");
        disp("Rotation")
        disp(pose.R);
        disp("---                       ---");
    end
end

% Finds the features that exist in left and right camera in current and previous frame.
function [current_features, old_features, lm, rm] = find_remaining_points(old_features, current_features)

    % Match the features between frames in the left image
    feed_forward_left_frame_matches = matchFeatures(current_features.l_desc, old_features.l_desc);
    lm = feed_forward_left_frame_matches;

    % Keep only the features that were found between frames in the left image
    old_features.l_desc = old_features.l_desc(feed_forward_left_frame_matches(:, 2), :);
    old_features.l_pos = old_features.l_pos(feed_forward_left_frame_matches(:, 2), :);
    old_features.r_desc = old_features.r_desc(feed_forward_left_frame_matches(:, 2), :);
    old_features.r_pos = old_features.r_pos(feed_forward_left_frame_matches(:, 2), :);

    % Match the features between frames in the right image
    feed_forward_right_frame_matches = matchFeatures(current_features.r_desc, old_features.r_desc);
    rm = feed_forward_right_frame_matches;
    
    % Keep only the features that were found between frames in the right image
    old_features.l_desc = old_features.l_desc(feed_forward_right_frame_matches(:, 2), :);
    old_features.l_pos = old_features.l_pos(feed_forward_right_frame_matches(:, 2), :);
    old_features.r_desc = old_features.r_desc(feed_forward_right_frame_matches(:, 2), :);
    old_features.r_pos = old_features.r_pos(feed_forward_right_frame_matches(:, 2), :);

    % The matched points from previous frame have now been acquired

    % Keep only the features that exists in both frames
    current_features.l_desc = current_features.l_desc(feed_forward_left_frame_matches(:, 1), :);
    current_features.l_pos = current_features.l_pos(feed_forward_left_frame_matches(:, 1));
    current_features.r_desc = current_features.r_desc(feed_forward_right_frame_matches(:, 1), :);
    current_features.r_pos = current_features.r_pos(feed_forward_right_frame_matches(:, 1));

    % Match the features between the left and right image in current frame
    current_frame_matches = matchFeatures(current_features.l_desc, current_features.r_desc);

    % Keep only the features that were found in the current frame
    current_features.l_desc = current_features.l_desc(current_frame_matches(:, 1), :);
    current_features.l_pos = current_features.l_pos(current_frame_matches(:, 1));
    current_features.r_desc = current_features.r_desc(current_frame_matches(:, 2), :);
    current_features.r_pos = current_features.r_pos(current_frame_matches(:, 2));

    % The matched points from current frame have now been acquired
    % Features that are out of view from at least one image can now be removed

    % Match the features between frames in the left image
    last_match = matchFeatures(current_features.l_desc, old_features.l_desc);

    % Keep only the features that were found between frames in the left image
    old_features.l_desc = old_features.l_desc(last_match(:, 2), :);
    old_features.l_pos = old_features.l_pos(last_match(:, 2), :);
    old_features.r_desc = old_features.r_desc(last_match(:, 2), :);
    old_features.r_pos = old_features.r_pos(last_match(:, 2), :);
    current_features.l_desc = current_features.l_desc(last_match(:, 1), :);
    current_features.l_pos = current_features.l_pos(last_match(:, 1), :);
    current_features.r_desc = current_features.r_desc(last_match(:, 1), :);
    current_features.r_pos = current_features.r_pos(last_match(:, 1), :);
end