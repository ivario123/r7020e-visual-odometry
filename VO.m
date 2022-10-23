% Coolcoolcool
% First of some late night notes, read this https://buq2.com/camera-position-estimation-from-known-3d-points/
% 1. The Projection matrix seems to inch close to correct ness but I that
% it still needs some work
% 2. The way I extract the rotation and translation data seems to be
% correct
% 3. We need to figure out how to recombine the poses, The old pose is
% expressed in world coordinates and the new pose is expressed in the
% previous poses world coordinates so the only thing we should have to
% rotate is the added vector if even that


% There are a few things that we need to do.
% 1. Look at the A matrix composition and the rest of PnP.m and validate
% that that is correct
% 2. Look in to how we reconstruct the wold coord pose from the previous
% camera coord pose
% 3. Hope that we fix it quickly
% 4. Merge this into main
% 5. Make a .m file that given a set of rigidtform3d's, a number of feature matches
% and an image visualizes the path of the camera and the ground truth in an
% a single or 2 seperate plots
% 6. Clean up the code, manage variable names and function names and just
% make it look good.
% 7. Write a report.

clear
clc
close all
% Get the camera feeds
cam0 = imageDatastore("./kitti/00/image_0/*", "FileExtensions", ".png");
cam1 = imageDatastore("./kitti/00/image_1/*", "FileExtensions", ".png");

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

% Get frame times
frame_times = readmatrix("./kitti/00/times.txt");

features = [];

hold on
scatter(0, 0)
old_translation = zeros(1, 3);
old_rot = eye(3, 3);
pose = rigidtform3d(old_rot, old_translation);
old_pose = []
all_poses = []

for i = 1:length(cam0.Files)
    lf = readimage(cam0, i);
    rf = readimage(cam1, i);
    figure(1);
    imshow(lf);
    % Get the features
    f_l = detectSIFTFeatures(lf);
    f_r = detectSIFTFeatures(rf);
    best = 1000;
    % Extract descriptors
    [l_desc, l_pos] = extractFeatures(lf, f_l);
    [r_desc, r_pos] = extractFeatures(rf, f_r);

    % If it's not the first itteration we want to match features
    if ~isempty(features)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                         MATCHING                           %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % First find the features that remain
        [remaining_features, remaining_left_frame_descriptors, left_frame_possitions, right_frame_descriptors, right_frame_possitions, last_match] = find_remaining_points(features, l_desc, l_pos, r_desc, r_pos);
        features = remaining_features;
        % Then triangulate the new points
        p = zeros([size(left_frame_possitions, 1), 3]);
        % Compute 3d coords
        for itter = 1:size(left_frame_possitions, 1)
            p(itter, :) = triangulate(round(left_frame_possitions(itter).Location), round(right_frame_possitions(itter).Location), p1, p2);
        end

        % Only keep the last remaining points
        p = p(last_match(:, 1), :);
        remaining_left_frame_descriptors = remaining_left_frame_descriptors(last_match(:, 1), :); % Intermediate descriptor
        right_frame_descriptors = right_frame_descriptors(last_match(:, 1), :); % Intermediate descriptor
        left_frame_possitions = left_frame_possitions(last_match(:, 1)); % Intermediate pos_l
        right_frame_possitions = right_frame_possitions(last_match(:, 1)); % Intermediate pos_r

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                           Odometry                          %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        intrinsics = cameraIntrinsics([fu1,fv1], [cu1 cv1], size(lf));

        % Find the transformation between the two frames

        % Formatting data for easier passing
        if isempty(old_pose)
            old_pose = struct("T", pose.Translation, "R", pose.R);
        end

        % If we have the 3d points in relation to the old pose
        % we can find the new pose in the old pose's coordinate system
        % and then transform it to the world coordinate system

        % Solve PnP and compute the old pose
        relative_pose = PnP(features.pos, left_frame_possitions.Location,intrinsics);

        % If we have a good pose this will work
        % pose = rigidtform3d(pose.A*rigidtform3d(relative_pose.R, relative_pose.T).A);

        % This sollution is a bit more transparant and allows us to
        % understand what is going wrong
        % We might not need to rotate this. Not sure though
        old_pose = struct("T", relative_pose.T*relative_pose.R  + old_pose.T, "R", relative_pose.R * old_pose.R, "Distance", relative_pose.Distance);
        all_poses = [all_poses
                old_pose.T
                ];

        fprintf("Distance traveled in that frame %.2f[m] \n", old_pose.Distance);
        fprintf("Velocity in that frame %.2f[km/h]\n", 3.6 * old_pose.Distance / (frame_times(i) - frame_times(i - 1)));
        disp("Current translation in world coords starting at first frame")
        disp(old_pose.T)
        disp("Current rotation in relation to the first frame")
        disp(old_pose.R)
        figure(2);
        % Plot the X Z plane, the Y plane should be static
        scatter(all_poses(:, 1), all_poses(:, 2));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     I still thik we need this part for improved stability   %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Find points that exist in both the old and the new right and left
        % images

        % Add new features
        %for index = 1:length(inter_frame)
        %    if isempty(find(lm(:, 1) == inter_frame(index, 1))) && isempty(find(rm(:, 2) == inter_frame(index, 2)))
        %        lm = [lm; inter_frame(index, 1), 0];
        %        rm = [rm; inter_frame(index, 2), 0];
        %    end
        %end

        % possibly add features
        %l_desc = l_desc(lm(:, 1), :);
        %l_pos = l_pos(lm(:, 1));
        %r_desc = r_desc(rm(:, 1), :);
        %r_pos = r_pos(rm(:, 1));

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                Basic boiler plate for the problem           %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Match the descriptors
    matched = matchFeatures(l_desc, r_desc);

    ptk = min([best, length(matched)]);

    l_pos = l_pos(matched(1:ptk, 1));
    r_pos = r_pos(matched(1:ptk, 2));
    l_desc = l_desc(matched(1:ptk, 1), :);
    r_desc = r_desc(matched(1:ptk, 2), :);

    x = l_pos.Location(:, 1);
    y = l_pos.Location(:, 2);
    dist = zeros(1, length(l_pos));

    p = [];
    % Triangulate the 3D points
    for itter = 1:length(dist)
        p = [p; triangulate(round(l_pos(itter).Location), round(r_pos(itter).Location), p1, p2)];
        %dist(itter) = sqrt(sum(p(end).^2));
    end


    %for j = 1:length(dist)
    %    lable = sprintf("%f [m]", norm(dist(j)));
    %    %text(x(j),y(j),lable,"Color",'g');
    %end


    % Store the features untill the next itteration
    features = struct( ...
    "pos", p, ... % 3d locations
        "l_desc", l_desc, ... % Feature descriptors in left image
        "r_desc", r_desc, ... % -||- right image
        "l_pos", l_pos, ...
        "matched", matched ...
    );

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         FUNCTIONS                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper functions to remove clutter from the main function

function [features, left_frame_descriptors, left_frame_possitions, right_frame_descriptors, right_frame_possitions, last_match] = find_remaining_points( ...
    old_features, left_frame_descriptors, left_frame_possitions, right_frame_descriptors, right_frame_possitions)

    % Rename variables for easier reading
    features = old_features;

    % Match the descriptors over time in the left image
    feed_formward_left_frame_matches = matchFeatures(left_frame_descriptors, features.l_desc); % Left intra frame match
    %p = p(lm(:, 2));

    features.l_desc = features.l_desc(feed_formward_left_frame_matches(:, 2), :);
    features.l_pos = features.l_pos(feed_formward_left_frame_matches(:, 2), :);
    features.r_desc = features.r_desc(feed_formward_left_frame_matches(:, 2), :);

    % Match the descriptors over time in the right image
    feed_formward_right_frame_matches = matchFeatures(right_frame_descriptors, features.r_desc); % Right intra frame match

    % Now we have the old points
    %old_points = p(rm(:, 2));
    features.l_desc = features.l_desc(feed_formward_right_frame_matches(:, 2), :);
    features.l_pos = features.l_pos(feed_formward_right_frame_matches(:, 2), :);
    features.r_desc = features.r_desc(feed_formward_right_frame_matches(:, 2), :);

    left_frame_descriptors = left_frame_descriptors(feed_formward_left_frame_matches(:, 1), :);
    left_frame_possitions = left_frame_possitions(feed_formward_left_frame_matches(:, 1));

    right_frame_descriptors = right_frame_descriptors(feed_formward_right_frame_matches(:, 1), :);
    right_frame_possitions = right_frame_possitions(feed_formward_right_frame_matches(:, 1));

    % Match the descriptors between the left and right image
    current_frame_matches = matchFeatures(left_frame_descriptors, right_frame_descriptors);

    % Get new valid points
    left_frame_descriptors = left_frame_descriptors(current_frame_matches(:, 1), :);
    right_frame_descriptors = right_frame_descriptors(current_frame_matches(:, 2), :);
    left_frame_possitions = left_frame_possitions(current_frame_matches(:, 1));
    right_frame_possitions = right_frame_possitions(current_frame_matches(:, 2));

    % Now that we know what features still exist between the two cameras we
    % can remove the ones that are out of view from atleast one of the images
    last_match = matchFeatures(left_frame_descriptors, features.l_desc);
    features.pos = features.pos(last_match(:, 2), :);
    features.l_desc = features.l_desc(last_match(:, 2), :);
    features.l_pos = features.l_pos(last_match(:, 2), :);

end
