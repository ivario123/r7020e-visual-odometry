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
proj_old = eye(4,4);
for i = 1:length(cam0.Files)
    lf = readimage(cam0, i);
    rf = readimage(cam1, i);
    % Get the features
    f_l = detectSIFTFeatures(lf);
    f_r = detectSIFTFeatures(rf);
    best = 1000;
    % I think that the intrinsics were in mm not in meters
    intrinsics_l = cameraIntrinsics([fu1,fv1], [cu1 cv1], size(lf));
    intrinsics_r = cameraIntrinsics([fu1,fv1], [cu1 cv1], size(rf));
    % Extract descriptors
    lf_rect = undistortImage(lf,intrinsics_l);
    rf_rect = undistortImage(rf,intrinsics_r);
    [l_desc, l_pos] = extractFeatures(lf_rect, f_l);
    [r_desc, r_pos] = extractFeatures(rf_rect, f_r);

    % If it's not the first itteration we want to match features
     if ~isempty(features)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                         MATCHING                           %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % First find the features that remain
        

        current_features = struct( ...
            "l_desc",l_desc,...
            "l_pos",l_pos,...
            "r_desc",r_desc,...
            "r_pos",r_pos...
            );
        old_features = features;
        [features_in_current_space,remaining_old_features,remaining_features] = find_remaining_points(old_features,current_features);
        features = remaining_old_features;
        features_in_current_space.l_desc = features_in_current_space.l_desc(remaining_features(:, 1), :); % Intermediate descriptor
        features_in_current_space.r_desc = features_in_current_space.r_desc(remaining_features(:, 1), :); % Intermediate descriptor
        features_in_current_space.l_pos = features_in_current_space.l_pos(remaining_features(:, 1)); % Intermediate pos_l
        features_in_current_space.r_pos = features_in_current_space.r_pos(remaining_features(:, 1)); % Intermediate pos_r
        % Then triangulate the new points
        p = zeros([size(features_in_current_space.l_pos.Location, 1), 3]);
        % Compute 3d coords
        for itter = 1:size(features_in_current_space.l_pos.Location, 1)
            features.pos(itter, :) = triangulate(remaining_old_features.l_pos(itter).Location,remaining_old_features.r_pos(itter).Location,p1,p2);
            p(itter,:) = triangulate(features_in_current_space.l_pos(itter).Location,features_in_current_space.r_pos(itter).Location, p1, p2);

        end

        % Only keep the last remaining points
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                           Odometry                          %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % I think that the intrinsics were in mm not in meters
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
        [relative_pose ,proj] = PnP(features.pos, features_in_current_space.l_pos.Location,intrinsics,false);
        est_pose= estworldpose(features_in_current_space.l_pos.Location,features.pos,intrinsics);
        %relative_pose = struct('R', est_pose.R, 'T', est_pose.Translation,"Distance",norm(est_pose.Translation));
        % If we have a good pose this will work
        pose = rigidtform3d(pose.A*est_pose.A );
        proj_old = proj_old*proj
        [R,T] = decomposeProjectionMatrix(proj_old,intrinsics,false)
        
        % This sollution is a bit more transparant and allows us to
        % understand what is going wrong
        % We might not need to rotate this. Not sure though
        % This should express the pose translation "flat" against the old
        % poses rotation. Poor explination.
        new_pose_t = old_pose.R*relative_pose.T';

        old_pose = struct( ...
            "T", new_pose_t'+old_pose.T,...%(new_pose_t_in_world_plane+old_pose.R'*old_pose.T')', ...
            "R",  relative_pose.R*old_pose.R, ...
            "Distance", relative_pose.Distance ...
            );
        all_poses = [all_poses
                pose.Translation
                % old_pose.T
                ];

        clc;
        fprintf("Distance traveled in that frame %.2f[m] \n", old_pose.Distance);
        fprintf("Velocity in that frame %.2f[km/h]\n", 3.6 * old_pose.Distance / (frame_times(i) - frame_times(i - 1)));
        disp("Current translation in world coords starting at first frame")
        disp(old_pose.T)
        disp("Current rotation in relation to the first frame")
        disp(old_pose.R)
        disp("Current rotation in relation to the 3d points")
        disp(relative_pose.R)
        fprintf("Estimated distance according to rtf %.2d",norm(est_pose.T))
        disp("Pose according to rigidtforms")
        disp(pose.Translation);
        disp("Rotation according to rigidtforms")
        disp(pose.R);

        
        figure(2);
        % Plot the X Z plane, the Y plane should be static
        %plot3(all_poses);
        hold on
        %scatter3(all_poses);
        hold off
        
        figure(1);
        imshow(lf_rect);
        figure(1);

        % Draw on the cars view
        hold on
        x = features_in_current_space.l_pos.Location(:,1);
        y = features_in_current_space.l_pos.Location(:,2);
        ox = features.l_pos.Location(:,1);
        oy = features.l_pos.Location(:,2);
        for i = 1:4:size(ox,1)
            plot([ox(i),x(i)],[oy(i),y(i)]);
            lable = sprintf("%.1f[m]", norm(p(i)));
            text(x(i),y(i),lable,"Color",'g');
            lable = sprintf("%.1f[m]", norm(features.pos(i)));
            text(ox(i),oy(i),lable,"Color",'r');
        end
        hold off
    


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %    I still think we need this part for improved stability   %
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
        p = [p; triangulate(l_pos(itter).Location, r_pos(itter).Location, p1, p2)];
        dist(itter) = sqrt(sum(p(end).^2));
    end

    


    % Store the features untill the next itteration
    features = struct( ...
        "pos", p, ...         % 3d locations
        "l_desc", l_desc, ... % Feature descriptors in left image
        "r_desc", r_desc, ... % -||- right image
        "l_pos", l_pos, ...
        "r_pos",r_pos,...
        "matched", matched ...
    );

end
function [R, T] = decomposeProjectionMatrix(P, K,use_k)
            if use_k
                M = K.K\P(1:3,1:3);
        
                % The first 3 columns of M are the rotation matrix
                R = M(:, 1:3);
                % The last column of M is the translation matrix
                T = linsolve(K.K,P(1:3,4))
            else
                R = P(1:3,1:3);
                T = P(1:3,4);
            end
            if det(R) < 0
                R = -R;
                %T = -T;
            end
            %if det(R) ~= 1
            %    R = R./((det(R)).^(1/3));
            %end
            if R*R' ~= eye(3,3)
                [U, ~, V] = svd(R); 
                R = U * V';
            end
            disp(R)
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         FUNCTIONS                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper functions to remove clutter from the main function

function [current_features,old_features,last_match] = find_remaining_points( old_features, current_features)

    

    % Match the descriptors over time in the left image
    feed_formward_left_frame_matches = matchFeatures(current_features.l_desc, old_features.l_desc);
    %p = p(lm(:, 2));

    old_features.l_desc = old_features.l_desc(feed_formward_left_frame_matches(:, 2), :);
    old_features.l_pos = old_features.l_pos(feed_formward_left_frame_matches(:, 2), :);
    old_features.r_pos = old_features.r_pos(feed_formward_left_frame_matches(:, 2), :);
    old_features.r_desc = old_features.r_desc(feed_formward_left_frame_matches(:, 2), :);

    % Match the descriptors over time in the right image
    feed_formward_right_frame_matches = matchFeatures(current_features.r_desc, old_features.r_desc);

    % Now we have the old points
    %old_points = p(rm(:, 2));
    old_features.l_desc = old_features.l_desc(feed_formward_right_frame_matches(:, 2), :);
    old_features.l_pos = old_features.l_pos(feed_formward_right_frame_matches(:, 2), :);
    old_features.r_pos = old_features.r_pos(feed_formward_right_frame_matches(:, 2), :);
    old_features.r_desc = old_features.r_desc(feed_formward_right_frame_matches(:, 2), :);

    current_features.l_desc = current_features.l_desc(feed_formward_left_frame_matches(:, 1), :);
    current_features.l_pos =  current_features.l_pos (feed_formward_left_frame_matches(:, 1));

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
    old_features.pos = old_features.pos(last_match(:, 2), :);
    old_features.l_desc = old_features.l_desc(last_match(:, 2), :);
    old_features.l_pos = old_features.l_pos(last_match(:, 2), :);
    old_features.r_pos = old_features.r_pos(last_match(:, 2), :);

end
