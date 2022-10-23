clear
clc
%close all
% Get the camera feeds
cam0 = imageDatastore("./kitti/00/image_0/*", "FileExtensions", ".png");
cam1 = imageDatastore("./kitti/00/image_1/*", "FileExtensions", ".png");

calib = readmatrix('./kitti/00/calib.txt');
frame_times = readmatrix("./kitti/00/times.txt");
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
clear fu1 fu2 fv1 fv2

% För varje frame:
%  1. Kolla efter features, SIFT
%  2. Generera descriptors, SIFT
%  3. Räkna ut djup för varje feature
%  4. Spara denna frames descriptors till nästa,
%  5. Räkna ut skillnad mellan matchande features ( i 3d )
%  6. Interpolera för att hitta förflyttningen
%  7. Addera till tidigare pose

features = [];
pose = []
figure;
hold on
scatter(0,0)

for i = 1:length(cam0.Files)
    lf = readimage(cam0, i);
    rf = readimage(cam1, i);

    % Get the features
    f_l = detectSIFTFeatures(lf);
    f_r = detectSIFTFeatures(rf);
    best = 1000;
    % Extract descriptors
    [l_desc, l_pos] = extractFeatures(lf, f_l);
    [r_desc, r_pos] = extractFeatures(rf, f_r);

    % If it's not the first itteration we want to match features
    if ~isempty(features)

        % This looks really messy and I will likely forget how it works
        % since it's spagetti

        % here's the idea

        % 1. Find all the matching features in the left images
        % 2. Find all the matching features in the right image
        % 3. Remove any feature not in these sets
        % 4. Find the position of all of the remaining points in both the
        % left and the right images at this timestep
        % 5. Use triangulate to find the 3d points for every feature
        % 6. Compute the difference between every feature

        % Match the descriptors
        inter_frame = matchFeatures(l_desc, r_desc);

        % Intra frame matches
        lm = matchFeatures(l_desc, features.l_desc); % Left intra frame match
        p = p(lm(:, 2));

        features.l_desc = features.l_desc(lm(:, 2), :);
        features.l_pos = features.l_pos(lm(:, 2), :);
        features.r_desc = features.r_desc(lm(:, 2), :);

        rm = matchFeatures(r_desc, features.r_desc); % Right intra frame match

        % Now we have the old points
        old_points = p(rm(:, 2));
        features.l_desc = features.l_desc(rm(:, 2), :);
        features.l_pos = features.l_pos(rm(:, 2), :);
        features.r_desc = features.r_desc(rm(:, 2), :);

        % Old points that still exist in atleast one perspective
        % but represented in the new images
        temp_l_desc = l_desc(lm(:, 1), :);
        temp_l_pos = l_pos(lm(:, 1));
        % Right side
        temp_r_desc = r_desc(rm(:, 1), :);
        temp_r_pos = r_pos(rm(:, 1));

        % Old pairs that exist
        temp_new_match = matchFeatures(temp_l_desc, temp_r_desc);

        % Get new valid points
        temp_l_desc = temp_l_desc(temp_new_match(:, 1), :); % Intermediate descriptor
        temp_r_desc = temp_r_desc(temp_new_match(:, 2), :); % Intermediate descriptor
        temp_l_pos = temp_l_pos(temp_new_match(:, 1)); % Intermediate pos_l
        temp_r_pos = temp_r_pos(temp_new_match(:, 2)); % Intermediate pos_r

        % Old pairs that exist in the left frame
        existant_points = matchFeatures(temp_l_desc, features.l_desc);
        features.pos = features.pos(existant_points(:, 2), :);
        p = [];
        % Compute 3d coord
        for itter = 1:size(temp_l_pos, 1)
            p = [p; triangulate(round(temp_l_pos(itter).Location), round(temp_r_pos(itter).Location), p1, p2)];
        end

        p = p(existant_points(:, 1), :);
        temp_l_desc = temp_l_desc(existant_points(:, 1), :); % Intermediate descriptor
        temp_r_desc = temp_r_desc(existant_points(:, 1), :); % Intermediate descriptor
        temp_l_pos = temp_l_pos(existant_points(:, 1)); % Intermediate pos_l
        temp_r_pos = temp_r_pos(existant_points(:, 1)); % Intermediate pos_r

        % Here we need to estimate the camera pose given the old and new 3d
        % The est worldpose minimizes the reprojection error and prodces an
        % image plane in the 3d space assuming the
        if isempty(pose)
            pose = estworldpose(temp_l_pos.Location, p, cameraIntrinsics(f, [cu1, cv1], size(lf)));
            old_translation = pose.Translation;
        else
            intrinsics_l = cameraIntrinsics(f, [cu1, cv1], size(lf));
            intrinsics_r = cameraIntrinsics(f, [cu2, cv2], size(rf));
            % Since we know that the points are in order, we can change the
            % origin of all of the points to be the first point in the
            % point array and recompute the distance to the camera in
            % relation to the first point, then in reality we should do
            % this for all points
            [relative_pose, rel_rot] = calc_rel_pose(temp_l_pos.Location, p, features.pos);
            %E = estimateEssentialMatrix(temp_l_pos.Location,temp_r_pos.Location,intrinsics_l,intrinsics_r);
            %translationFromEssentialMatrix(
            distance_moved = norm(relative_pose)
            velocity = distance_moved ./ (frame_times(i) - frame_times(i - 1)) .* 3.6
            % Something seems to be off about the rotation vector
            temp_old = old_translation;
            %rel_rot = eye(3,3)-rel_rot;

            % The rotation matrix scales the room too. it does not just
            % rotate, this should be solvable by scaling the matrix to be
            % det(A) == 1
            old_translation = double(old_translation + relative_pose')

            distance_after_rotation = norm(old_translation - temp_old)
            pose = rigidtform3d(pose.R*rel_rot, old_translation)
            scatter(old_translation(1),old_translation(2));
        end

        %disp(pose.Translation);

        % Last time we do this, letsgo
        temp_match = matchFeatures(temp_l_desc, features.l_desc);

        temp_l_pos = temp_l_pos(temp_match(:, 1)); % Intermediate pos_l
        temp_r_pos = temp_r_pos(temp_match(:, 1)); % Intermediate pos_r
        old_pos_l = features.l_pos(temp_match(:, 2), :);
        p = p(temp_new_match(temp_match(:, 1))); %
        old_points = old_points(temp_match(:, 1)); %

        distances = old_points - p;

        for d = 1:length(distances)
            distances(d) = norm(distances(d, :));
        end

        % Draw distances on the old image
        %%hold on
        %scatter(temp_l_pos.Location(:, 1), temp_l_pos.Location(:, 2));
        % Old x locations
        x = temp_l_pos.Location(:, 1);
        y = temp_l_pos.Location(:, 2);
        % Old y locations
        ox = old_pos_l.Location(:, 1);
        oy = old_pos_l.Location(:, 2);

        for index = 1:length(x)
            %plot([x(index), ox(index)], [y(index), oy(index)]);
        end

        for j = 1:length(distances)
            lable = sprintf("%f [m]", norm(distances(j)));
            %text(x(j),y(j),lable,"Color",'g');
        end

        %hold off

        % Find points that exist in both the old and the new right and left
        % images

        % Add new features
        for index = 1:length(inter_frame)

            if isempty(find(lm(:, 1) == inter_frame(index, 1))) && isempty(find(rm(:, 2) == inter_frame(index, 2)))
                lm = [lm; inter_frame(index, 1), 0];
                rm = [rm; inter_frame(index, 2), 0];
            end

        end

        % possibly add features
        l_desc = l_desc(lm(:, 1), :);
        l_pos = l_pos(lm(:, 1));
        r_desc = r_desc(rm(:, 1), :);
        r_pos = r_pos(rm(:, 1));

    end

    % Match the descriptors
    matched = matchFeatures(l_desc, r_desc);

    ptk = min([best, length(matched)]);

    l_pos = l_pos(matched(1:ptk, 1));
    r_pos = r_pos(matched(1:ptk, 2));
    l_desc = l_desc(matched(1:ptk, 1), :);
    r_desc = r_desc(matched(1:ptk, 2), :);

    %figure; imshow(lf);
    x = l_pos.Location(:, 1);
    y = l_pos.Location(:, 2);
    dist = zeros(1, length(l_pos));

    p = [];
    % Compute distance
    for itter = 1:length(dist)
        p = [p; triangulate(round(l_pos(itter).Location), round(r_pos(itter).Location), p1, p2)];
        dist(itter) = sqrt(sum(p(end).^2));
    end

    %hold on
    %scatter(x, y);

    for j = 1:length(dist)
        lable = sprintf("%f [m]", norm(dist(j)));
        %text(x(j),y(j),lable,"Color",'g');
    end

    %hold off

    features = struct( ...
        "pos", p, ... % 3d locations
        "l_desc", l_desc, ... % Feature descriptors in left image
        "r_desc", r_desc, ... % -||- right image
        "l_pos", l_pos, ...
        "matched", matched ...
    );

end

function [translation, rotation] = calc_rel_pose(points_img, ponts_world_current, points_world_old)

    % This is based on https://buq2.com/camera-position-estimation-from-known-3d-points/
    % Although I am not entierly sure how he solves the system,
    % does he use the

    % Computes the relative pose between two cameras given a set of
    % 3d points in the first camera and the corresponding 2d points
    % in the second camera

    % This is done by solving the following system of equations
    % [p1(1) p1(2) p1(3) 1 0 0 0 0 -p2(1)*p1(1) -p2(1)*p1(2) -p2(1)*p1(3) -p2(1)] [R11 R12 R13 T1] = 0
    % [0 0 0 0 p1(1) p1(2) p1(3) 1 -p2(2)*p1(1) -p2(2)*p1(2) -p2(2)*p1(3) -p2(2)] [R21 R22 R23 T2] = 0
    % [0 0 0 0 0 0 0 0 p1(1) p1(2) p1(3) 1 -p2(3)*p1(1) -p2(3)*p1(2) -p2(3)*p1(3) -p2(3)] [R31 R32 R33 T3] = 0

    A = zeros(3 * length(points_img), 12);

    for i = 1:size(points_img, 1)
        A(3 * i - 2, :) = [points_world_old(i, 1) points_world_old(i, 2) points_world_old(i, 3) 1 0 0 0 0 -points_img(i, 1) * points_world_old(i, 1) -points_img(i, 1) * points_world_old(i, 2) -points_img(i, 1) * points_world_old(i, 3) -points_img(i, 1)];
        A(3 * i - 1, :) = [0 0 0 0 points_world_old(i, 1) points_world_old(i, 2) points_world_old(i, 3) 1 -points_img(i, 2) * points_world_old(i, 1) -points_img(i, 2) * points_world_old(i, 2) -points_img(i, 2) * points_world_old(i, 3) -points_img(i, 2)];
    end

    % Solve the system of equations
    [~, ~, V] = svd(A);
    % Extract the last column
    P = V(:, end);

    % Reshape the result into the extrinsic matrix
    % The first 3 rows are the rotation matrix
    % The last row is the translation vector
    P = reshape(P, 4, 3)';

    % Decompose to rotation and translation
    [rotation, translation] = decomposeP(P);

    function [rotation, translation] = decomposeP(P)
        % Decompose the projection matrix into a rotation and
        % translation

        % Extract the rotation matrix
        rotation = P(:, 1:3);

        % Extract the translation vector
        translation = P(:, 4);

        % The rotation matrix needs to fullfill the following
        %def isRotationMatrix(M):
        %    tag = False
        %    I = np.identity(M.shape[0])
        %    if np.all((np.matmul(M, M.T)) == I) and (np.linalg.det(M)==1): tag = True
        %    return tag  
        
        function tag = isRotationMatrix(m)
            tag = false;
            I = eye(size(m, 1));
            if all(all(m * m' == I)) && det(m) == 1
                tag = true;
            end
        end

        % If the rotation matrix is not a rotation matrix, then
        % we need to correct it so the check %holds
        if ~isRotationMatrix(rotation)
            % Now we need to correct the rotation matrix
            if det(rotation) ~= 1
                % Adjust the determinant to be 1
                rotation = rotation./(sign(det(rotation))*abs(det(rotation)).^(1/3));
            end
            
            if all(all(rotation * rotation' ~= eye(size(rotation, 1))))
                % Adjust the rotation matrix to be orthogonal
                
                % First we need to make the rotation matrix orthonormal
                [U, ~, V] = svd(rotation);
                rotation = U * V';
            end
        end

        % Normalize the rotation matrix

        % Make sure that the rotation matrix is a rotation matrix
        % and not a reflection
        %if det(rotation) < 0
        %    rotation(:, 3) = -rotation(:, 3);
        %end

    end

    return
end

function [new_points, origin] = change_origin(index, points)
    origin = points(index);
    new_points = points - index; % Move all of the points relative to the new origin
end
