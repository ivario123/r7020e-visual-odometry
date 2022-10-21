close all
clear
clc


% Get the camera feeds
cam0 = imageDatastore("./kitti/00/image_0/*","FileExtensions",".png");
cam1 = imageDatastore("./kitti/00/image_1/*","FileExtensions",".png");


calib = readmatrix('./kitti/00/calib.txt');
% Collect the projection matrices
P11 = calib(1,2:13);
P11 = reshape(P11,4,3);
P1 = permute(P11, [2,1]); % Left projection matrix
P22 = calib(2,2:13);
P22 = reshape(P22,4,3);
P2 = permute(P22, [2,1]); % Right projection matrix

% Projection parameters
fu1 = P1(1,1); % focal lengths (pixels)
fv1 = P1(2,2);
cu1 = P1(1,3); % center of image (pixels)
cv1 = P1(2,3);
bx1 = -P1(1,4)/fu1;

fu2 = P2(1,1);
fv2 = P2(2,2);
cu2 = P2(1,3);
cv2 = P2(2,3);
bx2 = -P2(1,4)/fu2;

T = bx2 - bx1; % baseline
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
pose = [];
for i = 1:length(cam0.Files)
    lf = readimage(cam0,i);
    rf = readimage(cam1,i);
    
    
    % Get the features
    f_l = detectSIFTFeatures(lf);
    f_r = detectSIFTFeatures(rf);
    best = 1000;
    % Extract descriptors
    [l_desc,l_pos] = extractFeatures(lf,f_l);
    [r_desc,r_pos] = extractFeatures(rf,f_r);

    % If it's not the first iteration we want to match features
    if ~isempty(features)
        
        % This looks really messy and I will likely forget how it works
        % since it's spaghetti

        % here's the idea

        % 1. Find all the matching features in the left images 
        % 2. Find all the matching features in the right image 
        % 3. Remove any feature not in these sets (match)
        % 4. Find the position of all of the remaining points in both the 
        %       left and the right images at this timestep 
        % 5. Use triangulate to find the 3d points for every feature 
        % 6. Compute the difference between every feature NEEDED?


        % DOES INTER MEAN BETWEEN CAMERAS AND INTRA BETWEEN FRAMES AND SAME CAMERA?

        % Match the descriptors for new frame
        inter_frame = matchFeatures(l_desc,r_desc);

        % Left intra frame matches (match between previous and current frame)
        lm = matchFeatures(l_desc,features.l_desc);
        p = p(lm(:,2));

        % Remove features that aren't in both frames
        features.l_desc = features.l_desc(lm(:,2),:);
        features.l_pos = features.l_pos(lm(:,2),:);
        features.r_desc = features.r_desc(lm(:,2),:);
        
        % Right intra frame matches
        rm = matchFeatures(r_desc,features.r_desc);
        
        % Remove features that aren't in both frames
        features.l_desc = features.l_desc(rm(:,2),:);
        features.l_pos = features.l_pos(rm(:,2),:);
        features.r_desc = features.r_desc(rm(:,2),:);

        % Now we have the old points
        old_points = p(rm(:,2));


        % Old points that still exist in at least one perspective
        % but represented in the new images     REPLACE "temp" with "old"?
        % Left side
        temp_l_desc = l_desc(lm(:,1),:);
        temp_l_pos = l_pos(lm(:,1));
        % Right side
        temp_r_desc = r_desc(rm(:,1),:);
        temp_r_pos = r_pos(rm(:,1));
        

        % Old pairs that exist (IN CURRENT FRAME?)
        temp_new_match = matchFeatures(temp_l_desc,temp_r_desc);

        % Get new valid points (remove those that weren't found in both
        % frames)
        temp_l_desc = temp_l_desc(temp_new_match(:,1),:);     % Intermediate descriptor
        temp_r_desc = temp_r_desc(temp_new_match(:,2),:);     % Intermediate descriptor
        temp_l_pos = temp_l_pos(temp_new_match(:,1));         % Intermediate pos_l
        temp_r_pos = temp_r_pos(temp_new_match(:,2));         % Intermediate pos_r

        p = [];
        % Compute 3D coordinates for current frame features (relative to camera?) 
        for it = 1:size(temp_l_pos,1)
            p  = [p;triangulate(temp_l_pos(it).Location,temp_r_pos(it).Location,P11,P22)];
%             p  = [p;triangulate(round(temp_l_pos(it).Location),round(temp_r_pos(it).Location),P1,P2)];
        end

        


        % Estimate the camera pose given the old and new 3D
        intrinsics = cameraIntrinsics(f,[cu1,cv1],size(lf));
        if isempty(pose)
            worldPoints = p;
            pose = estworldpose(temp_l_pos.Location,p,intrinsics);
        else
%           True location = camera location + orientation*(points relative to camera)
            worldPoints = pose.translation + pose.R*p;
%           C(n) = C(n-1)*T(n)
            pose = rigidtform3d(pose.A*estworldpose(temp_l_pos.Location,worldPoints,intrinsics).A);
        end
        disp(pose.Translation);

        % 2-D points in an image that match across multiple views
%         viewIDs = 
%         points = 
        pointTracks = pointTrack(viewIDs,points);
        % Bundle adjustment (a more accurate estimate of local trajectory)
        % [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,intrinsics)
        [xyzRefinedPoints,refinedPoses] = bundleAdjustment(p+pose.Translation,pointTracks,T,intrinsics);



        
        % Match new left with old left
        temp_match = matchFeatures(temp_l_desc,features.l_desc);

        % remove those that weren't found in both frames
        temp_l_pos = temp_l_pos(temp_match(:,1));           % Intermediate pos_l
        temp_r_pos = temp_r_pos(temp_match(:,1));           % Intermediate pos_r
        old_pos_l = features.l_pos(temp_match(:,2),:);
        p = p(temp_new_match(temp_match(:,1)));             % What is this?
        old_points = old_points(temp_match(:,1));           %
        
        distances = old_points-p;
        for d = 1:length(distances)
            distances(d) = norm(distances(d,:));
        end

% Not needed I think
%         % Draw distances on the old image
%         hold on
%         scatter(temp_l_pos.Location(:,1),temp_l_pos.Location(:,2));
%         % Old x locations
%         x = temp_l_pos.Location(:,1);
%         y = temp_l_pos.Location(:,2);
%         % Old y locations
%         ox = old_pos_l.Location(:,1);
%         oy = old_pos_l.Location(:,2);
% 
%         for index = 1:length(x)
%             plot([x(index),ox(index)],[y(index),oy(index)]);
%         end
%         for j = 1:length(distances)
%             label = sprintf("%f [m]",norm(distances(j)));
%             text(x(j),y(j),lable,"Color",'g');
%         end
%         hold off

        

        
        % Find points that exist in both the old and the new right and left
        % images
        

        % Add new features
        for index = 1:length(inter_frame)
            if isempty(find(lm(:,1) == inter_frame(index,1)))  && isempty(find(rm(:,2) == inter_frame(index,2)))
                lm = [lm;inter_frame(index,1),0];
                rm = [rm;inter_frame(index,2),0];
            end
        end

        % Possibly add new features
        l_desc = l_desc(lm(:,1),:);
        l_pos = l_pos(lm(:,1));
        r_desc = r_desc(rm(:,1),:);
        r_pos = r_pos(rm(:,1));

    end

    % Match the descriptors
    matched = matchFeatures(l_desc,r_desc);

    % Number of matches
    ptk = min([best,length(matched)]);

    % Remove points that did not match
    l_pos = l_pos(matched(1:ptk,1));
    r_pos = r_pos(matched(1:ptk,2));
    l_desc = l_desc(matched(1:ptk,1),:);
    r_desc = r_desc(matched(1:ptk,2),:);

    dist = zeros(1,length(l_pos));
    p = [];
    % Compute distance
    for it = 1:length(dist)
        p  = [p;triangulate(l_pos(it).Location,r_pos(it).Location,P11,P22)];
%         [p;triangulate(round(l_pos(it).Location),round(r_pos(it).Location),P11,P22)];
        dist(it) = norm(p(end,:));
%         dist(it) = sqrt(sum(p(end).^2)); % remove commented lines
    end

    % Show left frame with matched features and their depths
    figure;imshow(lf);
    hold on
    x = l_pos.Location(:,1);
    y = l_pos.Location(:,2);
    scatter(x,y,'filled');
    for j = 1:length(dist)
        label = sprintf("%.2f",norm(dist(j)));
        text(x(j),y(j),label,"Color",'g');
    end
    hold off


    features = struct( ...          
        "pos", p ,...               % 3d locations
        "l_desc",l_desc, ...        % Feature descriptors in left image
        "r_desc",r_desc, ...        % -||- right image
        "l_pos",l_pos,...           % DO WE NEED POS FOR RIGHT IMAGE?
        "matched",matched...
        );


end