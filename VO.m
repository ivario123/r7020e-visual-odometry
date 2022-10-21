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
figure;
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
        % Match the descriptors for new frame
        inter_frame = matchFeatures(l_desc,r_desc);
        prev_match = matchFeatures(l_desc,features.l_desc);
        % Estimate the camera pose given the old and new 3D
        intrinsics = cameraIntrinsics(f,[cu1,cv1],size(lf));
        if isempty(pose)
            pose = estworldpose(l_pos(prev_match(:,1),:).Location,features.p(prev_match(:,2),:),intrinsics);
            relativePose = []; % Code crashed since it was uninit, just check if this is empty
        else 
            % Or is this the way to go?
            % Essential matrix
            matchedPoints1 = l_pos(inter_frame(:,1),:); % not sure if these are the right matchpoints
            matchedPoints2 = r_pos(inter_frame(:,2),:);
            cameraParams = intrinsics; % this argument should be supported as well
            [E,inliersIndex] = estimateEssentialMatrix(matchedPoints1,matchedPoints2,cameraParams); % Camera essential should not change between two timestamps

            relativePose = estrelpose(E,intrinsics,matchedPoints1,matchedPoints2);
            pose = rigidtform3d(pose.R*relativePose.R,relativePose.Translation+pose.Translation);
        end

        % Add the current view to the view set.
        viewId = i;
        currPoints = l_pos.Location; % These unnecessary lines can be inserted to the function and removed.
        vSet = addView(vSet, viewId, pose, "Points",l_pos,"Features",l_desc);

        % We can't make connections until the third itteration since the
        % second one does not yet have the point cloud information from the
        % first
        if ~isempty(relativePose)
            % Store the point matches between the previous and the current views. 
            % NOT SURE WHAT TO INSERT AS LAST ARGUMENT

            % Here we create an indexpair
            index_pair = matchFeatures(features.l_desc,features.r_desc);
            vSet = addConnection(vSet, viewId, viewId-1, relativePose,"Matches",round(l_pos(prev_match(:,1)).Location));
    
            % 2-D points in an image that match across multiple views
            % Each track contains 2-D projections of the same 3-D world point.
            tracks = findTracks(vSet);
            
            % Returns absolute poses associated with the views
            cameraPoses = poses(vSet);
            [xyzPoints,errors] = triangulateMultiview(tracks, cameraPoses, intrinsics);
            [xyzRefinedPoints,refinedPoses,reprojectionErrors] = bundleAdjustment(xyzPoints,tracks,cameraPoses,intrinsics);
    
            % Update viewset with refined poses
            vSet = updateView(vSet,refinedPoses);
            
        end
        disp("Current position : ")
        disp(vSet.Views(end,:).AbsolutePose.Translation);
        disp("---------------")
    end


    % Match the descriptors
    matched = matchFeatures(l_desc,r_desc);



    if i == 1
        % Image view set with data associated with each view
        vSet = imageviewset;

        % Add the first view to the view set.
        viewId = 1;
        %vSet = addView(vSet, viewId, rigidtform3d, points=matched);
        vSet = addView(vSet,1,"Features",l_desc(matched(:),:),"Points",l_pos(matched(:),:));
    end



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
        dist(it) = norm(p(end,:));
    end
    % Show left frame with matched features and their depths
    figure;
    if i > 2
        subplot(1,2,1);
    end
    imshow(lf);
    hold on
    x = l_pos.Location(:,1);
    y = l_pos.Location(:,2);
    scatter(x,y,'filled');
    for j = 1:length(dist)
        label = sprintf("%.2f",norm(dist(j)));
        text(x(j),y(j),label,"Color",'g');
    end
    hold off
    if i > 2
        subplot(1,2,2);
        z = p(:,3);
        %idx = errors < 5 & z > 0 & z < 20;
        pcshow(p(:, :),AxesVisibility="on",VerticalAxis="y",VerticalAxisDir="down",MarkerSize=30);
        hold on
        plotCamera(poses(vSet), Size=0.2);
        hold off
    end
    features = struct(...
        "l_desc",l_desc, ...        % Feature descriptors in left image
        "r_desc",r_desc, ...        % -||- right image
        "l_pos",l_pos,...           % DO WE NEED POS FOR RIGHT IMAGE?
        "r_pos",r_pos,...
        "p",p...
        );


end