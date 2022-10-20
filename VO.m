clear
clc


% Get the camera feeds
cam0 = imageDatastore("./kitti/00/image_0/*","FileExtensions",".png");
cam1 = imageDatastore("./kitti/00/image_1/*","FileExtensions",".png");


calib = readmatrix('./kitti/00/calib.txt');
% Collect the projection matricies
p11 = calib(1,2:13);
p11 = reshape(p11,4,3);
p1 = permute(p11, [2,1]); % Left projection matrix
p22 = calib(2,2:13);
p22 = reshape(p22,4,3);
p2 = permute(p22, [2,1]); % Right projection matrix

% Projection parameters
fu1 = p1(1,1); % focal lengths
fv1 = p1(2,2);
cu1 = p1(1,3);
cv1 = p1(2,3);
bx1 = -p1(1,4)/fu1; % baseline

fu2 = p2(1,1);
fv2 = p2(2,2);
cu2 = p2(1,3);
cv2 = p2(2,3);
bx2 = -p2(1,4)/fu2;

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

    % If it's not the first itteration we want to match features
    if ~isempty(features)
        % Match the descriptors
        inter_frame = matchFeatures(l_desc,r_desc);

        % Intra frame matches
        lm = matchFeatures(l_desc,features.l_desc);   % Left intra frame match
        p = p(lm(:,2));

        features.l_desc = features.l_desc(lm(:,2),:);
        features.r_desc = features.r_desc(lm(:,2),:);
        
        rm = matchFeatures(r_desc,features.r_desc);   % Right intra frame match
        

        % Now we have the old points
        old_points = p(rm(:,2));
        features.l_desc = features.l_desc(rm(:,2),:);
        features.r_desc = features.r_desc(rm(:,2),:);




        % Old points that still exist in atleast one perspective
        % but represented in the new images
        temp_l_desc = l_desc(lm(:,1),:);
        temp_l_pos = l_pos(lm(:,1));
        % Right side
        temp_r_desc = r_desc(rm(:,1),:);
        temp_r_pos = r_pos(rm(:,1));

        


        % Old pairs that exist
        temp_new_match = matchFeatures(temp_l_desc,temp_r_desc);

        % Get new valid points
        temp_l_desc = features.l_desc(temp_new_match(:,1));     % Intermediate descriptor
        temp_r_desc = features.r_desc(temp_new_match(:,2));     % Intermediate descriptor
        temp_l_pos = temp_l_pos(temp_new_match(:,1));           % Intermediate pos_l
        temp_r_pos = temp_r_pos(temp_new_match(:,2));           % Intermediate pos_r
        p = [];
        % Compute 3d coord
        for itter = 1:size(temp_l_pos,1)  
            p  = [p;triangulate(round(temp_l_pos(itter).Location),round(temp_r_pos(itter).Location),p1,p2)];
        end

        
        


        


        % Find 3d coords ( old points ) 
        % Find 3d coords ( matching old points )
        % Find translation and rotation
        % Interpolate
        % Extract pose
        

        % Find points that exist in both the old and the new right and left
        % images
        

        
        % Add new features
        for index = 1:length(inter_frame)
            if isempty(find(lm(:,1) == inter_frame(index,1)))  && isempty(find(rm(:,2) == inter_frame(index,2)))
                lm = [lm;inter_frame(index,1),0];
                rm = [rm;inter_frame(index,2),0];
            end
        end

        % possibly add features
        l_desc = l_desc(lm(:,1),:);
        l_pos = l_pos(lm(:,1));
        r_desc = r_desc(rm(:,1),:);
        r_pos = r_pos(rm(:,1));





    end
    % Match the descriptors
    matched = matchFeatures(l_desc,r_desc);


    
    


    ptk = min([best,length(matched) ]);


    l_pos = l_pos(matched(1:ptk,1));
    r_pos = r_pos(matched(1:ptk,2));
    l_desc = l_desc(matched(1:ptk,1),:);
    r_desc = r_desc(matched(1:ptk,2),:);

    figure;imshow(lf);
    x = l_pos.Location(:,1);
    y = l_pos.Location(:,2);
    dist = zeros(1,length(l_pos));

    p = []
    % Compute distance
    for itter = 1:length(dist)  
        p  = [p;triangulate(round(l_pos(itter).Location),round(r_pos(itter).Location),p1,p2)];
        dist(itter) = sqrt(sum(p(end).^2));
    end
    hold on
    scatter(x,y);
    for j = 1:length(dist)
        lable = sprintf("%f [m]",norm(dist(j)));
        %text(x(j),y(j),lable,"Color",'g');
    end
    hold off



    features = struct( ...          
        "pos", p ,...               % 3d locations
        "l_desc",l_desc, ...        % Feature descriptors in left image
        "r_desc",r_desc, ...        % -||- right image
        "matched",matched...
        );


end



