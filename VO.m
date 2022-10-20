clear
clc

% Rigid body transformation (contains all motion)
% T(k,k-1) = [R(k,k-1) t(k,k-1); 0 1];
% R is rotation matrix, t is translation vector

% C is the pose with respect to the initial coordinate frame C0 (k=0)
% Cn can be found by concatenating all Tk

% Projection/camera matrix (y = Px where image coordinate is y = (u, v, 1)^T and 
% world coordinate is x = (x, y, z, 1)^T if rectified, otherwise y = PRx
% where R is the rectifying rotation matrix.)
% See equation 4 in
% https://journals.sagepub.com/doi/epub/10.1177/0278364913491297
calib = readmatrix('kitti\00\calib.txt');
P11 = calib(1,2:13);
P11 = reshape(P11,4,3);
P1 = permute(P11, [2,1]); % Left projection matrix
P22 = calib(2,2:13);
P22 = reshape(P22,4,3);
P2 = permute(P22, [2,1]); % Right projection matrix

% Projection parameters
fu1 = P1(1,1); % focal lengths
fv1 = P1(2,2);
cu1 = P1(1,3);
cv1 = P1(2,3);
bx1 = -P1(1,4)/fu1; % baseline

fu2 = P2(1,1);
fv2 = P2(2,2);
cu2 = P2(1,3);
cv2 = P2(2,3);
bx2 = -P2(1,4)/fu2;

f = mean([fu1 fu2 fv1 fv2]);
clear fu1 fu2 fv1 fv2

%%% Algorithm 3. VO from 3-D-to-2-D Correspondences %%%

% Specify folder with calibrations images
calibrationFolder1 = 'C:\Users\emanu\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_0';
calibrationFolder2 = 'C:\Users\emanu\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_1';
% Specify what type of files to look for
fileType1 = fullfile(calibrationFolder1, '*.png');
fileType2 = fullfile(calibrationFolder2, '*.png');
% Create a file array
theFiles1 = dir(fileType1);
theFiles2 = dir(fileType2);

% 1) Do only once:
% 1.1) Capture two frames Ik2, Ik1
left1 = iread('C:\Users\emanu\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_0\000000.png','double');
right1 = iread('C:\Users\emanu\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_1\000000.png','double');

% 1.2) Extract and match features between them
strongest = 300;
[match1, match2] = match_sift(left1,right1,strongest);

% 1.3) Triangulate features from Ik2, Ik1
location1 = match1.Location;
location2 = match2.Location;
points3D = triangulate(location1,location2,P11,P22);

% Show points on top of image
% imshow(left1)
% hold on
% plot(location1(:,1),location1(:,2), 'r+', 'MarkerSize', 30, 'LineWidth', 2);

numImages = 10;
% 2) Do at each iteration:
for i = 1:numImages

    % 2.1) Capture new frame Ik
    leftImageName = theFiles1(i).name;
    rightImageName = theFiles1(i).name;
    leftImages{k} = fullfile(theFiles1(i).folder, leftImageName{k});
    rightImages{k} = fullfile(theFiles2(i).folder, rightImageName{k});
end





% Load images
% left = iread('C:\Users\Administratör\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_0/*.png','double');
% right = iread('C:\Users\Administratör\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_1/*.png','double');
% left = iread('C:\Users\emanu\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_0/*.png','double');
% right = iread('C:\Users\emanu\Documents\MATLAB\r7020e-visual-odometry\kitti\00\image_1/*.png','double');

% Rectify?

% [match1, match2] = match_sift()

% 3D-point Xk1 can be estimated from stereo data.
% The latter (pk) requires image correspondences across three views.
% p^i_k-1 is the reprojection of the 3-D point X^i_k1 into image I_k
% k = image index (column)
% i = point index (row)
% Find Tk that minimizes the image reprojection error:
% norm(p(i,k)-p(i,k-1))^2;

