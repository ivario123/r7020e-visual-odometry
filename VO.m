% Rigid body transformation (contains all motion)
% T(k,k-1) = [R(k,k-1) t(k,k-1); 0 1];
% R is rotation matrix, t is translation vector

% C is the pose with respect to the initial coordinate frame C0 (k=0)
% Cn can be found by concatenating all Tk

% Projection matrix (y = Px where image coordinate is y = (u, v, 1)^T and 
% world coordinate is x = (x, y, z, 1)^T if rectified, otherwise y = PRx
% where R is the rectifying rotation matrix.)
% See equation 4 in
% https://journals.sagepub.com/doi/epub/10.1177/0278364913491297
calib = readmatrix('kitti\00\calib.txt');
P1 = calib(1,2:13);
P1 = reshape(P1,4,3);
P1 = permute(P1, [2,1]); % Left projection matrix
P2 = calib(2,2:13);
P2 = reshape(P2,4,3);
P2 = permute(P2, [2,1]); % Right projection matrix

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



% Load images
left = iread('C:\Users\Administratör\Documents\MATLAB\Datorseende och Bildbehandling\Project6\kitti\00\image_0/*.png','double');
right = iread('C:\Users\Administratör\Documents\MATLAB\Datorseende och Bildbehandling\Project6\kitti\00\image_1/*.png','double');

% Rectify?

% [match1, match2] = match_sift()

% 3D-point Xk1 can be estimated from stereo data.
% The latter (pk) requires image correspondences across three views.
% p^i_k-1 is the reprojection of the 3-D point X^i_k1 into image I_k
% k = image index (column)
% i = point index (row)
% Find Tk that minimizes the image reprojection error:
% norm(p(i,k)-p(i,k-1))^2;

