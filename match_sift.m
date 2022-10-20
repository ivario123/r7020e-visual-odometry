function [match1, match2] = match_sift(I1,I2,strongest,EdgeThresh,ContrastThresh,NumLayersInOctave,Sigma)

I1 = im2gray(I1);
I2 = im2gray(I2);

if nargin == 2
    EdgeThresh = 10;
    ContrastThresh = 0.0133;
    NumLayersInOctave = 3;
    Sigma = 1.6;
    strongest = 'all';
elseif nargin == 3
    EdgeThresh = 10;
    ContrastThresh = 0.0133;
    NumLayersInOctave = 3;
    Sigma = 1.6;
end

points1 = detectSIFTFeatures(I1,EdgeThreshold=EdgeThresh,ContrastThreshold=ContrastThresh,NumLayersInOctave=NumLayersInOctave,Sigma=Sigma);
points2 = detectSIFTFeatures(I2,EdgeThreshold=EdgeThresh,ContrastThreshold=ContrastThresh,NumLayersInOctave=NumLayersInOctave,Sigma=Sigma);

% Extract features
if strcmp(strongest,'all')
    [features1,valid_points1] = extractFeatures(I1,points1);
    [features2,valid_points2] = extractFeatures(I2,points2);
else
    [features1,valid_points1] = extractFeatures(I1,points1.selectStrongest(strongest));
    [features2,valid_points2] = extractFeatures(I2,points2.selectStrongest(strongest));
end

% Left column has index in matched valid_leftpoints for left image
% Right column has index in matched valid_rightpoints for right image
indexPairs = matchFeatures(features1,features2); 

% Extract matchpoints
match1 = valid_points1(indexPairs(:,1),:);
match2 = valid_points2(indexPairs(:,2),:);

% Show matched features
% figure(10)
% showMatchedFeatures(I1,I2,match1,match2);

%%% Uncomment if not using live feed %%%

% figure(1)
% imshow(I1)
% hold on
% plot(valid_points1)
% 
% figure(2)
% imshow(I2)
% hold on
% plot(valid_points2)
% 
% hold off
%
% fprintf('Detected SIFT features in image 1: ');
% fprintf('%.u',valid_points1.Count);
% fprintf('\n');
% 
% fprintf('Detected SIFT features in image 2: ');
% fprintf('%.u',valid_points2.Count);
% fprintf('\n');
% 
% fprintf('Number of matched SIFT features between images: ');
% fprintf('%.u',match1.Count);
% fprintf('\n');

end