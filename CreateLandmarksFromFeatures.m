function landmarks = CreateLandmarksFromFeatures(features_l,features_r,intrinsics_l,intrinsics_r, pose,current_landmarks)
  % LandMarks: 3D landmarks from stereo images

  assert(size(features_l,1) == size(features_r,1));
  assert(size(features_l,2) == 2);
  assert(size(features_r,2) == 2);

  landmarks = zeros(size(features_l, 2),3);
  % Just keep every tenth landmark
  for i = 1:2:size(features_l, 1)
    % Triangulate the points

    coords = triangulate(features_l(i, :), features_r(i, :), intrinsics_l, intrinsics_r);
    % Only keep the points that are in front of the camera
    if coords(3) < 0
      continue
    end
    %  Only keep the points that are not too far away
    if coords(3) > 80
      continue
    end
    % Convert to world coordinates
    landmarks(i,:) = pose.transformPointsForward(coords);
  end
  % Add the current landmarks to the list
  landmarks = [current_landmarks; landmarks];
end