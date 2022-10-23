function pose = PnP(f1,f2,p1,p2,intrinsics1,intrinsics2)
% Estimates the camera pose by minimizing the reprojection error


% Step 1 Value correctness

% Step 1.0 Ensure that we have the same number of points for each view
assert(size(f1,1) == size(f2,1));
assert(size(f1,1) == size(p1,1));
assert(size(p1,1) == size(p2,1));
% Step 1.1 Ensure that the cameras matricies are the same size
assert(all(size(intrinsics1) == size(intrinsics2)));

% To do this we solve the Perspective from N Points problem
% here we need atleast 6 points
% Step 2 Constructing NpN matricies
% Step 2.0 Validating sizes
assert(size(f1,1)>6);





    function F = eight_point_alg(f1,f2,p1,p2,intrinsics1,intrinsics2)
        % The normalized 8 point algorithm
        

    end
end