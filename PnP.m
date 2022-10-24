function Pose = PnP(Points_3D, Points_2D,K,use_k)
    % PnP - Perspective from N Points solution
    % Returns the pose of the camera relative to the world frame
    % if the camera is the world frame, the poses translation will be zeros
    A = [];
    % Fill in the A matrix
    for index = 1:10:size(Points_3D, 1)

        % Create some variable names to make it readable
        X = Points_3D(index, 1);
        Y = Points_3D(index, 2);
        Z = Points_3D(index, 3);
        W = 1;              % Homogeneous coordinate, pulled to 1
        x = Points_2D(index, 1);
        y = Points_2D(index, 2);
        % Add 3 rows to the matrix A for each point
        % These are taken from https://buq2.com/camera-position-estimation-from-known-3d-points/
        A = [A
            0     0    0    0   -X   -Y   -Z   -W    X*y   Y*y   Z*y   W*y
            X     Y    Z    W    0    0    0    0   -X*x  -Y*x  -Z*x  -W*x
            %-X*y -Y*y -Z*y -W*y  X*x  Y*x  Z*x  W*x  0     0     0     0 
            0   0   0   0    0    0    0    0   0    0    0    0
        ];
    end
    % Single value decomposition of A to solve for the projection matrix
    [~ , ~, V] = svd(A);

    % The last column of V is the solution with the smallest eigenvalue
    P = V(:,end);
    P = reshape(P, [4,3])';
    

    [R,T] = decomposeProjectionMatrix(P,K,use_k);
    
    Pose = struct('R', R, 'T', T',"Distance",norm(T));
    
    function [R, T] = decomposeProjectionMatrix(P, K,use_k)
        if use_k
            M = K.K\P;
    
            % The first 3 columns of M are the rotation matrix
            R = M(:, 1:3);
            % The last column of M is the translation matrix
            T = M(:, 4);
        else
            R = P(:,1:3);
            T = P(:,4);
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
end

%         % Decompose the projection matrix into a rotation matrix and a
%         % translation vector.
%         T = P(:, 4);
%         R = P(:, 1:3);
%         % Ensure that R is not a reflection
%         if det(R) < 0
%             R = -R;
%         end
%         % Singular value decomposition to ensure that R is a rotation matrix
%         [U, ~, V] = svd(R);
%         R = U * V';
%         if det(R) < 0
%             R = -R;
%         end
%     end