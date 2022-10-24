function [R,t] = EPnP(X, x)
err = 1e10;
R = eye(3);
t = zeros(3, 1);
P = [R t];

[row_X, col_X] = size(X);
[row_x, col_x] = size(x);
if row_X ~= 3 && row_X ~= 4
    fprintf('mat X must be [3 x n] or [4 x n]\n')
    return
end
if row_x ~= 2 && row_x ~= 3
    fprintf('mat x must be [2 x n] or [3 x n]\n')
    return
end
if col_X ~= col_x
    fprintf('col(x) no equal to col(X)\n')
    return
end
n = col_X;
if n <= 3
    fprintf('col(x)/col(X) must greather than 3\n')
    return
end

if row_X == 4
    X = X(1:3, :) ./ X(4, :);
end
if row_x == 3
    x = x(1:2, :) ./ x(3, :);
end

%% choose control point
C = zeros(3, 4);
C(:, 1) = sum(X, 2) ./ n;
A = X - C(:, 1);
[v, lambda] = eig(A * A');
for i = 1:3
    C(:, i + 1) = C(:, 1) + sqrt(lambda(i, i) / n) * v(:, i);
end

%% compute barycentric coordinates
% 4 x n matrix
% [X; 1] = [C; 1] * alpha
alpha = pinv([C; ones(1, 4)]) * [X; ones(1, n)];

%% compute V
fx = 1;
fy = 1;
cx = 0;
cy = 0;
M = zeros(2 * n, 12);
for i = 1:n
    M(2 * i - 1, :) = [alpha(1, i) * fx, 0, alpha(1, i) * (cx - x(1, i)) ...
                       alpha(2, i) * fx, 0, alpha(2, i) * (cx - x(1, i)) ...
                       alpha(3, i) * fx, 0, alpha(3, i) * (cx - x(1, i)) ...
                       alpha(4, i) * fx, 0, alpha(4, i) * (cx - x(1, i))];
    M(2 * i, :) = [0, alpha(1, i) * fy, alpha(1, i) * (cy - x(2, i)) ...
                   0, alpha(2, i) * fy, alpha(2, i) * (cy - x(2, i)) ...
                   0, alpha(3, i) * fy, alpha(3, i) * (cy - x(2, i)) ...
                   0, alpha(4, i) * fy, alpha(4, i) * (cy - x(2, i))];
end
[~, ~, V] = svd(M);

%% compute L_6x10 and rho
% betas10 = [b11 b12 b22 b13 b23 b33 b14 b24 b34 b44]
% L = [l12'; l13'; l14'; l23'; l24'; l34';]
v = zeros(12, 4);
v(:, 1) = V(:, 12);
v(:, 2) = V(:, 11);
v(:, 3) = V(:, 10);
v(:, 4) = V(: , 9);

s = cell(4, 6);
for i = 1:4
    vtmp1 = v(1:3, i);
    vtmp2 = v(4:6, i);
    vtmp3 = v(7:9, i);
    vtmp4 = v(10:12, i);
    s{i, 1} = vtmp1 - vtmp2;
    s{i, 2} = vtmp1 - vtmp3;
    s{i, 3} = vtmp1 - vtmp4;
    s{i, 4} = vtmp2 - vtmp3;
    s{i, 5} = vtmp2 - vtmp4;
    s{i, 6} = vtmp3 - vtmp4;
end

L = zeros(6, 10);
for i = 1:6
    L(i, :) = [    s{1, i}' * s{1, i} ...
               2 * s{1, i}' * s{2, i} ...
                   s{2, i}' * s{2, i} ...
               2 * s{1, i}' * s{3, i} ...
               2 * s{2, i}' * s{3, i} ...
                   s{3, i}' * s{3, i} ...
               2 * s{1, i}' * s{4, i} ...
               2 * s{2, i}' * s{4, i} ...
               2 * s{3, i}' * s{4, i} ...
                   s{4, i}' * s{4, i}];
end

rho = zeros(6 , 1);
rho(1) = sum((C(:, 1) - C(:, 2)).^2);
rho(2) = sum((C(:, 1) - C(:, 3)).^2);
rho(3) = sum((C(:, 1) - C(:, 4)).^2);
rho(4) = sum((C(:, 2) - C(:, 3)).^2);
rho(5) = sum((C(:, 2) - C(:, 4)).^2);
rho(6) = sum((C(:, 3) - C(:, 4)).^2);

%% compute beta (N = 1)
% find betas
betas = zeros(4, 1);
b1 = linsolve(L(:, 1), rho);
betas(1) = sqrt(abs(b1));

betas = gauss_newton(L, rho, betas);

Xc = compute_Xc(alpha, betas, v);
[R1, t1] = compute_Rt(X, Xc);
err1 = reproject_error(X, x, R1, t1);

if err1 < err
    err = err1;
    R = R1;
    t = t1;
end

%% compute beta (N = 2)
% betas10 = [b11 b12 b22 b13 b23 b33 b14 b24 b34 b44]
% b3      = [b11 b12 b22]
% find beta
betas = zeros(4, 1);
b3 = linsolve(L(:, 1:3), rho);
betas(1) = sqrt(abs(b3(1)));
betas(2) = sqrt(abs(b3(3))) * sign(b3(2)) * sign(b3(1));

betas = gauss_newton(L, rho, betas);

Xc = compute_Xc(alpha, betas, v);
[R2, t2] = compute_Rt(X, Xc);

err2 = reproject_error(X, x, R2, t2);
if err2 < err
    err = err2;
    R = R2;
    t = t2;
end

%% compute beta (N = 3)
% betas10 = [b11 b12 b22 b13 b23 b33 b14 b24 b34 b44]
% b6      = [b11 b12 b22 b13 b23 b33]
% find beta
betas = zeros(4, 1);
b6 = linsolve(L(:, 1:6), rho);
betas(1) = sqrt(abs(b6(1)));
betas(2) = sqrt(abs(b6(3))) * sign(b6(2)) * sign(b6(1));
betas(3) = sqrt(abs(b6(6))) * sign(b6(4)) * sign(b6(1));

betas = gauss_newton(L, rho, betas);

Xc = compute_Xc(alpha, betas, v);
[R3, t3] = compute_Rt(X, Xc);

err3 = reproject_error(X, x, R3, t3);
if err3 < err
    err = err3;
    R = R3;
    t = t3;
end

%% compute beta (N = 4)
% betas10 = [b11 b12 b22 b13 b23 b33 b14 b24 b34 b44]
% b4      = [b11 b12     b13         b14]
% find beta
betas = zeros(4, 1);
b4 = linsolve([L(:, 1), L(:, 2), L(:, 4), L(:, 7)], rho);
betas(1) = sqrt(abs(b4(1)));
betas(2) = b4(2) / betas(1);
betas(3) = b4(3) / betas(1);
betas(4) = b4(4) / betas(1);

betas = gauss_newton(L, rho, betas);

Xc = compute_Xc(alpha, betas, v);
[R4, t4] = compute_Rt(X, Xc);

err4 = reproject_error(X, x, R4, t4);
if err4 < err
    err = err4;
    R = R4;
    t = t4;
end

%% return
P = [R t];
P = P ./ P(3, 4);

[R, t] = decomposeProjectionMatrix(P)
end

function betas = gauss_newton(L, rho, betas)
n_iter = 5;
for i = 1:n_iter
    % 6 x 10
    % err 12, 13, 14, 23, 24, 34
    % betas 11 12 22 13 23 33 14 24 34 44
    J = zeros(6, 4);
    for j = 1:6
        J(j, 1) = 2 * L(j, 1) * betas(1) +     L(j, 2) * betas(2) +     L(j, 4) * betas(3) +     L(j, 7) * betas(4);
        J(j, 2) =     L(j, 2) * betas(1) + 2 * L(j, 3) * betas(2) +     L(j, 5) * betas(3) +     L(j, 8) * betas(4);
        J(j, 3) =     L(j, 4) * betas(1) +     L(j, 5) * betas(2) + 2 * L(j, 6) * betas(3) +     L(j, 9) * betas(4);
        J(j, 4) =     L(j, 7) * betas(1) +     L(j, 8) * betas(2) +     L(j, 9) * betas(3) + 2 * L(j, 10) * betas(4);
    end
    
    % 6 x 1
    % err 12, 13, 14, 23, 24, 34
    r = zeros(6, 1);
    for j = 1:6
        r(j) = rho(j) ...
             - L(j, 1) * betas(1) * betas(1) ...
             - L(j, 2) * betas(1) * betas(2) ...
             - L(j, 3) * betas(2) * betas(2) ...
             - L(j, 4) * betas(1) * betas(3) ...
             - L(j, 5) * betas(2) * betas(3) ...
             - L(j, 6) * betas(3) * betas(3) ...
             - L(j, 7) * betas(1) * betas(4) ...
             - L(j, 8) * betas(2) * betas(4) ...
             - L(j, 9) * betas(3) * betas(4) ...
             - L(j, 10) * betas(4) * betas(4);
    end 
    
    A = J' * J;
    b = J' * r;
    dbetas = linsolve(A, b);
    
    betas = betas + dbetas;
    
end
end
function [R, t] = decomposeProjectionMatrix(P)
        %% QR decomposition
        [K, R] = rqGivens(P(1:3, 1:3));
        
        %% ensure that the diagonal is positive
        if K(3, 3) < 0
            K = -K;
            R = -R;
        end
        if K(2, 2) < 0
            S = [1  0  0 
                 0 -1  0
                 0  0  1];
            K = K * S;
            R = S * R;
        end
        if K(1, 1) < 0
            S = [-1  0  0 
                  0  1  0
                  0  0  1];
            K = K * S;
            R = S * R;
        end
        
        %% ensure R determinant == 1
        t = linsolve(K, P(:, 4));
        
        if det(R) < 0
            R = -R;
            t = -t;
        end
        
        K = K ./ K(3, 3); 
    end
function Xc = compute_Xc(alpha, betas, v)
x = betas(1) * v(:, 1) + betas(2) * v(:, 2) + betas(3) * v(:, 3) + betas(4) * v(:, 4);
C = [x(1:3, 1), x(4:6, 1), x(7:9, 1), x(10:12, 1)];
Xc = C * alpha;
end

function [R, t] = compute_Rt(Xw, Xc)
mean_Xw = mean(Xw, 2);
mean_Xc = mean(Xc, 2);

new_Xw = Xw - mean_Xw;
new_Xc = Xc - mean_Xc;

W = zeros(3, 3);
for j = 1:size(Xw, 2)
    W = W + new_Xc(:, j) * new_Xw(:, j)'; 
end
    
[U, ~, V] = svd(W);
R = U * V';
t = mean_Xc - R * mean_Xw;
end

function err = reproject_error(X, x, R, t)
x2 = R * X + t;
for i = 1:size(X, 2)
    x2(:, i) = x2(:, i) ./ x2(3, i);
end
x2 = x2(1:2, :);
err = mean(sqrt(sum((x2 - x).^2, 1)));
end
function [ R, Q ] = rqGivens( A )
% rqGivens Calculates RQ decomposition of A = RQ (3x3)
%
% Syntax:
%   [R, Q] = rqGivens(A);
%
% Input:
%   A - 3-by-3 matrix of rank 3
%
% Output:
%   R - Upper triangular matrix (3-by-3)
%   Q - Orthogonal matrix (3-by-3)
%
% Description:
%   This function calculates the 3-dimensional RQ decomposition of A using 
%   Givens rotations (equal to Euler rotations) Gx, Gy Gz:
%
%   Gx = [  1   0   0;
%           0   c   -s;
%           0   s   c];
%     
%   Gy = [  c   0   s;
%           0   1   0;
%           -s  0   c];
%
%   Gz = [  c   -s  0;
%           s   c   0;
%           0   0   1];
%
%   Ax = A * Gx to set Ax(3,2) to zero.
%   Axy = Ax * Gy to set Axy(3,1) to zero.
%   R = Axyz = Axy * Gz to set Axyz(2,1) to zero.
%
%   R = A * Gx * Gy * Gz 
%   -> R * Gz' * Gy' * Gx' = A
%   -> Q = Gz' * Gy' * Gx'
%
%   See also: 
%   - https://en.wikipedia.org/wiki/Givens_rotation#Dimension_3
%   - Hartley, Zisserman - Multiple View Geometry in Computer Vision
%     http://www.amazon.com/dp/0521540518 (Appendix 4, A4.1.1, page 579)     
%
% Author: Lars Meinel
% Email: lars.meinel@etit.tu-chemnitz.de
% Date: July 2015; Last revision: 2015-07-10
%% 1st - Set element 32 to zero
if A(3,2) == 0
    Ax = A;
    Gx = eye(3);
else
    r32 = sqrt(A(3,3)*A(3,3) + A(3,2)*A(3,2));
    c32 = A(3,3) / r32;
    s32 = -A(3,2) / r32;
    G32 = [ 1    0    0;
            0    c32  -s32;
            0    s32  c32   ];
    Gx = G32;   
    Ax = A * Gx;
end
%% 2nd - Set element 31 to zero
if A(3,1) == 0
    Axy = Ax;
    Gy = eye(3);
else
    r31 = sqrt(Ax(3,3)*Ax(3,3) + Ax(3,1)*Ax(3,1));
    c31 = Ax(3,3) / r31;
    s31 = Ax(3,1) / r31;
    G31 = [ c31     0   s31;
            0       1   0;
            -s31    0   c31];
    Gy = G31;   
    Axy = Ax * Gy;
end
%% 3rd - Set element 21 to zero
if A(2,1) == 0
    Axyz = Axy;
    Gz = eye(3);
else
    r21 = sqrt(Axy(2,2)*Axy(2,2) + Axy(2,1)*Axy(2,1));
    c21 = Axy(2,2) / r21;
    s21 = -Axy(2,1) / r21;
    G21 =    [  c21     -s21    0;
                s21     c21     0;
                0       0       1];
    Gz = G21;
    Axyz = Axy * Gz;
end
R = Axyz;
Q = Gz' * Gy' * Gx';
end