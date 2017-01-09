% MATLAB EXERCISE 2A

clear

% Absolute tolerance equality
isequalAbs = @(x,y) (abs(x-y) < eps('single'));

%% Using the Z—Y—X Euler angle convention, calculate the rotation matrix
%% R when the user enters the Euler angles.
% Test for two examples:
% i) Alpha = 10°, Beta = 20°, Gamma = 30°.
rpy1 = [10.0, 20.0, 30.0];
R1 = EULMAT(rpy1);
% ii) Alpha = 30°, Beta = 60°, Gamma = -55°.
rpy2 = [30.0, 60, -55.0];
R2 = EULMAT(rpy2);
%% Demonstrate the six constraints for unitary orthonormal rotation matrices
%% Each is a unit vector, and all three must be mutually perpendicular,
%% so we see that there are six constraints on the nine matrix elements.
R1_inv = inv(R1);
R1_t = R1';
assert(isequal(isequalAbs(R1_inv, R1_t), ones(3)), '(i) - Inverse/Transpose property not equal')

R2_inv = inv(R2);
R2_t = R2';
assert(isequal(isequalAbs(R2_inv, R2_t), ones(3)), '(ii) - Inverse/Transpose property not equal')

X1 = R1(:, 1);
Y1 = R1(:, 2);
Z1 = R1(:, 3);
assert(isequalAbs(norm(X1), 1.0))
assert(isequalAbs(norm(Y1), 1.0))
assert(isequalAbs(norm(Z1), 1.0))
assert(isequalAbs(dot(X1, Y1), 0.0))
assert(isequalAbs(dot(X1, Z1), 0.0))
assert(isequalAbs(dot(Y1, Z1), 0.0))

X2 = R2(:, 1);
Y2 = R2(:, 2);
Z2 = R2(:, 3);
assert(isequalAbs(norm(X2), 1.0))
assert(isequalAbs(norm(Y2), 1.0))
assert(isequalAbs(norm(Z2), 1.0))
assert(isequalAbs(dot(X2, Y2), 0.0))
assert(isequalAbs(dot(X2, Z2), 0.0))
assert(isequalAbs(dot(Y2, Z2), 0.0))

%% Calculate the Euler angles when the user
%% enters the rotation matrix R (the inverse problem).
%% Calculate both possible solutions.
[a1, b1] = MATEUL(R1);
[a2, b2] = MATEUL(R2);

%% Demonstrate this inverse solution for the two cases from part (a).
%% Use a circular check to verify your result
%% (two sets of answers—one should be the original user input, and the
%% second can be verified by once again using the code in part A.
%%  The first set is the original user input in (i) and (ii)
% For R1
if abs(R1(1,1)) < eps && abs(R1(2,1)) < eps
    % Although in this case, there are an infinite number of solutions to
    % the problem, in practice, one is often interested in finding one
    % solution. For this task, it is convenient to set roll = 0, pitch =
    % +/- 90.
    R1_comp = EULMAT(a1);
else
    assert(isequalAbs(rpy1(1), a1(1)))
    assert(isequalAbs(rpy1(2), a1(2)))
    assert(isequalAbs(rpy1(3), a1(3)))
    R1_comp = EULMAT(b1);
end

% For R2
if abs(R2(1,1)) < eps && abs(R2(2,1)) < eps
    R2_comp = EULMAT(a2);
else
    assert(isequalAbs(rpy2(1), a2(1)))
    assert(isequalAbs(rpy2(2), a2(2)))
    assert(isequalAbs(rpy2(3), a2(3)))
    R2_comp = EULMAT(b2);
end

% Check the results with the complementary set of angles
assert(isequal(isequalAbs(R1, R1_comp), ones(3)), 'The second Euler set produce different result (R1)')
assert(isequal(isequalAbs(R2, R2_comp), ones(3)), 'The second Euler set produce different result (R2)')

% For a simple rotation of about the Y axis only, for beta = 20 and
% B_P = [1 0 1]', calculate A_P
pitch = 20;
cy = cosd(pitch);
sy = sind(pitch);
R_Y = [cy 0 sy; 0 1 0; -sy 0 cy];
B_P = [1 0 1]';
A_P = roty(pitch, 'deg') * B_P;

%% Check all results
%% From MATLAB
R11 = rotz(rpy1(1), 'deg') * roty(rpy1(2), 'deg') * rotx(rpy1(3), 'deg');
assert(isequal(isequalAbs(R1, R11), ones(3)), '(i) - Rotations matrix are not equal')
R22 = rotz(rpy2(1), 'deg') * roty(rpy2(2), 'deg') * rotx(rpy2(3), 'deg');
assert(isequal(isequalAbs(R2, R22), ones(3)), '(ii) - Rotations matrix are not equal')
%% From Corke MATLAB Robotics Toolbox
% For R1
T1 = rpy2tr(rpy1(1), rpy1(2), rpy1(3), 'deg', 'zyx');
assert(isequal(isequalAbs(R1, T1(1:3, 1:3)), ones(3)), '(i) - Rotations matrix are not equal')
RPY1 = tr2rpy(R1, 'deg', 'zyx');
if abs(R1(1,1)) < eps && abs(R1(2,1)) < eps
else
    assert(isequalAbs(a1(1), RPY1(1)))
    assert(isequalAbs(a1(2), RPY1(2)))
    assert(isequalAbs(a1(3), RPY1(3)))
end
% For R2
T2 = rpy2tr(rpy2(1), rpy2(2), rpy2(3), 'deg', 'zyx');
assert(isequal(isequalAbs(R2, T2(1:3, 1:3)), ones(3)), '(i) - Rotations matrix are not equal')
RPY2 = tr2rpy(R2, 'deg', 'zyx');
if abs(R2(1,1)) < eps && abs(R2(2,1)) < eps
else
    assert(isequalAbs(a2(1), RPY2(1)))
    assert(isequalAbs(a2(2), RPY2(2)))
    assert(isequalAbs(a2(3), RPY2(3)))
end