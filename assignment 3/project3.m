% % MATLAB exercise 4 for project 3
%
clc;clear all;

%% inverse kinematics of the 3R planar manipulator
tool_frame = [1 0 0 9; 0 1 0 0; 0 0 1 0; 0 0 0 1]
% tool_frame = [0.5 -0.866 0 7.5373; 0.866 0.6 0 3.9266; 0 0 1 0; 0 0 0 1]
% tool_frame = [0 1 0 -3; -1 0 0 2; 0 0 1 0; 0 0 0 1]
% tool_frame = [0.866 0.5 0 -3.1245; -0.5 0.866 0 9.1674; 0 0 1 0; 0 0 0 1]


l1=4; l2=3; l3=2;
wrist_frame_transform = [1 0 0 l3; 0 1 0 0; 0 0 1 0; 0 0 0 1];
wrist_frame = tool_frame/wrist_frame_transform
c_phi = wrist_frame(1,1);
s_phi = wrist_frame(2,1);
x = wrist_frame(1,4);
y = wrist_frame(2,4);
c2 = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);

% In order for a solution to exist, the right-hand side of (4.14) must have a value
% between —1 and 1. In the solution algorithm, this constraint would be checked at
% this time to find out whether a solution exists. Physically, if this constraint is not
% satisfied, then the goal point is too far away for the manipulator to reach
tol = 4*eps;
if (abs(c2)-1) < tol
    s2 = sqrt(1-c2^2);
    theta2 = [atan2(s2, c2) atan2(-s2, c2)]; % "elbow-up" or the "elbow-down" solution.
    
    k1 = l1 + l2*c2;
    k2 = l2*s2;
    theta1 = [(atan2(y,x) - atan2(k2,k1)) (atan2(y,x) - atan2(-k2,k1))];
    
    phi = atan2(s_phi, c_phi);
    theta3 = phi - theta1 - theta2;
    
    fprintf('Theta1 = %3.3f, \t%3.3f\n', rad2deg(theta1));
    fprintf('Theta2 = %3.3f, \t%3.3f\n', rad2deg(theta2));
    fprintf('Theta3 = %3.3f, \t%3.3f\n', rad2deg(theta3));
else
    fprintf('The goal is out of reach (outside the reachable workspace) - no solution exists\n');
    return;
end
