function [ rpy1, rpy2 ] = MATEUL( m )
%MATEUL Extracts the Euler angles from the given transformation matrix
% Find the two possibles Euler angles set from a rotation matrix if no
% singularity else return one of the case and set rp2 to [0 0 0] (undefined).
if  abs(m(1,1)) < eps && abs(m(2,1)) < eps
    rpy1(1) = 0; % can be anything, set it to zero
    rpy1(2) = atan2(-m(3,1),m(1,1));
    rpy1(3) = atan2(-m(2,3),m(2,2));
    rpy2 = [0 0 0];
else
    rpy1(2) = -asin(m(3,1));
    rpy2(2) = pi-rpy1(2);
    sp1 = cos(rpy1(2));
    sp2 = cos(rpy2(2));
    rpy1(3) = atan2(m(3,2)/sp1,m(3,3)/sp1);
    rpy2(3) = atan2(m(3,2)/sp2,m(3,3)/sp2);
    rpy1(1) = atan2(m(2,1)/sp1,m(1,1)/sp1);
    rpy2(1) = atan2(m(2,1)/sp2,m(1,1)/sp2);
end
rpy1 = rpy1 * 180/pi;
rpy2 = rpy2 * 180/pi;
end

