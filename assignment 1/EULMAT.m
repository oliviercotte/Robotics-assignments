function [ r ] = EULMAT( rpy )
%EULMAT Generates a 3 x 3 transformation matrix from a given set of Euler
% angles (ZYX order)
pitch = rpy(:,2);
yaw = rpy(:,3);
roll = rpy(:,1);
% r = rotx(roll) * roty(pitch) * rotz(yaw);

cy = cosd(roll);
sy = sind(roll);
cp = cosd(pitch);
sp = sind(pitch);
cr = cosd(yaw);
sr = sind(yaw);

r11 = cy*cp;
r12 = cy*sp*sr - sy*cr;
r13 = cy*sp*cr + sy*sr;
r21 = sy*cp;
r22 = sy*sp*sr + cy*cr;
r23 = sy*sp*cr - cy*sr;
r31 = -sp;
r32 = cp*sr;
r33 = cp*cr;

r = [r11 r12 r13 ; r21 r22 r23; r31 r32 r33];
end

