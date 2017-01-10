% linktrans    returns the homogeneous transformation for a particular link
%
%       T=linktrans(a,d,alpha,theta) gives the Homogeneous transformation
%       T corresponding to DH parameters alpha, a,d, theta
%
%
%       alpha is the skew angle
%       a is the distance between the previous Z and current z coordinate
%       d is the distance between thee previous X and current x coordinate
%       theta is the angle between the x coordinates
%
%       Olivier Cotte
%       November 7th, 2016
function t=linktrans(a,b,c,d)

syms alpha an theta dn
alpha = a; % dhparam(1);
an = b; % dhparam(2);
dn = c ;% dhparam(3);
theta = d; % dhparam(4);
sa = sin(alpha); ca = cos(alpha);
st = sin(theta); ct = cos(theta) ;

t = [	ct	-st	0	an; ...
    st*ca	ct*ca	-sa	-sa*dn; ...
    st*sa	ct*sa	ca	ca*dn; ...
    0	0	0	1];
