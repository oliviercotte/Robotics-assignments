% % Problem 3 for project 2
%
 clear all;
 close all;
 clc;

%
po2 = pi/2;
 
% link length, joint variable
%
syms a1 a2 a3 t1 t2 t3

% DH parameters.: alpha(i-1), a(i-1), d(i), theta(i)
%
dhpr3 = [ 0, 0, 0, t1; ...
   0, a1, 0, t2; ...
   0, a2, 0, t3; ...
   0, a3, 0 0] ;

% Derive the forward-pose kinematic
% when it finishes execution, the results are returned in
% ti (inidividual joint transformations using mdh parameters)
% fk (forward kinematics transformation using mdh parameters)
%
dhtable = dhpr3
[ti, fk] = fkine(dhtable) 
simplify_fk = simplify(fk)

% joint coordinates use the subs expression
% NOTE: angles must be in radians
%
sym={a1, a2, a3, t1, t2, t3};
ai={4 3 2};
% qi={0 0 0};
% qi={deg2rad(10) deg2rad(20) deg2rad(30)};
qi={po2 po2 po2};
numval=horzcat(ai,qi);
tinum = subs(ti,sym,numval)
fksym = subs(fk,sym,numval)
fknum = double(subs(fk,sym,numval))

% Corke Robotic toolbox
%
qi = cell2mat(qi);
L(1) = Revolute('d', 0, 'a', ai(1), 'alpha', 0);
L(2) = Revolute('d', 0, 'a', ai(2), 'alpha', 0);
L(3) = Revolute('d', 0, 'a', ai(3), 'alpha', 0);
threelink = SerialLink(L, 'name', 'three-link');
rt_fknum = threelink.fkine(qi)
threelink.plot(qi)
