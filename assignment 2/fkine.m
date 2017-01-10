%       T=fkine(alpha,a,d,theta) gives the Homogeneous transformation T corresponding to
%       DH parameters alpha,a,d,theta
%           
%       alpha is the skew angle
%       a is the distance between the previous Z and current z coordinate
%       d is the distance between thee previous X and current x coordinate
%       theta is the angle between the x coordinates
%
%       tjnt:  individual homogeneous transforms for each joint
%       tfk :  final forward kinematics in symbolic form using mdh params.
%
%       Olivier Cotte
%       November 7th, 2016
function [tjnt, tfk] = fkine(dhtable)

   syms a b c d
   n = size(dhtable, 1) ; 
	
   % Initialize matrices for results
   %tjnt = [] ;
   tfk  = eye(4);
 
   % Find Individual Joint-Link Homo Transform
   % and forward kinematics
   for i=1:n
      a = dhtable(i,1);
      b = dhtable(i,2);
      c = dhtable(i,3);
      d = dhtable(i,4);
      tim1 = linktrans(a,b,c,d) ;
      tjnt(:,:,i) = tim1;
      tfk  = tfk * tim1 ;     
   end
   