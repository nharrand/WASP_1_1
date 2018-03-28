function q=dcm2q(R)
%
% function q=dcm2q(R)
% Function for transformation from directional cosine matrix to quaternions 
% From Farrel pp 42.
% Edit: Isaac Skog, 2007-05-24



q=zeros(4,1);

q(4)=0.5*sqrt(1+sum(diag(R)));

q(1)=(R(3,2)-R(2,3))/(4*q(4));

q(2)=(R(1,3)-R(3,1))/(4*q(4));

q(3)=(R(2,1)-R(1,2))/(4*q(4));

