function y=dcm2euler(U)
% y=dcm2euler(U)
% function for converting a DCM "U" to euler angles "y"
% Edit: Isaac Skog 2007-05-24
% Note: problems when y(2)=pi/2


y=zeros(3,1);

y(1)=atan2(U(3,2),U(3,3));
y(2)=asin(-U(3,1));%-atan(U(3,1)/sqrt(1-U(3,1)^2));
y(3)=-atan(U(2,1)/U(1,1));