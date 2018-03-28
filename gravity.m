function g=gravity(lambda,h)
% function g=gravity(lambda,h)
%
% function for calculation of the local gravity vector, in
% the geographic reference frame (same as tangent plane is 
% stationary).
%
% Based upon the WGS_84 Geodetic and Gravity model. For more 
% info see [pp 222-223,1].
%
% lambda -> Latitude [degrees]
% h -> Altitude [m]
% g 
%
% edit: Isaac Skog, 2006-08-17

% degrees to radians
lambda=pi/180*lambda;

gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);

g=gamma-((3.0877e-6)-(0.004e-6)*sin(lambda)^2)*h+(0.072e-12)*h^2;

g=[0 0 -g]';
return;




