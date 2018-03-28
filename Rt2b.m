function R=Rt2b(ang)
% function for calculation of the rotation matrix for
% rotaion from tangent frame to body frame.
% function R=Rt2b[roll,pitch,yaw];
% v_b=[u v d]'
% v_t=[N E D]'

cr=cos(ang(1));
sr=sin(ang(1));

cp=cos(ang(2));
sp=sin(ang(2));

cy=cos(ang(3));
sy=sin(ang(3));

R=[cy*cp sy*cp -sp; 
    -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; 
    sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];


