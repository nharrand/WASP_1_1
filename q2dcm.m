function R=q2dcm(q)
% function R=q2dcm(q)
% Function for transformation from quaternions to directional cosine matrix
% Farell pp.41
% Edit: Isaac Skog, 2007-05-24

p=zeros(6,1);

p(1:4)=q.^2;

p(5)=p(2)+p(3);

if p(1)+p(4)+p(5)~=0
   p(6)=2/(p(1)+p(4)+p(5)); 
else
   p(6)=0;
end


R(1,1)=1-p(6)*p(5);
R(2,2)=1-p(6)*(p(1)+p(3));
R(3,3)=1-p(6)*(p(1)+p(2));

p(1)=p(6)*q(1); 
p(2)=p(6)*q(2);
p(5)=p(6)*q(3)*q(4);
p(6)=p(1)*q(2);

R(1,2)=p(6)-p(5);
R(2,1)=p(6)+p(5);

p(5)=p(2)*q(4);
p(6)=p(1)*q(3);

R(1,3)=p(6)+p(5);
R(3,1)=p(6)-p(5);

p(5)=p(1)*q(4);
p(6)=p(2)*q(3);

R(2,3)=p(6)-p(5);
R(3,2)=p(6)+p(5);

% R(1,2)=2*(q(1)*q(2)-q(3)*q(4));
% R(1,3)=2*(q(1)*q(3)+q(2)*q(4));
% 
% % Row 2
% R(2,1)=2*(q(1)*q(2)+q(3)*q(4));
% R(2,2)=q(2)^2+q(4)^2-q(1)^2-q(3)^2;
% R(2,3)=2*(q(2)*q(3)-q(1)*q(4));
% 
% % Row 3
% R(3,1)=2*(q(1)*q(3)-q(2)*q(4));
% R(3,2)=2*(q(2)*q(3)+q(1)*q(4));
% R(3,3)=q(3)^2+q(4)^2-q(1)^2-q(2)^2;
% 
% % Okej, men kan förbätrras, se pp 41 Farrel