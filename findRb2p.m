function R=findRb2p(in_data,out_data)

ind=in_data.SPEEDOMETER.speed>10;
t_speed=in_data.SPEEDOMETER.t(ind);
s=in_data.SPEEDOMETER.speed(ind);

N=length(t_speed);
v=zeros(3,N);

for n=1:N
    ind=logical(in_data.IMU.t==t_speed(n));
    Rb2t=q2dcm(out_data.x_h(7:10,ind));
    v(:,n)=Rb2t'*out_data.x_h(4:6,ind);
end


x0=1*randn(3,1);
lb=-30*pi/180*ones(3,1);
ub=-lb;

x = lsqnonlin(@(x)my_cost_fun(x,v,s),x0,lb,ub);
disp(x*180/pi)
R=Rt2b(x);

end


function J=my_cost_fun(x,v,s)

v=Rt2b(x)*v;
v(1,:)=s-v(1,:);
J=[v(1,:).^2 v(2,:).^2 v(3,:).^2]';

end
