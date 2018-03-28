Ts=0.01;
Tmax=60;
t=0:Ts:Tmax;
N=length(t);

% Initialize state vector x
xS=zeros(10,1);
xS(end)=1;
xL=zeros(10,1);
xL(end)=1;

% Try out different biases
%Stockholm  59° 19′ 46″ nord
%Lund 55° 42′ nord
accbias = [0;0;0];
gyrobias = [0.01*pi/180;0;0];
%gyrobias = [0;0;0];

gS = gravity(59,0);
uS = [gS + accbias ; gyrobias];
gL = gravity(55,0);
uL = [gL + accbias ; gyrobias];

% simulate IMU standalone navigation system
posS=zeros(3,N);
posL=zeros(3,N);

for n=2:N
   xS = Nav_eq(xS,uS,Ts,gS);
   xL = Nav_eq(xL,uL,Ts,gL);
   posS(:,n) = xS(1:3);
   posL(:,n) = xL(1:3);
end
diff = posS - posL;
pos = [posS ; posL];

figure(1)
clf
%plot(t,pos)
%plot(t,posL')
plot(t,posS)
%plot(t,diff)
grid on
ylabel('Position error [m]')
%ylabel('Difference in position error [m]')
xlabel('Time [s]')
legend('x-axis error','y-axis error','z-axis error')


figure(2)
loglog(t,posS')
pos2=9.82*gyrobias(1)*t.^3/6;  % theoretical 
pos3=9.82*(gyrobias(1))^2*t.^4/24; % theoretical
hold on
loglog(t,pos2,'k--')
loglog(t,pos3,'k--')