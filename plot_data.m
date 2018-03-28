function plot_data(in_data,out_data,holdon)

if nargin < 3; holdon = 'False'; end
    
h=zeros(1,3);
figure(2)
if ~holdon clf; end
plot(in_data.GNSS.pos_ned(2,:),in_data.GNSS.pos_ned(1,:),'b-');
hold on
h(1)=plot(in_data.GNSS.pos_ned(2,:),in_data.GNSS.pos_ned(1,:),'b.');
h(2)=plot(out_data.x_h(2,:),out_data.x_h(1,:),'r');
h(3)=plot(in_data.GNSS.pos_ned(2,1),in_data.GNSS.pos_ned(1,1),'ks');
title('Trajectory')
ylabel('North [m]')
xlabel('East [m]')
legend(h,'GNSS position estimate','GNSS aided INS trajectory','Start point')
axis equal
grid on

h=zeros(1,3);
figure(3)
if ~holdon clf; end
h(1)=plot(in_data.GNSS.t,-in_data.GNSS.pos_ned(3,:),'b.');
hold on
h(2)=plot(in_data.IMU.t,-out_data.x_h(3,:),'r');
h(3)=plot(in_data.IMU.t,3*sqrt(out_data.diag_P(3,:))-out_data.x_h(3,:),'k--');
plot(in_data.IMU.t,-3*sqrt(out_data.diag_P(3,:))-out_data.x_h(3,:),'k--')
title('Height versus time')
ylabel('Height [m]')
xlabel('Time [s]')
grid on
legend(h,'GNSS estimate','GNSS aided INS estimate','3\sigma bound')

h=zeros(1,2);
figure(4)
if ~holdon clf; end
h(1)=plot(in_data.IMU.t,sqrt(sum(out_data.x_h(4:6,:).^2)),'r');
hold on
sigma=sqrt(sum(out_data.diag_P(4:6,:)));
speed=sqrt(sum(out_data.x_h(4:6,:).^2));
h(2)=plot(in_data.IMU.t,3*sigma+speed,'k--');
plot(in_data.IMU.t,-3*sigma+speed,'k--')
title('Speed versus time')
ylabel('Speed [m/s]')
xlabel('Time [s]')
grid on
legend(h,'GNSS aided INS estimate','3\sigma bound')

N=length(in_data.IMU.t);
attitude=zeros(3,N);

for n=1:N
    Rb2t=q2dcm(out_data.x_h(7:10,n));
    
    %Get the roll, pitch and yaw
    % roll
    attitude(1,n)=atan2(Rb2t(3,2),Rb2t(3,3))*180/pi;
    
    % pitch
    attitude(2,n)=-atan(Rb2t(3,1)/sqrt(1-Rb2t(3,1)^2))*180/pi;
    
    %yaw
    attitude(3,n)=atan2(Rb2t(2,1),Rb2t(1,1))*180/pi;
end

ylabels={'Roll [deg]','Pitch [deg]','Yaw [deg]'};
figure(5)
if ~holdon clf; end
h=zeros(1,2);
for k=1:3
    subplot(3,1,k)
    h(1)=plot(in_data.IMU.t,attitude(k,:),'r');
    hold on
    plot(in_data.IMU.t,3*180/pi*sqrt(out_data.diag_P(6+k,:))+attitude(k,:),'k--')
    h(2)=plot(in_data.IMU.t,-3*180/pi*sqrt(out_data.diag_P(6+k,:))+attitude(k,:),'k--');
    grid on
    ylabel(ylabels{k})
    if k==1
        title('Attitude versus time')
    end
end
xlabel('Time [s]')
legend(h,'GNSS aided INS estimate','3\sigma bound')


figure(6)
if ~holdon clf; end
ylabels={'X-axis bias [m/s^2]','Y-axis bias [m/s^2]','Z-axis bias [m/s^2]'};
h=zeros(1,2);
for k=1:3
    subplot(3,1,k)
    h(1)=plot(in_data.IMU.t,out_data.delta_u_h(k,:),'r');
    hold on
    h(2)=plot(in_data.IMU.t,3*sqrt(out_data.diag_P(9+k,:))+out_data.delta_u_h(k,:),'k--');
    plot(in_data.IMU.t,-3*sqrt(out_data.diag_P(9+k,:))+out_data.delta_u_h(k,:),'k--')
    grid on
    ylabel(ylabels{k})
    if k==1
        title('Accelerometer bias estimate versus time')
    end
end
xlabel('Time [s]')
legend(h,'GNSS aided INS estimate','3\sigma bound')


figure(7)
if ~holdon clf; end
ylabels={'X-axis bias [deg/s]','Y-axis bias [deg/s]','Z-axis bias [deg/s]'};
h=zeros(1,2);
for k=1:3
    subplot(3,1,k)
    h(1)=plot(in_data.IMU.t,180/pi*out_data.delta_u_h(3+k,:),'r');
    hold on
    h(2)=plot(in_data.IMU.t,3*180/pi*sqrt(out_data.diag_P(12+k,:))+180/pi*out_data.delta_u_h(3+k,:),'k--');
    plot(in_data.IMU.t,-3*180/pi*sqrt(out_data.diag_P(12+k,:))+180/pi*out_data.delta_u_h(3+k,:),'k--')
    grid on
    ylabel(ylabels{k})
    if k==1
        title('Gyroscope bias estimate versus time')
    end
end
xlabel('Time [s]')
legend(h,'GNSS aided INS estimate','3\sigma bound')

figure(8)
if ~holdon clf; end
xest = out_data.x_h(2,:);
yest = out_data.x_h(1,:);
xgps = interp1(in_data.GNSS.t,in_data.GNSS.pos_ned(2,:),in_data.IMU.t,'linear','extrap')';
ygps = interp1(in_data.GNSS.t,in_data.GNSS.pos_ned(1,:),in_data.IMU.t,'linear','extrap')';
xerr = xest - xgps; 
yerr = yest - ygps;
plot(in_data.IMU.t,xerr)
grid on
hold on
plot(in_data.IMU.t,yerr)
xlabel('time [s]')
ylabel('position difference [m]')
legend('x', 'y')

positionerr_RMS = sqrt(mean(xerr.^2+yerr.^2))
figure(2)
end

