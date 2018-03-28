%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Function that outputs a struct "settings" with the settings used in the
% GNSS-aided INS
% 
% Edit: Isaac Skog (skog@kth.se), 2016-09-01 
% Revised: Bo Bernhardsson 2018-01-01
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function settings=get_settings()



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%              GENERAL PARAMETERS         %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
settings.gnss_outage = 'on'; 
settings.outagestart = 200; 
settings.outagestop = Inf;
settings.non_holonomic = 'on';
settings.speed_aiding = 'on';

settings.init_heading = 320*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             FILTER PARAMETERS           %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Process noise covariance (Q)
% Standard deviations, need to be squared
settings.sigma_acc = 0.05; % [m/s^2]
settings.sigma_gyro = 0.1*pi/180; % [rad/s]
settings.sigma_acc_bias = 0.0001; % [m/s^2.5]
settings.sigma_gyro_bias = 0.01*pi/180; % [rad/s^1.5]


% GNSS position measurement noise covariance (R)
% Standard deviations, need to be squared
settings.sigma_gps = 3/sqrt(3); %[m] 
settings.sigma_speed = 1; %[m/s]  Trim here
%settings.sigma_speed = 24; %[m/s]  Trim here
settings.sigma_non_holonomic = 20; %[m/s] Trim here
%settings.sigma_non_holonomic = 3; %[m/s] Trim here


% Initial Kalman filter uncertainties (standard deviations)  
settings.factp(1) = 10;                                 % Position [m]
settings.factp(2) = 5;                                  % Velocity [m/s]
settings.factp(3:5) = (pi/180*[1 1 20]');               % Attitude (roll,pitch,yaw) [rad]
settings.factp(6) = 0.02;                               % Accelerometer biases [m/s^2]
settings.factp(7) = (0.05*pi/180);                      % Gyro biases [rad/s]                               

settings.gravity = gravity(59,0);
      
end




