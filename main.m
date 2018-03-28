%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                           
% Main script for the loosely-coupled feedback GNSS-aided INS system. 
%  
% Edit: Isaac Skog (skog@kth.se), 2016-09-01,  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Load data
disp('Loads data')
load('GNSSaidedINS_data.mat');

%% Load filter settings
disp('Loads settings')
settings=get_settings();

%% Run the GNSS-aided INS
disp('Runs the GNSS-aided INS')



% for j = 1:3
%     for i = 1:30
%         settings.sigma_speed = i/10;
%         settings.sigma_non_holonomic = j;
%         out_data=GPSaidedINS(in_data,settings);
%         xest = out_data.x_h(2,:);
%         yest = out_data.x_h(1,:);
%         xgps = interp1(in_data.GNSS.t,in_data.GNSS.pos_ned(2,:),in_data.IMU.t,'linear','extrap')';
%         ygps = interp1(in_data.GNSS.t,in_data.GNSS.pos_ned(1,:),in_data.IMU.t,'linear','extrap')';
%         xerr = xest - xgps; 
%         yerr = yest - ygps;
%         positionerr_RMS = sqrt(mean(xerr.^2+yerr.^2));
%         str = ['Run sigma_speed: ' num2str(i/10, '%i') ', sigma_non_holonomic: ' num2str(j, '%i') ' => RMS: ' num2str(positionerr_RMS, '%i')];
%         disp(str)
%     end
% end

out_data=GPSaidedINS(in_data,settings);

%% Plot the data 
disp('Plot data')
plot_data(in_data,out_data,'True');drawnow





