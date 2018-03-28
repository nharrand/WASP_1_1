% Function that returns the directional cosine matrix that that relates the
% body (IMU coordinate system) to the platform (vehicle coordinate system)
% coordinate frame.
%
% Edit: Isaac Skog (skog@kth.se), 2016-09-08
function   Rb2p=get_Rb2p()

Rb2p=[0.9880   -0.1472   -0.0463;
    0.1540    0.9605    0.2319;
    0.0103   -0.2363    0.9716];
   
end