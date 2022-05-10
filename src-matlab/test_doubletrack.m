close all;
clear all;
clc;

% define sample state
% x = [uy r ux dPsi e dFzlong dFzlat delta xPos yPos yawOrient]
X_init = [10, ... % uy
    0, ... % rdot 
    0, ... % ux
    0, ... % dFzlong
    0, ... % dFzlat
    pi/2, ... % delta
    0, ... % x-position
    0, ... % y-position
    0]; % rb

% define sample control input
% u = [deltadot Fxfbrake Fxr Fengine udiff]
U_init = [0, ... % deltadot
    100, ... % Fxf_enginebrake
    0]; % Fxr

double_track_car = DoubleTrackModel();

% step through dynamics 
dt = 0.1;
t = 0:dt:45;

xVect = [X_init];
xVect_discrete = [X_init];
for i=1:length(t)
%     i
    if i == 1
       temp_xVect = X_init;
%        temp_xVect_discrete = X_init;
    else
        temp_xVect = xVect(i,:);
%         temp_xVect_discrete = xVect_discrete(i,:);
    end
%     U_init(1) = (rand(1)-0.5)*(pi/2)+U_init_const_steer;
%     U_init(2) = (rand(1)-0.5)*200+U_init_const_throttle;
    X_n_1 = double_track_car.dynamics_rk4(temp_xVect, U_init, dt);
    xVect = [xVect ; X_n_1];
%     jac = double_track_car.discrete_jacobian(temp_xVect_discrete,U_init);
%     X_n_1_discrete = jac.A*temp_xVect_discrete' + jac.B*U_init';
%     xVect_discrete = [xVect_discrete ; X_n_1_discrete'];
%         disp(U_init);
end

% plot
figure()
hold on

plot(xVect(:,7),xVect(:,8),'-o')

title('varying random control inputs')
xlabel('x position')
ylabel('y position')
% ylim([-.1 .1])
% xlim([-.1 .1])
