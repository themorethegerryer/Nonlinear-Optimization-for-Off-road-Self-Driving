close all;
clear all;
clc;

% define sample state
X_init = [0 0 0 4 0 811 811 811 811];

% define sample control input
U_init = [0.0 0 150];

% accelerator between 0 and 70
% brake between 0 and 150

% declare double_track_car class
m = 2000;    % mass of the car (kg)
mw = 10;     % mass of each tire (kg) 
lw = 1.8;    % width of car - distance b/w center of tire (m)
lr = 2.25;   % distance from rear to center of mass (m)
lf = 2.25;   % distance from front to center of mass (m)
h = 1.5;     % height of car (m)
h_CoM = 0.5; % height of car's center of mass (m)  
Rw = 0.23;   % effective tire radius (m)
c = 0.04;    % rolling friction coefficient
double_track_car = DoubleTrackModel(m, mw, lw, lr, lf, Rw, c);

% step through dynamics 
dt = 0.1;
t = 0:dt:2;
xVect = [X_init];

for i=1:length(t)
    if i == 1
       temp_xVect = X_init;
    else
        temp_xVect = xVect(i,:);
    end
    X_n_1 = double_track_car.dynamics_rk4(temp_xVect, U_init, dt);
    xVect = [xVect ; X_n_1];
end

% X_init = [0 0 0.1 2 2 100 100 100 100];
% 
% jac = double_track_car.discrete_jacobian(X_init, U_init);
% jac = numeric_jacobian(double_track_car,X_init,U_init, 0.01);

xVect(end,:)

% plot
plot(xVect(:,1),xVect(:,2))
ylim([-1 1])
