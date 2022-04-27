close all;
clear all;
clc;

% define sample state
X_init = [0 0 0 4 0 100 100 100 100];

% define sample control input
U_init = [0 -30];

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
t = 0:dt:4;

U_init_const_steer = U_init(1);
U_init_const_throttle = U_init(2);

xVect = [X_init];
xVect_discrete = [X_init];
for i=1:length(t)
    if i == 1
       temp_xVect = X_init;
%        temp_xVect_discrete = X_init;
    else
        temp_xVect = xVect(i,:);
%         temp_xVect_discrete = xVect_discrete(i,:);
    end
%     U_init(1) = (rand(1)-0.5)*(pi/2)+U_init_const_steer;
%     U_init(2) = (rand(1)-0.5)*200+U_init_const_throttle;
    X_n_1 = double_track_car.dynamics_rk4(temp_xVect, U_init, t(i), dt);
    xVect = [xVect ; X_n_1];
%     jac = double_track_car.discrete_jacobian(temp_xVect_discrete,U_init);
%     X_n_1_discrete = jac.A*temp_xVect_discrete' + jac.B*U_init';
%     xVect_discrete = [xVect_discrete ; X_n_1_discrete'];
%         disp(U_init);
end

% plot
figure()
hold on

plot(xVect(:,1),xVect(:,2),'-o')

title('varying random control inputs')
xlabel('x position')
ylabel('y position')
ylim([-2 2])
