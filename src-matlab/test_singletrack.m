close all;
clear all;
clc;

% define sample state
% x = [uy r ux dFzlong delta x y yaw]
X_init = [0 0 0 0 pi/4 0 0 0];

% define sample control input
% u = [deltadot Fxfbrake Fxr Fengine]
U_init = [0 0 0 10];

single_track_car = DynamicBicycleModel();

% step through dynamics 
dt = 0.01;
t = 0:dt:45;
xpos = zeros(size(t));
ypos = zeros(size(t));

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
    X_n_1 = temp_xVect + single_track_car.continuous_dynamics(temp_xVect, U_init)*dt;
    xVect = [xVect ; X_n_1];
    xpos(i+1) = xpos(i) + X_n_1(3)*dt;
    ypos(i+1) = ypos(i) + X_n_1(1)*dt;
%     jac = double_track_car.discrete_jacobian(temp_xVect_discrete,U_init);
%     X_n_1_discrete = jac.A*temp_xVect_discrete' + jac.B*U_init';
%     xVect_discrete = [xVect_discrete ; X_n_1_discrete'];
%         disp(U_init);
end

% plot
figure()
hold on

plot(xVect(:,6),xVect(:,7),'-o')

title('varying random control inputs')
xlabel('x position')
ylabel('y position')
% ylim([-2 2])
xVect(end,3)
