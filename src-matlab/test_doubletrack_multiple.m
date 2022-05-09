close all;
clear all;
clc;

% define sample state
X_init = [0 0 0 0 0 0 0 0 0 0 0];

% define sample control input
U_init = [0 0 0 0 0];

% accelerator between 0 and 70
% brake between 0 and 150

% % declare double_track_car class
% m = 2000;    % mass of the car (kg)
% mw = 10;     % mass of each tire (kg) 
% lw = 1.8;    % width of car - distance b/w center of tire (m)
% lr = 2.25;   % distance from rear to center of mass (m)
% lf = 2.25;   % distance from front to center of mass (m)
% h = 1.5;     % height of car (m)
% h_CoM = 0.5; % height of car's center of mass (m)  
% Rw = 0.23;   % effective tire radius (m)
% c = 0.04;    % rolling friction coefficient
double_track_car = DoubleTrackModel();

% step through dynamics 
dt = 0.1;
t = 0:dt:30;

U_init_const_steer = 0;
U_init_const_throttle = 0;

for k=1:5
    xVect = [X_init];
    xVect_discrete = [X_init];
    for i=1:length(t)
        if i == 1
           temp_xVect = X_init;
%            temp_xVect_discrete = X_init;
        else
            temp_xVect = xVect(i,:);
%             temp_xVect_discrete = xVect_discrete(i,:);
        end
        U_init(1) = (rand(1)-0.5)*(pi/3)+U_init_const_steer;
        U_init(4) = (rand(1)-0.5)*100+U_init_const_throttle;
        X_n_1 = double_track_car.dynamics_rk4(temp_xVect, U_init, dt);
        xVect = [xVect ; X_n_1];
%         jac = double_track_car.discrete_jacobian(temp_xVect_discrete,U_init);
%         X_n_1_discrete = jac.A*temp_xVect_discrete' + jac.B*U_init';
%         xVect_discrete = [xVect_discrete ; X_n_1_discrete'];
%         disp(U_init);
    end
    xStore(:,:,k) = xVect;
%     xStore_discrete(:,:,k) = xVect_discrete;
end
% X_init = [0 0 0.1 2 2 100 100 100 100];
% 
% jac = double_track_car.discrete_jacobian(X_init, U_init);
% jac = numeric_jacobian(double_track_car,X_init,U_init, 0.01);

% xVect(end,:)

% plot
sizeArr = size(xStore);
figure()
hold on
for i=1:sizeArr(3)
    plot(xStore(:,9,i),xStore(:,10,i),'-o')
end
title('varying random control inputs')
xlabel('x position')
ylabel('y position')
% 
% U_init(1) = 0;
% % vary U_init(2)
% for k=1:17
%     xVect = [X_init];
%     U_init(2) = k*(4/17);
%     throttle(k) = string(U_init(2));
%     for i=1:length(t)
%         if i == 1
%            temp_xVect = X_init;
%         else
%             temp_xVect = xVect(i,:);
%         end
%         X_n_1 = double_track_car.dynamics_rk4(temp_xVect, U_init, t(i), dt);
%         xVect = [xVect ; X_n_1];
% %         disp(U_init);
%     end
% %     disp(X_n_1);
%     xStore_throttle(:,:,k) = xVect;
% end
% 
% % plot
% sizeArr = size(xStore_throttle);
% figure()
% hold on
% for i=1:sizeArr(3)
%     plot(xStore_throttle(end,1,i),xStore_throttle(end,2,i),'x')
% end
% legend(throttle)
% % plot(xStore_throttle(:,1,end), xStore_throttle(:,2,end), '-o');
% title('increasing control inputs (throttle)')
% xlabel('x position')
% ylabel('y position')
% hold off
% 
% % define sample state
% X_init = [0 0 0 4 0 100 100 100 100];
% U_init(1) = 0;
% % vary U_init(2)
% for k=1:17
%     xVect = [X_init];
%     U_init(2) = -k*(30/17);
%     throttle(k) = string(U_init(2));
%     for i=1:length(t)
%         if i == 1
%            temp_xVect = X_init;
%         else
%             temp_xVect = xVect(i,:);
%         end
%         X_n_1 = double_track_car.dynamics_rk4(temp_xVect, U_init, t(i), dt);
%         xVect = [xVect ; X_n_1];
% %         disp(U_init);
%     end
% %     disp(X_n_1);
%     xStore_throttle(:,:,k) = xVect;
% end
% 
% % plot
% sizeArr = size(xStore_throttle);
% figure()
% hold on
% for i=1:sizeArr(3)
%     plot(xStore_throttle(end,1,i),xStore_throttle(end,2,i),'x')
% end
% legend(throttle)
% % plot(xStore_throttle(:,1,end), xStore_throttle(:,2,end), '-o');
% title('decreasing control inputs (braking)')
% xlabel('x position')
% ylabel('y position')
% hold off
