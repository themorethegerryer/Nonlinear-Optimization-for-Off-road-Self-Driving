clc
clear all
close all
% MPPI
% x = [uy r ux dFzlong delta x y yaw]
% u = [deltadot Fxf_enginebrake Fxr]
% Defined constraints:
%   U: 
%       1: change in steering (+/-1 rad/s)
%       3: braking force 1500 Nm (>= 0)
%       4: torque limit 300 Nm (>= 0)
%   X:
%       7: steering angle +/- 45 deg = pi/4 rad

Nh = 25; %Rollout Horizon
% max_velocity = 15; % = 33.554mph
dt = 0.1;

x0 = [0 0 0 0 pi/4 0 0 0]';

% Create double track model
model = DynamicBicycleModel();

% Create reference trajectory
% [Xref, Uref, times] = genRefTraj_simplified(x0, xf, u_const, max_velocity, dt);
Xref = [0.01 0 0 0 0 0 0 0]';
posRef = [4 4 0]';
Uref = [0 0 0]';

% n = length(Xref(:,1));
% m = length(Uref(:,1));
% N = length(times);
n = length(Xref);
npos = 3;
m = length(Uref);
N = 2000;

% Define constraints
% xmin = [-Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -pi/4]';
% xmax = [Inf, Inf, Inf, Inf, Inf, Inf, pi/4]';
umin = [-0.2, -400, 0]';
umax = [0.2, 400, 0]';

% Define LQR costs... but for MPC
Q = diag([0 0 0 0 0 0 0 0]);
R = diag([0 0 0]);
% Q = diag([0 0 1 0 0 0 0]);
% R = diag([0 0 0 0]);
% xpos ypos yaw
Qpos = diag([1 1 0]);

% Create MPPI Controller
x_pos = zeros(N,1);
y_pos = zeros(N,1);
yaw_pos = zeros(N,1);
num_samples = 600;
X = zeros(n,N);
U = zeros(m,N-1);
X(:,1) = x0;
for k = 1:N-1
    disp(k)
    % Starting point for this iteration
    Nhu = length(k:min(k+Nh-2,N-1));
    u_start = Uref.*ones(m,Nhu);
    x_start = X(:,k);
    % Generate random perturbations between control constraints
%     filter_u = randi([0 1], [num_samples,Nhu]);
    u_perturb = zeros(m,Nhu,num_samples);
    u_perturb(1,:,:) = ((umax(1)-umin(1)).*rand(num_samples,Nhu)+umin(1))';
    u_perturb(2,:,:) = (((umax(2)-umin(2)).*rand(num_samples,Nhu)+umin(2)))';
    u_perturb(3,:,:) = (((umax(3)-umin(3)).*rand(num_samples,Nhu)+umin(3)))';
%     u_perturb(4,:,:) = (((umax(4)-umin(4)).*rand(num_samples,Nhu)+umin(4)).*~filter_u)';
    
    % Rollouts
    x_rollout = zeros(n,Nhu+1);
    x_rollout(:,1) = x_start;
    xpos_rollout = zeros(npos,Nhu+1);
    xpos_rollout(:,1) = [x_pos(k) ; y_pos(k); yaw_pos(k)];
    cost = zeros(num_samples,1);
    gamma = 1.0;
    for kk = 1:num_samples
        u_rollout = u_start+u_perturb(:,:,kk);
        for kkk = 1:Nhu
            x_rollout(:,kkk+1) = model.dynamics_rk4(x_rollout(:,kkk)', u_rollout(:,kkk)', dt); %A(:,:,k-1+kkk)*x_rollout(:,kkk)+B(:,:,k-1+kkk)*u_rollout(:,kkk); 
%             cost(kk) = cost(kk) + 0.5*(x_rollout(:,kkk+1)-Xref(:,k-1+kkk))'*Q*(x_rollout(:,kkk+1)-Xref(:,k-1+kkk)) + 0.5*(u_rollout(:,kkk)-Uref(:,k-1+kkk))'*R*(u_rollout(:,kkk)-Uref(:,k-1+kkk));
            cost(kk) = cost(kk) + gamma^(kkk-1)*(0.5*(x_rollout(:,kkk+1)-Xref)'*Q*(x_rollout(:,kkk+1)-Xref) + 0.5*(u_rollout(:,kkk)-Uref)'*R*(u_rollout(:,kkk)-Uref));
            xpos_rollout(:,kkk+1) = xpos_rollout(:,kkk) + [x_rollout(3,kkk) ; x_rollout(1,kkk) ; x_rollout(2,kkk)]*dt;
            cost(kk) = cost(kk) + gamma^(kkk-1)*(0.5*(xpos_rollout(:,kkk+1) - posRef)'*Qpos*(xpos_rollout(:,kkk+1) - posRef));
        end
    end
    % Get control trajectory weights based on cost
    sum_weights = 0.0;
    weighted_traj = zeros(size(u_perturb));
%     weighted_filter_u = zeros(num_samples,Nhu);
    scaling_factor = mean(cost);
%     scaling_factor = 1e4;
    for uk = 1:num_samples
        weight = exp(-(1/scaling_factor)*cost(uk));
        sum_weights = sum_weights + weight;
        weighted_traj(:,:,uk) = weight*u_perturb(:,:,uk);
%         weighted_filter_u(uk,:) = weight*(filter_u(uk,:)*2-1);
    end

    u_mppi = u_start+(sum(weighted_traj,3)/sum_weights);
%     filter_mppi = sum(weighted_filter_u,1);
%     u_mppi_braking_mean = u_mppi(2,1) + u_mppi(3,1);
%     u_mppi_throttle_mean = u_mppi(4,1);
%     if filter_mppi(1) < 0
%         u_mppi(2) = 0;
%         u_mppi(3) = 0;
%     else
%         u_mppi(4) = 0;
%     end
    U(:,k) = u_mppi(:,1);
    X(:,k+1) = model.dynamics_rk4(X(:,k)',U(:,k)', dt);%A(:,:,k)*x_start+B(:,:,k)*u_mppi(:,1);
    x_pos(k+1) = x_pos(k)+ X(3,k+1)*dt;
    y_pos(k+1) = y_pos(k)+ X(1,k+1)*dt;
    yaw_pos(k+1) = yaw_pos(k) + X(2,k+1)*dt;
    [x_pos(k+1) y_pos(k+1)]
    mean(cost)
end
figure(1)
hold on
plot(x_pos,y_pos,'-o')
% plot(xf(1),xf(2), 'r-o')
hold off
xlabel('x position')
ylabel('y position')
title('MPPI')
