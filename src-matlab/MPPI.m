clc
clear all
% MPPI
% x = [x y r xdot ydot wfl wfr wrl wrr]
% u = [theta_steer torque brake]
% Defined constraints:
%   U: 
%       0: steering angle +/- 45 deg = pi/4 rad
%       1: torque limit 300 Nm
%   X:
%       None?

Nh = 30; %Rollout Horizon
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0]; %Initial State
xf = [161.934; 161.934; pi/2; 0; 0; 0; 0; 0; 0]; %Final State
% xf(1) = 161.934; % = 0.1 miles
u_const = [0;10;0];
max_velocity = 15; % = 33.554mph
dt = 0.1;

% Create double track model
model = DoubleTrackModel();

% Create reference trajectory
[Xref, Uref, times] = genRefTraj_simplified(x0, xf, u_const, max_velocity, dt);

n = length(Xref(:,1));
m = length(Uref(:,1));
N = length(times);

% Define constraints
umin = [-pi/4; 0; 0];
umax = [pi/4; 300; 1500];

% Define LQR costs... but for MPC
Q = diag([1 1 1 1 1 0.01 0.01 0.01 0.01]);
R = diag([0.1 0.1 0.1]);
Qf = Q;

% Get A, B matrices (time-varying)
A = zeros(n,n,N-1);
B = zeros(n,m,N-1);
for k = 1:N-1
    jac = model.discrete_jacobian(Xref(:,k), Uref(:,k));
    A(:,:,k) = jac.A;
    B(:,:,k) = jac.B;
end

% Create MPPI Controller
num_samples = 100;
X = zeros(size(Xref));
U = zeros(size(Uref));
X(:,1) = x0;
for k = 1:N-1
    disp(k)
    % Starting point for this iteration
    u_start = Uref(:,k:min(k+Nh-2,N-1));
    x_start = X(:,k);
    Nhu = length(k:min(k+Nh-2,N-1));
    % Generate random perturbations between control constraints
    u_perturb = zeros(m,Nhu,num_samples);
    u_perturb(1,:,:) = ((umax(1)-umin(1)).*rand(num_samples,Nhu)+umin(1))';
    u_perturb(2,:,:) = ((umax(2)-umin(2)).*rand(num_samples,Nhu)+umin(2))';
    u_perturb(3,:,:) = ((umax(2)-umin(2)).*rand(num_samples,Nhu)+umin(2))';
    
    % Rollouts
    x_rollout = zeros(n,Nhu+1);
    x_rollout(:,1) = x_start;
    cost = zeros(num_samples,1);
    for kk = 1:num_samples
        u_rollout = u_start+u_perturb(:,:,kk);
        for kkk = 1:Nhu
            x_rollout(:,kkk+1) = model.dynamics_rk4(x_rollout(:,kkk)', u_rollout(:,kkk)', dt); %A(:,:,k-1+kkk)*x_rollout(:,kkk)+B(:,:,k-1+kkk)*u_rollout(:,kkk); 
            cost(kk) = cost(kk) + 0.5*(x_rollout(:,kkk+1)-Xref(:,k-1+kkk))'*Q*(x_rollout(:,kkk+1)-Xref(:,k-1+kkk)) + 0.5*(u_rollout(:,kkk)-Uref(:,k-1+kkk))'*R*(u_rollout(:,kkk)-Uref(:,k-1+kkk));
        end
    end
    % Get control trajectory weights based on cost
    sum_weights = 0.0;
    weighted_traj = zeros(size(u_perturb));
    scaling_factor = mean(cost);
    for uk = 1:num_samples
        weight = exp(-(1/scaling_factor)*cost(uk));
        sum_weights = sum_weights + weight;
        weighted_traj(:,:,kk) = weight*u_perturb(:,:,kk);
    end
    
    u_mppi = u_start+(sum(weighted_traj,3)/sum_weights);
    U(:,k) = u_mppi(:,1);
    X(:,k+1) = model.dynamics_rk4(X(:,k)',u_mppi(:,1)', dt);%A(:,:,k)*x_start+B(:,:,k)*u_mppi(:,1);
end
figure(1)
hold on
plot(X(1,:),X(2,:),'-o')
% plot(xf(1),xf(2), 'r-o')
hold off
xlabel('x position')
ylabel('y position')
title('MPPI')
