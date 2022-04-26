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

Nmpc = 10; %MPC Horizon
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0]; %Initial State
xf = [161.934; 0; 0; 0; 0; 0; 0; 0; 0]; %Final State
% xf(1) = 161.934; % = 0.1 miles
max_velocity = 15; % = 33.554mph
dt = 0.01;

% Create double track model
model = DoubleTrackModel();

% Create reference trajectory
[Xref, Uref, times] = genRefTraj_simplified(x0, xf, max_velocity, dt);

n = length(Xref(:,1));
m = length(Uref(:,1));
N = length(times);

% Define constraints
umin = [-pi/4; 0; 0];
umax = [pi/4; 30; 15];

% Define LQR costs... but for MPC
Q = diag([100 1 1 1 1 0.1 0.1 0.1 0.1]);
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
for k = 1:N-1
    
end