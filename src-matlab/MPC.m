% MPC
% x = [x y r xdot ydot wfl wfr wrl wrr]
% u = [theta_steer torque]
% Defined constraints:
%   U: 
%       0: steering angle +/- 45 deg = pi/4 rad
%       1: torque limit 300 Nm
%   X:
%       None?
Nmpc = 51; %MPC Horizon
x0 = zeros(9,1); %Initial State
xf = zeros(9,1); %Final State
xf(1) = 160.934; % = 0.1 miles
max_velocity = 15; % = 33.554mph

% Create double track model
model = DoubleTrackModel();

% Create reference trajectory
[Xref, Uref, times] = genRefTraj_simplified(x0, xf, 1, 0.1);

n = length(Xref(:,1));
m = length(Uref(:,1));

% Define LQR costs... but for MPC
Q = eye(length(x0));
R = [1 0; 0 1];
Qf = Q;

% Get A, B matrices (time-varying)
% jac = model.discrete_jacobian();
A = eye(length(x0));
B = eye(length(x0),2);

mpc = OSQPController(n, m, Nmpc, length(Xref), Nd);