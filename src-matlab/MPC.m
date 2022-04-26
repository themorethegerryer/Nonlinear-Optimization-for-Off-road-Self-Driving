clc
clear all
% MPC
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
dt = 1;

% Create double track model
model = DoubleTrackModel();

% Create reference trajectory
[Xref, Uref, times] = genRefTraj_simplified(x0, xf, max_velocity, dt);

n = length(Xref(:,1));
m = length(Uref(:,1));
N = length(times);

% Define constraints
umin = [-pi/4; 0; 0];
umax = [pi/4; 300; 150];

% Define LQR costs... but for MPC
Q = diag([10 10 10 1 1 0 0 0 0]);
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

% Create MPC as OSQP
Np = (Nmpc-1)*(n+m); % number of primals
Nd = (Nmpc-1)*n + (Nmpc-1)*m; % number of duals

P = blkdiag(kron(speye(Nmpc-2), blkdiag(sparse(R), sparse(Q))), sparse(R), sparse(Qf));

q = zeros((Nmpc-2)*(n+m),1);
% once k+i is less than the length of Xref/Uref, repeat Xref[end]/Uref[end] for remainder
refidx = 0;
for i = 1:Nmpc-2
    refidx = min(N-1,i);
    q((m+n)*(i-1)+(1:m)) = -R*Uref(:,refidx);
    q((m+n)*(i-1)+(m+1:m+n)) = -Q*Xref(:,refidx);
end
q((m+n)*(Nmpc-2)+(m+1:m+n)) = -Qf*Xref(:,refidx);

D(1:n,1:m) = B(:,:,1);
D(1:n,(m+1):(m+n)) = -eye(n);
for k = 1:(Nmpc-2)
    D((k*n)+(1:n), (m+(k-1)*(n+m))+(1:(2*n+m))) = [A(:,:,k+1) B(:,:,k+1) -eye(n)];
end
U = kron(eye(Nmpc-1), [eye(m) zeros(m,n)]);
C = [D;U];

lb_Uref = zeros(3,Nmpc-1);
ub_Uref = zeros(3,Nmpc-1);
for k = 1:Nmpc-1
    lb_Uref(:,k) = umin - Uref(:,k);
    ub_Uref(:,k) = umax - Uref(:,k);
end
lb = [zeros(n*(Nmpc-1),1); reshape(lb_Uref',1,[])'];
ub = [zeros(n*(Nmpc-1),1); reshape(ub_Uref',1,[])'];

mpc = osqp;
mpc.setup(P, q, C, lb, ub);


% SIMULATE!!!
x = x0;
for k = 1:length(times)-2
    disp(times(k))
    disp(x)
    % Solve
    res = mpc.solve();

    % Check solver status
    if ~strcmp(res.info.status, 'solved')
        error('OSQP did not solve the problem!')
    end

    % Apply first control input to the plant
    ctrl = res.x(n+m+1:n+m+m);
    disp(ctrl)
    x = A(:,:,k+1)*x + B(:,:,k+1)*ctrl;
    
    % Update initial state
    lb(1:n) = -x;
    ub(1:n) = -x;
    mpc.update('l', lb, 'u', ub);
end
disp("Final State")
disp(x)