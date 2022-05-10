clc
clear all
% MPC
% x = [uy r ux dPsi e dFzlong delta]
% u = [deltadot Fxfbrake Fxr Fengine]
% Defined constraints:
%   U: 
%       1: change in steering (+/-1 rad/s)
%       3: braking force 1500 Nm (>= 0)
%       4: torque limit 300 Nm (>= 0)
%   X:
%       7: steering angle +/- 45 deg = pi/4 rad

Nmpc = 30; %Rollout Horizon
max_velocity = 15; % = 33.554mph
dt = 0.1;

x0 = [0 0 10 0 0 0 0]';

% Create double track model
model = SingleTrackModel();

% Create reference trajectory
% [Xref, Uref, times] = genRefTraj_simplified(x0, xf, u_const, max_velocity, dt);
Xref = [0 0 15 0 0 0 0]';
Uref = [0 0 0 0]';
POSref = [30 0 0]';
TERRref = [0 0 0 0 0 0 0 0]';

% n = length(Xref(:,1));
% m = length(Uref(:,1));
% N = length(times);
n = length(Xref);
m = length(Uref);
pn = length(POSref);
tn = length(TERRref);
N = 100;

% Define constraints
xmin = [-Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -pi/4]';
xmax = [Inf, Inf, Inf, Inf, Inf, Inf, pi/4]';
umin = [-1, 0, 0, 0]';
umax = [1, 300, 300, 500]';

% Define LQR costs... but for MPC
Q = diag([1 1 10 10 10 0.1 0.1]);
Qf = Q;
R = diag([0.1 0.1 0.1 0.1]);
Cposition = diag([1 1 0]);
Cterrain = diag([1 1 1 1 1 1 1 1]);

% Get A, B matrices (time-varying)
% A = zeros(n,n,N-1);
% B = zeros(n,m,N-1);
% for k = 1:N-1
%     jac = model.discrete_jacobian(Xref(:,k), Uref(:,k));
%     A(:,:,k) = jac.A;
%     B(:,:,k) = jac.B;
% end
jac = model.discrete_jacobian([1 1 1 1 1 1 1], [1 1 1 1]);
A = jac.A;
B = jac.B;

% Create MPC as OSQP
Np = (Nmpc-1)*(n+m+pn+tn); % number of primals
Nd = (Nmpc-1)*n + (Nmpc-1)*m + (Nmpc-1)*pn + (Nmpc-1)*tn; % number of duals

P = blkdiag(kron(speye(Nmpc-2), blkdiag(sparse(R), sparse(Cterrain), sparse(Cposition), sparse(Q))), sparse(R), sparse(Cterrain), sparse(Cposition), sparse(Qf));

q = zeros((Nmpc-2)*(n+m+pn+tn),1);
% once k+i is less than the length of Xref/Uref, repeat Xref[end]/Uref[end] for remainder
refidx = 0;
for i = 1:Nmpc-2
%     refidx = min(N-1,i);
    q((m+pn+tn+n)*(i-1)+(1:m)) = -R*Uref;%(:,refidx);
    q((m+pn+tn+n)*(i-1)+(m+1:m+tn)) = -Cterrain*TERRref;
    q((m+pn+tn+n)*(i-1)+(m+tn+1:m+tn+pn)) = -Cposition*POSref;
    q((m+pn+tn+n)*(i-1)+(m+tn+pn+1:m+tn+pn+n)) = -Q*Xref;%(:,refidx);
end
q((m+pn+tn+n)*(Nmpc-2)+(m+tn+pn+1:m+tn+pn+n)) = -Qf*Xref;%(:,refidx);

D(1:n,1:m) = B(:,:,1);
D(1:n,(m+1):(m+n)) = -eye(n);
for k = 1:(Nmpc-2)
%     D((k*n)+(1:n), (m+(k-1)*(n+m))+(1:(2*n+m))) = [A(:,:,k+1) B(:,:,k+1) -eye(n)];
    D((k*n)+(1:n), (m+(k-1)*(n+m))+(1:(2*n+m))) = [A B -eye(n)];
end
U = kron(eye(Nmpc-1), [eye(m) zeros(m,n)]);
C = [D;U];

lb_Uref = zeros(m,Nmpc-1);
ub_Uref = zeros(m,Nmpc-1);
lb_Xref = zeros(n,Nmpc-1);
ub_Xref = zeros(n,Nmpc-1);
for k = 1:Nmpc-1
    lb_Uref(:,k) = umin - Uref;%(:,k);
    ub_Uref(:,k) = umax - Uref;%(:,k);
    lb_Xref(:,k) = xmin - Xref;
    ub_Xref(:,k) = xmax - Xref;
end
lb = [reshape(lb_Xref',1,[])'; reshape(lb_Uref',1,[])'];
ub = [reshape(ub_Xref',1,[])'; reshape(ub_Uref',1,[])'];

mpc = osqp;
mpc.setup(P, q, C, lb, ub);


% SIMULATE!!!
x_t = x0;
X = zeros(n,N);
for k = 1:N-1
    disp(N*dt)
    disp(x_t)
    X(:,k) = x_t;
    % Solve
    res = mpc.solve();

    % Check solver status
    if ~strcmp(res.info.status, 'solved')
        error('OSQP did not solve the problem!')
    end

    % Apply first control input to the plant
%     ctrl = res.x(n+m+1:n+m+m);
    ctrl = res.x(1:m);
    disp(ctrl)
%     x_t = A*x_t + B*ctrl;
%     x_t = x_t+model.continuous_dynamics(x_t,ctrl)';
    x_t = model.dynamics_rk4(x_t,ctrl, dt);

    % Update initial state
    lb(1:n) = -x_t;
    ub(1:n) = -x_t;
    mpc.update('l', lb, 'u', ub);
end
X(:,N) = x_t;
disp("Final State|Xf Ref")
disp([x_t xf])
figure(1)
hold on
plot(X(1,:),X(2,:),'-o')
% plot(xf(1),xf(2), 'r-o')
hold off
xlabel('x position')
ylabel('y position')
title('MPC')