clc
clear all
close all

model = DynamicBicycleModel();

dt = 0.1;

x0 = [0 0 0 0 pi/4 0 0 0]';
Xref = [0.01 0 0.01 0 0 2 2 0]';
Uref = [0 0 0]';

N = 1000;
n = length(Xref);
m = length(Uref);

X = zeros(n,N);
U = zeros(m,N);
X(:,1) = x0;
U(:,1) = Uref;

for k=1:N-1
    disp(k)
    U(:,k+1) = MPPI_Controller(X(:,k), Xref, Uref);
    X(:,k+1) = model.dynamics_rk4(X(:,k)', U(:,k+1)', dt);
%     [X(6,k+1) X(7,k+1)]
end

figure(1)
hold on
plot(X(6,:),X(7,:), '-o')
hold off
xlabel('x position')
ylabel('y position')
title('MPPI')