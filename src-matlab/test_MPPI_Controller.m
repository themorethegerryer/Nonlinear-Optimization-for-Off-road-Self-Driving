clc
clear all
close all

model = KinematicBicycleModel();

dt = 0.1;

x0 = [0 0 0 0 0 0 0 0 0]';
Xref = [0 0 0 0 0 0 20 10 0]';
x0(end) = atan2(Xref(8), Xref(7));
Uref = [0 0]';

N = 300;
n = length(Xref);
m = length(Uref);

X = zeros(n,N);
U = zeros(m,N);
X(:,1) = x0;
U(:,1) = Uref;

Uhorizon = Uref.*ones(m,30);

for k=1:N-1
    disp(k)
    [U(:,k+1), Uhorizon] = MPPI_Controller(X(:,k), Xref, Uhorizon);
    x_kinBicycle = model.dynamics_rk4(X(7:9,k)', U(:,k+1)', dt);
    X(7:9,k+1) = x_kinBicycle;
%     [X(7,k+1) X(8,k+1) U(1,k+1)]
end

figure(1)
hold on
plot(X(7,:),X(8,:), '-o', Color='blue')
plot(Xref(1), Xref(2), 'x', Color='red')
hold off
xlabel('x position')
ylabel('y position')
title('MPPI')