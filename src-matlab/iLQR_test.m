XDouble = [0 0 0 0 0 0 0 0 0];
XrefDouble = [0 0 0 0 0 0 20 10 0];
N = 100;
U = zeros(2,N-1);
% x = [x y theta]
% u = [delta v]

% XDouble = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
x = XDouble;
for k = 1:N-2
    [U,X] = MPC_iLQR(x, XrefDouble, U);
    u0 = U(:,1);
    U = U(:,2:end);
    x = [u0(2)*cos(X(3,2)) 0 u0(2)*sin(X(3,2)) 0 0 u0(1) X(1,2) X(2,2) X(3,2)];
    figure(1)
    clf(1)
    hold on
    plot(X(1,1),X(2,2),'b-o')
    plot(X(1,:),X(2,:),'y-o')
    plot(xg(1),xg(2),'r-o')
    axis([-10 30 -10 30])
    pause(0.1)
end