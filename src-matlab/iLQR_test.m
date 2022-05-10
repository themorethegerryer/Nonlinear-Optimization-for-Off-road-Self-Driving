XDouble = [0 0 0 0 0 0 0 0 0];
XrefDouble = [0 0 0 0 0 0 20 10 0];
U = zeros(2,25);
% x = [x y theta]
% u = [delta v]

% XDouble = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
x = XDouble;
for k = 1:100
    [u0,U,X] = MPC_iLQR(x, XrefDouble, U);
    x = [u0(2)*cos(X(3,2)) 0 u0(2)*sin(X(3,2)) 0 0 u0(1) X(1,2) X(2,2) X(3,2)];
    figure(1)
    clf(1)
    axis([-10 30 -10 30])
    plot(X(1,:),X(2,:),'-o'); hold on; plot(xg(1),xg(2),'r-o');
    pause(0.1)
    disp(k)
end