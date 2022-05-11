XDouble = [0 0 0 0 0 0 0 0 0];
XrefDouble = [0 0 0 0 0 0 100 100 pi/2];
N = 50;
U = zeros(2,N-1);
% x = [x y theta]
% u = [delta v]
xg = XrefDouble(7:9);
% XDouble = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
x = XDouble;
for k = 1:60
    [u0,U,X] = MPC_iLQR(x, XrefDouble, U);
%     U = U(:,2:end);
    x = [u0(2)*cos(X(3,2)) 0 u0(2)*sin(X(3,2)) 0 0 u0(1) X(1,2) X(2,2) X(3,2)];
    figure(1)
    clf(1)
    hold on
    plot(X(1,:),X(2,:),'y-o')
    plot(xg(1),xg(2),'r-o')
    plot(X(1,1),X(2,1),'b-o')
    axis([-5 105 -5 105])
    xlabel("x")
    ylabel("y")
    t = (k-1)*0.1;
    title("t = "+t)
    pause;%(0.1)
end