% XDouble = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
function [U, verror, derror] = ctrl2DynamicsPID(X0,vref,deltaref,verror_prev,derror_prev,dt)
    Kpa = 1;
    Kda = 1;
    
    Kpd = 1;
    Kdd = 1;

    v = X0(3)*cos(X(9))+X0(1)*sin(X(9));
    delta = X0(6);
    
    verror = vref - v;
    vnext = Kpv*verror+Kdv*(verror-verror_prev);
    
    derror = deltaref - delta;
    deltanext = Kpd*derror + Kdd*(derror-derror_prev);
    
    U = [(deltanext-delta)/dt;(vnext-v)/dt;0];
end