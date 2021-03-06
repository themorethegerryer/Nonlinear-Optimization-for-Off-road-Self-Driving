function [Xref, Uref, times]=genRefTraj_simplified(x0, xf, u_const, maxVel, dt)
    % x = [uy r ux dPsi e dFzlong delta]
    % u = [deltadot Fxfbrake Fxr Fengine]

    % compute delta between init and goal states
    delta = x0(1:3) - xf(1:3);
    num_steps = floor(norm(delta)*(1/maxVel)*(1/dt));

    % TODO implement yaw different between two vectors
    Xref = zeros([length(x0) num_steps]);
    Uref = zeros([length(u_const) num_steps-1]);
    times = zeros(num_steps,1);

    vx = maxVel;
    vy = 0;

    Xref(:,1) = x0;
    Uref(:,1) = u_const;
    times(1) = 0.0;
    x = x0;
    x(4:5) = [vx vy];
    for k = 2:num_steps-1
       x(1) = x(1) + vx*dt;
       Xref(:,k) = x;
       times(k) = k*dt;
       Uref(:,k) = u_const;
    end
    times(num_steps) = num_steps*dt;
    Xref(:,num_steps) = xf;
    Uref(:,num_steps-1) = [0;0;10];
end