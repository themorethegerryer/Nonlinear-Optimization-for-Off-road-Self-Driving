function X1 = DoubleTrackDynamicStepper(X0, U, ppitch, qroll, p, q)

single_track_car = DoubleTrackModel();

% x = [uy r ux dPsi e dFzlong delta]
% u = [deltadot Fxfbrake Fxr Fengine]

% step through dynamics 
dt = 0.1;
% xpos = zeros(size(t));
% ypos = zeros(size(t));

X1 = single_track_car.dynamics_rk4(X0,U,dt, ppitch, qroll, p, q);
% X1 = X0 + single_track_car.continuous_dynamics(X0, U)*dt;
% xVect = [xVect ; X_n_1];
% xpos(i+1) = xpos(i) + X_n_1(3)*dt;
% ypos(i+1) = ypos(i) + X_n_1(1)*dt;
end