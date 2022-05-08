function [Xref, Uref, times]=referenceTrajectory(init_pos, goal_pos, maxVel, dt)
    % x = [uy r ux dPsi e dFzlong delta]
    % u = [deltadot Fxfbrake Fxr Fengine]
    
    delta_pos = goal_pos - init_pos;
    k=0;
    
    

end