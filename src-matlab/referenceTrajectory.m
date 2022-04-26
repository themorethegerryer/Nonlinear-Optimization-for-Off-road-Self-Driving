function [stateRef, timeVect, uRef]=referenceTrajectory(initState, goalState, maxVel, dt)

if initState(2) ~= goalState(2)
   error("Currently testing for forward X-axis reference trajectory"); 
end
if initState(3) ~= goalState(3)
    error("Currently testing for forward X-axis reference trajectory"); 
end

%state:
%{
[x y z theta_fl theta_fr qx qy qz qw linvelx linvely linvelz rotvelx rotvely rotvelz]
%}

% compute delta between init and goal states
delta = goalState(1:3) - initState(1:3);
num_steps = floor(norm(delta)*(1/maxVel)*(1/dt));

% TODO implement yaw different between two vectors
stateRef = initState;
timeVect = 0;
uRef = [0 0];

reachedGoal = false;
vx = maxVel;
vy = 0;
vz = 0;

tempRef = initState;
tempRef(9:11) = [vx vy vz];
count = 0;
while ~reachedGoal
   tempRef(1) = tempRef(1) + vx*dt;
   stateRef = [stateRef ; tempRef];
   timeVect = [timeVect ; timeVect(end)+dt];
   % TODO placeholder for throttle and acceleration
   uRef = [uRef ; [2 0]];
   count = count + 1;
   if count == num_steps
       reachedGoal = true;
   end
end

stateRef = [stateRef ; goalState];
timeVect = [timeVect ; timeVect(end)+dt];
uRef = [uRef ; [2 0]];

end