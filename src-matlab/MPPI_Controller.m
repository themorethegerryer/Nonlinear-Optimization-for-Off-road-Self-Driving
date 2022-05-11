function [control, control_Nh] = MPPI_Controller(XDouble, XrefDouble, Uref)
% inputs:
% X: current state of DoubleTrackModel car
% Xref: reference state of DoubleTrackModel car
% Uref: reference control input

    % input: 9-long state -> output: 7-long state
%     X = [XDouble(1:3) ; XDouble(6:9)];
%     Xref = [XrefDouble(1:3) ; XrefDouble(6:9)];

    X = XDouble(7:9);
    Xref = XrefDouble(7:9);

%     X = XDouble;
%     Xref = XrefDouble;

    % define DynamicBicycleModel car
    model = KinematicBicycleModel();
    
    % define hyperparameters
    [m, cols] = size(Uref);
    Nh = cols; %Rollout Horizon
    dt = 0.1;
    n = length(Xref);
    
%     m = length(Uref);
    gamma = 0.1; % discount factor
    num_samples = 800; % Number of sampled trajectories
    
    umin = [-pi/4, -20]';
    umax = [pi/4, 20]';
    
    Q = diag([1 1 0]);
    R = diag([0 0]);
    
    x_rollout = zeros(n,Nh);
    x_rollout(:,1) = X;
    costs = zeros(num_samples,1);


%     % compute reference u_start
%     heuristicThrottle = 200;
%     heuristicBrake = -200;
%     distanceFromGoal = sqrt((X(5) - Xref(5))^2+(X(6) - Xref(6))^2);
%     generalSpeed = sqrt(X(3)^2 + X(1)^2);
%     if generalSpeed == 0
%         numStepsToGoal = 1e6;
%     else
%         numStepsToGoal = (distanceFromGoal/generalSpeed)/dt;
%     end
%     uref_internal = zeros(m,Nh);
    u_start = zeros(m,Nh);
%     u_start = Uref;
%     if numStepsToGoal >= Nh
%         u_start(2,:) = ones(1,Nh).*heuristicThrottle;
%     elseif numStepsToGoal > 1 && numStepsToGoal < Nh
% %         brakeIdx = mod(Nh,floor(numStepsToGoal));
%         brakeIdx = floor(numStepsToGoal);
%         u_start(2,:) = ones(1,Nh).*heuristicThrottle;
%         u_start(2,brakeIdx:end) = u_start(2,brakeIdx:end).*-1;
%     else
%         u_start(2,:) = ones(1,Nh).*heuristicBrake;
%     end
% 
%     if X(5) > Xref(5)
%         u_start(2,:) = ones(1,Nh).*heuristicBrake;
%     end

%     u_start = Uref.*ones(m,Nh);
%     u_start = Uref;
%     u_start = ones(m,Nh);
%     
%     for i=1:m
%         for j=1:cols
%             u_start(i,j) = u_start(i,j) * Uref(i,j);
%         end
%     end
    
    u_perturb = zeros(m,Nh,num_samples);
    u_perturb(1,:,:) = ((umax(1)-umin(1)).*rand(num_samples,Nh)+umin(1))';
    u_perturb(2,:,:) = ((umax(2)-umin(2)).*rand(num_samples,Nh)+umin(2))';
%     u_perturb(3,:,:) = ((umax(3)-umin(3)).*rand(num_samples,Nh)+umin(3))';

%     % compute u_perturb where enginebrake is between -200:-100 and 100:200
%     abs_soft_enginebrake = 0.5*umax(2);
%     filtered_throttle = u_perturb(2,:,:);
%     idx = abs(filtered_throttle)<abs_soft_enginebrake;
%     filtered_throttle(idx) = abs_soft_enginebrake*sign(filtered_throttle(idx));
%     u_perturb(2,:,:) = filtered_throttle;
    
    for k=1:num_samples
        u_rollout = u_start+u_perturb(:,:,k);
        for kk=1:Nh-1
            x_rollout(:,kk+1) = model.dynamics_rk4(x_rollout(:,kk)', u_rollout(:,kk)', dt);
            costs(k) = costs(k) + gamma^(kk-1)*(0.5*(x_rollout(:,kk+1)-Xref)'*Q*(x_rollout(:,kk+1)-Xref) + 0.5*(u_rollout(:,kk)-u_start(:,kk))'*R*(u_rollout(:,kk)-u_start(:,kk)));
        end
    end

    % Get control trajectory weights based on cost
    sum_weights = 0.0;
    weighted_traj = zeros(size(u_perturb));
    scaling_factor = mean(costs);
    for uk = 1:num_samples
        weight = exp(-(1/scaling_factor)*costs(uk));
        sum_weights = sum_weights + weight;
        weighted_traj(:,:,uk) = weight*u_perturb(:,:,uk);
    end

%     [mean(costs)]
    control_Nh = u_start+(sum(weighted_traj,3)/sum_weights);
    control = control_Nh(:,1);
%     control = mean(control_Nh,2);
end