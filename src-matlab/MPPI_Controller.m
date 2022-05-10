function [control, control_Nh] = MPPI_Controller(XDouble, XrefDouble, Uref)
% inputs:
% X: current state of DoubleTrackModel car
% Xref: reference state of DoubleTrackModel car
% Uref: reference control input

    X = [XDouble(1:4) ; XDouble(6:9)];
    Xref = [XrefDouble(1:4) ; XrefDouble(6:9)];

    % define DynamicBicycleModel car
    model = DynamicBicycleModel();
    
    % define hyperparameters
    Nh = 25; %Rollout Horizon
    dt = 0.1;
    n = length(Xref);
    [m, cols] = size(Uref);
%     m = length(Uref);
    gamma = 0.75; % discount factor
    num_samples = 600; % Number of sampled trajectories
    
    umin = [-0.1, -200, 0]';
    umax = [0.1, 200, 0]';
    
    Q = diag([0 0 0 0 0 1 1 0]);
    R = diag([0 0 0]);
    
    x_rollout = zeros(n,Nh);
    x_rollout(:,1) = X;
    costs = zeros(num_samples,1);

%     u_start = Uref.*ones(m,Nh);
    u_start = Uref;
%     u_start = ones(m,Nh);
%     
%     for i=1:m
%         u_start(i,:) = u_start(i,:) * Uref(i);
%     end
    
    u_perturb = zeros(m,Nh,num_samples);
    u_perturb(1,:,:) = ((umax(1)-umin(1)).*rand(num_samples,Nh)+umin(1))';
    u_perturb(2,:,:) = ((umax(2)-umin(2)).*rand(num_samples,Nh)+umin(2))';
    u_perturb(3,:,:) = ((umax(3)-umin(3)).*rand(num_samples,Nh)+umin(3))';

    % compute u_perturb where enginebrake is between -200:-100 and 100:200
    abs_soft_enginebrake = 0.5*100;
    filtered_throttle = u_perturb(2,:,:);
    idx = abs(filtered_throttle)<abs_soft_enginebrake;
    filtered_throttle(idx) = abs_soft_enginebrake*sign(filtered_throttle(idx));
    u_perturb(2,:,:) = filtered_throttle;
    
    for k=1:num_samples
        u_rollout = u_start+u_perturb(:,:,k);
        for kk=1:Nh-1
            x_rollout(:,kk+1) = model.dynamics_rk4(x_rollout(:,kk)', u_rollout(:,kk)', dt);
            costs(k) = costs(k) + gamma^(kk-1)*(0.5*(x_rollout(:,kk+1)-Xref)'*Q*(x_rollout(:,kk+1)-Xref) + 0.5*(u_rollout(:,kk)-Uref(:,kk))'*R*(u_rollout(:,kk)-Uref(:,kk)));
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

    [mean(costs)]
    control_Nh = u_start+(sum(weighted_traj,3)/sum_weights);
    control = control_Nh(:,1);
end