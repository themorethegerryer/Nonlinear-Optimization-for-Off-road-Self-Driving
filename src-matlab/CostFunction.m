function [costVector, costGrad, costHess] = CostFunction(X0, car)
% Wheel translations in terms of z height
% Terrain values in terms of y height

% Do we need to calculate the cost, cost grad, and cost hessian with
% respect to the control input?

% X0 = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
delta = X0(6);
CGx = X0(7);
CGy = X0(8);
CGyaw = X0(9);

% Front wheel yaw is vehicle yaw plus delta (slip angle)
init_parameters = load('init_parameters');
datamap = load('TerrainData').data_map;

% Load in terrain parameters
terrain_x_indices = init_parameters.terrain_x_indices;
terrain_y_indices = init_parameters.terrain_y_indices;
terrain_x_scale = init_parameters.terrain_x_scale;
terrain_y_scale = init_parameters.terrain_y_scale;
terrain_translation = init_parameters.TerrainTranslation;

[State3d, CGT, CGR, FLWheelT, FLWheelR, FRWheelT, FRWheelR, BLWheelT, BLWheelR, BRWheelT, BRWheelR] = TireLocator(CGx, CGy, CGyaw, delta, car.a, car.b, car.df, car.dr, car.hcg, 0.2, 0.3, terrain_x_indices, terrain_y_indices, terrain_x_scale, terrain_y_scale, terrain_translation, datamap);
BWheelYaw = CGyaw;
FWheelYaw = CGyaw+delta;

[FLGrad, FRGrad, BLGrad, BRGrad, FAxleGrad, BAxleGrad, LSideGrad, RSideGrad, FL2BRGrad, FR2BLGrad] = GradCalc(FLWheelT, FRWheelT, BLWheelT, BRWheelT, FWheelYaw, BWheelYaw, terrain_translation, terrain_x_scale, terrain_y_scale);

% % Grab gradients at tire locations
% % Front Left Wheel
% FLx = ceil((FLWheelT(1) - terrain_translation(1)) / terrain_x_scale);
% FLy = ceil((FLWheelT(2) - terrain_translation(3)) / terrain_y_scale);
% FLGrad = cos(FWheelYaw) * datamap(FLx, FLy, 2) + sin(FWheelYaw) * datamap(FLx, FLy, 3);
% % Front Right Wheel
% FRx = ceil((FRWheelT(1) - terrain_translation(1)) / terrain_x_scale);
% FRy = ceil((FRWheelT(2) - terrain_translation(3)) / terrain_y_scale);
% FRGrad = cos(FWheelYaw) * datamap(FRx, FRy, 2) + sin(FWheelYaw) * datamap(FRx, FRy, 3);
% % Back Left Wheel
% BLx = ceil((BLWheelT(1) - terrain_translation(1)) / terrain_x_scale);
% BLy = ceil((BLWheelT(2) - terrain_translation(3)) / terrain_y_scale);
% BLGrad = cos(BWheelYaw) * datamap(BLx, BLy, 2) + sin(BWheelYaw) * datamap(BLx, BLy, 3);
% % Back Right Wheel
% BRx = ceil((BRWheelT(1) - terrain_translation(1)) / terrain_x_scale);
% BRy = ceil((BRWheelT(2) - terrain_translation(3)) / terrain_y_scale);
% BRGrad = cos(BWheelYaw) * datamap(BRx, BRy, 2) + sin(BWheelYaw) * datamap(BRx, BRy, 3);

% % Calculate gradients between tires
% FAxleGrad = abs(norm(FLWheelT(1:2) - FRWheelT(1:2)) / (FLWheelT(3) - FRWheelT(3)));
% BAxleGrad = abs(norm(BLWheelT(1:2) - BRWheelT(1:2)) / (BLWheelT(3) - BRWheelT(3)));
% LSideGrad = abs(norm(FLWheelT(1:2) - BLWheelT(1:2)) / (FLWheelT(3) - BLWheelT(3)));
% RSideGrad = abs(norm(FRWheelT(1:2) - BRWheelT(1:2)) / (FRWheelT(3) - BRWheelT(3)));
% FL2BRGrad = abs(norm(FLWheelT(1:2) - BRWheelT(1:2)) / (FLWheelT(3) - BRWheelT(3)));
% FR2BLGrad = abs(norm(FRWheelT(1:2) - BLWheelT(1:2)) / (FRWheelT(3) - BLWheelT(3)));

% Calculate the mean cost
%costVector = [FLGrad, FRGrad, BLGrad, BRGrad, FAxleGrad, BAxleGrad, LSideGrad, RSideGrad];
Terrain_std = std([FLGrad, FRGrad, BLGrad, BRGrad]);
Axle_std = std([FAxleGrad, BAxleGrad]);
Side_std = std([LSideGrad, RSideGrad]);
costVector = [Terrain_std, Axle_std, Side_std];

% Calculate the cost derivative for each state numerically
costGrad = zeros(length(costVector), length(x0));
costGrad2 = zeros(length(costVector), length(x0));
costHess = zeros(length(costVector), length(x0));
derivative_step = 0.1;
for jj=1:2
    for ii=1:length(x0)
        temp_x0 = x0;
        if jj == 0
            temp_x0(ii) = temp_x0(ii) + derivative_step;
        else
            temp_x0(ii) = temp_x0(ii) + 2 * derivative_step;
        end
        % Calculate the stepped tire positions
        delta = temp_x0(6);
        CGx = temp_x0(7);
        CGy = temp_x0(8);
        CGyaw = temp_x0(9);
        [State3d, CGT, CGR, FLWheelT, FLWheelR, FRWheelT, FRWheelR, BLWheelT, BLWheelR, BRWheelT, BRWheelR] = TireLocator(CGx, CGy, CGyaw, delta, car.a, car.b, car.df, car.dr, car.hcg, 0.2, 0.3, terrain_x_indices, terrain_y_indices, terrain_x_scale, terrain_y_scale, terrain_translation, datamap);
        BWheelYaw = CGyaw;
        FWheelYaw = CGyaw+delta;
        % Calculate the stepped gradients
        [FLGrad, FRGrad, BLGrad, BRGrad, FAxleGrad, BAxleGrad, LSideGrad, RSideGrad, FL2BRGrad, FR2BLGrad] = GradCalc(FLWheelT, FRWheelT, BLWheelT, BRWheelT, FWheelYaw, BWheelYaw, terrain_translation, terrain_x_scale, terrain_y_scale);
        % Calculate the stepped cost
        Terrain_std = std([FLGrad, FRGrad, BLGrad, BRGrad]);
        Axle_std = std([FAxleGrad, BAxleGrad]);
        Side_std = std([LSideGrad, RSideGrad]);
        stepped_costVector = [Terrain_std, Axle_std, Side_std];
        if jj == 0
            costGrad(1:end,ii) = (stepped_costVector' - costVector') / derivative_step;
        else
            costGrad2(1:end,ii) = (stepped_costVector' - costVector') / (2 * derivative_step);
        end
    end
    costHess = (costGrad2 - costGrad) / derivative_step;
end

%%% Attempts at Analytical cost differentiation

% % x = [uy r ux dFzlong delta x y yaw]
% % y-velocity, yaw-dot, x-velocity, longitudinal load transfer, steering
% % angle, x, y, yaw
% % u = [deltadot Fxf_enginebrake Fxr]
% % yaw-dot, engine force, rear braking force
% % Costs: Terrain_std, Axle_std, Side_std
% 
% % Terrain_std = std([FLGrad, FRGrad, BLGrad, BRGrad]);
% % FLGrad = cos(FWheelYaw) * datamap(FLx, FLy, 2) + sin(FWheelYaw) * datamap(FLx, FLy, 3);
% dFLGrad_dyaw = -sin(yaw+delta)*datamap(FLx, FLy, 2) + cos(yaw+delta)*datamap(FLx, FLy, 3);
% dFLGrad_dr = -cos(yaw+delta)*datamap(FLx, FLy, 2) - sin(yaw+delta)*datamap(FLx, FLy, 3);
% d2FLGrad_dr2 = sin(yaw+delta)*datamap(FLx, FLy, 2) - cos(yaw+delta)*datamap(FLx, FLy, 3);
% dFLGrad_dx = [0 dFLGrad_dr 0 0 dFLGrad_dyaw 0 0 dFLGrad_dyaw];
% d2FLGrad_dx2 = [0 d2FLGrad_dr2 0 0 dFLGrad_dr 0 0 dFLGrad_dr];
% % How does this convert to standard deviation?
% % std = sqrt((SUM(x - mu))^2 / N)
% dstd/dx = 
% dTerrain_std__duy = 0;
% dTerrain_std__dr = ;
% 
% Grad_Terrain_std = 0;
% Grad_Axle_std = std([FLGrad, FRGrad]) + std([BLGrad, BRGrad]);
% Grad_Side_std = std([FLGrad, BLGrad]) + std([FRGrad, BRGrad]);
% Hess_Terrain_std = 0;
% Hess_Axle_std = 0;
% Hess_Side_std = 0;
end

function [FLGrad, FRGrad, BLGrad, BRGrad, FAxleGrad, BAxleGrad, LSideGrad, RSideGrad, FL2BRGrad, FR2BLGrad] = GradCalc(FLWheelT, FRWheelT, BLWheelT, BRWheelT, FWheelYaw, BWheelYaw, terrain_translation, terrain_x_scale, terrain_y_scale)
% Grab terrain gradients at tire locations
% Front Left Wheel
FLx = ceil((FLWheelT(1) - terrain_translation(1)) / terrain_x_scale);
FLy = ceil((FLWheelT(2) - terrain_translation(3)) / terrain_y_scale);
FLGrad = cos(FWheelYaw) * datamap(FLx, FLy, 2) + sin(FWheelYaw) * datamap(FLx, FLy, 3);
% Front Right Wheel
FRx = ceil((FRWheelT(1) - terrain_translation(1)) / terrain_x_scale);
FRy = ceil((FRWheelT(2) - terrain_translation(3)) / terrain_y_scale);
FRGrad = cos(FWheelYaw) * datamap(FRx, FRy, 2) + sin(FWheelYaw) * datamap(FRx, FRy, 3);
% Back Left Wheel
BLx = ceil((BLWheelT(1) - terrain_translation(1)) / terrain_x_scale);
BLy = ceil((BLWheelT(2) - terrain_translation(3)) / terrain_y_scale);
BLGrad = cos(BWheelYaw) * datamap(BLx, BLy, 2) + sin(BWheelYaw) * datamap(BLx, BLy, 3);
% Back Right Wheel
BRx = ceil((BRWheelT(1) - terrain_translation(1)) / terrain_x_scale);
BRy = ceil((BRWheelT(2) - terrain_translation(3)) / terrain_y_scale);
BRGrad = cos(BWheelYaw) * datamap(BRx, BRy, 2) + sin(BWheelYaw) * datamap(BRx, BRy, 3);

% Calculate Wheel Gradients
FAxleGrad = abs(norm(FLWheelT(1:2) - FRWheelT(1:2)) / (FLWheelT(3) - FRWheelT(3)));
BAxleGrad = abs(norm(BLWheelT(1:2) - BRWheelT(1:2)) / (BLWheelT(3) - BRWheelT(3)));
LSideGrad = abs(norm(FLWheelT(1:2) - BLWheelT(1:2)) / (FLWheelT(3) - BLWheelT(3)));
RSideGrad = abs(norm(FRWheelT(1:2) - BRWheelT(1:2)) / (FRWheelT(3) - BRWheelT(3)));
FL2BRGrad = abs(norm(FLWheelT(1:2) - BRWheelT(1:2)) / (FLWheelT(3) - BRWheelT(3)));
FR2BLGrad = abs(norm(FRWheelT(1:2) - BLWheelT(1:2)) / (FRWheelT(3) - BLWheelT(3)));
end