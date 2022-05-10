function [costVector] = CostFunction(car, CGx, CGy, CGyaw, delta)
% Wheel translations in terms of z height
% Terrain values in terms of y height

% Front wheel yaw is vehicle yaw plus delta (slip angle)
init_parameters = load('init_parameters');
datamap = load('TerrainData').data_map;

% Load in terrain parameters
terrain_x_indices = init_parameters.terrain_x_indices;
terrain_y_indices = init_parameters.terrain_y_indices;
terrain_x_scale = init_parameters.terrain_x_scale;
terrain_y_scale = init_parameters.terrain_y_scale;
terrain_translation = init_parameters.TerrainTranslation;

[CGT, CGR, FLWheelT, FLWheelR, FRWheelT, FRWheelR, BLWheelT, BLWheelR, BRWheelT, BRWheelR] = TireLocator(CGx, CGy, CGyaw, delta, car.a, car.b, car.df, car.dr, car.hcg, 0.2, 0.3, terrain_x_indices, terrain_y_indices, terrain_x_scale, terrain_y_scale, terrain_translation, datamap);
BWheelYaw = CGyaw;
FWheelYaw = CGyaw+delta;

% Grab gradients at tire locations
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

% Calculate gradients between tires
FAxleGrad = abs(norm(FLWheelT(1:2) - FRWheelT(1:2)) / (FLWheelT(3) - FRWheelT(3)));
BAxleGrad = abs(norm(BLWheelT(1:2) - BRWheelT(1:2)) / (BLWheelT(3) - BRWheelT(3)));
LSideGrad = abs(norm(FLWheelT(1:2) - BLWheelT(1:2)) / (FLWheelT(3) - BLWheelT(3)));
RSideGrad = abs(norm(FRWheelT(1:2) - BRWheelT(1:2)) / (FRWheelT(3) - BRWheelT(3)));
FL2BRGrad = abs(norm(FLWheelT(1:2) - BRWheelT(1:2)) / (FLWheelT(3) - BRWheelT(3)));
FR2BLGrad = abs(norm(FRWheelT(1:2) - BLWheelT(1:2)) / (FRWheelT(3) - BLWheelT(3)));

%costVector = [FLGrad, FRGrad, BLGrad, BRGrad, FAxleGrad, BAxleGrad, LSideGrad, RSideGrad];
Terrain_std = std([FLGrad, FRGrad, BLGrad, BRGrad]);
Axle_std = std([FAxleGrad, BAxleGrad]);
Side_std = std([LSideGrad, RSideGrad]);
costVector = [Terrain_std, Axle_std, Side_std];
end