function [costVector] = CostFunction(FLWheelT, FRWheelT, BLWheelT, BRWheelT)
% Wheel translations in terms of z height
% Terrain values in terms of y height

% Load in terrain parameters
terrain_x_indices = init_parameters.terrain_x_indices;
terrain_y_indices = init_parameters.terrain_y_indices;
terrain_x_scale = init_parameters.terrain_x_scale;
terrain_y_scale = init_parameters.terrain_y_scale;
terrain_translation = init_parameters.TerrainTranslation;

% Grab gradients at tire locations
% Front Left Wheel
FLx = ceil((FLWheelT(1) - terrain_translation(1)) / terrain_x_scale);
FLy = ceil((FLWheelT(2) - terrain_translation(3)) / terrain_y_scale);
FLAvgGrad = mean(datamap(FLx, FLy, 2:3));
% Front Right Wheel
FRx = ceil((FRWheelT(1) - terrain_translation(1)) / terrain_x_scale);
FRy = ceil((FRWheelT(2) - terrain_translation(3)) / terrain_y_scale);
FRAvgGrad = mean(datamap(FRx, FRy, 2:3));
% Back Left Wheel
BLx = ceil((BLWheelT(1) - terrain_translation(1)) / terrain_x_scale);
BLy = ceil((BLWheelT(2) - terrain_translation(3)) / terrain_y_scale);
BLAvgGrad = mean(datamap(BLx, BLy, 2:3));
% Back Right Wheel
BRx = ceil((BRWheelT(1) - terrain_translation(1)) / terrain_x_scale);
BRy = ceil((BRWheelT(2) - terrain_translation(3)) / terrain_y_scale);
BRAvgGrad = mean(datamap(BRx, BRy, 2:3));

% Calculate gradients between tires
FAxleGrad = abs(norm(FLWheelT(1:2) - FRWheelT(1:2)) / (FLWheelT(3) - FRWheelT(3)));
BAxleGrad = abs(norm(BLWheelT(1:2) - BRWheelT(1:2)) / (BLWheelT(3) - BRWheelT(3)));
LSideGrad = abs(norm(FLWheelT(1:2) - BLWheelT(1:2)) / (FLWheelT(3) - BLWheelT(3)));
RSideGrad = abs(norm(FRWheelT(1:2) - BRWheelT(1:2)) / (FRWheelT(3) - BRWheelT(3)));
FL2BRGrad = abs(norm(FLWheelT(1:2) - BRWheelT(1:2)) / (FLWheelT(3) - BRWheelT(3)));
FR2BLGrad = abs(norm(FRWheelT(1:2) - BLWheelT(1:2)) / (FRWheelT(3) - BLWheelT(3)));

costVector = [FLAvgGrad, FRAvgGrad, BLAvgGrad, BRAvgGrad, FAxleGrad, BAxleGrad, LSideGrad, RSideGrad];
end