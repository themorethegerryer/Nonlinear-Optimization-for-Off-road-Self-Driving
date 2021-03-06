function [State3d, CGT, CGR, FLWheelT, FLWheelR, FRWheelT, FRWheelR, BLWheelT, BLWheelR, BRWheelT, BRWheelR] = TireLocator(CGx, CGy, CGyaw, delta, CarA, CarB, CarDf, CarDr, CarCGH, tire_width, tire_radius, terrain_x_indices, terrain_y_indices, terrain_x_scale, terrain_y_scale, terrain_translation, data_map)
% T: x, y, z
% R: x, y, z, w
% All variables in terms of "normal" axes (z: height) EXCEPT for
% terrain_translation, which is "Matlab VR" axes (y: height)

% Load in terrain parameters
% terrain_x_indices = init_parameters.terrain_x_indices;
% terrain_y_indices = init_parameters.terrain_y_indices;
% terrain_x_scale = init_parameters.terrain_x_scale;
% terrain_y_scale = init_parameters.terrain_y_scale;
% terrain_translation = init_parameters.TerrainTranslation;
% terrain_x_indices = evalin('base', 'init_parameters.terrain_x_indices');
% terrain_y_indices = evalin('base', 'init_parameters.terrain_y_indices');
% terrain_x_scale = evalin('base', 'init_parameters.terrain_x_scale');
% terrain_y_scale = evalin('base', 'init_parameters.terrain_y_scale');
% terrain_translation = evalin('base', 'init_parameters.TerrainTranslation');


%%%%%
% Method A: Assuming that the center of gravity is perfectly aligned with
% the terrain directly underneath.  It is "flat" on the ground.

% Tires in body frame
% Front Left Wheel
B_FLWheelT = [CarA, CarDf / 2 + tire_width / 2, -CarCGH / 2];
B_FLWheelRotm = eul2rotm([pi/2, 0, delta],'XYZ'); % First rotation set y to be vertical, second follows delta angle
B_FLWheelH = [B_FLWheelRotm, B_FLWheelT'; 0, 0, 0, 1];
% Front Right Wheel
B_FRWheelT = [CarA, - CarDf / 2 - tire_width / 2, -CarCGH / 2];
B_FRWheelRotm = eul2rotm([pi/2, 0, delta],'XYZ'); % First rotation set y to be vertical, second follows delta angle
B_FRWheelH = [B_FRWheelRotm, B_FRWheelT'; 0, 0, 0, 1];
% Back Left Wheel
B_BLWheelT = [-CarB, CarDr / 2 + tire_width / 2, -CarCGH / 2];
B_BLWheelRotm = eul2rotm([pi/2, 0, 0],'XYZ');
B_BLWheelH = [B_BLWheelRotm, B_BLWheelT'; 0, 0, 0, 1];
% Back Right Wheel
B_BRWheelT = [-CarB, - CarDr / 2 - tire_width / 2, -CarCGH / 2];
B_BRWheelRotm = eul2rotm([pi/2, 0, 0],'XYZ');
B_BRWheelH = [B_BRWheelRotm, B_BRWheelT'; 0, 0, 0, 1];

% Center of Gravity (of body frame) in global frame
CGx_indices = (CGx - terrain_translation(1)) / terrain_x_scale;
CGy_indices = (CGy - terrain_translation(3)) / terrain_y_scale;
CGz = 0;
CGx_indices;
CGy_indices;
if CGx_indices < 1
    CGx_indices = 1;
elseif CGx_indices > terrain_x_indices
    CGx_indices = terrain_x_indices;
end
if CGy_indices < 1
    CGy_indices = 1;
elseif CGy_indices > terrain_y_indices
    CGy_indices = terrain_y_indices;
end
upper_left_z = data_map(floor(CGx_indices), ceil(CGy_indices), 1);
lower_right_z = data_map(ceil(CGx_indices), floor(CGy_indices), 1);
if CGx_indices - floor(CGx_indices) + CGy_indices - floor(CGy_indices) <= 1
    % CG is in lower-left half of planar triangle
    lower_left_z = data_map(floor(CGx_indices), floor(CGy_indices), 1);
    x_ratio = (1.0 - (CGx_indices - floor(CGx_indices)));
    y_ratio = (1.0 - (CGy_indices - floor(CGy_indices)));
    CGz = lower_left_z + x_ratio * (lower_right_z - lower_left_z) + y_ratio * (upper_left_z - lower_left_z);
    x_angle = atan((lower_right_z - lower_left_z) / terrain_x_scale);
    y_angle = atan((upper_left_z - lower_left_z) / terrain_y_scale);
else
    % CG is in upper-right half of planar triangle
    upper_right_z = data_map(ceil(CGx_indices), ceil(CGy_indices), 1);
    x_ratio = ceil(CGx_indices) - CGx_indices;
    y_ratio = ceil(CGy_indices) - CGy_indices;
    CGz = upper_right_z - x_ratio * (upper_right_z - upper_left_z) - y_ratio * (upper_right_z - lower_right_z);
    x_angle = atan((upper_right_z - upper_left_z) / terrain_x_scale);
    y_angle = atan((upper_right_z - lower_right_z) / terrain_y_scale);
end
B2GT = [CGx, CGy, CGz + tire_radius + CarCGH / 2];
x_angle;
y_angle;
CGpitch = cos(CGyaw) * x_angle + sin(CGyaw) * y_angle;
CGroll = sin(CGyaw) * x_angle + cos(CGyaw) * y_angle;
YawMat = eul2rotm([-CGyaw, 0, 0], 'ZYX');
Pitchmat = eul2rotm([0, -CGpitch, 0], 'ZYX');
Rollmat = eul2rotm([0, 0, CGroll], 'ZYX');
B2GR = Rollmat * Pitchmat * YawMat;
B2GH = [B2GR, B2GT'; 0, 0, 0, 1];
CGT = B2GT;
YawMat = eul2rotm([CGyaw, 0, 0], 'ZYX');
Pitchmat = eul2rotm([0, CGpitch, 0], 'ZYX');
Rollmat = eul2rotm([0, 0, -CGroll], 'ZYX');
B2GR = Rollmat * Pitchmat * YawMat;
CGR = rotm2axang(B2GR);

% Send roll and pitch information
DpitchDyaw = -sin(CGyaw) * x_angle + cos(CGyaw) * y_angle;
DrollDyaw = cos(CGyaw) * x_angle - sin(CGyaw) * y_angle;
State3d = [-CGpitch; CGroll; -DpitchDyaw; DrollDyaw];

% Tires in global frame
% Front left wheel
G_FLWheelH = B2GH * B_FLWheelH;
FLWheelT = G_FLWheelH(1:3,4);
FLWheelR = rotm2axang(G_FLWheelH(1:3,1:3));
FLWheelR(3) = -FLWheelR(3);
% Front right wheel
G_FRWheelH = B2GH * B_FRWheelH;
FRWheelT = G_FRWheelH(1:3,4);
FRWheelR = rotm2axang(G_FRWheelH(1:3,1:3));
FRWheelR(3) = -FRWheelR(3);
% Back left wheel
G_BLWheelH = B2GH * B_BLWheelH;
BLWheelT = G_BLWheelH(1:3,4);
BLWheelR = rotm2axang(G_BLWheelH(1:3,1:3));
BLWheelR(3) = -BLWheelR(3);
% Front right wheel
G_BRWheelH = B2GH * B_BRWheelH;
BRWheelT = G_BRWheelH(1:3,4);
BRWheelR = rotm2axang(G_BRWheelH(1:3,1:3));
BRWheelR(3) = -BRWheelR(3);
end