% Parameter Initialization
% Terrain Parameters
terrain_x_indices = 20;
terrain_y_indices = 20;
terrain_x_scale = 3;
terrain_y_scale = 3;
grad_scale = 0.3; % z + prev_grad * grad_scale
terrain_scale = 0.5;  % z + terrain_scale * gaussian(0,1)
% Car Parameters
tire_width = 0.3;
tire_radius = 0.42;              % effective tire radius (m)
car_lr = 2.25;                  % distance from rear to center of mass (m)
car_lf = 2.25;                  % distance from front to center of mass (m)
car_length = car_lf + car_lr;
car_width = 1.8;                % width of car - distance b/w center of tire (m)
car_height = 1.5;               % height of car (m)
car_elevation = tire_radius;    % height of car undercarriage(m)

% Terrain Generation
starting_zone_buffer = 2.0;
starting_zone = [(car_width + tire_width), car_length + tire_radius*2];
starting_zone = starting_zone * starting_zone_buffer;
data_map = zeros(terrain_x_indices, terrain_y_indices, 3); % [Height, Grad_x, Grad_y]
for y_idx = 1:terrain_y_indices
    for x_idx = 1:terrain_x_indices
        if x_idx-1 <= starting_zone(1) / terrain_x_scale && y_idx-1 <= starting_zone(2) / terrain_x_scale
            data_map(y_idx, x_idx, :) = [0,0,0];
        else
            if x_idx == 1
                prev_y_grad = data_map(y_idx-1, x_idx, 3);
                prev_y_height = data_map(y_idx-1, x_idx, 1);
                z = prev_y_height + prev_y_grad * grad_scale + terrain_scale * normrnd(0,1);
                data_map(y_idx, x_idx, :) = [z, 0, z - prev_y_height];
            elseif y_idx == 1
                prev_x_grad = data_map(y_idx, x_idx-1, 2);
                prev_x_height = data_map(y_idx, x_idx-1, 1);
                z = prev_x_height + prev_x_grad * grad_scale + terrain_scale * normrnd(0,1);
                data_map(y_idx, x_idx, :) = [z, z - prev_x_height, 0];
            else
                avg_grad = (data_map(y_idx, x_idx-1, 2) + data_map(y_idx-1, x_idx, 3)) / 2;
                prev_x_height = data_map(y_idx, x_idx-1, 1);
                prev_y_height = data_map(y_idx-1, x_idx, 1);
                z = (prev_x_height + prev_y_height) / 2 + avg_grad * grad_scale + terrain_scale * normrnd(0,1);
                data_map(y_idx, x_idx, :) = [z, z - prev_x_height, z - prev_y_height];
            end
        end
    end
end
% reshape the data into a one-dimensional array
height_vector = data_map(:,:,1);
height_vector = height_vector(:);

% Open Virtual World and create handles
filename = "TerrainSimulation.x3d";
myworld = vrworld(filename);
open(myworld);
% Create Car handles
% Body
CarBody_node = vrnode(myworld, 'CarBody');
CarBody_shape = vrnode(myworld, 'CarBody_Shape');
CarBody_geo = vrnode(CarBody_shape, 'geometry', 'CarBody_Geometry','Box');
% FLWheel
CarFLWheel_node = vrnode(myworld, 'FLWheel');
CarFLWheel_shape = vrnode(myworld, 'FLWheel_Shape');
CarFLWheel_geo = vrnode(CarFLWheel_shape, 'geometry', 'FLWheel_Geometry','Sphere');
% FRWheel
CarFRWheel_node = vrnode(myworld, 'FRWheel');
CarFRWheel_shape = vrnode(myworld, 'FRWheel_Shape');
CarFRWheel_geo = vrnode(CarFRWheel_shape, 'geometry', 'FRWheel_Geometry','Sphere');
% BLWheel
CarBLWheel_node = vrnode(myworld, 'BLWheel');
CarBLWheel_shape = vrnode(myworld, 'BLWheel_Shape');
CarBLWheel_geo = vrnode(CarBLWheel_shape, 'geometry', 'BLWheel_Geometry','Sphere');
% BRWheel
CarBRWheel_node = vrnode(myworld, 'BRWheel');
CarBRWheel_shape = vrnode(myworld, 'BRWheel_Shape');
CarBRWheel_geo = vrnode(CarBRWheel_shape, 'geometry', 'BRWheel_Geometry','Sphere');
% Create Terrain handles
Terrain_node = vrnode(myworld, 'Terrain');
Terrain_shape = vrnode(myworld, 'Terrain_Shape');
Terrain_appear = vrnode(Terrain_shape, 'appearance', 'Terrain_Appearance', 'Appearance');
Terrain_mat = vrnode(Terrain_appear, 'material', 'Terrain_Material','Material');
Terrain_grid = vrnode(Terrain_shape, 'geometry', 'Terrain_Height','ElevationGrid');
% assign properties for the material field
Terrain_mat.ambientIntensity = 0.25;
Terrain_mat.diffuseColor = [0.9 0.6 0.6];
Terrain_mat.shininess = 0.078125;
Terrain_mat.specularColor = [0.0955906 0.0955906 0.0955906];
% assign properties for the geometry field - use DEM data
Terrain_grid.creaseAngle = 3.14;
Terrain_grid.xDimension = terrain_y_indices;
Terrain_grid.zDimension = terrain_x_indices;
Terrain_grid.xSpacing = terrain_y_scale;
Terrain_grid.zSpacing = terrain_x_scale;
Terrain_grid.height = height_vector;
Terrain_grid.ccw = 'TRUE';
% This setting will make the terrain surface visible from both sides
Terrain_grid.solid = 'FALSE';

% terrain elevation is used to color the image
cmap = demcmap(height_vector, 256);
% create texture subdirectory of the current directory
% output arguments used only to avoid warning message when the directory
% already exists
[~, ~] = mkdir('texture');
% scale the height values to use the full colormap range
% scaling relies on the fact that this terrain begins at zero height
Zscaled = height_vector .* (size(cmap,1)-1) ./ max(height_vector);

% Center terrain on screen
Terrain_node.translation = [-terrain_x_indices/2*terrain_x_scale, 0, -terrain_y_indices/2*terrain_y_scale];
% Update view?

% Update Car
% Body
CarBody_node.translation = [-terrain_x_indices/2*terrain_x_scale+car_length/2+tire_radius, car_elevation+0.5*car_height, -terrain_y_indices/2*terrain_y_scale+0.5*car_width+0.5*tire_width];
CarBody_node.rotation = [0, 0, 1, 0];
CarBody_geo.size = [car_length, car_height, car_width];
% FL Wheel
CarFLWheel_node.translation = [-terrain_x_indices/2*terrain_x_scale+car_length+tire_radius, tire_radius, -terrain_y_indices/2*terrain_y_scale+0.5*tire_width];
CarFLWheel_node.rotation = [1, 0, 0, pi/2];
% CarFLWheel_geo.height = tire_width;
CarFLWheel_geo.radius = tire_radius;
% FR Wheel
CarFRWheel_node.translation = [-terrain_x_indices/2*terrain_x_scale+car_length+tire_radius, tire_radius, -terrain_y_indices/2*terrain_y_scale+car_width+0.5*tire_width];
CarFRWheel_node.rotation = [1, 0, 0, pi/2];
% CarFRWheel_geo.height = tire_width;
CarFRWheel_geo.radius = tire_radius;
% BL Wheel
CarBLWheel_node.translation = [-terrain_x_indices/2*terrain_x_scale+tire_radius, tire_radius, -terrain_y_indices/2*terrain_y_scale+0.5*tire_width];
CarBLWheel_node.rotation = [1, 0, 0, pi/2];
% CarBLWheel_geo.height = tire_width;
CarBLWheel_geo.radius = tire_radius;
% BR Wheel
CarBRWheel_node.translation = [-terrain_x_indices/2*terrain_x_scale+tire_radius, tire_radius, -terrain_y_indices/2*terrain_y_scale+car_width+0.5*tire_width];
CarBRWheel_node.rotation = [1, 0, 0, pi/2];
% CarBRWheel_geo.height = tire_width;
CarBRWheel_geo.radius = tire_radius;

% Place Car in Center of Terrain
% CarBody_node.translation = [car_length/2+tire_radius, car_elevation+0.5*car_height, 0.5*car_width+0.5*tire_width];
% CarFLWheel_node.translation = [car_length+tire_radius, tire_radius, 0.5*tire_width];
% CarFRWheel_node.translation = [car_length+tire_radius, tire_radius, car_width+0.5*tire_width];
% CarBLWheel_node.translation = [tire_radius, tire_radius, 0.5*tire_width];
% CarBRWheel_node.translation = [tire_radius, tire_radius, car_width+0.5*tire_width];

% Save Generated Terrain, Simulation, and Parameters
save(myworld, get(myworld, 'FileName'))
save('TerrainData.mat','data_map')
init_parameters = struct;
% Save car part starting locations
init_parameters.BodyRotation = CarBody_node.rotation;
init_parameters.BodyTranslation = CarBody_node.translation;
init_parameters.FLWheelRotation = CarFLWheel_node.rotation;
init_parameters.FLWheelTranslation = CarFLWheel_node.translation;
init_parameters.FRWheelRotation = CarFRWheel_node.rotation;
init_parameters.FRWheelTranslation = CarFRWheel_node.translation;
init_parameters.BLWheelRotation = CarBLWheel_node.rotation;
init_parameters.BLWheelTranslation = CarBLWheel_node.translation;
init_parameters.BRWheelRotation = CarBRWheel_node.rotation;
init_parameters.BRWheelTranslation = CarBRWheel_node.translation;
% Save terrain parameters
init_parameters.terrain_x_indices = terrain_x_indices;
init_parameters.terrain_y_indices = terrain_y_indices;
init_parameters.terrain_x_scale = terrain_x_scale;
init_parameters.terrain_y_scale = terrain_y_scale;
init_parameters.TerrainTranslation = Terrain_node.translation;

save('init_parameters.mat', '-struct', 'init_parameters')