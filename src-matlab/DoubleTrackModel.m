classdef DoubleTrackModel
    % Model dynamics for the double track model for the purpose of off-road
    % autonomous driving
    
    properties
        lw = 1.8    % width of car (distance b/w center of tires)
        lr = 2.25   % distance from rear to center of mass
        lf = 2.25   % distance from front to center of mass
        Rw = 0.23   % effective tire radius
        c = 0.04    % rolling friction coefficient
        
    end
    
    methods
        function obj = DoubleTrackModel()
            % Double Track Model Constructor
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

