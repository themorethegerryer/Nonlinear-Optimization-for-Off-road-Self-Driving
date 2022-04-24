classdef DoubleTrackModel
    % Model dynamics for the double track model for the purpose of off-road
    % autonomous driving
    
    properties
        m = 2000    % mass of the car (kg)
        mw = 10     % mass of each tire (kg) 
        lw = 1.8    % width of car - distance b/w center of tire (m)
        lr = 2.25   % distance from rear to center of mass (m)
        lf = 2.25   % distance from front to center of mass (m)
        h = 1.5     % height of car (m)
        h_CoM = 0.5 % height of car's center of mass (m)  
        Rw = 0.23   % effective tire radius (m)
        c = 0.04    % rolling friction coefficient
        
        length
        Iz
        Iy
        Iwheel
    end
    
    methods
        function car = DoubleTrackModel(m, mw, lw, lr, lf, Rw, c)
            % Double Track Model Constructor
            if nargin < 2
                car.m = 2000;
                car.mw = 10;
                car.lw = 1.8;
                car.lr = 2.25;
                car.lf = 2.25;
                car.Rw = 0.23;
                car.c = 0.04;
            else
                car.m = m;
                car.mw = mw;
                car.lw = lw;
                car.lr = lr;
                car.lf = lf;
                car.Rw = Rw;
                car.c = c;
            end
            car.length = car.lr+car.lf;
            car.Iz = (car.m)*(car.length^2 + car.lw^2)/12;
            car.Iy = (car.m)*(car.length^2 + car.height^2)/12;
            car.Iwheel = (car.mw)*((car.Rw/2)^2)/2; %effective radius divided by 2 because tire mass is focused at center
        end
        
        function dynamics(car,X,U,dt)
            steering_angle = U[1];
            torque = U[2];
            w_tire = zeros(1,4)
            for tire=1:4
                F_weight = -car.m*9.81
                F_moment_pair = car.Iy*car.h_CoM*
                Fr = weight_on_tire*car.c
                w_tire = torque-Fr*car.Rw %F_xi assumed to be zero
            end
            
        end
        
        function cost(car,X,U,Q,R,Qf)
            
        end
        
        function theta = ackermann_turn(car, steering_angle)
            radius = car.length*tan((pi/2)-steering_angle);
            if steering_angle < 0
                theta.left = atan(car.length/(radius+(car.lw/2)))*(180/pi);
                theta.right = atan(car.length/(radius-(car.lw/2)))*(180/pi);
            else
                theta.right = atan(car.length/(radius+(car.lw/2)))*(180/pi);
                theta.left = atan(car.length/(radius-(car.lw/2)))*(180/pi);
            end
        end
    end
end

