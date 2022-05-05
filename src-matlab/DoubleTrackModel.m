classdef DoubleTrackModel
    % Model dynamics for the double track model for the purpose of off-road
    % autonomous driving
    % Based on MODEL FIDELITY AND TRAJECTORY PLANNING FOR AUTONOMOUS 
    % VEHICLES AT THE LIMIT by John K. Subosits
    
    properties
        a = 1.19        % Distance from c.g. to front axle (m)
        b = 1.44        % Distance from c.g. to rear axle (m)
        df = 1.54       % Front track width (m)
        dr = 1.52       % Rear track width (m)
        hcg = 0.56      % Center of gravity height (m)
        gamma = 0.64    % Front lateral load transfer fraction
        m = 1776        % Vehicle mass (kg)
        Ixx = 1200      % Moment of inertia about longitudinal axis (kg*m^2)
        Iyy = 3200      % Moment of inertia about lateral axis (kg*m^2)
        Izz = 3950      % Moment of inertia about vertical axis (kg*m^2)
        Klat = 23.2     % Lateral load transfer time constant (1/s)
        Klong = 15.7    % Longitudinal load transfer time constant (1/s)
    end
    
    methods
        function car = DoubleTrackModel()
            % Double Track Model Constructor
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            
        end
        
        function X_n_1 = dynamics_rk4(car,X,U,timeVal,dt)
            uTemp = U;
%             uTemp(1) = uTemp(1) + 0.1*timeVal;
%             if mod(timeVal,3) == 0
%                uTemp(1) = -1 * uTemp(1); 
%             end
            f1 = continuous_dynamics(car, X, uTemp);
            f2 = continuous_dynamics(car, X + 0.5*dt*f1, uTemp);
            f3 = continuous_dynamics(car, X + 0.5*dt*f2, uTemp);
            f4 = continuous_dynamics(car, X + dt*f3, uTemp);
            
            X_n_1 = X + (dt / 6)*(f1 + 2*f2 + 2*f3 + f4);
            X_n_1(isnan(X_n_1)) = 0;
        end
        
        function jac = discrete_jacobian(car, x, u)

        end
        
        function Fwheel = tire_model(car, longSlip, sideSlip, Fz)
            
        end
        
        function slip = slip_function(car, x,u)
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
        end
        
        function theta = ackermann_turn(car, steering_angle)
            radius = car.length*tan((pi/2)-steering_angle);
            if steering_angle < 0
                theta.left = atan(car.length/(radius+(car.lw/2))); % *(180/pi);
                theta.right = atan(car.length/(radius-(car.lw/2))); % *(180/pi);
            else
                theta.right = atan(car.length/(radius+(car.lw/2))); % *(180/pi);
                theta.left = atan(car.length/(radius-(car.lw/2))); % *(180/pi);
            end
        end
    end
end

