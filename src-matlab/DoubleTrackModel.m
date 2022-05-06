classdef DoubleTrackModel
    % Model dynamics for the double track model for the purpose of off-road
    % autonomous driving
    % Based on MODEL FIDELITY AND TRAJECTORY PLANNING FOR AUTONOMOUS 
    % VEHICLES AT THE LIMIT by John K. Subosits
    
    properties
        % vehicle properties
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

        % vehicle and surface properties
        % parameters for ice
        Fz0 = 4470      % Nominal normal load (N)
        mu0 = 0.267     % Nominal friction coefficient
        mu_dFz = -0.011 % Friction sensitivity
        c1 = 22.0       % Lateral stiffness parameter
        c2 = 2.14       % Lateral stiffness peak
        n_coup = 4.23   % Lateral-longitudinal coupling
        c8 = 10         % Longitudinal stiffness
    end
    
    methods
        function car = DoubleTrackModel()
            % Double Track Model Constructor
            car.L = car.a + car.b;
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            % TODO equation of motion go here
        end
        
        function x_next = dynamics_rk4(car,x,u,dt)
            uTemp = u;
%             uTemp(1) = uTemp(1) + 0.1*timeVal;
%             if mod(timeVal,3) == 0
%                uTemp(1) = -1 * uTemp(1); 
%             end
            f1 = continuous_dynamics(car, x, uTemp);
            f2 = continuous_dynamics(car, x + 0.5*dt*f1, uTemp);
            f3 = continuous_dynamics(car, x + 0.5*dt*f2, uTemp);
            f4 = continuous_dynamics(car, x + dt*f3, uTemp);
            
            x_next = x + (dt / 6)*(f1 + 2*f2 + 2*f3 + f4);
            x_next(isnan(x_next)) = 0;
        end
        
        function jac = discrete_jacobian(car, x, u)
            % TODO fill out
        end

        function Fz_bundle = tire_fz_function(car, x, Fz)
            Fz_bundle.fl = (((car.b/car.L)*Fz - x(6))/2) - car.gamma*u(7); % eq 4.13
            Fz_bundle.fr = (((car.b/car.L)*Fz - x(6))/2) + car.gamma*u(7); % eq 4.14
            Fz_bundle.rl = (((car.a/car.L)*Fz + x(6))/2) - (1 - car.gamma)*u(7); % eq 4.15
            Fz_bundle.rr = (((car.a/car.L)*Fz + x(6))/2) + (1 - car.gamma)*u(7); % eq 4.16
        end

        function tire_corner_stiffness = cornering_stiffnes(car, tire_mu, Fz, Fx) % eq 4.20
            % TODO replace Fx with the state of the car
            tire_normal_stiffness = car.c1*car.Fz0*sin(2*atan(Fz/(car.c2*car.Fz0))); % eq 4.21
            
            term1 = 0.5*(tire_mu*Fz - Fx);
            term2_1 = (1 - (abs(Fx)/(tire_mu*Fz))^car.n_coup)^(1/car.n_coup);
            term2_2 = tire_normal_stiffness-(0.5*tire_mu*Fz);
            tire_corner_stiffness = term1 + (term2_1*term2_2);
        end

        function tire_mu = tire_friction(car,Fz)
            tire_mu = car.mu0 + (car.mu_dFz)*((Fz - car.Fz0) / car.Fz0); % eq 4.19
        end

        
        function Fwheel = tire_model(car, longSlip, sideSlip, Fz, Fx, u, front_bool) 
            % TODO replace Fx with the state of the car
            % computer lateral force Fy
            tire_mu = tire_friction(car,Fz);
            c_stiffness = cornering_stiffness(car, tire_mu, Fz, Fx);
            Fy_max = sqrt((tire_mu*Fz)^2 - Fx); % eq. 4.17
            
            alpha_slide = (3*Fy_max) / c_stiffness; % eq 4.18


            Fwheel.Fy = -c_stiffness*tan(sideSlip) + (c_stiffness^2 / (3*Fy_max))*tan(sideSlip)*abs(tan(sideSlip));
            if abs(sideSlip) <= alpha_slide
                Fwheel.Fy = Fwheel.Fy - (c_stiffness^3 / (27*Fy_max^2))*tan(sideSlip)^3;
            else
                Fwheel.Fy = Fwheel.Fy - Fy_max*sign(sideSlip);
            end

            % compute longitudinal force Fx
            % assume constant open differential
            torque_engine = u(6);
            if front_bool == true
                Fwheel.Fx_left = torque_engine/2 - u(2);
                Fwheel.Fx_right = torque_engine/2 - u(3);
            else
                Fwheel.Fx_left = 0;
                Fwheel.Fx_right = 0;
            end

        end
        
        function theta = ackermann_turn(car, steering_angle)
            % TODO may not need to use for this model
            radius = car.L*tan((pi/2)-steering_angle);
            if steering_angle < 0
                theta.left = atan(car.L/(radius+(car.df/2))); % *(180/pi);
                theta.right = atan(car.L/(radius-(car.df/2))); % *(180/pi);
            else
                theta.right = atan(car.L/(radius+(car.df/2))); % *(180/pi);
                theta.left = atan(car.L/(radius-(car.df/2))); % *(180/pi);
            end
        end
    end
end

