classdef SingleTrackModel
    % Model dynamics for the single track model for the controller state
    % Based on MODEL FIDELITY AND TRAJECTORY PLANNING FOR AUTONOMOUS 
    % VEHICLES AT THE LIMIT by John K. Subosits

    
    properties
        a = 1.19        % Distance from c.g. to front axle (m)
        b = 1.44        % Distance from c.g. to rear axle (m)
        df = 0          % Front track width (m) - 0 for single track
        dr = 0          % Rear track width (m) - 0 for single track
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
        function car = SingleTrackModel()
            % Single Track Model Constructor
            car.L = car.a + car.b; % vehicle's wheelbase
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
        % x = [uy r ux dPsi e dFzlong delta]
        % uy: vehicle's lateral velocity ~ 1 m/s
        % r: yaw rate ~ 0.1 rad/s
        % ux: vehicle's longitudinal velocity (straight forward) ~ 1 m/s
        % dPsi = z-axis rotation difference between body frame and path (angle between b_x and p_x) ~ 0.01 rad
        % e: distance from path to car's c.g. in p_y direction (e*p_y = distance from path) ~ 1 m
        % dFzlong: longitudinal load transfer ~ 500 N
        % delta: current steering angle ~ 0.1 rad
        %
        % u = [deltadot Fxfbrake Fxr Fengine]
        % deltadot: change in steering angle ~ 0.1 rad/s
        % Fxfbrake: Front brake force ~ 500 N
        % Fxr: Rear driving force ~ 300 N
        % Fengine: Torque applied to rear tires by engine ~ 500 N
            dxdt = zeros(size(x));

            % Pull state and control variables out for easy equation reference
            uy = x(1);
            r = x(2);
            ux = x(3);
            dPsi = x(4);
            e = x(5);
            dFzlong = x(6);
            delta = x(7);

            deltadot = u(1);
            Fxfbrake = u(2);
            Fxr = u(3);
            Fengine = u(4);

            % Pre-calcuate variables needed in equations
            p = 1;% roll rate
            pdot = 1; % roll acceleration
            q = 1;% pitch rate
            qdot = 1; % pitch acceleration
            uz = 1;% vertical velocity
            k = 1; % curvature describing how path tangent roates about the normal to the road surface with distance traveled along the path - Page 33

            % g = g_x*b_x + g_y*b_y + g_z*b_z, gravity in the body frame
            gx = 0;
            gy = 9.81;
            gz = 0;

            Fyf = 1;
            Fyr = 1;
            Fxf = 1;

            Fdrag = 0;

            % duy/dt - Page 95
            dxdt(1) = ((Fyf*cos(delta) + Fyr + Fxf*sin(delta))/car.m)-r*ux+p*uz+gy;

            % dr/dt - Page 95
            % For single track, Fxrr = Fxrl, Fxfr = Fxfl, and Fyfl = Fyfr so 2nd and 3rd terms are 0.
            dxdt(2) = ((car.a*(Fyf*cos(delta)+Fxf*sin(delta)) - car.b*Fyr)/car.Izz) + (((car.Ixx - car.Iyy)*p*q)/car.Izz);

            % dux/dt - Page 95
            dxdt(3) = ((Fxf*cos(delta)+Fxr-Fyf*sin(delta)-Fdrag)/car.m) + r*uy-q*uz+gx;

            % ddPsi/dt - Page 96
            dxdt(4) = r - ((ux*cos(dPsi)-uy*sin(dPsi))/(1-k*e))*k;

            % de/dt - Page 96
            dxdt(5) = ux*sin(dPsi)+uy*cos(dPsi);

            % ddFzlong/dt - Page 96
            %confirm that h = hcg here
            dxdt(6) = -car.Klong(dFzlong+((p*r*(car.Izz-car.Ixx))/car.L)-((car.hcg*(Fxf*cos(delta)+Fxr-Fyf*sin(delta))+car.Iyy*qdot)/car.L));

            % ddelta/dt
            dxdt(7) = delta + deltadot;
        end
        
        function x_next = dynamics_rk4(car,x,u,dt)
            f1 = continuous_dynamics(car, x, u);
            f2 = continuous_dynamics(car, x + 0.5*dt*f1, u);
            f3 = continuous_dynamics(car, x + 0.5*dt*f2, u);
            f4 = continuous_dynamics(car, x + dt*f3, u);
            
            x_next = x + (dt / 6)*(f1 + 2*f2 + 2*f3 + f4);
        end
        
        function jac = discrete_jacobian(car, x, u)
            % Calculate discrete Jacobian of car.continuous_dynamics at
            % given x and u
            % Standard finite forward difference method
            epsilon = 0.1;
            epsilon_inv = 1/(epsilon); % (1 / (2*epsilon))
            nx = length(x); % Dimension of the input x;
            f0 = car.continuous_dynamics(x,u);
            jac.A = zeros(length(f0), nx);
            % Do perturbation
            for i = 1 : nx
                xplus = x;
            %     xminus = x;
                xplus(i) =  x(i) + epsilon;
            %     xminus(i) = x(i) - epsilon;
                jac.A(:, i) = (car.continuous_dynamics(xplus,u) - f0) .* epsilon_inv;
            end

            jac.B = zeros(length(f0), length(u));

            for i = 1 : length(u)
                uplus = u;
            %     uminus = u;
                uplus(i) =  u(i) + epsilon;
            %     uminus(i) = u(i) - epsilon;
                jac.B(:,i) = (car.continuous_dynamics(x,uplus) - f0) .* epsilon_inv;
            end

        end
    end
end

