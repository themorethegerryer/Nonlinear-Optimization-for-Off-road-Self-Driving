classdef KinematicBicycleModel
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
        
        % vehicle and surface properties
        % parameters for dry asphalt - Page 123
        mu_f = 0.873        % Front friction coefficient
        mu_r = 1.03         % Rear friction coefficient
        Calpha_f = 1.76e5   % Front cornering stiffness
        Calpha_r = 2.26e5   % Rear cornering stiffness
        n_f = 8.0           % Front lateral-longitudinal coupling
        n_r = 3.0           % Rear lateral-longitudinal coupling
        
        L
    end
    
    methods
        function car = KinematicBicycleModel()
            % Single Track Model Constructor
            car.L = car.a + car.b; % vehicle's wheelbase
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
        % x = [x y theta]
        % x: global x position
        % y: global y position
        % theta: heading (angle from global x-axis)
        %
        % u = [v alpha]
        % v: forward velocity of back tire
        % alpha: steering angle
            dxdt = zeros(size(x));
            
            theta = x(3);
            
            v = u(1);
            alpha = u(2);
            
            dxdt(1) = v*cos(theta);
            dxdt(2) = v*sin(theta);
            dxdt(3) = v*tan(alpha)/car.L;
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

