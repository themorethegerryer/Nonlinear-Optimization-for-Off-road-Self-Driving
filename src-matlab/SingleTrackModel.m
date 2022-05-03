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
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
            % x = [uy r ux dPsi e dFzlong delta]
            % u = [deltadot Fxfbrake Fxr Fengine]            
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

