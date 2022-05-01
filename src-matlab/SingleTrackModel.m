classdef SingleTrackModel
    % Model dynamics for the single track model for the controller state
    
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
        w_wheel = 0.3 % width of tire (m) (i.e. average is 12.5 inches)
        
        length
        Iz
        Iy
        Iwheel
        
    end
    
    methods
        function car = SingleTrackModel(m, mw, lw, lr, lf, Rw, c)
            % Single Track Model Constructor
            if nargin < 2
                car.m = 2000;
                car.mw = 10;
                car.lw = 1.8;
                car.lr = 2.25;
                car.lf = 2.25;
                car.Rw = 0.23;
                car.c = 0.04;
                car.w_wheel = 0.3;
            else
                car.m = m;
                car.mw = mw;
                car.lw = lw;
                car.lr = lr;
                car.lf = lf;
                car.Rw = Rw;
                car.c = c;
                car.w_wheel = 0.3;
            end
            car.length = car.lr+car.lf;
            car.Iz = (car.m)*(car.length^2 + car.lw^2)/12;
            car.Iy = (car.m)*(car.length^2 + car.h^2)/12;
            car.Iwheel = (car.mw)*((car.Rw/2)^2)/2; %effective radius divided by 2 because tire mass is focused at center
        end
        
        function dxdt = continuous_dynamics(car, state, controls)% ax, ay, rdot, wdot
            % state = [x y r xdot ydot wfl wfr wrl wrr]
            % dxdt = [xdot ydot rdot xdotdot ydotdot wfldot wfrdot wrldot wrrdot]
            
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
            % Calculate discrete Jacobian of car.continuous_dynamics at
            % given x and u
            % Standard finite forward difference method
            %
%             if nargin < 3
%                 epsilon = 1e-5; 
%             end
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

