classdef DynamicBicycleModel
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
        function car = DynamicBicycleModel()
            % Single Track Model Constructor
            car.L = car.a + car.b; % vehicle's wheelbase
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
        % x = [uy r ux dFzlong delta x y yaw]
        % uy: vehicle's lateral velocity ~ 1 m/s
        % r: yaw rate ~ 0.1 rad/s
        % ux: vehicle's longitudinal velocity (straight forward) ~ 1 m/s
        % dFzlong: longitudinal load transfer ~ 500 N
        % delta: current steering angle ~ 0.1 rad
        %
        % u = [deltadot Fxf_enginebrake Fxr]
        % deltadot: change in steering angle ~ 0.1 rad/s
        % Fxfbrake: Front brake force ~ 500 N
        % Fxr: Rear driving force ~ 300 N
        % Fengine: Torque applied to front tires by engine ~ 500 N
            dxdt = zeros(size(x));

            % Pull state and control variables out for easy equation reference
            uy = x(1);
            r = x(2);
            ux = x(3);
            dFzlong = x(4);
            delta = x(5);

            deltadot = u(1);
            Fxf_enginebrake = u(2);
            Fxr = u(3);

            % Pre-calcuate variables needed in equations
            p = 0;% roll rate
            pdot = 0; % roll acceleration
            q = 0;% pitch rate
            qdot = 0; % pitch acceleration
            uz = 0;% vertical velocity
            k = 0; % curvature describing how path tangent rotates about the normal to the road surface with distance traveled along the path - Page 33

            % g = g_x*b_x + g_y*b_y + g_z*b_z, gravity in the body frame
            gx = 0;
            gy = 0;
            gz = -9.81;
            
            % compute Fz of each tire
            % wheelFz_bundle: [Fzf, Fzr]
            wheelFz_bundle = tire_fz_function(car, x, car.m*9.81);
            % wheelFxFy_bundle: [Fxf, Fxr, Fyf, Fyr]
            wheelFxFy_bundle = tire_model(car, wheelFz_bundle, x, u);

            Fyf = wheelFxFy_bundle(3);
            Fyr = wheelFxFy_bundle(4);
            Fxf = wheelFxFy_bundle(1);

            Fdrag = 0;

            % duy/dt - Page 95
            dxdt(1) = ((Fyf*cos(delta) + Fyr + Fxf*sin(delta))/car.m)-r*ux+p*uz+gy;

            % dr/dt - Page 95
            % For single track, Fxrr = Fxrl, Fxfr = Fxfl, and Fyfl = Fyfr so 2nd and 3rd terms are 0.
            dxdt(2) = ((car.a*(Fyf*cos(delta)+Fxf*sin(delta)) - car.b*Fyr)/car.Izz) + (((car.Ixx - car.Iyy)*p*q)/car.Izz);

            % dux/dt - Page 95
            dxdt(3) = ((Fxf*cos(delta)+Fxr-Fyf*sin(delta)-Fdrag)/car.m) + r*uy-q*uz+gx;

            % ddFzlong/dt - Page 96
            %confirm that h = hcg here
            dxdt(4) = -car.Klong*(dFzlong+((p*r*(car.Izz-car.Ixx))/car.L)-((car.hcg*(Fxf*cos(delta)+Fxr-Fyf*sin(delta))+car.Iyy*qdot)/car.L));

            % ddelta/dt
            dxdt(5) = deltadot;

            % dx/dt
            dxdt(6) = x(3);

            % dy/dt
            dxdt(7) = x(1);

            % dyaw/dt
            dxdt(8) = x(2);
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
        
        function Fz_bundle = tire_fz_function(car, x, Fz)
            % x = [uy r ux dFzlong delta x y yaw]
            % u = [deltadot Fxfbrake Fxr Fengine]
            Fz_bundle = zeros(1,2);
            Fz_bundle(1) = (((car.b/car.L)*Fz - x(4))/2); % eq 4.13
            Fz_bundle(2) = (((car.a/car.L)*Fz + x(4))/2); % eq 4.15
        end

        function tire_corner_stiffness = cornering_stiffness(car, tire_mu, tire_normal_stiffness, n_coup, Fz, Fx) % eq 4.20
            % TODO replace Fx with the state of the car
            % x = [uy r ux dFzlong delta x y yaw]
            % u = [deltadot Fxfbrake Fxr Fengine]
            
            term1 = 0.5*(tire_mu*Fz - Fx);
            term2_1 = (1 - (abs(Fx)/(tire_mu*Fz))^n_coup)^(1/n_coup);
            term2_2 = tire_normal_stiffness-(0.5*tire_mu*Fz);
            tire_corner_stiffness = term1 + (term2_1*term2_2);
        end
        
        function Fwheel = tire_model(car, Fz_bundle, x, u) 
            % x = [uy r ux dFzlong delta x y yaw]
            % u = [deltadot Fxf_enginebrake Fxr]
            % TODO replace Fx with the state of the car
            % computer lateral force Fy

            % compute longitudinal force Fx
            % assume constant open differential
            Fwheel = zeros(1,3);

            engine_brake = u(2);
            Fxf = engine_brake;
            Fxr = u(3);

            Fwheel(1) = Fxf;
            Fwheel(2) = Fxr;

            % compute the slip
            slip_angle_bundle = slip_function(car, x);

            % compute Fyf
            c_stiffness_f = car.cornering_stiffness(car.mu_f, car.Calpha_f, car.n_f, Fz_bundle(1), Fxf);
            Fyf_max = sqrt((car.mu_f*Fz_bundle(1))^2 - Fxf); % eq. 4.17
            alpha_slide_f = (3*Fyf_max) / c_stiffness_f; % eq 4.18
            Fwheel(3) = -c_stiffness_f*tan(slip_angle_bundle(1)) + (c_stiffness_f^2 / (3*Fyf_max))*tan(slip_angle_bundle(1))*abs(tan(slip_angle_bundle(1)));
            if abs(slip_angle_bundle(1)) <= alpha_slide_f
                Fwheel(3) = Fwheel(3) - (c_stiffness_f^3 / (27*Fyf_max^2))*tan(slip_angle_bundle(1))^3;
            else
                Fwheel(3) = Fwheel(3) - Fyf_max*sign(slip_angle_bundle(1));
            end

            % compute Fyr
            c_stiffness_r = car.cornering_stiffness(car.mu_r, car.Calpha_r, car.n_r, Fz_bundle(2), Fxr);
            Fyr_max = sqrt((car.mu_r*Fz_bundle(2))^2 - Fxr); % eq. 4.17
            alpha_slide_r = (3*Fyr_max) / c_stiffness_r; % eq 4.18
            Fwheel(4) = -c_stiffness_r*tan(slip_angle_bundle(2)) + (c_stiffness_r^2 / (3*Fyr_max))*tan(slip_angle_bundle(2))*abs(tan(slip_angle_bundle(2)));
            if abs(slip_angle_bundle(2)) <= alpha_slide_r
                Fwheel(4) = Fwheel(4) - (c_stiffness_r^3 / (27*Fyr_max^2))*tan(slip_angle_bundle(2))^3;
            else
                Fwheel(4) = Fwheel(4) - Fyr_max*sign(slip_angle_bundle(2));
            end
        end

        function sideSlip = slip_function(car, x) % POTENTIALLY TOXIC
            % x = [uy r ux dFzlong delta x y yaw]
            % u = [deltadot Fxfbrake Fxr Fengine]
            sideSlip = zeros(2);
%             r = x(2);
            Vx = x(3);
            Vy = x(1);
%             theta = ackermann_turn(car, U(1));
            steering_angle = x(4);
            
            % compute sideSlip for each wheel
            sideSlip_f = 0;%steering_angle - atan((Vy+ car.a*r)/Vx);
            sideSlip_r = 0;% - atan((Vy- car.b*r)/Vx);

            sideSlip(1) = sideSlip_f;
            sideSlip(2) = sideSlip_r;

%             sideSlip = [sideSlip_f sideSlip_r];
            % set NaN values to zero (i.e. car moving foward)
            sideSlip(isnan(sideSlip)) = 0;
        end
    end
end

