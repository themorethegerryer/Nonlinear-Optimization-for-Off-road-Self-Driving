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
        % parameters for wet asphalt
        Fz0 = 4470      % Nominal normal load (N)
        mu0 = 0.81     % Nominal friction coefficient
        mu_dFz = -0.232 % Friction sensitivity
        c1 = 22.8       % Lateral stiffness parameter
        c2 = 1.5       % Lateral stiffness peak
        n_coup = 2.5   % Lateral-longitudinal coupling
        c8 = 6.55         % Longitudinal stiffness

        L
        D
    end
    
    methods
        function car = DoubleTrackModel()
            % Double Track Model Constructor
            car.L = car.a + car.b; % vehicle's wheelbase
            car.D = max(car.dr,car.df); % width of car

            % load TerrainData.mat file
%             load("Terrain Generation and Simulation/TerrainData.mat");
        end
        
        function dxdt = continuous_dynamics(car, x, u, ppitch, qroll, p, q)% ax, ay, rdot, wdot
        % x = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
        % uy: vehicle's lateral velocity ~ 1 m/s
        % r: yaw rate ~ 0.1 rad/s
        % ux: vehicle's longitudinal velocity (straight forward) ~ 1 m/s
        % dFzlong: longitudinal load transfer ~ 500 N
        % dFzlat: lateral load transfer ~ 500 N
        % delta: current steering angle ~ 0.1 rad
        % xPos: global x-position of the car
        % yPos: global y-position of the car
        % yawOrient: global yaw orientation of the car
        %
        % u = [deltadot Fxf_enginebrake Fxr]
        % deltadot: change in steering angle ~ 0.1 rad/s
        % Fxfl_enginebrake: Front left and right tire throttle or brake force ~ 500 N
        % Fxrl: Rear left and right tire driving force ~ 300 N
            dxdt = zeros(size(x));
    
            % Pull state and control variables out for easy equation reference
            uy = x(1);
            r = x(2);
            ux = x(3);
            dFzlong = x(4);
            dFzlat = x(5);
            delta = x(6);
    
            deltadot = u(1);
            Fxf_enginebrake = u(2);
            Fxr = u(3);
%             udiff = u(5); % TODO differential is always open

%             slip_bundle = slip_function(car, x);
    
            % Pre-calcuate variables needed in equations
            % given the interaction between the car and the environment
            % TODO assume values are zero for now
%             p = 0;% roll rate
            pdot = 0; % roll acceleration
%             q = 0;% pitch rate
            qdot = 0; % pitch acceleration
            uz = 0;% vertical velocity
            k = 0; % curvature describing how path tangent roates about the normal to the road surface with distance traveled along the path - Page 3
    
            % g = g_x*b_x + g_y*b_y + g_z*b_z, gravity in the body frame

            pitchRotMat = [cos(ppitch) 0 sin(ppitch) ; 0 1 0 ; -sin(ppitch) 0 cos(ppitch)];
            rollRotMat = [1 0 0 ; 0 cos(qroll) -sin(qroll) ; 0 sin(qroll) cos(qroll)];

            gVect = [0 ; 0 ; 9.81];
            gVect = rollRotMat*pitchRotMat*gVect;

            gx = gVect(1);
            gy = gVect(2);
            gz = gVect(3);

            Fxfl_enginebrake = Fxf_enginebrake/2; % Fxflbrake
            Fxfr_enginebrake = Fxf_enginebrake/2; % Fxfrbrake
            Fxrl = Fxr/2; % 
            Fxrr = Fxr/2;

            % compute Fz of each tire
            % wheelFz_bundle: [Fzfl, Fzfr, Fzrl, Fzrr]
            wheelFz_bundle = tire_fz_function(car, x, car.m*9.81);
            % wheelFxFy_bundle: [Fxfl, Fxfr, Fxrl, Fxrr, Fyfl, Fyfr, Fyrl, Fyrr]
            wheelFxFy_bundle = tire_model(car, wheelFz_bundle, x, u);

            % TODO placeholder for Fxfl
            Fxfl = wheelFxFy_bundle(1);
            Fxfr = wheelFxFy_bundle(2);
%             Fxrl = 1;
%             Fxrr = 1;
            Fyfl = wheelFxFy_bundle(5);
            Fyfr = wheelFxFy_bundle(6);
            Fyrl = wheelFxFy_bundle(7);
            Fyrr = wheelFxFy_bundle(8);

    
            Fyf = Fyfl + Fyfr;
            Fyr = Fyrl + Fyrr;
            Fxf = Fxfl + Fxfr;
            Fxr = Fxrl + Fxrr;
    
            Fdrag = 0;
            
    
            % duy/dt - Page 95 - eq. 4.4
            dxdt(1) = ((Fyf*cos(delta) + Fyr + Fxf*sin(delta))/car.m)-r*ux+p*uz+gy;
    
            % dr/dt - Page 95 - eq. 4.5
            dxdt(2) = ((car.a*(Fyf*cos(delta)+Fxf*sin(delta)) - car.b*Fyr)/car.Izz) + (((0.5*car.dr*(Fxrr - Fxrl))+(0.5*car.df*(Fxfr - Fxfl)*cos(delta)))/car.Izz) + (((Fyfl - Fyfr)*sin(delta))/car.Izz) + (((car.Ixx - car.Iyy)*p*q)/car.Izz);
%             dxdt(2)
            % dux/dt - Page 95 - eq. 4.6
            dxdt(3) = ((Fxf*cos(delta)+Fxr-Fyf*sin(delta)-Fdrag)/car.m) + r*uy-q*uz+gx;
    
            % ddFzlong/dt - Page 96 - eq. 4.10
            %confirm that h = hcg here
            dxdt(4) = -car.Klong*(dFzlong+((p*r*(car.Izz-car.Ixx))/car.L)-((car.hcg*(Fxf*cos(delta)+Fxr-Fyf*sin(delta))+car.Iyy*qdot)/car.L));
            
            % ddFzlat/dt - Page 96 - eq. 4.11
            %confirm that h = hcg here
            dxdt(5) = -car.Klat*(dFzlat+((q*r*(car.Izz-car.Iyy))/car.D)-((car.hcg*(Fyf*cos(delta)+Fyr+Fxf*sin(delta))-car.Ixx*pdot)/car.D));
    
            % ddelta/dt
            % TODO redo using state equations
            dxdt(6) = deltadot;

            % TODO placeholder x-position
            dxdt(7) = x(3);

            % TODO placeholder y-position
            dxdt(8) = x(1);

            % TODO placeholder yaw orientation
            dxdt(9) = x(2);

        end
        
        function x_next = dynamics_rk4(car,x,u,dt,ppitch,qroll,p,q)
            % x = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
            % u = [deltadot Fxf_enginebrake Fxr]
            if nargin<8
                ppitch = 0;
                qroll = 0;
                p = 0;
                q = 0;
            end
            uTemp = u;
%             uTemp(1) = uTemp(1) + 0.1*timeVal;
%             if mod(timeVal,3) == 0
%                uTemp(1) = -1 * uTemp(1); 
%             end
            f1 = continuous_dynamics(car, x, uTemp,ppitch,qroll,p,q);
            f2 = continuous_dynamics(car, x + 0.5*dt*f1, uTemp,ppitch,qroll,p,q);
            f3 = continuous_dynamics(car, x + 0.5*dt*f2, uTemp,ppitch,qroll,p,q);
            f4 = continuous_dynamics(car, x + dt*f3, uTemp,ppitch,qroll,p,q);
            
            x_next = x + (dt / 6)*(f1 + 2*f2 + 2*f3 + f4);
            x_next(isnan(x_next)) = 0;
        end

        function Fz_bundle = tire_fz_function(car, x, Fz)
            % x = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
            % u = [deltadot Fxf_enginebrake Fxr]
            Fz_bundle = zeros(1,4);
            Fz_bundle(1) = (((car.b/car.L)*Fz - x(4))/2) - car.gamma*x(5); % eq 4.13
            Fz_bundle(2) = (((car.b/car.L)*Fz - x(4))/2) + car.gamma*x(5); % eq 4.14
            Fz_bundle(3) = (((car.a/car.L)*Fz + x(4))/2) - (1 - car.gamma)*x(5); % eq 4.15
            Fz_bundle(4) = (((car.a/car.L)*Fz + x(4))/2) + (1 - car.gamma)*x(5); % eq 4.16
        end

        function tire_corner_stiffness = cornering_stiffness(car, tire_mu, Fz, Fx) % eq 4.20
            % TODO replace Fx with the state of the car
            % x = [uy r ux dPsi e dFzlong dFzlat delta xPos yPos yawOrient]
            % u = [deltadot Fxf_enginebrake Fxr]
            tire_normal_stiffness = car.c1*car.Fz0*sin(2*atan(Fz/(car.c2*car.Fz0))); % eq 4.21
%             tire_normal_stiffness = car.c1*car.Fz0*sin(2*atan2(car.c2*car.Fz0,Fz)); % eq 4.21
            
            term1 = 0.5*(tire_mu*Fz - Fx);
            term2_1 = (1 - (abs(Fx)/(tire_mu*Fz))^car.n_coup)^(1/car.n_coup);
            term2_2 = tire_normal_stiffness-(0.5*tire_mu*Fz);
            tire_corner_stiffness = term1 + (term2_1*term2_2);
%             tire_corner_stiffness = term1;
        end

        function tire_mu = tire_friction(car,Fz)
            % x = [uy r ux dPsi e dFzlong dFzlat delta xPos yPos yawOrient]
            % u = [deltadot Fxf_enginebrake Fxr]
            tire_mu = car.mu0 + (car.mu_dFz)*((Fz - car.Fz0) / car.Fz0); % eq 4.19
        end

        
        function Fwheel = tire_model(car, Fz_bundle, x, u) 
            % x = [uy r ux dPsi e dFzlong dFzlat delta xPos yPos yawOrient]
            % u = [deltadot Fxf_enginebrake Fxr]
            % TODO replace Fx with the state of the car
            % computer lateral force Fy

            % compute longitudinal force Fx
            % assume constant open differential
            Fwheel = zeros(1,8);
            torque_enginebrake = u(2);
            Fxr = u(3);

            Fxfl = torque_enginebrake/2;
            Fxfr = torque_enginebrake/2;
            Fxrl = Fxr/2;
            Fxrr = Fxr/2;

            Fwheel(1) = Fxfl;
            Fwheel(2) = Fxfr;
            Fwheel(3) = Fxrl;
            Fwheel(4) = Fxrr;

            % compute the slip
            slip_angle_bundle = slip_function(car, x);

            % compute Fyfl
            tire_mu_fl = tire_friction(car,Fz_bundle(1));
            c_stiffness_fl = cornering_stiffness(car, tire_mu_fl, Fz_bundle(1), Fxfl);
            Fyfl_max = sqrt((tire_mu_fl*Fz_bundle(1))^2 - Fxfl); % eq. 4.17
            alpha_slide_fl = (3*Fyfl_max) / c_stiffness_fl; % eq 4.18
            Fwheel(5) = -c_stiffness_fl*tan(slip_angle_bundle(1)) + (c_stiffness_fl^2 / (3*Fyfl_max))*tan(slip_angle_bundle(1))*abs(tan(slip_angle_bundle(1)));
            if abs(slip_angle_bundle(1)) <= alpha_slide_fl
                Fwheel(5) = Fwheel(5) - (c_stiffness_fl^3 / (27*Fyfl_max^2))*tan(slip_angle_bundle(1))^3;
            else
                Fwheel(5) = Fwheel(5) - Fyfl_max*sign(slip_angle_bundle(1));
            end

            % compute Fyfr
            tire_mu_fr = tire_friction(car,Fz_bundle(2));
            c_stiffness_fr = cornering_stiffness(car, tire_mu_fr, Fz_bundle(2), Fxfr);
            Fyfr_max = sqrt((tire_mu_fr*Fz_bundle(2))^2 - Fxfr); % eq. 4.17
            alpha_slide_fr = (3*Fyfr_max) / c_stiffness_fr; % eq 4.18
            Fwheel(6) = -c_stiffness_fr*tan(slip_angle_bundle(2)) + (c_stiffness_fr^2 / (3*Fyfr_max))*tan(slip_angle_bundle(2))*abs(tan(slip_angle_bundle(2)));
            if abs(slip_angle_bundle(2)) <= alpha_slide_fr
                Fwheel(6) = Fwheel(6) - (c_stiffness_fr^3 / (27*Fyfr_max^2))*tan(slip_angle_bundle(2))^3;
            else
                Fwheel(6) = Fwheel(6) - Fyfr_max*sign(slip_angle_bundle(2));
            end

            % compute Fyrl
            tire_mu_rl = tire_friction(car,Fz_bundle(3));
            c_stiffness_rl = cornering_stiffness(car, tire_mu_rl, Fz_bundle(3), Fxrl);
            Fyrl_max = sqrt((tire_mu_rl*Fz_bundle(3))^2 - Fxrl); % eq. 4.17
            alpha_slide_rl = (3*Fyrl_max) / c_stiffness_rl; % eq 4.18
            Fwheel(7) = -c_stiffness_rl*tan(slip_angle_bundle(3)) + (c_stiffness_rl^2 / (3*Fyrl_max))*tan(slip_angle_bundle(3))*abs(tan(slip_angle_bundle(3)));
            if abs(slip_angle_bundle(3)) <= alpha_slide_rl
                Fwheel(7) = Fwheel(7) - (c_stiffness_rl^3 / (27*Fyrl_max^2))*tan(slip_angle_bundle(3))^3;
            else
                Fwheel(7) = Fwheel(7) - Fyrl_max*sign(slip_angle_bundle(3));
            end

            % compute Fyrr
            tire_mu_rr = tire_friction(car,Fz_bundle(4));
            c_stiffness_rr = cornering_stiffness(car, tire_mu_rr, Fz_bundle(4), Fxrr);
            Fyrr_max = sqrt((tire_mu_rr*Fz_bundle(4))^2 - Fxrr); % eq. 4.17
            alpha_slide_rr = (3*Fyrr_max) / c_stiffness_rr; % eq 4.18
            Fwheel(8) = -c_stiffness_rr*tan(slip_angle_bundle(4)) + (c_stiffness_rr^2 / (3*Fyrr_max))*tan(slip_angle_bundle(4))*abs(tan(slip_angle_bundle(4)));
            if abs(slip_angle_bundle(4)) <= alpha_slide_rr
                Fwheel(8) = Fwheel(8) - (c_stiffness_rr^3 / (27*Fyrr_max^2))*tan(slip_angle_bundle(4))^3;
            else
                Fwheel(8) = Fwheel(8) - Fyrr_max*sign(slip_angle_bundle(4));
            end

%             tire_mu = tire_friction(car,Fz);
%             c_stiffness = cornering_stiffness(car, tire_mu, Fz, Fx);
%             Fy_max = sqrt((tire_mu*Fz)^2 - Fx); % eq. 4.17
%             alpha_slide = (3*Fy_max) / c_stiffness; % eq 4.18
%             Fwheel.Fy = -c_stiffness*tan(sideSlip) + (c_stiffness^2 / (3*Fy_max))*tan(sideSlip)*abs(tan(sideSlip));
%             if abs(sideSlip) <= alpha_slide
%                 Fwheel.Fy = Fwheel.Fy - (c_stiffness^3 / (27*Fy_max^2))*tan(sideSlip)^3;
%             else
%                 Fwheel.Fy = Fwheel.Fy - Fy_max*sign(sideSlip);
%             end

            

        end

        function sideSlip = slip_function(car, x)
            % x = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
            % u = [deltadot Fxf_enginebrake Fxr]
            sideSlip = zeros(1,4);
            r = x(9);
            Vx = x(3);
            Vy = x(1);
%             theta = ackermann_turn(car, U(1));
            steering_angle = x(6);
            sideSlip(1) = 0;
            sideSlip(2) = 0;
            sideSlip(3) = 0;
            sideSlip(4) = 0;
            
            % compute sideSlip for each wheel
%             sideSlip_fl = steering_angle - atan((Vy + car.a*r)/(Vx - (car.df/2)*r));
%             sideSlip_fr = steering_angle - atan((Vy + car.a*r)/(Vx + (car.df/2)*r));
%             sideSlip_rl = -atan((Vy - car.b*r)/(Vx-(car.dr/2)*r));
%             sideSlip_rr = -atan((Vy - car.b*r)/(Vx+(car.dr/2)*r));

%             sideSlip = [sideSlip_fl sideSlip_fr sideSlip_rl sideSlip_rr];
            % set NaN values to zero (i.e. car moving foward)
            sideSlip(isnan(sideSlip)) = 0; 
%             slip.side_slip = sideSlip;
        end
        
        function theta = ackermann_turn(car, steering_angle)
            % TODO may not need to use for this model
            theta = zeros(1,2);
            radius = car.L*tan((pi/2)-steering_angle);
            if steering_angle < 0
                theta(1) = atan(car.L/(radius+(car.D/2))); % *(180/pi);
                theta(2) = atan(car.L/(radius-(car.D/2))); % *(180/pi);
            else
                theta(2) = atan(car.L/(radius+(car.D/2))); % *(180/pi);
                theta(1) = atan(car.L/(radius-(car.D/2))); % *(180/pi);
            end
        end
    end
end

