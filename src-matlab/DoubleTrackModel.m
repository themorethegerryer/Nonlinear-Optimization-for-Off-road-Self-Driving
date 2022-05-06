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
            car.L = car.a + car.b; % vehicle's wheelbase
            car.D = max(car.dr,car.df); % width of car
        end
        
        function dxdt = continuous_dynamics(car, x, u)% ax, ay, rdot, wdot
        % x = [uy r ux dPsi e dFzlong dFzlat delta]
        % uy: vehicle's lateral velocity ~ 1 m/s
        % r: yaw rate ~ 0.1 rad/s
        % ux: vehicle's longitudinal velocity (straight forward) ~ 1 m/s
        % dPsi = z-axis rotation difference between body frame and path (angle between b_x and p_x) ~ 0.01 rad
        % e: distance from path to car's c.g. in p_y direction (e*p_y = distance from path) ~ 1 m
        % dFzlong: longitudinal load transfer ~ 500 N
        % dFzlat: lateral load transfer ~ 500 N
        % delta: current steering angle ~ 0.1 rad
        %
        % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
        % deltadot: change in steering angle ~ 0.1 rad/s
        % Fxflbrake: Front left tire brake force ~ 500 N
        % Fxfrbrake: Front right tire brake force ~ 500 N
        % Fxrl: Rear left tire driving force ~ 300 N
        % Fxrr: Rear right tire driving force ~ 300 N
        % Fengine: Torque applied to rear tires by engine ~ 500 N
        % udiff: Degree of locking in the differential [0,1] ~ 1
            dxdt = zeros(size(x));
    
            % Pull state and control variables out for easy equation reference
            uy = x(1);
            r = x(2);
            ux = x(3);
            dPsi = x(4);
            e = x(5);
            dFzlong = x(6);
            dFzlat = x(7);
            delta = x(8);
    
            deltadot = u(1);
            Fxflbrake = u(2); % Fxflbrake
            Fxfrbrake = u(3); % Fxfrbrake
            Fxrl = u(4);
            Fxrr = u(5);
            Fengine = u(6);
            udiff = u(7);

            slip_bundle = slip_function(car, x);
    
            % Pre-calcuate variables needed in equations
            p = 1;% roll rate
            pdot = 1; % roll acceleration
            q = 1;% pitch rate
            qdot = 1; % pitch acceleration
            uz = 1;% vertical velocity
            k = 1; % curvature describing how path tangent roates about the normal to the road surface with distance traveled along the path - Page 3
    
            % g = g_x*b_x + g_y*b_y + g_z*b_z, gravity in the body frame
            gx = 0;
            gy = 9.81;
            gz = 0;

            % compute Fz of each tire
            wheelFz_bundle = tire_fz_function(car, x, car.m*9.81);
            wheelFxFy_bundle = tire_model(car, wheelFz_bundle, x, u);

            % TODO placeholder for Fxfl
            Fxfl = wheelFxFy_bundle.Fxfl;
            Fxfr = wheelFxFy_bundle.Fxfr;
%             Fxrl = 1;
%             Fxrr = 1;
            Fyfl = wheelFxFy_bundle.Fyfl;
            Fyfr = wheelFxFy_bundle.Fyfr;
            Fyrl = wheelFxFy_bundle.Fyrl;
            Fyrr = wheelFxFy_bundle.Fyrr;

    
            Fyf = Fyfl + Fyfr;
            Fyr = Fyrl + Fyrr;
            Fxf = Fxfl + Fxfr;
            Fxr = Fxrl + Fxrr;
    
            Fdrag = 0;
            
    
            % duy/dt - Page 95 - eq. 4.4
            dxdt(1) = ((Fyf*cos(delta) + Fyr + Fxf*sin(delta))/car.m)-r*ux+p*uz+gy;
    
            % dr/dt - Page 95 - eq. 4.5
            dxdt(2) = ((car.a*(Fyf*cos(delta)+Fxf*sin(delta)) - car.b*Fyr)/car.Izz) + (((0.5*car.dr*(Fxrr - Fxrl))+(0.5*car.df*(Fxfr - Fxfl)*cos(delta)))/car.Izz) + (((Fyfl - Fyfr)*sin(delta))/car.Izz) + (((car.Ixx - car.Iyy)*p*q)/car.Izz);
    
            % dux/dt - Page 95 - eq. 4.6
            dxdt(3) = ((Fxf*cos(delta)+Fxr-Fyf*sin(delta)-Fdrag)/car.m) + r*uy-q*uz+gx;
    
            % ddPsi/dt - Page 96 - eq. 4.7
            dxdt(4) = r - ((ux*cos(dPsi)-uy*sin(dPsi))/(1-k*e))*k;
    
            % de/dt - Page 96  - eq. 4.9
            dxdt(5) = ux*sin(dPsi)+uy*cos(dPsi);
    
            % ddFzlong/dt - Page 96 - eq. 4.10
            %confirm that h = hcg here
            dxdt(6) = -car.Klong*(dFzlong+((p*r*(car.Izz-car.Ixx))/car.L)-((car.hcg*(Fxf*cos(delta)+Fxr-Fyf*sin(delta))+car.Iyy*qdot)/car.L));
            
            % ddFzlong/dt - Page 96 - eq. 4.11
            %confirm that h = hcg here
            dxdt(7) = -car.Klat*(dFzlat+((q*r*(car.Izz-car.Iyy))/car.D)-((car.hcg*(Fyf*cos(delta)+Fyr-Fxf*sin(delta))-car.Ixx*pdot)/car.D));
    
            % ddelta/dt
            % TODO redo using state equations
            dxdt(8) = deltadot;
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

        function Fz_bundle = tire_fz_function(car, x, Fz)
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            Fz_bundle.fl = (((car.b/car.L)*Fz - x(6))/2) - car.gamma*u(7); % eq 4.13
            Fz_bundle.fr = (((car.b/car.L)*Fz - x(6))/2) + car.gamma*u(7); % eq 4.14
            Fz_bundle.rl = (((car.a/car.L)*Fz + x(6))/2) - (1 - car.gamma)*u(7); % eq 4.15
            Fz_bundle.rr = (((car.a/car.L)*Fz + x(6))/2) + (1 - car.gamma)*u(7); % eq 4.16
        end

        function tire_corner_stiffness = cornering_stiffnes(car, tire_mu, Fz, Fx) % eq 4.20
            % TODO replace Fx with the state of the car
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            tire_normal_stiffness = car.c1*car.Fz0*sin(2*atan(Fz/(car.c2*car.Fz0))); % eq 4.21
            
            term1 = 0.5*(tire_mu*Fz - Fx);
            term2_1 = (1 - (abs(Fx)/(tire_mu*Fz))^car.n_coup)^(1/car.n_coup);
            term2_2 = tire_normal_stiffness-(0.5*tire_mu*Fz);
            tire_corner_stiffness = term1 + (term2_1*term2_2);
        end

        function tire_mu = tire_friction(car,Fz)
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            tire_mu = car.mu0 + (car.mu_dFz)*((Fz - car.Fz0) / car.Fz0); % eq 4.19
        end

        
        function Fwheel = tire_model(car, Fz_bundle, x, u) 
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            % TODO replace Fx with the state of the car
            % computer lateral force Fy

            % compute longitudinal force Fx
            % assume constant open differential
            torque_engine = u(6);
            Fxfl = torque_engine/2 - u(2);
            Fxfr = torque_engine/2 - u(3);
            Fxrl = u(4);
            Fxrr = u(5);

            Fwheel.Fxfl = Fxfl;
            Fwheel.Fxfr = Fxfr;
            Fwheel.Fxrl = Fxrl;
            Fwheel.Fxrr = Fxrr;

            % compute the slip
            slip_angle_bundle = slip_function(car, x);

            % compute Fyfl
            tire_mu_fl = tire_friction(car,Fz_bundle.fl);
            c_stiffness_fl = cornering_stiffness(car, tire_mu_fl, Fz_bundle.fl, Fxfl);
            Fyfl_max = sqrt((tire_mu_fl*Fz_bundle.fl)^2 - Fxfl); % eq. 4.17
            alpha_slide_fl = (3*Fyfl_max) / c_stiffness_fl; % eq 4.18
            Fwheel.Fyfl = -c_stiffness_fl*tan(slip_angle_bundle.side_slip(1)) + (c_stiffness_fl^2 / (3*Fyfl_max))*tan(slip_angle_bundle.side_slip(1))*abs(tan(slip_angle_bundle.side_slip(1)));
            if abs(slip_angle_bundle.side_slip(1)) <= alpha_slide_fl
                Fwheel.Fyfl = Fwheel.Fyfl - (c_stiffness_fl^3 / (27*Fyfl_max^2))*tan(slip_angle_bundle.side_slip(1))^3;
            else
                Fwheel.Fyfl = Fwheel.Fyfl - Fyfl_max*sign(slip_angle_bundle.side_slip(1));
            end

            % compute Fyfr
            tire_mu_fr = tire_friction(car,Fz_bundle.fr);
            c_stiffness_fr = cornering_stiffness(car, tire_mu_fr, Fz_bundle.fr, Fxfr);
            Fyfr_max = sqrt((tire_mu_fr*Fz_bundle.fr)^2 - Fxfr); % eq. 4.17
            alpha_slide_fr = (3*Fyfr_max) / c_stiffness_fr; % eq 4.18
            Fwheel.Fyfr = -c_stiffness_fr*tan(slip_angle_bundle.side_slip(2)) + (c_stiffness_fr^2 / (3*Fyfr_max))*tan(slip_angle_bundle.side_slip(2))*abs(tan(slip_angle_bundle.side_slip(2)));
            if abs(slip_angle_bundle.side_slip(2)) <= alpha_slide_fr
                Fwheel.Fyfr = Fwheel.Fyfr - (c_stiffness_fr^3 / (27*Fyfr_max^2))*tan(slip_angle_bundle.side_slip(2))^3;
            else
                Fwheel.Fyfr = Fwheel.Fyfr - Fyfr_max*sign(slip_angle_bundle.side_slip(2));
            end

            % compute Fyrl
            tire_mu_rl = tire_friction(car,Fz_bundle.rl);
            c_stiffness_rl = cornering_stiffness(car, tire_mu_rl, Fz_bundle.rl, Fxrl);
            Fyrl_max = sqrt((tire_mu_rl*Fz_bundle.rl)^2 - Fxrl); % eq. 4.17
            alpha_slide_rl = (3*Fyrl_max) / c_stiffness_rl; % eq 4.18
            Fwheel.Fyrl = -c_stiffness_rl*tan(slip_angle_bundle.side_slip(3)) + (c_stiffness_rl^2 / (3*Fyrl_max))*tan(slip_angle_bundle.side_slip(3))*abs(tan(slip_angle_bundle.side_slip(3)));
            if abs(slip_angle_bundle.side_slip(3)) <= alpha_slide_rl
                Fwheel.Fyrl = Fwheel.Fyrl - (c_stiffness_rl^3 / (27*Fyrl_max^2))*tan(slip_angle_bundle.side_slip(3))^3;
            else
                Fwheel.Fyrl = Fwheel.Fyrl - Fyrl_max*sign(slip_angle_bundle.side_slip(3));
            end

            % compute Fyrr
            tire_mu_rr = tire_friction(car,Fz_bundle.rr);
            c_stiffness_rr = cornering_stiffness(car, tire_mu_rr, Fz_bundle.rr, Fxrr);
            Fyrr_max = sqrt((tire_mu_rr*Fz_bundle.rr)^2 - Fxrr); % eq. 4.17
            alpha_slide_rr = (3*Fyrr_max) / c_stiffness_rr; % eq 4.18
            Fwheel.Fyrr = -c_stiffness_rr*tan(slip_angle_bundle.side_slip(4)) + (c_stiffness_rr^2 / (3*Fyrr_max))*tan(slip_angle_bundle.side_slip(4))*abs(tan(slip_angle_bundle.side_slip(4)));
            if abs(slip_angle_bundle.side_slip(4)) <= alpha_slide_rr
                Fwheel.Fyrr = Fwheel.Fyrr - (c_stiffness_rr^3 / (27*Fyrr_max^2))*tan(slip_angle_bundle.side_slip(4))^3;
            else
                Fwheel.Fyrr = Fwheel.Fyrr - Fyrr_max*sign(slip_angle_bundle.side_slip(4));
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

        function slip = slip_function(car, x)
            % x = [uy r ux dPsi e dFzlong dFzlat delta]
            % u = [deltadot Fxflbrake Fxfrbrake Fxrl Fxrr Fengine udiff]
            r = x(2);
            Vx = x(3);
            Vy = x(1);
%             theta = ackermann_turn(car, U(1));
            steering_angle = x(8);
            
            % compute sideSlip for each wheel
            sideSlip_fl = steering_angle - atan((Vy + car.df*r)/(Vx - (car.L/2)*r));
            sideSlip_fr = steering_angle - atan((Vy + car.df*r)/(Vx + (car.L/2)*r));
            sideSlip_rl = -atan((Vy - car.dr*r)/(Vx-(car.L/2)*r));
            sideSlip_rr = -atan((Vy - car.dr*r)/(Vx+(car.L/2)*r));

            sideSlip = [sideSlip_fl sideSlip_fr sideSlip_rl sideSlip_rr];
            % set NaN values to zero (i.e. car moving foward)
            sideSlip(isnan(sideSlip)) = 0; 
            slip.side_slip = sideSlip;
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

