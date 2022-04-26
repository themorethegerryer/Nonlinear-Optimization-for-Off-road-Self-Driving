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
        w_wheel = 0.3 % width of tire (m) (i.e. average is 12.5 inches)
        
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
            
            % TODO: assume Fz is equal across all wheels
            Fz = car.m*9.81/4;
            if state(4) > 0
                brake_force = controls(3);
                roll_frict_force = min(car.c*Fz*car.Rw, controls(2));
            elseif state(4) < 0
                brake_force = -controls(3);
                roll_frict_force = max(-car.c*Fz*car.Rw, controls(2));
            else
                brake_force = 0;
                roll_frict_force = 0;
            end
            
            theta = ackermann_turn(car, controls(1));
            slip = slip_function(car, state, controls(1));
            
            Fwheel_fl = tire_model(car, slip.long_slip(1), slip.side_slip(1), Fz);
            Fwheel_fr = tire_model(car, slip.long_slip(2), slip.side_slip(2), Fz);
            Fwheel_rl = tire_model(car, slip.long_slip(3), slip.side_slip(3), Fz);
            Fwheel_rr = tire_model(car, slip.long_slip(4), slip.side_slip(4), Fz);
            
            % xdot
            dxdt(1) = state(4);
            % ydot
            dxdt(2) = state(5);
            % rdot
            dxdt(3) = (1/car.Iz)*(Fwheel_fl.Fy*cos(theta.left) + Fwheel_fr.Fy*cos(theta.right) + Fwheel_fl.Fx*sin(theta.left) + Fwheel_fr.Fx*sin(theta.right) + (car.lw/2)*(Fwheel_fl.Fy*sin(theta.left) - Fwheel_fr.Fy*sin(theta.right) - Fwheel_fl.Fx*cos(theta.left) + Fwheel_fr.Fx*cos(theta.right)) + car.lr*(-Fwheel_rl.Fy - Fwheel_rr.Fy) + (car.lw/2)*(-Fwheel_rl.Fx+Fwheel_rr.Fx));
            % xdotdot
            dxdt(4) = (1/car.m)*(Fwheel_fl.Fx*cos(theta.left) + Fwheel_fr.Fx*cos(theta.right) + Fwheel_rl.Fx + Fwheel_rr.Fx - Fwheel_fl.Fy*sin(theta.left) - Fwheel_fr.Fy*sin(theta.right));
            % ydotdot
            dxdt(5) = (1/car.m)*(Fwheel_fl.Fy*cos(theta.left) + Fwheel_fr.Fy*cos(theta.right) + Fwheel_rl.Fy + Fwheel_rr.Fy + Fwheel_fl.Fx*sin(theta.left) + Fwheel_fr.Fx*sin(theta.right));
            % w_fl
            dxdt(6) = (1/car.Iwheel)*(controls(2) - brake_force - roll_frict_force);
            % w_fr
            dxdt(7) = (1/car.Iwheel)*(controls(2) - brake_force - roll_frict_force);
            % w_rl
            dxdt(8) = (1/car.Iwheel)*(controls(2) - brake_force - roll_frict_force);
            % w_rr
            dxdt(9) = (1/car.Iwheel)*(controls(2) - brake_force - roll_frict_force);
        end
        
        function X_n_1 = dynamics_rk4(car,X,U, dt)
            uTemp = U;
%             uTemp(1) = uTemp(1) + 0.1*timeVal;
%             if timeVal > 1.0
%                uTemp(1) = 0.1; 
%             end
            f1 = continuous_dynamics(car, X, uTemp);
            f2 = continuous_dynamics(car, X + 0.5*dt*f1, uTemp);
            f3 = continuous_dynamics(car, X + 0.5*dt*f2, uTemp);
            f4 = continuous_dynamics(car, X + dt*f3, uTemp);
            
            X_n_1 = X + (dt / 6)*(f1 + 2*f2 + 2*f3 + f4);
        end
        
        function jac = discrete_jacobian(car, x, u)
            % Calculate discrete Jacobian of car.continuous_dynamics at
            % given x and u
            % Standard finite forward difference method
            %
%             if nargin < 3
%                 epsilon = 1e-5; 
%             end
            epsilon = 1e-3;
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
        
%         function dynamics(car,X,U,dt) % output: X_n+1
%             steering_angle = U[1];
%             torque = U[2];
%             w_tire = zeros(1,4)
%             for tire=1:4
%                 F_weight = -car.m*9.81
%                 F_moment_pair = car.Iy*car.h_CoM*
%                 Fr = weight_on_tire*car.c
%                 w_tire = torque-Fr*car.Rw %F_xi assumed to be zero
%             end
%             
%         end
        
%         function cost(car,X,U,Q,R,Qf)
%             
%         end
        
        function Fwheel = tire_model(car, longSlip, sideSlip, Fz)
            % refer to link for nominal values and formulas
            % https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/

            % define b0-b12 values
%             b0 = 1.5;
%             b1 = 0;
%             b2 = 1100;
%             b3 = 0;
%             b4 = 300;
%             b5 = 0;
%             b6 = 0;
%             b7 = 0;
%             b8 = -2;
%             b9 = 0;
%             b10 = 0;
%             b11 = 0;
%             b12 = 0;
            
            % define B, C, D, E, Sh, Sv
%             C = b0;
%             D = Fz*(b1*Fz + b2);
%             bcdVal = (b3*(Fz^2) + b4*Fz)*exp(-b5*Fz);
%             B = bcdVal / (C*D);
%             E = (b6*(Fz^2) + b7*Fz + b8);
%             Sh = b9*Fz + b10;
%             Sv = b11*Fz + b12;

%             Fwheel.Fx = D*sin(C*atan(B*(longSlip + Sh) - E*(B*(longSlip + Sh) - atan(B*(longSlip + Sh))))) + Sv;
%             Fwheel.Fx = Fwheel.Fx/1e3;

%             Fwheel.Fy = D*sin(C*atan(B*(sideSlip + Sh) - E*(B*(sideSlip + Sh) - atan(B*(sideSlip + Sh))))) + Sv;
%             Fwheel.Fy = Fwheel.Fy/1e3;

            % constant values from Matlab documentation
            % https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html
            B = 10;
            C = 1.9;
            D = 1;
            E = 0.97;

            % compute Fx and Fy
            Fwheel.Fx = Fz*D*sin(C*atan(B*longSlip - E*(B*longSlip - atan(B*longSlip))));

            Fwheel.Fy = Fz*D*sin(C*atan(B*sideSlip - E*(B*sideSlip - atan(B*sideSlip))));
%             disp(Fwheel);
        end
        
        function slip = slip_function(car, X,U)
            % X = [x y r xdot ydot wfl wfr wrl wrr]
            % U = [steering_angle wheel_torque]
            r = X(3);
            Vx = X(4);
            Vy = X(5);
            theta = ackermann_turn(car, U(1));
            
            % compute sideSlip for each wheel
            sideSlip_fl = theta.left - atan((Vy + car.lf*r)/(Vx - (car.lw/2)*r));
            sideSlip_fr = theta.right - atan((Vy + car.lf*r)/(Vx + (car.lw/2)*r));
            sideSlip_rl = -atan((Vy - car.lr*r)/(Vx-(car.lw/2)*r));
            sideSlip_rr = -atan((Vy - car.lr*r)/(Vx+(car.lw/2)*r));

            sideSlip = [sideSlip_fl sideSlip_fr sideSlip_rl sideSlip_rr];
            % set NaN values to zero (i.e. car moving foward)
            sideSlip(isnan(sideSlip)) = 0; 
            slip.side_slip = sideSlip;

            w_fl = X(6);
            w_fr = X(7);
            w_rl = X(8);
            w_rr = X(9);
            
            % compute longitudinal wheel velocity
            Vx_fl = Vx*cos(theta.left);
            Vx_fr = Vx*cos(theta.right);
            Vx_rl = Vx;
            Vx_rr = Vx;

            % compute longSlip for each wheel
            if (w_fl*car.Rw - Vx_fl) < 0
                lambda_fl = (w_fl*car.Rw - Vx_fl) / Vx_fl;
            else
                lambda_fl = (w_fl*car.Rw - Vx_fl) / (w_fl*car.Rw);
            end

            if (w_fr*car.Rw - Vx_fr) < 0
                lambda_fr = (w_fr*car.Rw - Vx_fr) / Vx_fr;
            else
                lambda_fr = (w_fr*car.Rw - Vx_fr) / (w_fr*car.Rw);
            end

            if (w_rl*car.Rw - Vx_rl) < 0
                lambda_rl = (w_rl*car.Rw - Vx_rl) / Vx_rl;
            else
                lambda_rl = (w_rl*car.Rw - Vx_rl) / (w_rl*car.Rw);
            end

            if (w_rr*car.Rw - Vx_rr) < 0
                lambda_rr = (w_rr*car.Rw - Vx_rr) / Vx_rr;
            else
                lambda_rr = (w_rr*car.Rw - Vx_rr) / (w_rr*car.Rw);
            end

            longSlip = [lambda_fl lambda_fr lambda_rl lambda_rr];
            % set NaN values to zero (i.e. vechile is not moving)
            longSlip(isnan(longSlip)) = 0;
            slip.long_slip = longSlip;
%             disp(slip);
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

