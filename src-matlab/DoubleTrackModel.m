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
            else
                car.m = m;
                car.mw = mw;
                car.lw = lw;
                car.lr = lr;
                car.lf = lf;
                car.Rw = Rw;
                car.c = c;
            end
            car.length = car.lr+car.lf;
            car.Iz = (car.m)*(car.length^2 + car.lw^2)/12;
            car.Iy = (car.m)*(car.length^2 + car.h^2)/12;
            car.Iwheel = (car.mw)*((car.Rw/2)^2)/2; %effective radius divided by 2 because tire mass is focused at center
        end
        
        function dxdt = continuous_dynamics(car, state, controls)% ax, ay, rdot, wdot
            % state = [x y r xdot ydot wfl wfr wrl wrr]
            % rdot xdotdot ydotdot wfldot wfrdot wrldot wrrdot]
            
            % TODO: assume Fz is equal across all wheels
            Fz = car.m*9.81/4;
            roll_frict_force = car.c*Fz*car.Rw;
            
            theta = ackermann_turn(car, controls(1));
            slip = slip_function(car, state, controls(1));
            
            Fwheel_fl = tire_model(car, slip.long_slip(1), slip.side_slip(1), Fz);
            Fwheel_fr = tire_model(car, slip.long_slip(2), slip.side_slip(2), Fz);
            Fwheel_rl = tire_model(car, slip.long_slip(3), slip.side_slip(3), Fz);
            Fwheel_rr = tire_model(car, slip.long_slip(4), slip.side_slip(4), Fz);
            
            dxdt(1) = state(4); % xdot
            dxdt(2) = state(5); % ydot
            dxdt(3) = (1/car.Iz)*(Fwheel_fl.Fy*cos(theta.left) + Fwheel_fr.Fy*cos(theta.right) + Fwheel_fl.Fx*sin(theta.left) + Fwheel_fr.Fx*sin(theta.right) + (car.lw/2)*(Fwheel_fl.Fy*sin(theta.left) - Fwheel_fr.Fy*sin(theta.right) - Fwheel_fl.Fx*cos(theta.left) + Fwheel_fr.Fx*cos(theta.right)) + car.lr*(-Fwheel_rl.Fy - Fwheel_rr.Fy) + car.lw*(-Fwheel_rl.Fx+Fwheel_rr.Fx));
            dxdt(4) = (1/car.m)*(Fwheel_fl.Fx*cos(theta.left) + Fwheel_fr.Fx*cos(theta.right) + Fwheel_rl.Fx + Fwheel_rr.Fx - Fwheel_fl.Fy*sin(theta.left) - Fwheel_fr.Fy*sin(theta.right));
            dxdt(5) = (1/car.m)*(Fwheel_fl.Fy*cos(theta.left) + Fwheel_fr.Fy*cos(theta.right) + Fwheel_rl.Fy + Fwheel_rr.Fy + Fwheel_fl.Fx*sin(theta.left) + Fwheel_fr.Fx*sin(theta.right));
            dxdt(6) = (1/car.Iwheel)*(controls(2) - roll_frict_force);
            dxdt(7) = (1/car.Iwheel)*(controls(2) - roll_frict_force);
            dxdt(8) = (1/car.Iwheel)*(controls(2) - roll_frict_force);
            dxdt(9) = (1/car.Iwheel)*(controls(2) - roll_frict_force);
        end
        
        function X_n_1 = dynamics_rk4(car,X,U,dt)
            f1 = continuous_dynamics(car, X, U);
            f2 = continuous_dynamics(car, X + 0.5*dt*f1, U);
            f3 = continuous_dynamics(car, X + 0.5*dt*f2, U);
            f4 = continuous_dynamics(car, X + dt*f3, U);
            
            X_n_1 = X + (dt / 6)*(f1 + 2*f2 + 2*f3 + f4);
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
            b0 = 1.5;
            b1 = 0;
            b2 = 1100;
            b3 = 0;
            b4 = 300;
            b5 = 0;
            b6 = 0;
            b7 = 0;
            b8 = -2;
            b9 = 0;
            b10 = 0;
            b11 = 0;
            b12 = 0;
            
            % define B, C, D, E, Sh, Sv
%             C = b0;
%             D = Fz*(b1*Fz + b2);
%             bcdVal = (b3*(Fz^2) + b4*Fz)*exp(-b5*Fz);
%             B = bcdVal / (C*D);
%             E = (b6*(Fz^2) + b7*Fz + b8);
            Sh = b9*Fz + b10;
            Sv = b11*Fz + b12;
            % constant values from Matlab documentation
            % https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html
            B = 10;
            C = 1.9;
            D = 1;
            E = 0.97;

            % compute Fx and Fy
            Fwheel.Fx = Fz*D*sin(C*atan(B*longSlip));
%             Fwheel.Fx = D*sin(C*atan(B*(longSlip + Sh) - E*(B*(longSlip + Sh) - atan(B*(longSlip + Sh))))) + Sv;
%             Fwheel.Fx = Fwheel.Fx/1e3;
            % Fx = D*sin(C*atan(B*(longSlip + Sh) - E*(B*(longSlip + Sh) -
            % atan(B*(longSlip + Sh))))) + Sv;

            Fwheel.Fy = D*sin(C*atan(B*(sideSlip + Sh) - E*(B*(sideSlip + Sh) - atan(B*(sideSlip + Sh))))) + Sv;
%             Fwheel.Fy = Fwheel.Fy/1e3;
            disp(Fwheel)
        end
        
        function slip = slip_function(car, X,U)
            % state = [x y r xdot ydot wfl wfr wrl wrr]
            r = X(3);
            Vx = X(4);
            Vy = X(5);
            theta = ackermann_turn(car, U(1));
            
            sideSlip_fl = theta.left - atan2(Vx - (car.lw/2)*r, Vy + car.lf*r);
            sideSlip_fr = theta.right - atan2(Vx + (car.lw/2)*r, Vy + car.lf*r);
            sideSlip_rl = -atan2(Vx-(car.lw/2)*r, Vy - car.lr*r);
            sideSlip_rr = -atan2(Vx+(car.lw/2)*r, Vy - car.lr*r);

            sideSlip = [sideSlip_fl sideSlip_fr sideSlip_rl sideSlip_rr];
            sideSlip(isnan(sideSlip)) = 0;
            slip.side_slip = sideSlip;

            % compute longSlip for each wheel
            w_fl = X(6);
            w_fr = X(7);
            w_rl = X(8);
            w_rr = X(9);
            
            Vx_fl = Vx*cos(theta.left);
            Vx_fr = Vx*cos(theta.right);
            Vx_rl = Vx;
            Vx_rr = Vx;

%             vx_wheels = vehicle_params('Vxwheel');
%             Vx_fl = vx_wheels(1);
%             Vx_fr = vx_wheels(2);
%             Vx_rl = vx_wheels(3);
%             Vx_rr = vx_wheels(4);

            if (w_fl*car.Rw - Vx_fl) < 0
                lambda_fl = (w_fl*car.Rw - Vx_fl) / Vx_fl;
            else
                lambda_fl = (w_fl*car.Rw - Vx_fl) / w_fl*car.Rw;
            end

            if (w_fr*car.Rw - Vx_fr) < 0
                lambda_fr = (w_fr*car.Rw - Vx_fr) / Vx_fr;
            else
                lambda_fr = (w_fr*car.Rw - Vx_fr) / w_fr*car.Rw;
            end

            if (w_rl*car.Rw - Vx_rl) < 0
                lambda_rl = (w_rl*car.Rw - Vx_rl) / Vx_rl;
            else
                lambda_rl = (w_rl*car.Rw - Vx_rl) / w_rl*car.Rw;
            end

            if (w_rr*car.Rw - Vx_rr) < 0
                lambda_rr = (w_rr*car.Rw - Vx_rr) / Vx_rr;
            else
                lambda_rr = (w_rr*car.Rw - Vx_rr) / w_rr*car.Rw;
            end

            longSlip = [lambda_fl lambda_fr lambda_rl lambda_rr];
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

