% MPC iLQR
% x = [x y theta]
% u = [delta v]

% XDouble = [uy r ux dFzlong dFzlat delta xPos yPos yawOrient]
function [u0,U,X] = MPC_iLQR(XDouble, XrefDouble, U)
    x0 = XDouble(7:9);
    xg = XrefDouble(7:9);

    model = KinematicBicycleModel();
    N = length(U)+1;
    dt = 0.1;
    Q = diag([1 1 1]);
    R = diag([1 2]);
    Qf = diag([1 1 1]);
    
    nx = length(x0);
    nu = length(R);
    
    X = zeros(nx,N);
    X(:,1) = x0;
    Xref = zeros(nx,N);
    for k = 1:N
        Xref(:,k) = xg;
    end
    Uref = zeros(nu,N-1);
    
    iter = -1;
    [d, K, del_J] = backward_pass(model,X,U,Xref,Uref,Q,R,Qf,N);
    iter = iter + 1;
    while iter < 20 && max(vecnorm(d)) > 1
        [X, U] = forward_pass(model,X,U,Xref,Uref,K,d,del_J,Q,R,Qf,N,dt);
        iter = iter + 1;
        [d, K, del_J] = backward_pass(model,X,U,Xref,Uref,Q,R,Qf,N);
        disp(iter)
        disp(max(vecnorm(d)))
    end
    u0 = U(:,1);
end

function [d, K, del_J] = backward_pass(model,X,U,Xref,Uref,Q,R,Qf,N)
    nx = length(X(:,1));
    nu = length(U(:,1));
    P = zeros(nx,nx,N);
    p = zeros(nx,N);
    d = ones(nu,N-1);
    K = zeros(nu,nx,N-1);
    del_J = 0.0;
    
    [P(:,:,N), p(:,N)] = term_cost_expansion(X(:,length(X)),Xref(:,length(X)),Qf);
    
    for k = (N-1):-1:1
        jac = model.discrete_jacobian(X(:,k),U(:,k));
        A = jac.A;
        B = jac.B;
        
        [Jxx, Jx, Juu, Ju] = stage_cost_expansion(X(:,k),U(:,k),Xref(:,k),Uref(:,k), Q, R);
        
        gx = Jx + A'*p(:,k+1);
        gu = Ju + B'*p(:,k+1);
        
        Gxx = Jxx + A'*P(:,:,k+1)*A;
        Guu = R + B'*P(:,:,k+1)*B;
        Gxu = A'*P(:,:,k+1)*B;
        Gux = B'*P(:,:,k+1)*A;
        
        d(:,k) = Guu\gu;
        K(:,:,k) = Guu\Gux;
        
        p(:,k) = gx - K(:,:,k)'*gu + K(:,:,k)'*Guu*d(:,k) - Gxu*d(:,k);
        P(:,:,k) = Gxx + K(:,:,k)'*Guu*K(:,:,k) - Gxu*K(:,:,k) - K(:,:,k)'*Gux;
        
        del_J = del_J + gu'*d(:,k);
    end
end

function [Xn, Un] = forward_pass(model,X,U,Xref,Uref,K,d,del_J,Q,R,Qf,N,dt)
    Xn = X(:,:);
    Un = U(:,:);
    alpha = 1.0;
    J = trajectory_cost(model,X,U,Xref,Uref,Q,R,Qf);

    for k = 1:(N-1)
        Un(:,k) = U(:,k) - alpha*d(:,k) - K(:,:,k)*(Xn(:,k)-X(:,k));
        Xn(:,k+1) = model.dynamics_rk4(Xn(:,k),Un(:,k),dt);
    end
    Jn = trajectory_cost(model,Xn,Un,Xref,Uref,Q,R,Qf);
    
    linesearch_iters = 0;
    while isnan(Jn) || Jn > (J - 1e-2*alpha*del_J)
        linesearch_iters = linesearch_iters + 1;
        alpha = 0.5*alpha;
        for k = 1:(N-1)
            Un(:,k) = U(:,k) - alpha*d(:,k) - K(:,:,k)*(Xn(:,k)-X(:,k));
            Xn(:,k+1) = model.dynamics_rk4(Xn(:,k),Un(:,k),dt);
        end
        Jn = trajectory_cost(model,Xn,Un,Xref,Uref,Q,R,Qf);
        if linesearch_iters >= 10
            break
        end
    end
end

function J = stage_cost(model,x,u,xref,uref,Q,R)
%     terrain_cost = CostFunction(model,x(1),x(2),x(3),u(2));
    J = 0.5*(x-xref)'*Q*(x-xref)+0.5*(u-uref)'*R*(u-uref);
end

function J = term_cost(x,xref,Qf)
    J = 0.5*(x-xref)'*Qf*(x-xref);
end

function J = trajectory_cost(model,X,U,Xref,Uref,Q,R,Qf)
    J = 0;
    for k = 1:length(U)
        stage_cost_J = stage_cost(model,X(:,k),U(:,k),Xref(:,k),Uref(:,k),Q,R);
        J = J + stage_cost_J;
    end
    term_cost_J = term_cost(X(:,length(X)),Xref(:,length(X)),Qf);
    J = J + term_cost_J;
end

function [Jxx, Jx, Juu, Ju] = stage_cost_expansion(x,u,xref,uref, Q, R)
    Jxx = Q;
    Jx = Q*(x-xref);
    Juu = R;
    Ju = R*(u-uref);
end

function [Jxx, Jx] = term_cost_expansion(x,xref, Qf)
    Jxx = Qf;
    Jx = Qf*(x-xref);
end