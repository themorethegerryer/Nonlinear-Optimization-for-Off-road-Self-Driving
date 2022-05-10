% MPC iLQR
% x = [x y theta]
% u = [delta v]

function [X,U,K,P] = MPC_iLQR(x0, xg, model, N, dt, Q, R, Qf)
    if nargin < 3
        model = KinematicBicycleModel();
        N = 100;
        dt = 0.1;
        Q = diag([1 1 1]);
        R = diag([0.1 0.1]);
        Qf = diag([10 10 1]);
    end
    nx = length(x0);
    nu = 2;
    
    X = zeros(nx,N);
    X(:,1) = x0;
    Xref = ones(nx,N).*xg;
    
    Uref = zeros(nu,N-1);
    U = Uref(:,:);
    
    iter = -1;
    [d, K, P, del_J] = backward_pass(model,X,U,Xref,Uref,Q,R,Qf,N);
    iter = iter + 1;
    while iter < 200 && max(vecnorm(d)) > 6
        [X, U, Jr, alpha] = forward_pass(model,X,U,Xref,Uref,K,d,del_J,Q,R,Qf,N,dt);
        iter = iter + 1;
        [d, K, P, del_J] = backward_pass(model,X,U,Xref,Uref,Q,R,Qf,N);
        disp(iter)
        disp(max(vecnorm(d)))
    end
end

function [d, K, P, del_J] = backward_pass(model,X,U,Xref,Uref,Q,R,Qf,N)
    nx = length(X(:,1));
    nu = length(U(:,1));
    P = zeros(nx,nx,N);
    p = zeros(nx,N);
    d = ones(nu,N-1);
    K = zeros(nu,nx,N-1);
    del_J = 0.0;
    
    p(:,N) = Qf*(X(:,N)-Xref(:,N));
    P(:,:,N) = Qf;
    
    for k = (N-1):-1:1
        q = Q*(X(:,k)-Xref(:,k));
        r = R*(U(:,k)-Uref(:,k));
        
        jac = model.discrete_jacobian(X(:,k),U(:,k));
        A = jac.A;
        B = jac.B;
        
        gx = q + A'*p(:,k+1);
        gu = r + B'*p(:,k+1);
        
        Gxx = Q + A'*P(:,:,k+1)*A;
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

function [Xn, Un, Jn, alpha] = forward_pass(model,X,U,Xref,Uref,K,d,del_J,Q,R,Qf,N,dt)
    Xn = X(:,:);
    Un = U(:,:);
    alpha = 1.0;
    J = trajectory_cost(X,U,Xref,Uref,Q,R,Qf);

    for k = 1:(N-1)
        Un(:,k) = U(:,k) - alpha*d(:,k) - K(:,:,k)*(Xn(:,k)-X(:,k));
        Xn(:,k+1) = model.dynamics_rk4(Xn(:,k),Un(:,k),dt);
    end
    Jn = trajectory_cost(Xn,Un,Xref,Uref,Q,R,Qf);
    
    linesearch_iters = 0;
    while isnan(Jn) || Jn > (J - 1e-2*alpha*del_J)
        linesearch_iters = linesearch_iters + 1;
        alpha = 0.5*alpha;
        for k = 1:(N-1)
            Un(:,k) = U(:,k) - alpha*d(:,k) - K(:,:,k)*(Xn(:,k)-X(:,k));
            Xn(:,k+1) = model.dynamics_rk4(Xn(:,k),Un(:,k),dt);
        end
        Jn = trajectory_cost(Xn,Un,Xref,Uref,Q,R,Qf);
        if linesearch_iters >= 10
            break
        end
    end
end

function J = stage_cost(x,u,xref,uref,Q,R)
    J = 0.5*(x-xref)'*Q*(x-xref)+0.5*(u-uref)'*R*(u-uref);
end

function J = term_cost(x,xref,Qf)
    J = 0.5*(x-xref)'*Qf*(x-xref);
end

function J = trajectory_cost(X,U,Xref,Uref,Q,R,Qf)
    J = 0;
    for k = 1:length(U)
        stage_cost_J = stage_cost(X(:,k),U(:,k),Xref(:,k),Uref(:,k),Q,R);
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