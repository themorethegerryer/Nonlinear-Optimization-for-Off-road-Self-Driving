function K = lqr(A,B,Q,R)
    [n,m] = size(B);
    K = zeros(m,n);
    K_prev = K;
    
    P = Q;
    for k = 1:200
        K = (R + B'*P*B) \ (B'*P*A);
        P = Q + A'*P*A - A'*P*B*K;
        if norm(K-K_prev,Inf) < 1e-6
            break
        end
        K_prev = K;
    end
end