function [K,P] = tvlqr(A,B,Q,R,Qf)
    T = length(A)+1;
    [n,m] = size(B(1));
    K = zeros(m,n,T-1);
    P = zeros(n,n,T);
    
    P(length(P)) = Qf;
    for k = T-1:-1:1
        K(:,:,k) = (R + B(:,:,k)'*P(:,:,k+1)*B(:,:,k))\(B(:,:,k)'*P(:,:,k+1)*A(:,:,k));
        P(:,:,k) = Q+ A(:,:,k)'*P(:,:,k+1)*A(:,:,k) - A(:,:,k)'*P(:,:,k+1)*B(:,:,k)*K(:,:,k);
    end
end