function G = attitude_jacobian(q)
    disp(q)
    s = q(1);
    v = q(2:4);
    vhat = [0 -q(4) q(3); q(4) 0 -q(2); -q(3) q(2) 0];
    G = zeros(4,3);
    G(1,:) = -v';
    G(2:4,:) = s.*eye(3)+vhat;
end