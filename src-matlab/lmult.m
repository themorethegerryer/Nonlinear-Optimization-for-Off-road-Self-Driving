function Lq = lmult(q)
    s = q(1);
    v = q(2:4);
    vhat = [0 -q(4) q(3); q(4) 0 -q(2); -q(3) q(2) 0];
    Lq = zeros(4,4);
    Lq(1,1) = s;
    Lq(1,2:4) = -v';
    Lq(2:4,1) = v;
    Lq(2:4,2:4) = s*eye(3)+vhat;
end