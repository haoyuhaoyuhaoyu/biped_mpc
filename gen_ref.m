function state_ref = gen_ref(pcx, pcy, vx, vy, desired_height, desired_vx, desired_vy, p_h, Ts)
    % p_h prediction horizon
    % state dim 6 [pcx, pcy, sita, dpcx, dpcy, dsita]
    state_ref = zeros(p_h, 6);
    for i = 1:p_h
        state_ref(i, 1) = pcx + desired_vx * Ts * (i-1);
    end
    for i = 1:p_h
        state_ref(i, 2) = (p_h-i)/p_h*pcy + i/p_h*desired_height;
    end
    for i = 1:p_h
        state_ref(i, 3) = 0;
    end
    for i = 1:p_h
        state_ref(i, 4) = (1-i/p_h)*vx + i/p_h*desired_vx;
    end
    for i = 1:p_h
        state_ref(i, 5) = (1-i/p_h)*vy + i/p_h*desired_vy;
    end
    for i = 1:p_h
        state_ref(i, 6) = 0;
    end
end