function pos = gen_parabola(phase, start_pos, end_pos, max_clearance)
    x = (1 - phase) * start_pos(1) + phase * end_pos(1);    
    mid_phase = 0.5;
    mid_pos = max(end_pos(2), start_pos(2)) + max_clearance;
    delta_1 = mid_pos - start_pos(2);
    delta_2 = end_pos(2) - start_pos(2);
    delta_3 = mid_phase^2 - mid_phase;
    coef_a = (delta_1 - delta_2 * mid_phase) / delta_3;
    coef_b = (delta_2 * mid_phase^2 - delta_1) / delta_3;
    coef_c = start_pos(2);
    y = coef_a * phase^2 + coef_b * phase + coef_c;
    pos = [x,y];
end