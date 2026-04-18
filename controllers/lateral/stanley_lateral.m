function delta = stanley_lateral(x, y, yaw, v, ref, L, p, idx_hint, window)
    if nargin < 8
        idx_hint = [];
    end
    if nargin < 9
        window = [];
    end

    [idx, xr, yr, psi_ref] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);
    kappa = ref.kappa(idx);
    e_ct = -sin(psi_ref) * (x - xr) + cos(psi_ref) * (y - yr);
    e_psi = angle_wrap(psi_ref - yaw);

    delta_ff = p.delta_ff_gain * atan(L * kappa);
    delta_fb = p.k_yaw * e_psi + atan2(p.k_cte * e_ct, p.k_soft + max(v, 0.5));

    delta = delta_ff + delta_fb;
end
