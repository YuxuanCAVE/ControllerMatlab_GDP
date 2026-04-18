function delta = pure_pursuit_lateral(x, y, yaw, v, ref, L, p, idx_hint, window)
    if nargin < 8
        idx_hint = [];
    end
    if nargin < 9
        window = [];
    end

    idx0 = nearest_path_ref_point(x, y, ref.x, ref.y, idx_hint, window);

    Ld = p.Ld_min + p.Ld_gain * max(v, 0);
    Ld = min(max(Ld, p.Ld_min), p.Ld_max);

    idx = idx0;
    dist_sum = 0;
    while idx < numel(ref.x)
        ds = hypot(ref.x(idx+1) - ref.x(idx), ref.y(idx+1) - ref.y(idx));
        dist_sum = dist_sum + ds;
        idx = idx + 1;
        if dist_sum >= Ld
            break;
        end
    end

    tx = ref.x(idx);
    ty = ref.y(idx);
    alpha = angle_wrap(atan2(ty - y, tx - x) - yaw);

    delta_pp = atan2(2 * L * sin(alpha), max(Ld, 1e-3));
    delta_ff = p.delta_ff_gain * atan(L * ref.kappa(idx));

    delta = p.k_pp * delta_pp + delta_ff;
end
