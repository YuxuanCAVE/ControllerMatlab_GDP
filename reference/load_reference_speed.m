function ref = load_reference_speed(ref, speed_cfg)
    switch speed_cfg.mode
        case "constant"
            ref.v_ref = speed_cfg.constant_value * ones(size(ref.x));
            ref.v_ref_mode = "constant";

        case "profile"
            S = load(speed_cfg.profile_file);
            if ~isfield(S, 'pathv_ref')
                error('Speed profile file must contain pathv_ref: %s', speed_cfg.profile_file);
            end

            v_profile = S.pathv_ref(26:384).';

            if numel(v_profile) ~= numel(ref.x)
                error(['Speed profile length mismatch after slicing pathv_ref(26:384). ', ...
                       'Expected %d samples, got %d from file: %s'], ...
                       numel(ref.x), numel(v_profile), speed_cfg.profile_file);
            end

            ref.v_ref = v_profile;
            ref.v_ref_mode = "profile";
            ref.v_ref_source = speed_cfg.profile_file;

        otherwise
            error('Unknown speed reference mode: %s', speed_cfg.mode);
    end

    ref.v_ref = ref.v_ref(:);

    % ── Smooth the speed profile to respect acceleration limits ──────
    % Converts step-like transitions into physically achievable ramps.
    % Uses slightly below actual vehicle limits to leave headroom for
    % the feedback controller (PID/LQR) to correct without saturating.
    ref.v_ref_raw = ref.v_ref;   % keep original for comparison
    ref.v_ref = smooth_speed_profile(ref.v_ref, 0.05, 1.8, -2.5);
end