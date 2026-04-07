function v_smooth = smooth_speed_profile(v_ref, dt, a_max, a_min)
% SMOOTH_SPEED_PROFILE  Rate-limit a speed reference to respect acceleration bounds.
%
%   v_smooth = smooth_speed_profile(v_ref, dt, a_max, a_min)
%
%   Converts step-like speed transitions into physically achievable ramps
%   by clamping the rate of change to [a_min, a_max] m/s^2.
%
%   Inputs:
%       v_ref  - original speed reference vector (from .mat file)
%       dt     - timestep (s), used to compute per-step speed change
%       a_max  - maximum acceleration (m/s^2), e.g. 2.0
%       a_min  - maximum deceleration (m/s^2), e.g. -3.0 (negative)
%
%   Output:
%       v_smooth - smoothed speed reference that the vehicle can actually follow
%
%   Example:
%       v_smooth = smooth_speed_profile(ref.v_ref, 0.05, 1.8, -2.5);
%
%   Note: Using slightly lower limits than the actual vehicle capability
%   (e.g. a_max=1.8 instead of 2.0) leaves headroom for the feedback
%   controller to correct small errors without saturating.

    if nargin < 4
        a_min = -3.0;
    end
    if nargin < 3
        a_max = 2.0;
    end

    n = numel(v_ref);
    v_smooth = zeros(n, 1);
    v_smooth(1) = v_ref(1);

    dv_max = a_max * dt;   % max speed increase per step
    dv_min = a_min * dt;   % max speed decrease per step (negative)

    for k = 2:n
        dv_desired = v_ref(k) - v_smooth(k-1);
        dv_clamped = min(max(dv_desired, dv_min), dv_max);
        v_smooth(k) = v_smooth(k-1) + dv_clamped;
    end

    v_smooth = max(v_smooth, 0);  % speed cannot be negative
end