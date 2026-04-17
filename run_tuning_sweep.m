%% run_tuning_sweep.m
%  GDP sweep for the retained lateral controller family.
%  Acceleration limits derived from actuator .mat files.
%
%  Usage:  run_tuning_sweep          % run all
%          run_tuning_sweep(true)    % dry-run

function run_tuning_sweep(dry_run)
    if nargin < 1, dry_run = false; end

    project_root = fileparts(which('main'));
    if isempty(project_root), project_root = pwd; end
    addpath(fullfile(project_root,'config'));
    addpath(fullfile(project_root,'reference'));
    addpath(fullfile(project_root,'controllers','lateral'));
    addpath(fullfile(project_root,'controllers','longitudinal'));
    addpath(fullfile(project_root,'model'));
    addpath(fullfile(project_root,'simulation'));
    addpath(fullfile(project_root,'plotting'));
    addpath(fullfile(project_root,'utils'));

    LAT_LIM = 0.6; LON_LIM = 0.8;

    cfg_base = default_config();
    ref_base = load_reference_path(cfg_base.ref.path_file);
    veh = load_vehicle_params(cfg_base.vehicle.accel_map_file, cfg_base.vehicle.brake_map_file);
    veh.max_steer = cfg_base.vehicle.max_steer;
    veh.max_steer_rate = cfg_base.vehicle.max_steer_rate;

    a_max_map = veh.a_max_from_map;
    a_min_map = veh.a_min_from_map;
    cfg_base.lon_pid.a_max = a_max_map; cfg_base.lon_pid.a_min = a_min_map;
    cfg_base.lon_lqr.a_max = a_max_map; cfg_base.lon_lqr.a_min = a_min_map;

    combos = {
        struct('lateral',"mpc_kinematic",'lon',"pid",'label',"mpc_kinematic+pid");
        struct('lateral',"mpc_kinematic",'lon',"lqr",'label',"mpc_kinematic+lqr");
        struct('lateral',"mpc_kinematic",'lon',"lqr_force_balance",'label',"mpc_kinematic+lqr_force");
    };

    speed_conditions = {};
    speed_conditions{end+1} = struct('mode',"constant",'value',0.5,'file',"",'scenario',"GDP-S1",'peak_speed',0.5);
    speed_conditions{end+1} = struct('mode',"constant",'value',2.0,'file',"",'scenario',"GDP-S2",'peak_speed',2.0);
    pf = dictionary(3,fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_3.mat'),...
        4,fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_4.mat'),...
        5,fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_5.mat'),...
        7,fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_7.mat'));
    for s = [3,4,5,7]
        speed_conditions{end+1} = struct('mode',"profile",'value',s,'file',pf(s),'scenario',"GDP-S3",'peak_speed',s); %#ok
    end
    speed_conditions{end+1} = struct('mode',"constant",'value',10.0,'file',"",'scenario',"GDP-S3-EXT",'peak_speed',10.0);

    cases = {};
    for ci = 1:numel(combos)
        for si = 1:numel(speed_conditions)
            cases{end+1} = struct('combo',combos{ci},'speed',speed_conditions{si}); %#ok
        end
    end
    n_cases = numel(cases);

    fprintf('================================================================\n');
    fprintf('  GDP SWEEP: %d combos x %d speeds = %d cases\n', numel(combos), numel(speed_conditions), n_cases);
    fprintf('  a_max=%.3f, a_min=%.3f (from actuator maps at 60%% pedal)\n', a_max_map, a_min_map);
    fprintf('================================================================\n\n');

    fprintf('%-5s %-20s %-8s %-8s\n','Case','Controller','Speed','Scenario');
    fprintf('%s\n',repmat('-',1,45));
    for i=1:n_cases
        fprintf('%-5d %-20s %-8.1f %s\n',i,char(cases{i}.combo.label),cases{i}.speed.peak_speed,char(cases{i}.speed.scenario));
    end
    fprintf('\n');
    if dry_run, fprintf('Dry run complete.\n'); return; end

    stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
    sweep_dir = fullfile(project_root,'run',sprintf('%s_GDP_sweep',stamp));
    if ~exist(sweep_dir,'dir'), mkdir(sweep_dir); end

    T = table(strings(n_cases,1),strings(n_cases,1),strings(n_cases,1),...
        zeros(n_cases,1),strings(n_cases,1),...
        zeros(n_cases,1),zeros(n_cases,1),zeros(n_cases,1),...
        zeros(n_cases,1),zeros(n_cases,1),zeros(n_cases,1),...
        zeros(n_cases,1),zeros(n_cases,1),...
        strings(n_cases,1),strings(n_cases,1),...
        strings(n_cases,1),strings(n_cases,1),strings(n_cases,1),...
        'VariableNames',{'Controller','Longitudinal','GDP_Scenario',...
        'Peak_Speed_mps','Speed_Mode',...
        'RMS_CTE_m','Peak_CTE_m','RMS_Heading_Err_deg',...
        'RMS_Speed_Err_mps','Peak_Speed_Err_mps','Peak_Lon_Dev_m',...
        'RMS_Lon_Dev_m','Loop_Time_s',...
        'Goal_Reached','Termination',...
        'REQ1_Lat_Pass','REQ1_Lon_Pass','Overall_Pass'});

    all_results = cell(n_cases,1);

    for i = 1:n_cases
        cb = cases{i}.combo; sc = cases{i}.speed;
        fprintf('\n[%2d/%d] %s @ %.1f (%s) ...',i,n_cases,char(cb.label),sc.peak_speed,char(sc.scenario));
        t0 = tic;
        cfg = cfg_base;
        cfg.controller.lateral = cb.lateral;
        cfg.controller.longitudinal = cb.lon;
        cfg.speed.mode = sc.mode;
        if sc.mode=="constant", cfg.speed.constant_value = sc.value;
        else, cfg.speed.profile_file = sc.file; end
        if sc.peak_speed<=1, cfg.sim.T_end=400; cfg.sim.max_travel_time=400;
        elseif sc.peak_speed<=3, cfg.sim.T_end=250; cfg.sim.max_travel_time=250;
        else, cfg.sim.T_end=150; cfg.sim.max_travel_time=150; end

        ref = ref_base; ref = load_reference_speed(ref, cfg.speed);

        try
            result = run_closed_loop(cfg, ref, veh);
            m = result.metrics; dt_s = cfg.sim.dt;
            se = result.log.v_ref - result.log.v;
            ld = cumsum(se)*dt_s; pk_ld = max(abs(ld)); rm_ld = sqrt(mean(ld.^2));
            lok = m.peak_cte<LAT_LIM; lonk = pk_ld<LON_LIM;

            T.Controller(i)=cb.label; T.Longitudinal(i)=cb.lon;
            T.GDP_Scenario(i)=sc.scenario; T.Peak_Speed_mps(i)=sc.peak_speed;
            T.Speed_Mode(i)=sc.mode;
            T.RMS_CTE_m(i)=round(m.rms_cte,4); T.Peak_CTE_m(i)=round(m.peak_cte,4);
            T.RMS_Heading_Err_deg(i)=round(m.rms_epsi_deg,4);
            T.RMS_Speed_Err_mps(i)=round(m.rms_speed_error,4);
            T.Peak_Speed_Err_mps(i)=round(m.peak_speed_error,4);
            T.Peak_Lon_Dev_m(i)=round(pk_ld,4); T.RMS_Lon_Dev_m(i)=round(rm_ld,4);
            T.Loop_Time_s(i)=round(m.single_loop_time_s,2);
            T.Goal_Reached(i)=string(m.goal_reached); T.Termination(i)=m.termination_reason;
            T.REQ1_Lat_Pass(i)=iff(lok,"PASS","FAIL");
            T.REQ1_Lon_Pass(i)=iff(lonk,"PASS","FAIL");
            T.Overall_Pass(i)=iff(lok&&lonk,"PASS","FAIL");
            all_results{i}=result; all_results{i}.lon_dev=ld;

            cd_tag = sprintf('case_%02d_%s_%.0f',i,strrep(char(cb.label),'+','_'),sc.peak_speed*10);
            cd = fullfile(sweep_dir,cd_tag); if ~exist(cd,'dir'), mkdir(cd); end
            save(fullfile(cd,'result.mat'),'cfg','ref','veh','result','ld','pk_ld','rm_ld');
            fprintf(' %.1fs | CTE=%.3f[%s] LonDev=%.3f[%s]',toc(t0),...
                m.peak_cte,iff(lok,"OK","X"),pk_ld,iff(lonk,"OK","X"));
        catch ME
            fprintf(' ERR: %s',ME.message);
            T.Controller(i)=cb.label; T.Longitudinal(i)=cb.lon;
            T.GDP_Scenario(i)=sc.scenario; T.Peak_Speed_mps(i)=sc.peak_speed;
            T.Speed_Mode(i)=sc.mode; T.Termination(i)="error: "+string(ME.message);
            T.REQ1_Lat_Pass(i)="ERROR"; T.REQ1_Lon_Pass(i)="ERROR"; T.Overall_Pass(i)="ERROR";
        end
    end

    writetable(T, fullfile(sweep_dir,'GDP_sweep_results.csv'));
    save(fullfile(sweep_dir,'GDP_sweep_results.mat'),'T','all_results','cases');

    fprintf('\n\n================================================================\n');
    fprintf('  RESULTS (a_lim from maps: [%.3f, %.3f] m/s^2)\n', a_min_map, a_max_map);
    fprintf('================================================================\n\n');
    fprintf('%-20s %-8s %-6s %-10s %-8s %-12s %-8s %-8s\n',...
        'Controller','Scen','Speed','PeakCTE','LatOK','PkLonDev','LonOK','Overall');
    fprintf('%s\n',repmat('-',1,95));
    for i=1:n_cases
        fprintf('%-20s %-8s %-6.1f %-10.3f %-8s %-12.3f %-8s %-8s\n',...
            char(T.Controller(i)),char(T.GDP_Scenario(i)),T.Peak_Speed_mps(i),...
            T.Peak_CTE_m(i),char(T.REQ1_Lat_Pass(i)),...
            T.Peak_Lon_Dev_m(i),char(T.REQ1_Lon_Pass(i)),char(T.Overall_Pass(i)));
    end

    ctrls = unique(T.Controller,'stable');
    fprintf('\n--- PASS RATE BY CONTROLLER ---\n');
    for ci=1:numel(ctrls)
        m=T.Controller==ctrls(ci); nt=sum(m); np=sum(T.Overall_Pass(m)=="PASS");
        fprintf('  %-20s: %d/%d (%.0f%%)\n',char(ctrls(ci)),np,nt,100*np/max(nt,1));
    end

    scens = unique(T.GDP_Scenario,'stable');
    fprintf('\n--- BEST PER SCENARIO ---\n');
    for si=1:numel(scens)
        m=T.GDP_Scenario==scens(si); sub=T(m,:);
        if height(sub)>0
            [~,bi]=min(sub.Peak_CTE_m+sub.Peak_Lon_Dev_m);
            fprintf('  %s: %s (CTE=%.3f, LonDev=%.3f)\n',char(scens(si)),...
                char(sub.Controller(bi)),sub.Peak_CTE_m(bi),sub.Peak_Lon_Dev_m(bi));
        end
    end

    % Save text summary
    fid = fopen(fullfile(sweep_dir,'GDP_compliance_summary.txt'),'w');
    fprintf(fid,'GDP COMPLIANCE SUMMARY\nGenerated: %s\n\n',char(datetime('now')));
    fprintf(fid,'Accel limits from maps: a_max=%.3f, a_min=%.3f m/s^2\n',a_max_map,a_min_map);
    fprintf(fid,'Thresholds: Lateral<%.1fm, Longitudinal<%.1fm\n\n',LAT_LIM,LON_LIM);
    fprintf(fid,'%-20s %-8s %-6s %-10s %-8s %-12s %-8s %-8s\n',...
        'Controller','Scen','Speed','PeakCTE','LatOK','PkLonDev','LonOK','Overall');
    fprintf(fid,'%s\n',repmat('-',1,95));
    for i=1:n_cases
        fprintf(fid,'%-20s %-8s %-6.1f %-10.4f %-8s %-12.4f %-8s %-8s\n',...
            char(T.Controller(i)),char(T.GDP_Scenario(i)),T.Peak_Speed_mps(i),...
            T.Peak_CTE_m(i),char(T.REQ1_Lat_Pass(i)),...
            T.Peak_Lon_Dev_m(i),char(T.REQ1_Lon_Pass(i)),char(T.Overall_Pass(i)));
    end
    np=sum(T.Overall_Pass=="PASS");
    fprintf(fid,'\nOVERALL: %d/%d passed (%.0f%%)\n',np,height(T),100*np/height(T));
    fclose(fid);

    fprintf('\n\nOutputs saved to: %s\n', sweep_dir);
end

function r = iff(c,t,f), if c, r=t; else, r=f; end, end
