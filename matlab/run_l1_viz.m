% run_l1_viz.m  –  CanSat L1 guidance & motor real-time visualization.
%
% File-based IPC: reads viz_data.json written by sim_matlab_bridge.py.
% No toolboxes required. Close the figure to stop.
%
% Run tests/sim_matlab_bridge.py first, then run this script.

% Stop ALL previously running viz timers BEFORE close all,
% so no callback is mid-render when figures are destroyed.
for tname = {'L1VizTimer','L1ShotTimer'}
    told = timerfind('Name',tname{1});
    for ti = 1:length(told)
        try; stop(told(ti)); delete(told(ti)); catch; end
    end
end
close all; clc;

% ── Configuration ──────────────────────────────────────────────────────────────
DATA_FILE   = 'C:\workspace\CANSAT_AAS_2026_FSW\matlab\viz_data.json';
TRACE_FILE  = 'C:\workspace\CANSAT_AAS_2026_FSW\matlab\viz_trace.json';
SHOT_FILE   = 'C:\workspace\CANSAT_AAS_2026_FSW\matlab\viz_live.png';
HISTORY_LEN = 300;
ARM_LEN     = 0.75;
NEUTRAL_DEG = 80.0;
arm_cx      = [-0.75, 0.75];
arm_cy      = [0.15,  0.15];

use_batch_trace = exist(TRACE_FILE, 'file') == 2;

if ~use_batch_trace && ~exist(DATA_FILE, 'file')
    fprintf("Waiting for %s ...\n", DATA_FILE);
    for k = 1:40
        pause(0.5);
        if exist(DATA_FILE, 'file'), break; end
    end
end
if ~use_batch_trace && ~exist(DATA_FILE, 'file')
    error('Data file not found. Start sim_matlab_bridge.py first.');
end

% ── Create figure ──────────────────────────────────────────────────────────────
fig = figure("Name", "CanSat L1 Guidance Verification", ...
             "NumberTitle", "off", ...
             "Position", [50, 50, 1500, 860], ...
             "Color", [0.13 0.13 0.13]);

% ══════════════════════════════════════════════════════════════════════════════
% Panel 1 – Top-down trajectory map
% ══════════════════════════════════════════════════════════════════════════════
ax_map = subplot(2, 3, [1, 2]);
hold(ax_map, "on"); grid(ax_map, "on"); axis(ax_map, "equal");
ax_map.Color = [0.07 0.07 0.10]; ax_map.GridColor = [0.33 0.33 0.33];
ax_map.XColor = [0.82 0.82 0.82]; ax_map.YColor = [0.82 0.82 0.82];
xlabel(ax_map, "East  (m)",  "Color", [0.78 0.78 0.78]);
ylabel(ax_map, "North (m)", "Color", [0.78 0.78 0.78]);
title(ax_map, "Trajectory  |  waiting for data ...", "Color", [0.95 0.95 0.95], "FontSize", 10);
th_circ     = linspace(0, 2*pi, 120);
ln_refpath  = plot(ax_map, [0 0],[0 0], "--","Color",[.50 .50 .50],"LineWidth",1.0,"DisplayName","Ref path");
ln_trail    = plot(ax_map, NaN, NaN, "-", "Color",[.28 .52 1.00],"LineWidth",1.5,"DisplayName","Trajectory");
sc_carrot_h = scatter(ax_map, NaN, NaN, 14,[1.0 .55 0.0],"filled","DisplayName","Carrot trail");
ln_l1circle = plot(ax_map, NaN, NaN,"--","Color",[.55 .90 .55],"LineWidth",0.9,"DisplayName","L1 circle");
ln_tocarrot = plot(ax_map, NaN, NaN, "-", "Color",[1.0 .55 0.0],"LineWidth",1.3,"DisplayName","to Carrot");
qv_heading  = quiver(ax_map,0,0,0,8,"Color",[.95 .95 .35],"LineWidth",2,"MaxHeadSize",2,"AutoScale","off","DisplayName","Heading");
pl_carrot   = plot(ax_map, NaN,NaN,"d","Color",[1.0 .55 0.0],"MarkerFaceColor",[1.0 .55 0.0],"MarkerSize",11,"DisplayName","Carrot");
pl_pos      = plot(ax_map, NaN,NaN,"o","Color",[.28 .52 1.00],"MarkerFaceColor",[.28 .52 1.00],"MarkerSize",11,"DisplayName","Position");
              plot(ax_map, 0,0,"^","Color",[.38 1.00 .38],"MarkerFaceColor",[.38 1.00 .38],"MarkerSize",13,"DisplayName","Start");
pl_target   = plot(ax_map, NaN,NaN,"p","Color",[1.00 .22 .22],"MarkerFaceColor",[1.00 .22 .22],"MarkerSize",17,"DisplayName","Target");
qv_wind     = quiver(ax_map,0,0,0,0,"Color",[.90 .90 .30],"LineWidth",2.5,"MaxHeadSize",3,"AutoScale","off","DisplayName","Wind");
txt_wind    = text(ax_map,0,0,"wind","Color",[.90 .90 .30],"FontSize",8,"HorizontalAlignment","left");
legend(ax_map,"Location","northeast","TextColor",[.83 .83 .83],"Color",[.13 .13 .13],"FontSize",7);
xlim(ax_map,[-50 50]); ylim(ax_map,[-50 50]);

% ══════════════════════════════════════════════════════════════════════════════
% Panel 2 – Motor arms
% ══════════════════════════════════════════════════════════════════════════════
ax_arm = subplot(2, 3, 3);
hold(ax_arm,"on"); axis(ax_arm,"off"); axis(ax_arm,"equal");
ax_arm.Color = [0.07 0.07 0.10];
xlim(ax_arm,[-1.55 1.55]); ylim(ax_arm,[-1.05 1.30]);
title(ax_arm,"Motor Arms  (0deg = up,  80deg = neutral,  160deg = max brake)","Color",[.95 .95 .95],"FontSize",9);
arm_colors = {[.28 .52 1.00],[1.00 .33 .33]};
arm_labels  = {"LEFT","RIGHT"};
ln_arm  = cell(1,2); txt_arm = cell(1,2);
for k = 1:2
    cx = arm_cx(k); cy = arm_cy(k);
    th_arc = linspace(0,deg2rad(160),80);
    plot(ax_arm,cx+ARM_LEN.*sin(th_arc),cy+ARM_LEN.*cos(th_arc),"-","Color",[.42 .42 .42],"LineWidth",0.9);
    for ref_deg = [0, NEUTRAL_DEG, 160]
        plot(ax_arm,[cx, cx+ARM_LEN*1.08*sin(deg2rad(ref_deg))],[cy, cy+ARM_LEN*1.08*cos(deg2rad(ref_deg))],"--","Color",[.48 .48 .48],"LineWidth",0.7);
        text(ax_arm,cx+ARM_LEN*1.26*sin(deg2rad(ref_deg)),cy+ARM_LEN*1.22*cos(deg2rad(ref_deg)),sprintf("%gdeg",ref_deg),"HorizontalAlignment","center","FontSize",7,"Color",[.68 .68 .68]);
    end
    plot(ax_arm,cx,cy,"o","Color",[.78 .78 .78],"MarkerFaceColor",[.78 .78 .78],"MarkerSize",6);
    ex0 = cx+ARM_LEN*sin(deg2rad(NEUTRAL_DEG)); ey0 = cy+ARM_LEN*cos(deg2rad(NEUTRAL_DEG));
    ln_arm{k}  = plot(ax_arm,[cx ex0],[cy ey0],"-","Color",arm_colors{k},"LineWidth",6);
    txt_arm{k} = text(ax_arm,cx,cy-0.52,sprintf("%.1fdeg",NEUTRAL_DEG),"HorizontalAlignment","center","FontSize",14,"FontWeight","bold","Color",arm_colors{k});
    text(ax_arm,cx,cy-0.72,arm_labels{k},"HorizontalAlignment","center","FontSize",10,"FontWeight","bold","Color",[.82 .82 .82]);
end
txt_pw_l = text(ax_arm,arm_cx(1),arm_cy(1)-0.90,"PW: - us","HorizontalAlignment","center","FontSize",8,"Color",[.70 .70 .70]);
txt_pw_r = text(ax_arm,arm_cx(2),arm_cy(2)-0.90,"PW: - us","HorizontalAlignment","center","FontSize",8,"Color",[.70 .70 .70]);

% ══════════════════════════════════════════════════════════════════════════════
% Panel 3 – yaw_rate + lat_acc
% ══════════════════════════════════════════════════════════════════════════════
ax_yaw = subplot(2,3,4);
hold(ax_yaw,"on"); grid(ax_yaw,"on");
ax_yaw.Color=[.07 .07 .10]; ax_yaw.GridColor=[.28 .28 .28];
ax_yaw.XColor=[.78 .78 .78]; ax_yaw.YColor=[.28 .52 1.00];
yline(ax_yaw,0,"--","Color",[.48 .48 .48],"LineWidth",0.8);
ln_yaw_cmd=plot(ax_yaw,NaN,NaN,"-","Color",[.28 .52 1.00],"LineWidth",1.6,"DisplayName","yaw rate cmd (rad/s)");
xlabel(ax_yaw,"Time (s)","Color",[.78 .78 .78]); ylabel(ax_yaw,"rad/s","Color",[.28 .52 1.00]);
title(ax_yaw,"yaw rate cmd  &  lat acc cmd","Color",[.92 .92 .92],"FontSize",9);
yyaxis(ax_yaw,"right"); ax_yaw.YColor=[1.00 .52 .18];
ln_lat_acc=plot(ax_yaw,NaN,NaN,"-","Color",[1.00 .52 .18],"LineWidth",1.6,"DisplayName","lat acc cmd (m/s2)");
ylabel(ax_yaw,"m/s2","Color",[1.00 .52 .18]);
legend(ax_yaw,"Location","best","TextColor",[.83 .83 .83],"Color",[.13 .13 .13],"FontSize",8);

% ══════════════════════════════════════════════════════════════════════════════
% Panel 4 – nu / nu1 / nu2
% ══════════════════════════════════════════════════════════════════════════════
ax_nu = subplot(2,3,5);
hold(ax_nu,"on"); grid(ax_nu,"on");
ax_nu.Color=[.07 .07 .10]; ax_nu.GridColor=[.28 .28 .28];
ax_nu.XColor=[.78 .78 .78]; ax_nu.YColor=[.83 .83 .83];
yline(ax_nu,0,"--","Color",[.48 .48 .48],"LineWidth",0.8);
ln_nu =plot(ax_nu,NaN,NaN,"-", "Color",[.28 .52 1.00],"LineWidth",1.6,"DisplayName","nu (total)");
ln_nu1=plot(ax_nu,NaN,NaN,"--","Color",[.92 .32 .32],"LineWidth",1.6,"DisplayName","nu1 (xtrack)");
ln_nu2=plot(ax_nu,NaN,NaN,":", "Color",[.38 .92 .38],"LineWidth",1.6,"DisplayName","nu2 (along)");
xlabel(ax_nu,"Time (s)","Color",[.78 .78 .78]); ylabel(ax_nu,"rad","Color",[.83 .83 .83]);
title(ax_nu,"nu  /  nu1  /  nu2  (rad)","Color",[.92 .92 .92],"FontSize",9);
legend(ax_nu,"Location","best","TextColor",[.83 .83 .83],"Color",[.13 .13 .13],"FontSize",8);

% ══════════════════════════════════════════════════════════════════════════════
% Panel 5 – crossTrack / alongTrack / L1_distance
% ══════════════════════════════════════════════════════════════════════════════
ax_trk = subplot(2,3,6);
hold(ax_trk,"on"); grid(ax_trk,"on");
ax_trk.Color=[.07 .07 .10]; ax_trk.GridColor=[.28 .28 .28];
ax_trk.XColor=[.78 .78 .78]; ax_trk.YColor=[.83 .83 .83];
yline(ax_trk,0,"--","Color",[.48 .48 .48],"LineWidth",0.8);
ln_cross =plot(ax_trk,NaN,NaN,"-", "Color",[.28 .52 1.00],"LineWidth",1.6,"DisplayName","crossTrack (m)");
ln_along =plot(ax_trk,NaN,NaN,"--","Color",[.92 .32 .32],"LineWidth",1.6,"DisplayName","alongTrack (m)");
ln_l1dist=plot(ax_trk,NaN,NaN,"-", "Color",[.38 .92 .38],"LineWidth",1.6,"DisplayName","L1 distance (m)");
xlabel(ax_trk,"Time (s)","Color",[.78 .78 .78]); ylabel(ax_trk,"m","Color",[.83 .83 .83]);
title(ax_trk,"crossTrack  /  alongTrack  /  L1 dist  (m)","Color",[.92 .92 .92],"FontSize",9);
legend(ax_trk,"Location","best","TextColor",[.83 .83 .83],"Color",[.13 .13 .13],"FontSize",8);

drawnow;

% ── State struct ───────────────────────────────────────────────────────────────
NaN_row = nan(1,HISTORY_LEN);
S = struct();
S.data_file    = DATA_FILE;
S.shot_file    = SHOT_FILE;
S.HISTORY_LEN  = HISTORY_LEN;
S.ARM_LEN      = ARM_LEN;
S.arm_cx       = arm_cx; S.arm_cy = arm_cy;
S.th_circ      = th_circ;
S.target_init  = false;
S.last_shot_t  = -999;
S.map_scale_m  = 0;          % set on first frame from target distance
S.all_pos_N    = []; S.all_pos_E = [];
S.TRAIL_MAX    = 600;        % max trajectory points kept on map
S.last_step    = -1;
S.h_t          =NaN_row; S.h_pos_N=NaN_row; S.h_pos_E=NaN_row;
S.h_carrot_N   =NaN_row; S.h_carrot_E=NaN_row;
S.h_yaw_cmd    =NaN_row; S.h_lat_acc=NaN_row;
S.h_nu         =NaN_row; S.h_nu1=NaN_row; S.h_nu2=NaN_row;
S.h_crossTrack =NaN_row; S.h_alongTrack=NaN_row; S.h_L1_distance=NaN_row;
S.ax_map=ax_map; S.ax_yaw=ax_yaw; S.ax_nu=ax_nu; S.ax_trk=ax_trk;
S.ln_refpath=ln_refpath; S.ln_trail=ln_trail; S.sc_carrot_h=sc_carrot_h;
S.ln_l1circle=ln_l1circle; S.ln_tocarrot=ln_tocarrot; S.qv_heading=qv_heading;
S.pl_carrot=pl_carrot; S.pl_pos=pl_pos; S.pl_target=pl_target;
S.qv_wind=qv_wind; S.txt_wind=txt_wind;
S.ln_arm1=ln_arm{1}; S.ln_arm2=ln_arm{2};
S.txt_arm1=txt_arm{1}; S.txt_arm2=txt_arm{2};
S.txt_pw_l=txt_pw_l; S.txt_pw_r=txt_pw_r;
S.ln_yaw_cmd=ln_yaw_cmd; S.ln_lat_acc=ln_lat_acc;
S.ln_nu=ln_nu; S.ln_nu1=ln_nu1; S.ln_nu2=ln_nu2;
S.ln_cross=ln_cross; S.ln_along=ln_along; S.ln_l1dist=ln_l1dist;
fig.UserData = S;

if use_batch_trace
    try
        raw_trace = fileread(TRACE_FILE);
        batch_trace = jsondecode(raw_trace);
        l1_viz_render_batch(fig, batch_trace);
        fprintf("Batch trace rendered from %s\n", TRACE_FILE);
        return;
    catch ME
        warning("Batch trace render failed: %s", ME.message);
    end
end

% ── Main update timer (5 Hz – graphics only, no file I/O blocking) ────────────
viz_timer = timer("Name","L1VizTimer","ExecutionMode","fixedRate","Period",0.20, ...
    "ErrorFcn",@(~,~)disp('[timer err]'), ...
    "TimerFcn",@(~,~)l1_viz_update(fig));

% ── Screenshot timer (0.1 Hz = every 10 s, runs off-thread from main viz) ─────
shot_timer = timer("Name","L1ShotTimer","ExecutionMode","fixedRate","Period",10.0, ...
    "ErrorFcn",@(~,~)[], ...
    "TimerFcn",@(~,~)l1_viz_shot(fig));

fig.DeleteFcn = @(~,~)l1_viz_cleanup(viz_timer, shot_timer);
start(viz_timer);
start(shot_timer);
fprintf("Live visualization running at 5 Hz. Screenshots every 10 s -> viz_live.png\n");

% ══════════════════════════════════════════════════════════════════════════════
function l1_viz_update(fig)
    try
    if ~ishandle(fig), return; end
    S = fig.UserData;

    % Read JSON file
    if ~exist(S.data_file,'file'), return; end
    try
        raw  = fileread(S.data_file);
        d    = jsondecode(raw);
    catch
        return;
    end

    % Skip if same step as last update
    if isfield(d,'step') && d.step == S.last_step, return; end
    if isfield(d,'step'), S.last_step = d.step; end

    % Unpack
    t          = d.t;
    pos_N      = d.pos_N;        pos_E      = d.pos_E;
    target_N   = d.target_N;     target_E   = d.target_E;
    carrot_N   = d.carrot_N;     carrot_E   = d.carrot_E;
    start_N    = d.start_N;      start_E    = d.start_E;
    yaw_cmd    = d.yaw_rate_cmd_rad_s;
    lat_acc    = d.lat_acc_cmd_mps2;
    nu_v       = d.nu;           nu1_v      = d.nu1;    nu2_v = d.nu2;
    crossTrack = d.crossTrack;   alongTrack = d.alongTrack;
    L1_dist    = d.L1_distance;
    heading    = d.heading_rad;
    left_ang   = d.left_angle_deg;
    right_ang  = d.right_angle_deg;
    left_pw    = d.left_pw;      right_pw   = d.right_pw;
    alt_m      = d.alt_m;
    mode_str   = d.mode;
    wind_N     = getFieldSafe(d,'wind_N',0);
    wind_E     = getFieldSafe(d,'wind_E',0);

    % Shift history
    S.h_t           = [S.h_t(2:end),          t];
    S.h_pos_N       = [S.h_pos_N(2:end),      pos_N];
    S.h_pos_E       = [S.h_pos_E(2:end),      pos_E];
    S.h_carrot_N    = [S.h_carrot_N(2:end),   carrot_N];
    S.h_carrot_E    = [S.h_carrot_E(2:end),   carrot_E];
    S.h_yaw_cmd     = [S.h_yaw_cmd(2:end),    yaw_cmd];
    S.h_lat_acc     = [S.h_lat_acc(2:end),    lat_acc];
    S.h_nu          = [S.h_nu(2:end),         nu_v];
    S.h_nu1         = [S.h_nu1(2:end),        nu1_v];
    S.h_nu2         = [S.h_nu2(2:end),        nu2_v];
    S.h_crossTrack  = [S.h_crossTrack(2:end), crossTrack];
    S.h_alongTrack  = [S.h_alongTrack(2:end), alongTrack];
    S.h_L1_distance = [S.h_L1_distance(2:end),L1_dist];
    % Bounded trail buffer (avoids unbounded growth and slow set() calls)
    S.all_pos_N(end+1) = pos_N;
    S.all_pos_E(end+1) = pos_E;
    if numel(S.all_pos_N) > S.TRAIL_MAX
        S.all_pos_N = S.all_pos_N(end-S.TRAIL_MAX+1:end);
        S.all_pos_E = S.all_pos_E(end-S.TRAIL_MAX+1:end);
    end

    % ── Map ───────────────────────────────────────────────────────────────────
    if ~S.target_init
        set(S.pl_target, "XData",target_E,"YData",target_N);
        set(S.ln_refpath,"XData",[start_E,target_E],"YData",[start_N,target_N]);
        % Fix map scale once based on target distance (prevents runaway zoom)
        S.map_scale_m = max(hypot(target_N-start_N, target_E-start_E) * 1.5, 80);
        S.target_init = true;
    end
    set(S.ln_trail,   "XData",S.all_pos_E,  "YData",S.all_pos_N);
    valid = ~isnan(S.h_carrot_E);
    set(S.sc_carrot_h,"XData",S.h_carrot_E(valid),"YData",S.h_carrot_N(valid));
    set(S.pl_carrot,  "XData",carrot_E,"YData",carrot_N);
    set(S.ln_tocarrot,"XData",[pos_E,carrot_E],"YData",[pos_N,carrot_N]);
    cx=pos_E+L1_dist.*sin(S.th_circ); cy=pos_N+L1_dist.*cos(S.th_circ);
    set(S.ln_l1circle,"XData",cx,"YData",cy);
    set(S.pl_pos,     "XData",pos_E,"YData",pos_N);
    al = max(L1_dist*0.32,8.0);
    set(S.qv_heading, "XData",pos_E,"YData",pos_N,"UData",al*sin(heading),"VData",al*cos(heading));
    % Map zoom: center tracks cansat position, scale fixed from initial target distance
    sc = S.map_scale_m;
    S.ax_map.XLim = [pos_E - sc, pos_E + sc];
    S.ax_map.YLim = [pos_N - sc, pos_N + sc];
    title(S.ax_map,sprintf("Trajectory  |  mode: %s  |  t=%.1fs  |  alt=%.1fm  |  wind=(N%.1f E%.1f) m/s",mode_str,t,alt_m,wind_N,wind_E),"Color",[.95 .95 .95],"FontSize",9);
    % Wind arrow: place at current position, scaled 5x for visibility
    wscale = 5.0;
    set(S.qv_wind,  "XData",pos_E,"YData",pos_N,"UData",wind_E*wscale,"VData",wind_N*wscale);
    set(S.txt_wind, "Position",[pos_E+wind_E*wscale*1.15, pos_N+wind_N*wscale*1.15, 0], ...
        "String",sprintf("wind %.1fm/s",hypot(wind_N,wind_E)));

    % ── Arms ──────────────────────────────────────────────────────────────────
    arms_ln={S.ln_arm1,S.ln_arm2}; arms_txt={S.txt_arm1,S.txt_arm2};
    for k=1:2
        cx2=S.arm_cx(k); cy2=S.arm_cy(k); ang=[left_ang,right_ang]; pw=[left_pw,right_pw];
        ex=cx2+S.ARM_LEN*sin(deg2rad(ang(k))); ey=cy2+S.ARM_LEN*cos(deg2rad(ang(k)));
        set(arms_ln{k},"XData",[cx2,ex],"YData",[cy2,ey]);
        set(arms_txt{k},"String",sprintf("%.1fdeg",ang(k)));
    end
    set(S.txt_pw_l,"String",sprintf("PW: %d us",left_pw));
    set(S.txt_pw_r,"String",sprintf("PW: %d us",right_pw));

    % ── Time series ───────────────────────────────────────────────────────────
    tw=[max(0,t-15),t+0.5];
    set(S.ln_yaw_cmd,"XData",S.h_t,"YData",S.h_yaw_cmd);
    set(S.ln_lat_acc,"XData",S.h_t,"YData",S.h_lat_acc); S.ax_yaw.XLim=tw;
    set(S.ln_nu, "XData",S.h_t,"YData",S.h_nu);
    set(S.ln_nu1,"XData",S.h_t,"YData",S.h_nu1);
    set(S.ln_nu2,"XData",S.h_t,"YData",S.h_nu2); S.ax_nu.XLim=tw;
    set(S.ln_cross, "XData",S.h_t,"YData",S.h_crossTrack);
    set(S.ln_along, "XData",S.h_t,"YData",S.h_alongTrack);
    set(S.ln_l1dist,"XData",S.h_t,"YData",S.h_L1_distance); S.ax_trk.XLim=tw;

    fig.UserData = S;
    drawnow limitrate;

    catch ME
        fprintf('[viz] %s\n', ME.message);
    end
end

function l1_viz_shot(fig)
    % Runs on separate slow timer – exportgraphics is slow (~1s), keep it off the main timer.
    try
        if ~ishandle(fig), return; end
        S = fig.UserData;
        exportgraphics(fig, S.shot_file, 'Resolution', 90);
    catch
    end
end

function l1_viz_cleanup(viz_tmr, shot_tmr)
    try; stop(viz_tmr);  delete(viz_tmr);  catch; end
    try; stop(shot_tmr); delete(shot_tmr); catch; end
    fprintf("Visualization stopped.\n");
end

function l1_viz_render_batch(fig, trace)
    if ~ishandle(fig) || isempty(trace), return; end
    S = fig.UserData;
    n = numel(trace);
    last = trace(end);

    t          = arrayfun(@(d)d.t, trace);
    pos_N      = arrayfun(@(d)d.pos_N, trace);
    pos_E      = arrayfun(@(d)d.pos_E, trace);
    carrot_N   = arrayfun(@(d)d.carrot_N, trace);
    carrot_E   = arrayfun(@(d)d.carrot_E, trace);
    yaw_cmd    = arrayfun(@(d)d.yaw_rate_cmd_rad_s, trace);
    lat_acc    = arrayfun(@(d)d.lat_acc_cmd_mps2, trace);
    nu_v       = arrayfun(@(d)d.nu, trace);
    nu1_v      = arrayfun(@(d)d.nu1, trace);
    nu2_v      = arrayfun(@(d)d.nu2, trace);
    crossTrack = arrayfun(@(d)d.crossTrack, trace);
    alongTrack = arrayfun(@(d)d.alongTrack, trace);
    L1_dist    = arrayfun(@(d)d.L1_distance, trace);

    start_N  = last.start_N;  start_E  = last.start_E;
    target_N = last.target_N; target_E = last.target_E;

    set(S.pl_target, "XData",target_E,"YData",target_N);
    set(S.ln_refpath,"XData",[start_E,target_E],"YData",[start_N,target_N]);
    set(S.ln_trail,"XData",pos_E,"YData",pos_N);
    set(S.sc_carrot_h,"XData",carrot_E,"YData",carrot_N);
    set(S.pl_carrot,"XData",last.carrot_E,"YData",last.carrot_N);
    set(S.ln_tocarrot,"XData",[last.pos_E,last.carrot_E],"YData",[last.pos_N,last.carrot_N]);
    cx = last.pos_E + last.L1_distance .* sin(S.th_circ);
    cy = last.pos_N + last.L1_distance .* cos(S.th_circ);
    set(S.ln_l1circle,"XData",cx,"YData",cy);
    set(S.pl_pos,"XData",last.pos_E,"YData",last.pos_N);
    al = max(last.L1_distance * 0.32, 8.0);
    set(S.qv_heading,"XData",last.pos_E,"YData",last.pos_N, ...
        "UData",al*sin(last.heading_rad),"VData",al*cos(last.heading_rad));

    pad = max(20,last.L1_distance*1.5);
    all_E = [pos_E(:).',target_E,start_E];
    all_N = [pos_N(:).',target_N,start_N];
    S.ax_map.XLim = [min(all_E)-pad,max(all_E)+pad];
    S.ax_map.YLim = [min(all_N)-pad,max(all_N)+pad];
    wind_N = getFieldSafe(last,'wind_N',0);
    wind_E = getFieldSafe(last,'wind_E',0);
    title(S.ax_map,sprintf("Trajectory  |  batch=%d frames  |  mode: %s  |  t=%.1fs  |  alt=%.1fm", ...
        n,last.mode,last.t,last.alt_m),"Color",[.95 .95 .95],"FontSize",9);
    wscale = 5.0;
    set(S.qv_wind,"XData",last.pos_E,"YData",last.pos_N,"UData",wind_E*wscale,"VData",wind_N*wscale);
    set(S.txt_wind,"Position",[last.pos_E+wind_E*wscale*1.15, last.pos_N+wind_N*wscale*1.15, 0], ...
        "String",sprintf("wind %.1fm/s",hypot(wind_N,wind_E)));

    arms_ln={S.ln_arm1,S.ln_arm2}; arms_txt={S.txt_arm1,S.txt_arm2};
    ang=[last.left_angle_deg,last.right_angle_deg];
    pw=[last.left_pw,last.right_pw];
    for k=1:2
        cx2=S.arm_cx(k); cy2=S.arm_cy(k);
        ex=cx2+S.ARM_LEN*sin(deg2rad(ang(k))); ey=cy2+S.ARM_LEN*cos(deg2rad(ang(k)));
        set(arms_ln{k},"XData",[cx2,ex],"YData",[cy2,ey]);
        set(arms_txt{k},"String",sprintf("%.1fdeg",ang(k)));
    end
    set(S.txt_pw_l,"String",sprintf("PW: %d us",pw(1)));
    set(S.txt_pw_r,"String",sprintf("PW: %d us",pw(2)));

    set(S.ln_yaw_cmd,"XData",t,"YData",yaw_cmd);
    set(S.ln_lat_acc,"XData",t,"YData",lat_acc);
    set(S.ln_nu,"XData",t,"YData",nu_v);
    set(S.ln_nu1,"XData",t,"YData",nu1_v);
    set(S.ln_nu2,"XData",t,"YData",nu2_v);
    set(S.ln_cross,"XData",t,"YData",crossTrack);
    set(S.ln_along,"XData",t,"YData",alongTrack);
    set(S.ln_l1dist,"XData",t,"YData",L1_dist);
    xlim(S.ax_yaw,[min(t),max(t)+0.5]);
    xlim(S.ax_nu,[min(t),max(t)+0.5]);
    xlim(S.ax_trk,[min(t),max(t)+0.5]);

    fig.UserData = S;
    drawnow;
    try
        exportgraphics(fig, S.shot_file, 'Resolution', 90);
    catch
    end
end

function v = getFieldSafe(s, fname, default)
    if isfield(s, fname); v = s.(fname); else; v = default; end
end
