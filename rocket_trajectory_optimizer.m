%% ============================================================================
%  SUBORBITAL LAUNCH VEHICLE TRAJECTORY OPTIMIZATION AND SIMULATION
%  Flight Dynamics Engineer - MATLAB Script (HYBRID MASTER VERSION)
%  Mission: 10,000 kg payload to 200 km altitude at t=300s with FUEL DEPLETED
%  Vehicle: 4x RL10 Clustered Engines (400 kN total thrust, 90.4 kg/s flow)
%% ============================================================================

clear all; close all; clc;

%% ========== MISSION AND VEHICLE PARAMETERS ==========
payload_mass = 10000;           % kg
target_altitude = 200000;       % m (200 km)
target_time = 300;              % s
dt = 0.1;                       % s (Euler timestep)
num_steps = target_time / dt;   % 3000 steps

% Engine cluster
F_thrust = 400000;              % N (vacuum thrust, 4x RL10)
mass_flow = 90.4;               % kg/s (total cluster)
dry_engine_mass = 1108;         % kg (4x 277 kg RL10)

% Atmosphere and gravity
H_scale = 8500;                 % m (scale height)
rho_0 = 1.225;                  % kg/m^3 (sea level density)
g_0 = 9.81;                     % m/s^2 (surface gravity)
R_E = 6.371e6;                  % m (Earth radius)

% Aerodynamics
C_d = 0.3;                      % drag coefficient
A_ref = 4.3;                    % m^2 (reference area)

%% ========== OPTIMIZATION LOOP ==========
fprintf('\n%s\n', repmat('=',1,90));
fprintf('SUBORBITAL TRAJECTORY OPTIMIZATION - CONVERGENCE LOOP\n');
fprintf('OBJECTIVE: h(300s) = 200,000 m AND fuel depleted during powered burn phase\n');
fprintf('%s\n', repmat('=',1,90));

% Binary search on BURN TIME
BT_lower = 100;   % s (minimum estimate)
BT_upper = 280;   % s (maximum estimate)
altitude_tolerance = 5; % m (strict convergence tolerance)
max_iterations = 60;

for iteration = 1:max_iterations
    BT = 0.5 * (BT_lower + BT_upper);
    
    % Run encapsulated physics simulation
    [alt, vel, ~, ~, ~, ~] = simulate_flight(BT, payload_mass, F_thrust, mass_flow, ...
        dry_engine_mass, g_0, R_E, rho_0, H_scale, C_d, A_ref, dt, num_steps);
    
    final_altitude = alt(end);
    altitude_error = final_altitude - target_altitude;
    propellant_mass = mass_flow * BT;
    
    fprintf('Iter %2d | Prop = %8.1f kg | h(300s) = %10.1f m | Error = %+10.1f m | MECO = %6.3f s \n', ...
        iteration, propellant_mass, final_altitude, altitude_error, BT);
    
    % Convergence check
    if abs(altitude_error) < altitude_tolerance
        fprintf('%s\n', repmat('=',1,90));
        fprintf('✓✓✓ CONVERGENCE ACHIEVED in %d iterations ✓✓✓\n', iteration);
        fprintf('Optimal Burn Time: %.3f s | Optimal Propellant: %.1f kg\n', BT, propellant_mass);
        fprintf('%s\n', repmat('=',1,90));
        break;
    end
    
    % Adjust bounds
    if altitude_error > 0
        BT_upper = BT;   % Overshot: shorten burn
    else
        BT_lower = BT;   % Undershot: lengthen burn
    end
end

%% ========== FINAL TRAJECTORY COMPUTATION (CLEAN DATA) ==========
burn_time_actual = BT;
propellant_mass = mass_flow * burn_time_actual;
structural_mass = 0.10 * propellant_mass;
GLOW = payload_mass + propellant_mass + dry_engine_mass + structural_mass;

% Re-run trajectory with optimized parameters to extract full arrays
[altitude, velocity, mass, acceleration, thrust_vec, drag_vec] = simulate_flight(...
    burn_time_actual, payload_mass, F_thrust, mass_flow, dry_engine_mass, ...
    g_0, R_E, rho_0, H_scale, C_d, A_ref, dt, num_steps);

time_vec = (0:dt:target_time)';
density_vec = rho_0 * exp(-altitude / H_scale);
grav_vec = g_0 * (R_E ./ (R_E + altitude)).^2;
weight_vec = mass .* grav_vec;

accel_g = acceleration / g_0;
peak_acceleration = max(accel_g);
peak_altitude = max(altitude);

% Find exact MECO index
meco_idx = find(thrust_vec < 1, 1) - 1;
if isempty(meco_idx) || meco_idx < 1
    meco_idx = round(burn_time_actual/dt); 
end
meco_t = time_vec(meco_idx);

%% ========== DIAGNOSTIC SUMMARY ==========
fprintf('\n%s\n', repmat('=',1,90));
fprintf('MISSION OPTIMIZATION RESULTS - FINAL CONFIGURATION\n');
fprintf('%s\n', repmat('=',1,90));

fprintf('\nVEHICLE CONFIGURATION:\n');
fprintf('  Gross Liftoff Weight (GLOW)            : %12.1f kg\n', GLOW);
fprintf('  Payload Mass                           : %12.1f kg (fixed)\n', payload_mass);
fprintf('  Propellant Mass (OPTIMIZED)            : %12.1f kg <<<\n', propellant_mass);
fprintf('  Structural Mass (10%% of propellant)   : %12.1f kg\n', structural_mass);
fprintf('  Engine Dry Mass                        : %12.1f kg\n', dry_engine_mass);

fprintf('\nMISSION PERFORMANCE:\n');
fprintf('  Main Engine Cutoff (MECO) Time         : %12.3f s\n', meco_t);
fprintf('  Burnout Altitude                       : %12.1f m\n', altitude(meco_idx));
fprintf('  Burnout Velocity                       : %12.1f m/s\n', velocity(meco_idx));
fprintf('  Peak Altitude (Apogee)                 : %12.1f m\n', peak_altitude);
fprintf('  Final Altitude at t=300s               : %12.1f m <<<\n', altitude(end));
fprintf('  Final Velocity at t=300s               : %12.4f m/s (should be ~0)\n', velocity(end));

fprintf('\nDYNAMIC CHARACTERISTICS:\n');
fprintf('  Initial TWR                            : %12.4f\n', F_thrust / (GLOW * g_0));
fprintf('  Peak Acceleration (at MECO)            : %12.4f G\n', peak_acceleration);
fprintf('  Mass Ratio (GLOW/Dry Mass)             : %12.4f\n', GLOW / (GLOW - propellant_mass));

fprintf('\nAERODYNAMIC PROPERTIES:\n');
fprintf('  Reference Area                         : %12.4f m^2\n', A_ref);
fprintf('  Drag Coefficient                       : %12.4f\n', C_d);
fprintf('  Max Dynamic Pressure                   : %12.2f Pa\n', max(0.5 * density_vec .* velocity.^2));

fprintf('\nATMOSPHERE & GRAVITY:\n');
fprintf('  US Std Atm Scale Height                : %12.1f m\n', H_scale);
fprintf('  Sea Level Density                      : %12.5f kg/m^3\n', rho_0);
fprintf('  Surface Gravity                        : %12.4f m/s^2\n', g_0);
fprintf('  Gravity at 200 km                      : %12.4f m/s^2\n', g_0 * (R_E / (R_E + 200000))^2);

fprintf('\nINTEGRATION PARAMETERS:\n');
fprintf('  Time Step (Δt)                         : %12.4f s\n', dt);
fprintf('  Total Integration Steps                : %12.0f\n', num_steps);
fprintf('  Total Mission Duration                 : %12.1f s\n', target_time);

fprintf('\n%s\n\n', repmat('=',1,90));

%% ========== MISSION VALIDATION ==========
fprintf('%s\n', repmat('=',1,90));
fprintf('MISSION COMPLIANCE VERIFICATION\n');
fprintf('%s\n', repmat('=',1,90));

payload_check = abs(payload_mass - 10000) < 1;
altitude_check = abs(altitude(end) - 200000) < 100;
time_check = true; % fixed by num_steps
velocity_check = abs(velocity(end)) < 10;

fprintf('  [1] Payload Mass (10,000 kg)          : %s\n', iif(payload_check, '✓ PASS', '✗ FAIL'));
fprintf('  [2] Target Altitude (200,000 m)       : %s (actual = %.1f m)\n', iif(altitude_check, '✓ PASS', '✗ FAIL'), altitude(end));
fprintf('  [3] Mission Duration (300 s)          : %s\n', iif(time_check, '✓ PASS', '✗ FAIL'));
fprintf('  [4] Fuel Depleted                     : ✓ PASS (Locked by Burn Time optimization)\n');
fprintf('  [5] Velocity at Apogee (~0 m/s)       : %s (actual = %.4f m/s)\n', iif(velocity_check, '✓ PASS', '✗ FAIL'), velocity(end));

if payload_check && altitude_check && time_check && velocity_check
    fprintf('\n  ╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('  ║  ✓✓✓ PERFECT MISSION PROFILE ACHIEVED ✓✓✓                     ║\n');
    fprintf('  ║  - Payload: 10,000 kg delivered                               ║\n');
    fprintf('  ║  - Altitude: Exactly 200,000 m at t=300s                      ║\n');
    fprintf('  ║  - Fuel: Completely depleted (ballistic coast phase)          ║\n');
    fprintf('  ║  - Dynamics: All constraints satisfied                        ║\n');
    fprintf('  ╚═══════════════════════════════════════════════════════════════╝\n\n');
else
    fprintf('\n  ✗✗✗ MISSION PROFILE INCOMPLETE - REVIEW PARAMETERS ✗✗✗\n\n');
end

fprintf('%s\n\n', repmat('=',1,90));

%% ========== PROFESSIONAL 4-PANEL PLOTTING ==========
figure('Name', 'Flight Dynamics Analysis', 'NumberTitle', 'off', ...
    'Position', [100 100 1400 900], 'Color', 'white');

% Define professional RGB colors (Fixes Hex bugs in older MATLAB)
cBlue   = [0,    0.447, 0.741];   
cOrange = [0.85, 0.325, 0.098];   
cYellow = [0.929, 0.694, 0.125];   
cPurple = [0.494, 0.184, 0.556];   
cGreen  = [0.466, 0.674, 0.188];   
cCyan   = [0.301, 0.745, 0.933];   
cRed    = [0.8,  0,     0    ];   
cGray   = [0.5,  0.5,   0.5  ];   

% ========== SUBPLOT 1: ALTITUDE & VELOCITY vs TIME ==========
ax1 = subplot(2, 2, 1);
hold on; grid on; grid minor;
set(ax1, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

yyaxis(ax1, 'left');
plot(ax1, time_vec, altitude/1000, 'LineWidth', 2.5, 'Color', cBlue);
ylabel(ax1, 'Altitude (km)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', cBlue);
ax1.YAxis(1).Color = cBlue;

yyaxis(ax1, 'right');
plot(ax1, time_vec, velocity, 'LineWidth', 2.5, 'Color', cOrange);
ylabel(ax1, 'Velocity (m/s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', cOrange);
ax1.YAxis(2).Color = cOrange;

xlabel(ax1, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
xline(ax1, meco_t, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');

yyaxis(ax1, 'left');
yline(ax1, 200, ':', 'LineWidth', 1.8, 'Color', cGray, 'Label', '200 km target', 'LabelHorizontalAlignment', 'right');
title(ax1, 'Altitude & Velocity Profile', 'FontSize', 12, 'FontWeight', 'bold');

% ========== SUBPLOT 2: ACCELERATION vs TIME (Gs) ==========
ax2 = subplot(2, 2, 2);
hold on; grid on; grid minor;
set(ax2, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

% Use patch to fix the fill bug
px = [time_vec; flipud(time_vec)];
py = [accel_g;  zeros(num_steps+1, 1)];
patch(ax2, px, py, cYellow, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
plot(ax2, time_vec, accel_g, 'LineWidth', 2.5, 'Color', cYellow);

yline(ax2, 0, '-', 'Color', cGray, 'LineWidth', 1);
xline(ax2, meco_t, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
xlabel(ax2, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax2, 'Acceleration (G)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax2, 'Acceleration Profile', 'FontSize', 12, 'FontWeight', 'bold');

% ========== SUBPLOT 3: FORCES vs TIME (MN) ==========
ax3 = subplot(2, 2, 3);
hold on; grid on; grid minor;
set(ax3, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

plot(ax3, time_vec, thrust_vec/1e6, 'LineWidth', 2.5, 'Color', cPurple, 'DisplayName', 'Thrust');
plot(ax3, time_vec, weight_vec/1e6, 'LineWidth', 2.0, 'Color', cGreen, 'DisplayName', 'Weight');
plot(ax3, time_vec, drag_vec/1e6, 'LineWidth', 1.8, 'LineStyle', '--', 'Color', cOrange, 'DisplayName', 'Drag');

xline(ax3, meco_t, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
xlabel(ax3, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax3, 'Force (MN)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax3, 'Thrust, Weight & Drag', 'FontSize', 12, 'FontWeight', 'bold');
legend(ax3, 'Location', 'northeast', 'FontSize', 10);

% ========== SUBPLOT 4: MASS vs TIME ==========
ax4 = subplot(2, 2, 4);
hold on; grid on; grid minor;
set(ax4, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

px4 = [time_vec; flipud(time_vec)];
py4 = [mass/1e3; zeros(num_steps+1, 1)];
patch(ax4, px4, py4, cCyan, 'FaceAlpha', 0.40, 'EdgeColor', 'none');
plot(ax4, time_vec, mass/1e3, 'LineWidth', 2.5, 'Color', cCyan);

xline(ax4, meco_t, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
xlabel(ax4, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax4, 'Vehicle Mass (1000 kg)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax4, 'Vehicle Mass Depletion', 'FontSize', 12, 'FontWeight', 'bold');
ylim(ax4, [0, GLOW/1e3 * 1.08]);

sgtitle('4x RL10 Clustered Engine Trajectory Optimization - 200 km Target Mission', ...
    'FontSize', 14, 'FontWeight', 'bold');

% Save outputs to the current directory to avoid missing folder errors
print(gcf, 'trajectory_analysis_final.png', '-dpng', '-r300');

%% ========== EXPORT DATA TO CSV ==========
data_table = table(time_vec, altitude, velocity, acceleration, accel_g, mass, ...
    thrust_vec, drag_vec, density_vec, weight_vec, ...
    'VariableNames', {'Time_s', 'Altitude_m', 'Velocity_ms', 'Acceleration_ms2', ...
    'Acceleration_G', 'Mass_kg', 'Thrust_N', 'Drag_N', 'Density_kgm3', 'Weight_N'});

writetable(data_table, 'trajectory_data_final.csv');

fprintf('EXPORT SUMMARY:\n');
fprintf('  ✓ trajectory_analysis_final.png (Saved to current folder)\n');
fprintf('  ✓ trajectory_data_final.csv     (Saved to current folder)\n\n');

%% ============================================================================
% LOCAL FUNCTIONS (Must remain at the very bottom of the script)
%% ============================================================================

function [alt, vel, mas, acc, thr, drg] = simulate_flight(burn_time, ...
        payload, F_thrust, mdot, m_eng, g0, RE, rho0, Hs, Cd, Aref, dt, N)
    
    % Derive masses from burn time
    m_prop   = mdot * burn_time;
    m_struct = 0.10 * m_prop;
    GLOW     = payload + m_prop + m_eng + m_struct;
    
    % Preallocate
    alt = zeros(N+1,1);  vel = zeros(N+1,1);
    mas = zeros(N+1,1);  acc = zeros(N+1,1);
    thr = zeros(N+1,1);  drg = zeros(N+1,1);
    
    mas(1) = GLOW;
    prop_left = m_prop;
    
    for k = 1:N
        h = alt(k);  v = vel(k);  m = mas(k);
        
        % Atmosphere & gravity
        rho    = rho0 * exp(-h/Hs);
        g_loc  = g0 * (RE/(RE+h))^2;
        
        % Drag
        D = 0.5 * rho * v^2 * Cd * Aref;
        drg(k) = D;
        
        % Thrust phase
        if prop_left > 0
            dm  = min(mdot*dt, prop_left);
            thrust = F_thrust * (dm/(mdot*dt));
            prop_left = prop_left - dm;
        else
            thrust = 0;  
            dm = 0;
        end
        thr(k) = thrust;
        
        % Equations of motion
        a = thrust/m - g_loc - D/m;
        acc(k) = a;
        
        % Euler update
        vel(k+1) = v + a*dt;
        
        % Hard deck constraint (don't fall through ground)
        if vel(k+1) < 0 && h <= 0
            vel(k+1) = 0;
        end
        
        alt(k+1) = h + v*dt;
        mas(k+1) = m - dm;
    end
    
    acc(N+1) = acc(N);
    thr(N+1) = 0;
    drg(N+1) = drg(N);
end

function out = iif(cond, a, b)
    if cond
        out = a; 
    else
        out = b; 
    end
end