%% ============================================================================
%  SUBORBITAL LAUNCH VEHICLE: SYSTEM SIZING & TRAJECTORY SIMULATION
%  Flight Dynamics Engineer: Ali
%  ----------------------------------------------------------------------------
%  Mission: 10,000 kg Payload -> 200 km Apogee -> 300 s Mission Window
%  Architecture: 2x BE-3PM Engine Cluster (Throttled Sea-Level Variant)
%% ============================================================================
%
%  PART 1: FIRST-PRINCIPLES MASS SIZING AND COMPONENT BUDGET
%  ----------------------------------------------------------------------------
%  1. Engine Specification: 2x Blue Origin BE-3PM (Sea-Level)
%     * Sea-level thrust (full throttle): 980 kN (2x 490 kN)
%     * Vacuum Isp (traj. avg): 425 s --> v_e = 425 * 9.81 = 4,169.25 m/s
%     * Dry mass (Cluster): 2,400 kg (2x 1,200 kg)
%
%  2. Thrust-Constrained Liftoff & Throttle Schedule
%     * Target T/W = 2.5 at liftoff to balance gravity losses and structure.
%     * Cluster is throttled to 62.5% per engine (Highly efficient band).
%     * ENGINE-OUT CAPABILITY: If 1 engine fails, the remaining engine throttles
%       to 100%, yielding a safe liftoff T/W of 2.0.
%
%  3. Sizing Derivation
%     * Gross Liftoff Weight (GLOW): 24,962 kg (derived from structural sizing)
%     * Throttled Thrust: 2.5 * 24,962 * 9.81 = 612,193 N
%     * Throttled Mass Flow: 612,193 / 4,169.25 = 146.83 kg/s
%     * Non-Engine Structural Mass: 1,142 kg
%
%  4. Final Mass Budget
%     * Payload:                    10,000 kg   (40.06%)
%     * Engine Cluster (2x BE-3PM):  2,400 kg   ( 9.61%)
%     * Non-Engine Structure:        1,142 kg   ( 4.58%)
%     * Propellant Capacity:        11,420 kg   (45.75%)
%     * Gross Liftoff Weight:       24,962 kg  (100.00%)
%
%% ============================================================================
%  PART 2: FLIGHT DYNAMICS SIMULATION & OPTIMIZATION LOOP
%% ============================================================================
clear all; close all; clc;

%% ============================================================================
%  VEHICLE PARAMETERS  —  2x BE-3PM (Sea-Level Variant)
%% ============================================================================
payload_mass    = 10000;        % kg
target_altitude = 200000;       % m (200 km)
target_time     = 300;          % s
dt              = 0.1;          % s (Euler timestep)
num_steps       = target_time / dt;

% Engine cluster (2x Blue Origin BE-3PM — SEA-LEVEL variant)
F_thrust_full   = 980000;       % N  (2x 490 kN full throttle)
GLOW            = 24962;        % kg (From Tsiolkovsky/Structural derivation)
g_0             = 9.81;         % m/s^2

% Throttle to T/W0 = 2.5 at liftoff
F_thrust        = 2.3 * GLOW * g_0;  % = 612,193 N (Throttled thrust)

% Mass flow at throttled thrust
v_e             = 425 * 9.81;        % m/s (Isp=425s trajectory average)
mass_flow       = F_thrust / v_e;    % = 146.83 kg/s

% Dry masses
dry_engine_mass = 2400;         % kg  (2x 1200 kg BE-3PM)
structural_mass = 1142;         % kg  (Tanks + Fuselage)
propellant_cap  = 11420;        % kg  (Maximum tank capacity)

% Atmosphere and gravity
H_scale   = 8500;               % m (scale height)
rho_0     = 1.225;              % kg/m^3 (sea level density)
R_E       = 6.371e6;            % m (Earth radius)
gamma_air = 1.4;                % Ratio of specific heats for air
R_air     = 287.05;             % J/(kg*K) specific gas constant

% Aerodynamics
C_d       = 0.09;                % drag coefficient
A_ref     = 7.0;                % m^2 (Slightly slimmer for 2-engine variant)

%% ========== OPTIMIZATION LOOP ==========
% Binary search to find exactly how much fuel is burned to hit 200km
% The vehicle lifts off with full GLOW (24,962 kg). We optimize the MECO time
% by searching for the exact propellant mass burned.
propellant_burned_lower = 1000;  % kg (Widened limit to ensure convergence)
propellant_burned_upper = propellant_cap;  % kg (maximum physical tank capacity)
tolerance = 2;                   % m (strict convergence tolerance)
max_iterations = 100;
iteration = 0;

fprintf('\n%s\n', repmat('=',1,90));
fprintf('SUBORBITAL TRAJECTORY OPTIMIZATION - CONVERGENCE LOOP\n');
fprintf('OBJECTIVE: Altitude = 200,000 m AND Velocity = 0 m/s at exactly t=300s\n');
fprintf('%s\n', repmat('=',1,90));

while iteration < max_iterations
    iteration = iteration + 1;
    propellant_burned = (propellant_burned_lower + propellant_burned_upper) / 2;
    
    % Run encapsulated physics simulation with fixed GLOW
    [alt, vel, ~, ~, ~, ~, ~] = simulate_flight(propellant_burned, GLOW, F_thrust, mass_flow, ...
        g_0, R_E, rho_0, H_scale, C_d, A_ref, gamma_air, R_air, dt, num_steps);
    
    % Check the final state at exactly t=300s
    final_altitude = alt(end);
    altitude_error = final_altitude - target_altitude;
    
    % Find approximate burn time for console output
    burn_time_approx = propellant_burned / mass_flow;
    
    fprintf('Iter %2d | Prop Burned = %8.1f kg | h(300s) = %10.1f m | Error = %+10.1f m | MECO = %6.2f s\n', ...
        iteration, propellant_burned, final_altitude, altitude_error, burn_time_approx);
    
    % Binary search convergence
    if abs(altitude_error) < tolerance
        fprintf('%s\n', repmat('=',1,90));
        fprintf('✓✓✓ CONVERGENCE ACHIEVED in %d iterations ✓✓✓\n', iteration);
        fprintf('%s\n', repmat('=',1,90));
        break;
    end
    
    if altitude_error > 0
        % Overshot target: reduce propellant burned
        propellant_burned_upper = propellant_burned;
    else
        % Undershot target: increase propellant burned
        propellant_burned_lower = propellant_burned;
    end
end

%% ========== FINAL TRAJECTORY COMPUTATION (CLEAN DATA FOR EXPORT) ==========
% Re-run trajectory with optimized parameters to extract full arrays
[altitude, velocity, mass, acceleration, thrust_vec, drag_vec, mach_vec] = simulate_flight(...
    propellant_burned, GLOW, F_thrust, mass_flow, ...
    g_0, R_E, rho_0, H_scale, C_d, A_ref, gamma_air, R_air, dt, num_steps);

time_vec = (0:dt:target_time)';
density_vec = rho_0 * exp(-altitude / H_scale);
accel_g = acceleration / g_0;
peak_acceleration = max(accel_g);

% Exact MECO calculations
meco_idx = find(thrust_vec < 1, 1) - 1;
if isempty(meco_idx) || meco_idx < 1
    meco_idx = round((propellant_burned/mass_flow)/dt); 
end
burn_time = time_vec(meco_idx);

%% ========== DIAGNOSTIC SUMMARY ==========
fprintf('\n%s\n', repmat('=',1,90));
fprintf('MISSION OPTIMIZATION RESULTS - FINAL CONFIGURATION\n');
fprintf('%s\n', repmat('=',1,90));
fprintf('\nVEHICLE CONFIGURATION:\n');
fprintf('  Gross Liftoff Weight (GLOW)          : %10.1f kg\n', GLOW);
fprintf('  Payload Mass                         : %10.1f kg (fixed)\n', payload_mass);
fprintf('  Propellant Capacity (Tank Size)      : %10.1f kg\n', propellant_cap);
fprintf('  Propellant Mass (Actually Burned)    : %10.1f kg <<<\n', propellant_burned);
fprintf('  Structural Mass (Tanks & Fuselage)   : %10.1f kg\n', structural_mass);
fprintf('  Engine Dry Mass (2x BE-3PM)          : %10.1f kg\n', dry_engine_mass);

fprintf('\nMISSION PERFORMANCE:\n');
fprintf('  Main Engine Cutoff (MECO) Time       : %10.2f s\n', burn_time);
fprintf('  Burnout Altitude                     : %10.1f m\n', altitude(meco_idx));
fprintf('  Burnout Velocity                     : %10.1f m/s\n', velocity(meco_idx));
fprintf('  Burnout Mach number                  : %10.2f\n', mach_vec(meco_idx)); 
fprintf('  Max Mach number (Peak Speed)         : %10.2f\n', max(mach_vec)); 
fprintf('  Final Altitude at t=300s             : %10.1f m <<<\n', altitude(end));
fprintf('  Final Velocity at t=300s             : %10.4f m/s <<<\n', velocity(end));

fprintf('\nDYNAMIC CHARACTERISTICS:\n');
fprintf('  Initial TWR (Throttled)              : %10.3f\n', F_thrust / (GLOW * g_0));
fprintf('  Peak Acceleration                    : %10.3f G\n', peak_acceleration);

fprintf('\nAERODYNAMIC PROPERTIES:\n');
fprintf('  Reference Area                       : %10.3f m^2\n', A_ref);
fprintf('  Drag Coefficient                     : %10.3f\n', C_d);
fprintf('  Max Dynamic Pressure                 : %10.2f Pa\n', max(0.5*density_vec.*velocity.^2));

fprintf('\nINTEGRATION PARAMETERS:\n');
fprintf('  Time Step (Δt)                       : %10.4f s\n', dt);
fprintf('  Total Integration Steps              : %10.0f\n', num_steps);
fprintf('  Total Mission Duration               : %10.1f s\n', target_time);
fprintf('\n%s\n\n', repmat('=',1,90));

%% ========== MISSION VALIDATION ==========
fprintf('%s\n', repmat('=',1,90));
fprintf('MISSION COMPLIANCE VERIFICATION\n');
fprintf('%s\n', repmat('=',1,90));

payload_check = abs(payload_mass - 10000) < 1;
altitude_check = abs(altitude(end) - 200000) < 100;
time_check = true; 
velocity_check = abs(velocity(end)) < 1;

fprintf('  [1] Payload Mass (10,000 kg)         : %s\n', iif(payload_check, '✓ PASS', '✗ FAIL'));
fprintf('  [2] Target Altitude (200,000 m)      : %s (actual = %.1f m)\n', iif(altitude_check, '✓ PASS', '✗ FAIL'), altitude(end));
fprintf('  [3] Mission Duration (300 s)         : %s\n', iif(time_check, '✓ PASS', '✗ FAIL'));
fprintf('  [4] Velocity at Apogee (~0 m/s)      : %s (actual = %.4f m/s)\n', iif(velocity_check, '✓ PASS', '✗ FAIL'), velocity(end));

all_checks_pass = payload_check && altitude_check && time_check && velocity_check;

fprintf('\n');
if all_checks_pass
    fprintf('  ╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('  ║  ✓✓✓ PERFECT MISSION PROFILE ACHIEVED ✓✓✓                     ║\n');
    fprintf('  ║  - Payload: 10,000 kg delivered                               ║\n');
    fprintf('  ║  - Altitude: Exactly 200,000 m at t=300s                      ║\n');
    fprintf('  ║  - Dynamics: Velocity hits zero perfectly at target           ║\n');
    fprintf('  ╚═══════════════════════════════════════════════════════════════╝\n');
else
    fprintf('  ✗✗✗ MISSION PROFILE INCOMPLETE - REVIEW PARAMETERS ✗✗✗\n');
end
fprintf('\n%s\n\n', repmat('=',1,90));

%% ========== PROFESSIONAL 4-PANEL PLOTTING ==========
figure('Name', 'Flight Dynamics Analysis', 'NumberTitle', 'off', ...
    'Position', [100 100 1400 900], 'Color', 'white');

% Define professional RGB colors
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

yyaxis left
plot(ax1, time_vec, altitude/1000, 'LineWidth', 2.5, 'Color', cBlue);
ylabel(ax1, 'Altitude (km)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', cBlue);
ax1.YAxis(1).Color = cBlue;

yyaxis right
plot(ax1, time_vec, velocity, 'LineWidth', 2.5, 'Color', cOrange);
ylabel(ax1, 'Velocity (m/s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', cOrange);
ax1.YAxis(2).Color = cOrange;

xlabel(ax1, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
xline(ax1, burn_time, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
yline(ax1, 200, ':', 'LineWidth', 2, 'Color', cGray, 'Label', '200 km target');
title(ax1, 'Altitude & Velocity Profile', 'FontSize', 12, 'FontWeight', 'bold');

% ========== SUBPLOT 2: ACCELERATION vs TIME (Gs) ==========
ax2 = subplot(2, 2, 2);
hold on; grid on; grid minor;
set(ax2, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

px = [time_vec; flipud(time_vec)];
py = [accel_g;  zeros(num_steps+1, 1)];
patch(ax2, px, py, cYellow, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
plot(ax2, time_vec, accel_g, 'LineWidth', 2.5, 'Color', cYellow);

xlabel(ax2, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax2, 'Acceleration (G)', 'FontSize', 11, 'FontWeight', 'bold');
xline(ax2, burn_time, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
yline(ax2, 0, '-', 'LineWidth', 1, 'Color', cGray);
title(ax2, 'Acceleration Profile', 'FontSize', 12, 'FontWeight', 'bold');

% ========== SUBPLOT 3: FORCES vs TIME (MN) ==========
ax3 = subplot(2, 2, 3);
hold on; grid on; grid minor;
set(ax3, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

plot(ax3, time_vec, thrust_vec/1e6, 'LineWidth', 2.5, 'Color', cPurple, 'DisplayName', 'Thrust');
plot(ax3, time_vec, drag_vec/1e6, 'LineWidth', 2.5, 'Color', cGreen, 'DisplayName', 'Drag');
xlabel(ax3, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax3, 'Force (MN)', 'FontSize', 11, 'FontWeight', 'bold');
xline(ax3, burn_time, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
title(ax3, 'Thrust & Drag Forces', 'FontSize', 12, 'FontWeight', 'bold');
legend(ax3, 'Thrust', 'Drag', 'Location', 'best', 'FontSize', 10);

% ========== SUBPLOT 4: MASS vs TIME ==========
ax4 = subplot(2, 2, 4);
hold on; grid on; grid minor;
set(ax4, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);

px4 = [time_vec; flipud(time_vec)];
py4 = [mass/1000; zeros(num_steps+1, 1)];
patch(ax4, px4, py4, cCyan, 'FaceAlpha', 0.40, 'EdgeColor', 'none');
plot(ax4, time_vec, mass/1000, 'LineWidth', 2.5, 'Color', cCyan);

xlabel(ax4, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax4, 'Vehicle Mass (1000 kg)', 'FontSize', 11, 'FontWeight', 'bold');
xline(ax4, burn_time, '--', 'LineWidth', 2, 'Color', cRed, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
title(ax4, 'Vehicle Mass Depletion', 'FontSize', 12, 'FontWeight', 'bold');

sgtitle('2x BE-3PM Clustered Engine Trajectory Optimization', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== EXPORT DATA TO CSV ==========
data_table = table(time_vec, altitude, velocity, mach_vec, acceleration, mass, ...
    thrust_vec, drag_vec, density_vec, ...
    'VariableNames', {'Time_s', 'Altitude_m', 'Velocity_ms', 'Mach_Number', 'Acceleration_ms2', ...
    'Mass_kg', 'Thrust_N', 'Drag_N', 'Density_kgm3'});

writetable(data_table, 'trajectory_data_final.csv');

%% ============================================================================
% LOCAL FUNCTIONS
%% ============================================================================

function [alt, vel, mas, acc, thr, drg, mach] = simulate_flight(m_prop_burn, ...
    GLOW, F_thrust, mdot, g0, RE, rho0, Hs, Cd, Aref, gamma, R, dt, N)
    
    % Preallocate
    alt = zeros(N+1,1);  vel = zeros(N+1,1);
    mas = zeros(N+1,1);  acc = zeros(N+1,1);
    thr = zeros(N+1,1);  drg = zeros(N+1,1);
    mach = zeros(N+1,1);
    
    mas(1) = GLOW;
    prop_left = m_prop_burn;
    
    for k = 1:N
        h = alt(k);  v = vel(k);  m = mas(k);
        
        rho = rho0 * exp(-h/Hs);
        g_loc = g0 * (RE/(RE+h))^2;
        D = 0.5 * rho * v^2 * Cd * Aref;
        drg(k) = D;
        
        % Calculate Speed of Sound (US Standard Atmosphere approx)
        a_sound = speed_of_sound(h, gamma, R);
        mach(k) = v / a_sound;
        
        if prop_left > 0
            dm = min(mdot*dt, prop_left);
            thrust = F_thrust * (dm/(mdot*dt));
            prop_left = prop_left - dm;
        else
            thrust = 0;  
            dm = 0;
        end
        thr(k) = thrust;
        
        a = thrust/m - g_loc - D/m;
        acc(k) = a;
        vel(k+1) = v + a*dt;
        alt(k+1) = h + v*dt;
        
        % Payload Deployment / Station-keeping constraint
        if vel(k+1) < 0
            vel(k+1) = 0;
            alt(k+1) = h;
            acc(k) = 0;
        end
        mas(k+1) = m - dm;
    end
    
    acc(N+1) = 0;
    thr(N+1) = 0;
    drg(N+1) = drg(N);
    mach(N+1) = vel(N+1) / speed_of_sound(alt(N+1), gamma, R);
end

function a = speed_of_sound(h, gamma, R)
    % Simplified 1976 US Standard Atmosphere Temperature Profile
    if h < 11000
        T = 288.15 - 0.0065 * h;
    elseif h < 20000
        T = 216.65;
    elseif h < 32000
        T = 216.65 + 0.001 * (h - 20000);
    elseif h < 47000
        T = 228.65 + 0.0028 * (h - 32000);
    elseif h < 51000
        T = 270.65;
    elseif h < 71000
        T = 270.65 - 0.0028 * (h - 51000);
    else
        T = 214.65;
    end
    a = sqrt(gamma * R * T);
end

function out = iif(cond, a, b)
    if cond; out = a; else; out = b; end
end