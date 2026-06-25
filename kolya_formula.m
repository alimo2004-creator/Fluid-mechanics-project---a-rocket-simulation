%% ============================================================================
%  SUBORBITAL LAUNCH VEHICLE: TRAJECTORY & NASA MASS RATIO SIZING
%  Flight Dynamics Engineer: Ali
%  ----------------------------------------------------------------------------
%  Mission: 10,000 kg Payload -> 200 km Apogee -> 300 s Mission Window
%  Architecture: 3x BE-3PM Engine Cluster (Fixed Mass Sizing from Table)
%% ============================================================================
clear all; close all; clc;

%% ============================================================================
%  VEHICLE CONSTANTS  —  EXACT MATCH TO 3x BE-3PM IMAGE TABLE
%% ============================================================================
target_altitude = 200000;       % m (200 km)
target_time     = 300;          % s
dt              = 0.1;          % s (Euler timestep)
num_steps       = target_time / dt;

% Fixed Mass Parameters (from the table)
m_d = 10000;  % kg (Payload Mass)
m_p = 11000;  % kg (Propellant Mass Capacity)
m_s = 1500;   % kg (Structural/Dry Mass)

% NASA Empty and Full Masses
m_e = m_d + m_s;       % 11,500 kg
m_f = m_e + m_p;       % 22,500 kg (Total Liftoff Wet Mass)

% Engine cluster constants
F_thrust_full = 1470000;      % N  (3x 490 kN full throttle)
v_e           = 425 * 9.81;   % m/s (Isp=425s trajectory average)
g_0           = 9.81;         % m/s^2

% Thrust-to-Weight Ratio from the table
TWR_liftoff = 2;
F_thrust = TWR_liftoff * m_f * g_0;  % ~441,450 N (Throttled thrust)
mass_flow = F_thrust / v_e;          % kg/s

% Atmosphere and gravity
H_scale   = 8500;               % m (scale height)
rho_0     = 1.225;              % kg/m^3 (sea level density)
R_E       = 6.371e6;            % m (Earth radius)
gamma_air = 1.4;                % Ratio of specific heats for air
R_air     = 287.05;             % J/(kg*K) specific gas constant

% Aerodynamics
C_d       = 0.09;               % drag coefficient
A_ref     = 8.5;                % m^2 

%% ========== DYNAMIC OPTIMIZATION LOOP ==========
m_p_burn_lower = 1000;  % kg (Minimum possible propellant burned)
m_p_burn_upper = m_p;   % kg (Maximum capacity: 11,000 kg)
tolerance = 2;          % m (strict convergence tolerance)
max_iterations = 80;
iteration = 0;

fprintf('\n%s\n', repmat('=',1,90));
fprintf('SUBORBITAL TRAJECTORY OPTIMIZATION - FIXED TABLE SIZING\n');
fprintf('OBJECTIVE: Natural Apogee (Peak) = 200,000 m (Velocity inherently 0 m/s)\n');
fprintf('%s\n', repmat('=',1,90));

while iteration < max_iterations
    iteration = iteration + 1;
    m_p_burn = (m_p_burn_lower + m_p_burn_upper) / 2;  % Burned Propellant
    
    % Run encapsulated physics simulation
    [alt, vel, ~, ~, ~, ~, ~, ~] = simulate_flight(m_p_burn, m_f, F_thrust, mass_flow, ...
        g_0, R_E, rho_0, H_scale, C_d, A_ref, gamma_air, R_air, dt, num_steps);
    
    % Find the natural Peak (Apogee) instead of the final altitude
    [peak_altitude, peak_idx] = max(alt);
    peak_velocity = vel(peak_idx); % Mathematically, velocity at peak crosses 0
    
    altitude_error = peak_altitude - target_altitude;
    burn_time_approx = m_p_burn / mass_flow;
    
    % Velocity added to the display menu
    fprintf('Iter %2d | Prop Burned = %8.1f kg | Peak Alt = %9.1f m | Error = %+7.1f m | Apogee Vel = %5.2f m/s\n', ...
        iteration, m_p_burn, peak_altitude, altitude_error, peak_velocity);
    
    if abs(altitude_error) < tolerance
        fprintf('%s\n', repmat('=',1,90));
        fprintf('✓✓✓ CONVERGENCE ACHIEVED in %d iterations ✓✓✓\n', iteration);
        fprintf('%s\n', repmat('=',1,90));
        break;
    end
    
    if altitude_error > 0
        m_p_burn_upper = m_p_burn; % Overshot target
    else
        m_p_burn_lower = m_p_burn; % Undershot target
    end
end

%% ========== NASA RATIO CALCULATIONS ==========
lambda  = m_d / (m_p + m_s);
epsilon = m_s / (m_p + m_s);
MR_actual = m_f / m_e;
MR_nasa_formula = (1 + lambda) / (epsilon + lambda);

%% ========== FINAL TRAJECTORY COMPUTATION (CLEAN DATA) ==========
[altitude, velocity, mass, acceleration, thrust_vec, drag_vec, mach_vec, vel_hw_cum] = simulate_flight(...
    m_p_burn, m_f, F_thrust, mass_flow, g_0, R_E, rho_0, H_scale, C_d, A_ref, gamma_air, R_air, dt, num_steps);

time_vec = (0:dt:target_time)';
density_vec = rho_0 * exp(-altitude / H_scale);
accel_g = acceleration / g_0;
peak_acceleration = max(accel_g);

meco_idx = find(thrust_vec < 1, 1) - 1;
if isempty(meco_idx) || meco_idx < 1
    meco_idx = round((m_p_burn/mass_flow)/dt); 
end
burn_time = time_vec(meco_idx);

[final_apogee, apogee_idx] = max(altitude);
apogee_time = time_vec(apogee_idx);

%% ========== DIAGNOSTIC SUMMARY ==========
fprintf('\n%s\n', repmat('=',1,90));
fprintf('MISSION OPTIMIZATION RESULTS\n');
fprintf('%s\n', repmat('=',1,90));

fprintf('\nEXACT MASS DEFINITIONS (FROM TABLE):\n');
fprintf('  Payload Mass (md)                    : %10.0f kg\n', m_d);
fprintf('  Propellant Capacity (mp)             : %10.0f kg\n', m_p);
fprintf('  Structural/Dry Mass (ms)             : %10.0f kg\n', m_s);
fprintf('  Total Liftoff Wet Mass (m0 / mf)     : %10.0f kg\n', m_f);
fprintf('  Empty Mass (me = md + ms)            : %10.0f kg\n', m_e);

fprintf('\nNASA MASS RATIOS & VALIDATION:\n');
fprintf('  Payload Ratio (lambda)               : %10.4f\n', lambda);
fprintf('  Structural Coefficient (epsilon)     : %10.4f\n', epsilon);
fprintf('  Propellant Mass Ratio (MR = mf/me)   : %10.4f\n', MR_actual);
fprintf('  Validation: (1+lam)/(eps+lam)        : %10.4f  (Matches MR!)\n', MR_nasa_formula);

fprintf('\nMISSION PERFORMANCE:\n');
fprintf('  Propellant ACTUALLY Burned           : %10.1f kg (%.1f%% of capacity)\n', m_p_burn, (m_p_burn/m_p)*100);
fprintf('  Main Engine Cutoff (MECO) Time       : %10.2f s\n', burn_time);
fprintf('  Time of Apogee (Peak Altitude)       : %10.2f s\n', apogee_time);
fprintf('  Apogee Altitude                      : %10.1f m <<<\n', final_apogee);
fprintf('  Apogee Velocity (Target = 0 m/s)     : %10.3f m/s <<<\n', velocity(apogee_idx));
fprintf('  Final Altitude (End of Sim)          : %10.1f m\n', altitude(end));
fprintf('  Final Velocity (Falling)             : %10.1f m/s\n', velocity(end));

fprintf('\nDYNAMIC CHARACTERISTICS:\n');
fprintf('  Total Combined Liftoff Thrust        : %10.1f kN\n', F_thrust_full / 1000);
fprintf('  Thrust-to-Weight Ratio (TWR)         : %10.2f \n', F_thrust / (m_f * g_0));
fprintf('  Required Engine Throttle             : %10.2f %% \n', (F_thrust/F_thrust_full)*100);
fprintf('  Peak Acceleration                    : %10.3f G\n', peak_acceleration);

fprintf('\nAERODYNAMIC PROPERTIES:\n');
fprintf('  Reference Area                       : %10.3f m^2\n', A_ref);
fprintf('  Drag Coefficient                     : %10.3f\n', C_d);
fprintf('  Max Dynamic Pressure                 : %10.2f Pa\n', max(0.5*density_vec.*velocity.^2));
fprintf('\n%s\n\n', repmat('=',1,90));

%% ========== MISSION VALIDATION ==========
fprintf('%s\n', repmat('=',1,90));
fprintf('MISSION COMPLIANCE VERIFICATION\n');
fprintf('%s\n', repmat('=',1,90));

payload_check = abs(m_d - 10000) < 1;
altitude_check = abs(final_apogee - 200000) < 100;
velocity_check = abs(velocity(apogee_idx)) < 1;

fprintf('  [1] Payload Mass md (10,000 kg)      : %s\n', iif(payload_check, '✓ PASS', '✗ FAIL'));
fprintf('  [2] Peak Apogee Altitude (200,000 m) : %s (actual = %.1f m)\n', iif(altitude_check, '✓ PASS', '✗ FAIL'), final_apogee);
fprintf('  [3] Velocity at Apogee (~0 m/s)      : %s (actual = %.4f m/s)\n', iif(velocity_check, '✓ PASS', '✗ FAIL'), velocity(apogee_idx));

all_checks_pass = payload_check && altitude_check && velocity_check;
fprintf('\n');
if all_checks_pass
    fprintf('  ╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('  ║  ✓✓✓ EXACT-SIZED MISSION PROFILE ACHIEVED ✓✓✓                 ║\n');
    fprintf('  ║  - Payload: 10,000 kg delivered                               ║\n');
    fprintf('  ║  - Dynamics: Rocket naturally hits 0 m/s without artificial   ║\n');
    fprintf('  ║    walls, reaches exactly 200,000 m apogee, and falls back.   ║\n');
    fprintf('  ╚═══════════════════════════════════════════════════════════════╝\n');
else
    fprintf('  ✗✗✗ MISSION PROFILE INCOMPLETE - REVIEW PARAMETERS ✗✗✗\n');
end
fprintf('\n%s\n\n', repmat('=',1,90));

%% ========== PROFESSIONAL 4-PANEL PLOTTING ==========
figure('Name', 'Flight Dynamics Analysis', 'NumberTitle', 'off', ...
    'Position', [100 100 1400 900], 'Color', 'white');

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
plot(ax1, time_vec, velocity, 'LineWidth', 2.5, 'Color', cOrange, 'DisplayName', 'Standard Velocity');
plot(ax1, time_vec, vel_hw_cum, '--', 'LineWidth', 2.5, 'Color', cRed, 'DisplayName', 'Handwritten Eq. Velocity');
ylabel(ax1, 'Velocity (m/s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', cOrange);
ax1.YAxis(2).Color = cOrange;

xlabel(ax1, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
xline(ax1, burn_time, '--', 'LineWidth', 2, 'Color', cGray, 'Label', 'MECO', 'LabelVerticalAlignment', 'bottom');
xline(ax1, apogee_time, ':', 'LineWidth', 2, 'Color', cGreen, 'Label', 'Apogee (v=0)', 'LabelVerticalAlignment', 'bottom');
title(ax1, 'Altitude & Velocity Profile', 'FontSize', 12, 'FontWeight', 'bold');
legend(ax1, 'Location', 'best', 'FontSize', 10);

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

sgtitle('3x BE-3PM (Natural Apogee Target) Optimization', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== EXPORT DATA TO CSV ==========
data_table = table(time_vec, altitude, velocity, vel_hw_cum, mach_vec, acceleration, mass, ...
    thrust_vec, drag_vec, density_vec, ...
    'VariableNames', {'Time_s', 'Altitude_m', 'Velocity_ms', 'Handwritten_Velocity_ms', 'Mach_Number', 'Acceleration_ms2', ...
    'Mass_kg', 'Thrust_N', 'Drag_N', 'Density_kgm3'});

writetable(data_table, 'trajectory_data_final.csv');

%% ============================================================================
% LOCAL FUNCTIONS
%% ============================================================================

function [alt, vel, mas, acc, thr, drg, mach, vel_hw_cum] = simulate_flight(m_p_burn, ...
    m_f, F_thrust, mdot, g0, RE, rho0, Hs, Cd, Aref, gamma, R, dt, N)
    
    % Preallocate arrays
    alt = zeros(N+1,1);  vel = zeros(N+1,1);
    mas = zeros(N+1,1);  acc = zeros(N+1,1);
    thr = zeros(N+1,1);  drg = zeros(N+1,1);
    mach = zeros(N+1,1); vel_hw_cum = zeros(N+1,1);
    
    mas(1) = m_f;  
    prop_left = m_p_burn;
    
    % Base parameters for Handwritten Quadratic Equation
    V_e = F_thrust / mdot; 
    rho_e_Ae = mdot / V_e; 
    v_hw_current = 0; % Accumulator
    
    for k = 1:N
        h = alt(k);  v = vel(k);  m = mas(k);
        
        rho = rho0 * exp(-h/Hs);
        g_loc = g0 * (RE/(RE+h))^2;
        D = 0.5 * rho * v^2 * Cd * Aref;
        drg(k) = D;
        
        a_sound = speed_of_sound(h, gamma, R);
        mach(k) = v / a_sound;
        
        if prop_left > 0
            dm = min(mdot*dt, prop_left);
            thrust = F_thrust * (dm/(mdot*dt));
            prop_left = prop_left - dm;
            current_mdot = mdot;
        else
            thrust = 0;  
            dm = 0;
            current_mdot = 0;
        end
        thr(k) = thrust;
        
        % -------------------------------------------------------------
        % SOLVING THE HANDWRITTEN QUADRATIC EQUATION
        % -------------------------------------------------------------
        A_hw = rho_e_Ae - 0.5 * Cd * rho * Aref;
        B_hw = 2 * rho_e_Ae * V_e + current_mdot - (m / dt);
        C_hw = rho_e_Ae * V_e^2 + current_mdot * g_loc * dt - m * g_loc;
        
        if A_hw ~= 0
            disc = B_hw^2 - 4 * A_hw * C_hw;
            if disc >= 0
                root1 = (-B_hw + sqrt(disc)) / (2 * A_hw);
                root2 = (-B_hw - sqrt(disc)) / (2 * A_hw);
                if abs(root1) < abs(root2)
                    V_r_step = root1;
                else
                    V_r_step = root2;
                end
            else
                V_r_step = 0;
            end
        else
            V_r_step = -C_hw / B_hw;
        end
        
        v_hw_current = v_hw_current + V_r_step;
        vel_hw_cum(k+1) = v_hw_current;
        
        % -------------------------------------------------------------
        % STANDARD EULER INTEGRATION (WALL-FREE)
        % -------------------------------------------------------------
        % Notice: sign(v) applied to Drag so air resistance points UP when falling
        a = thrust/m - g_loc - sign(v)*(D/m);
        acc(k) = a;
        vel(k+1) = v + a*dt;
        alt(k+1) = h + v*dt;
        
        mas(k+1) = m - dm;
        
        % Terminate simulation if rocket hits the ground after liftoff
        if alt(k+1) < 0 && k > 10
            alt(k+1:end) = 0;
            vel(k+1:end) = 0;
            break;
        end
    end
    
    acc(N+1) = 0;
    thr(N+1) = 0;
    drg(N+1) = drg(N);
    mach(N+1) = vel(N+1) / speed_of_sound(alt(N+1), gamma, R);
end

function a = speed_of_sound(h, gamma, R)
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