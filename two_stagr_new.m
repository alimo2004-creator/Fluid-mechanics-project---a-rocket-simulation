%% ============================================================================
%  SUBORBITAL LAUNCH VEHICLE: TWO-STAGE DYNAMIC SIZING & TRAJECTORY
%  Flight Dynamics Engineer: Ali
%  ----------------------------------------------------------------------------
%  Mission: 10,000 kg Payload -> 200 km Natural Apogee
%  Architecture (Real-World Hydrolox): 
%     - STAGE 1: 1x Snecma Vulcain 2 (Ariane 5 Main Engine)
%     - STAGE 2: 2x Aerojet Rocketdyne RL10C-1 (Atlas V Upper Stage)
%% ============================================================================
clear all; close all; clc;

%% ============================================================================
%  VEHICLE CONSTANTS & REAL-WORLD ENGINE SPECIFICATIONS
%% ============================================================================
target_altitude = 200000;       % m (200 km)
target_time     = 300;          % s
dt              = 0.1;          % s (Euler timestep)
num_steps       = target_time / dt;

m_d = 10000;  % kg (Fixed Payload Mass)

% --- STAGE 1 (Booster Stage - Ariane 5 Sea-Level Hydrolox) ---
dry_engine_mass1 = 1800;        % kg (1x Snecma Vulcain 2)
F_thrust_full1   = 960000;      % N  (1x 960 kN Max Thrust)
v_e1             = 318 * 9.81;  % m/s (Isp=318s)
P0_1             = 11.5e6;      % Pa (Vulcain 2 Chamber Pressure)
T0_1             = 3500;        % K  (Vulcain 2 Chamber Temperature)

% --- STAGE 2 (Upper Stage - Atlas V / Vulcan Vacuum Hydrolox) ---
dry_engine_mass2 = 380;         % kg (2x Aerojet Rocketdyne RL10C-1 @ 190 kg each)
F_thrust_full2   = 212600;      % N  (2x 106.3 kN Max Thrust)
v_e2             = 449.7 * 9.81;% m/s (Isp=449.7s)
P0_2             = 4.0e6;       % Pa (RL10C-1 Chamber Pressure)
T0_2             = 3300;        % K  (RL10C-1 Chamber Temperature)

g_0       = 9.81;               % m/s^2
H_scale   = 8500;               % m
rho_0     = 1.225;              % kg/m^3
R_E       = 6.371e6;            % m
gamma_air = 1.4;                
R_air     = 287.05;             % J/(kg*K)
C_d       = 0.09;               % drag coefficient
D_rocket  = 3.5;                % m (Rocket Diameter)
A_ref     = pi * (D_rocket^2) / 4; % m^2 (Cross-sectional area: ~9.62 m^2)

%% ========== TRUE DYNAMIC OPTIMIZATION LOOP ==========
% Real engines like Vulcain 2 run at fixed 100% thrust.
% The optimizer calculates the EXACT propellant mass needed to hit 200 km.
% We allocate 70% of it to Stage 1, and 30% of it to Stage 2.

m_p_total_lower = 1000;  
m_p_total_upper = 50000; 
tolerance = 1;      
max_iterations = 80;
iteration = 0;

fprintf('\n%s\n', repmat('=',1,90));
fprintf('SUBORBITAL TRAJECTORY OPTIMIZATION - REAL-WORLD HYDROLOX SIZING\n');
fprintf('OBJECTIVE: Natural Apogee (Peak) = 200,000 m \n');
fprintf('%s\n', repmat('=',1,90));

while iteration < max_iterations
    iteration = iteration + 1;
    m_p_total = (m_p_total_lower + m_p_total_upper) / 2;  
    
    % --- STAGING MASS DISTRIBUTION ---
    m_p1 = 0.70 * m_p_total; % Stage 1 gets 70% of fuel
    m_p2 = 0.30 * m_p_total; % Stage 2 gets 30% of fuel
    
    % Hydrolox tanks (LH2/LOX) require high volume: 10% mass fraction
    m_s1_tanks = 0.10 * m_p1;
    m_s1 = dry_engine_mass1 + m_s1_tanks; % Stage 1 Structure
    
    m_s2_tanks = 0.10 * m_p2;
    m_s2 = dry_engine_mass2 + m_s2_tanks; % Stage 2 Structure
    
    % Mass cascade (Top down)
    m_stage2_full = m_d + m_s2 + m_p2;          % What Stage 2 weighs at ignition
    GLOW = m_stage2_full + m_s1 + m_p1;         % Gross Liftoff Weight
    
    % --- FIXED THRUST LOGIC ---
    F_thrust1 = F_thrust_full1;
    mass_flow1 = F_thrust1 / v_e1;
    
    F_thrust2 = F_thrust_full2;
    mass_flow2 = F_thrust2 / v_e2;
    
    % Run physics simulation with discrete staging
    [alt, vel, ~, ~, ~, ~, ~, ~, ~, ~, ~] = simulate_2stage_flight(...
        m_p1, m_p2, m_s1, GLOW, F_thrust1, F_thrust2, mass_flow1, mass_flow2, ...
        P0_1, T0_1, P0_2, T0_2, g_0, R_E, rho_0, H_scale, C_d, A_ref, gamma_air, R_air, dt, num_steps);
    
    [peak_altitude, peak_idx] = max(alt);
    peak_velocity = vel(peak_idx); 
    
    altitude_error = peak_altitude - target_altitude;
    
    fprintf('Iter %2d | Prop Total = %7.1f kg | GLOW = %7.1f kg | Error = %+8.1f m | Apogee Vel = %5.2f m/s\n', ...
        iteration, m_p_total, GLOW, altitude_error, peak_velocity);
    
    if abs(altitude_error) < tolerance
        fprintf('%s\n', repmat('=',1,90));
        fprintf('✓✓✓ PERFECT CONVERGENCE ACHIEVED in %d iterations ✓✓✓\n', iteration);
        fprintf('%s\n', repmat('=',1,90));
        break;
    end
    
    if altitude_error > 0
        m_p_total_upper = m_p_total; 
    else
        m_p_total_lower = m_p_total; 
    end
end

%% ========== NASA RATIO CALCULATIONS ==========
lambda  = m_d / (m_p_total + m_s1 + m_s2);
epsilon = (m_s1 + m_s2) / (m_p_total + m_s1 + m_s2);
m_e_total = m_d + m_s1 + m_s2;
MR_actual = GLOW / m_e_total;
MR_nasa_formula = (1 + lambda) / (epsilon + lambda);

%% ========== GEOMETRIC SIZING CALCULATION (O/F = 6.0) ==========
% Density: LH2 = 71 kg/m^3 | LOX = 1141 kg/m^3
m_LH2 = m_p_total / 7; 
m_LOX = 6 * m_p_total / 7;
vol_LH2 = m_LH2 / 71; 
vol_LOX = m_LOX / 1141;
vol_total_prop = vol_LH2 + vol_LOX;

% Tank Length = Volume / Cross-sectional Area (plus 10% for domes/ullage)
L_tanks = (vol_total_prop / A_ref) * 1.1;
L_payload_fairing = 8.0; % Estimated 8m for 10-ton payload
L_engines_interstage = 6.0; % Estimated 6m for engines and stage separation gaps
Total_Length = L_tanks + L_payload_fairing + L_engines_interstage;

%% ========== FINAL TRAJECTORY COMPUTATION (CLEAN DATA) ==========
[altitude, velocity, mass, acceleration, thrust_vec, drag_vec, mach_vec, vel_hw_cum, weight_vec, P_comb_vec, T_comb_vec] = simulate_2stage_flight(...
        m_p1, m_p2, m_s1, GLOW, F_thrust1, F_thrust2, mass_flow1, mass_flow2, ...
        P0_1, T0_1, P0_2, T0_2, g_0, R_E, rho_0, H_scale, C_d, A_ref, gamma_air, R_air, dt, num_steps);

time_vec = (0:dt:target_time)';
density_vec = rho_0 * exp(-altitude / H_scale);
accel_g = acceleration / g_0;
peak_acceleration = max(accel_g);

[final_apogee, apogee_idx] = max(altitude);
apogee_time = time_vec(apogee_idx);

% Extract MECO based on Stage 2 shutoff
meco_idx = find(thrust_vec < 1 & time_vec > 10, 1) - 1;
if isempty(meco_idx), meco_idx = apogee_idx; end
burn_time = time_vec(meco_idx);

%% ========== THERMODYNAMIC NOZZLE CALCULATIONS ==========
% Stage 1 (Vulcain 2 Throat)
rho0_1 = P0_1 / (R_air * T0_1);
T_star_1 = (2 * T0_1) / (gamma_air + 1);
rho_star_1 = rho0_1 / ((gamma_air + 1) / 2)^(1 / (gamma_air - 1));
A_star_S1 = mass_flow1 / (rho_star_1 * sqrt(gamma_air * R_air * T_star_1)); 

% Stage 2 (RL10 Throat)
rho0_2 = P0_2 / (R_air * T0_2);
T_star_2 = (2 * T0_2) / (gamma_air + 1);
rho_star_2 = rho0_2 / ((gamma_air + 1) / 2)^(1 / (gamma_air - 1));
A_star_S2 = mass_flow2 / (rho_star_2 * sqrt(gamma_air * R_air * T_star_2));

%% ========== DASHBOARD SUMMARY (COMPLETE ROCKET DESIGN) ==========
fprintf('\n%s\n', repmat('=',1,90));
fprintf('🚀 COMPLETE ROCKET DESIGN & DYNAMICS DASHBOARD\n');
fprintf('%s\n', repmat('=',1,90));

fprintf('\nGEOMETRIC SIZING & DESIGN:\n');
fprintf('  Vehicle Diameter                     : %10.1f m\n', D_rocket);
fprintf('  Total Estimated Length               : %10.1f m\n', Total_Length);
fprintf('  Propellant Volume Required           : %10.1f m^3\n', vol_total_prop);

fprintf('\nSTAGE 1 (BOOSTER) - 1x Snecma Vulcain 2:\n');
fprintf('  Propellant Mass (mp1)                : %10.1f kg\n', m_p1);
fprintf('  Structural Mass (ms1)                : %10.1f kg\n', m_s1);
fprintf('  Chamber Pressure & Temp              : %10.1f MPa | %.0f K\n', P0_1/1e6, T0_1);
fprintf('  Nozzle Throat Area (A*)              : %10.4f m^2\n', A_star_S1);
fprintf('  Engine Output (Fixed 100%% Thrust)    : %10.1f kN\n', F_thrust1/1000);

fprintf('\nSTAGE 2 (UPPER STAGE) - 2x Aerojet Rocketdyne RL10C-1:\n');
fprintf('  Payload Mass (md)                    : %10.1f kg\n', m_d);
fprintf('  Propellant Mass (mp2)                : %10.1f kg\n', m_p2);
fprintf('  Structural Mass (ms2)                : %10.1f kg\n', m_s2);
fprintf('  Chamber Pressure & Temp              : %10.1f MPa | %.0f K\n', P0_2/1e6, T0_2);
fprintf('  Combined Nozzle Throat Area (A*)     : %10.4f m^2\n', A_star_S2);
fprintf('  Engine Output (Fixed 100%% Thrust)    : %10.1f kN\n', F_thrust2/1000);

fprintf('\nSYSTEM WEIGHTS & RATIOS:\n');
fprintf('  Total Liftoff Wet Mass (GLOW)        : %10.1f kg\n', GLOW);
fprintf('  Total System Propellant              : %10.1f kg\n', m_p_total);
fprintf('  Total Liftoff Weight                 : %10.1f kN\n', (GLOW * g_0) / 1000);
fprintf('  Payload Ratio (lambda)               : %10.4f\n', lambda);
fprintf('  Propellant Mass Ratio (MR = mf/me)   : %10.4f\n', MR_actual);

fprintf('\nMISSION DYNAMICS (AT APOGEE):\n');
fprintf('  Final MECO Time (Stage 2 Cutoff)     : %10.2f s\n', burn_time);
fprintf('  Time of Apogee (Peak Altitude)       : %10.2f s\n', apogee_time);
fprintf('  Apogee Altitude                      : %10.1f m <<<\n', final_apogee);
fprintf('  Apogee Velocity (Target = 0 m/s)     : %10.4f m/s <<<\n', velocity(apogee_idx));
fprintf('  Final Vehicle Mass (Empty)           : %10.1f kg\n', mass(end)); 
fprintf('  Final Vehicle Weight (in Orbit)      : %10.1f kN\n', weight_vec(end) / 1000); 
fprintf('  Liftoff TWR (Stage 1)                : %10.3f \n', F_thrust1 / (GLOW * g_0));
fprintf('  Ignition TWR (Stage 2)               : %10.3f \n', F_thrust2 / (m_stage2_full * g_0));
fprintf('  Peak Acceleration                    : %10.3f G\n', peak_acceleration);
fprintf('\n%s\n\n', repmat('=',1,90));

%% ========== MISSION VALIDATION ==========
payload_check = abs(m_d - 10000) < 1;
altitude_check = abs(final_apogee - 200000) < 2; 
velocity_check = abs(velocity(apogee_idx)) < 1;

if payload_check && altitude_check && velocity_check
    fprintf('  ╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('  ║  ✓✓✓ REAL-WORLD MISSION PROFILE PERFECTLY ACHIEVED ✓✓✓        ║\n');
    fprintf('  ║  - Architecture: Vulcain 2 & RL10C-1 fully documented engines.║\n');
    fprintf('  ║  - Dynamics: 100%% fixed thrust yields a highly realistic,    ║\n');
    fprintf('  ║    high-acceleration suborbital launch trajectory.            ║\n');
    fprintf('  ╚═══════════════════════════════════════════════════════════════╝\n');
end


%% ============================================================================
%  CSV TELEMETRY EXPORT (WEIGHT OMITTED AS REQUESTED)
%% ============================================================================
data_table = table(time_vec, altitude, velocity, mass, thrust_vec, drag_vec, ...
    acceleration, T_atm_vec, P_atm_vec, rho_atm_vec, a_sound_vec, mach_vec, mdot_out_vec, ...
    'VariableNames', {'Time_s','Altitude_m','Velocity_ms','Mass_kg', ...
    'Thrust_N','Drag_N','Acceleration_ms2','Temp_K','Press_Pa','Density_kgm3', ...
    'Speed_of_Sound_ms','Mach_Number','Mass_Flow_Rate_kgs'});
csv_path = fullfile(pwd, 'Delta_IV_Comprehensive_Telemetry.csv');
writetable(data_table, csv_path);
fprintf('  CSV saved to: %s\n\n', csv_path);

%% ========== PROFESSIONAL 6-PANEL PLOTTING ==========
figure('Name', 'Complete Dynamics vs Time Analysis', 'NumberTitle', 'off', ...
    'Position', [50 50 1800 1000], 'Color', 'white');

cBlue = [0, 0.447, 0.741]; cOrange = [0.85, 0.325, 0.098]; cYellow = [0.929, 0.694, 0.125]; 
cPurple = [0.494, 0.184, 0.556]; cGreen = [0.466, 0.674, 0.188]; cRed = [0.8, 0, 0]; cGray = [0.5, 0.5, 0.5]; cCyan = [0.301, 0.745, 0.933];

% PANEL 1: ALTITUDE & VELOCITY
ax1 = subplot(2, 3, 1); hold on; grid on; grid minor; set(ax1, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);
yyaxis left; plot(ax1, time_vec, altitude/1000, 'LineWidth', 2.5, 'Color', cBlue); ylabel('Altitude (km)'); ax1.YAxis(1).Color = cBlue;
yyaxis right; plot(ax1, time_vec, velocity, 'LineWidth', 2.5, 'Color', cOrange, 'DisplayName', 'Standard Velocity'); 
plot(ax1, time_vec, vel_hw_cum, '--', 'LineWidth', 2.5, 'Color', cRed, 'DisplayName', 'Handwritten Eq. Velocity');
ylabel('Velocity (m/s)'); ax1.YAxis(2).Color = cOrange;
xlabel('Time (s)'); xline(ax1, apogee_time, ':', 'LineWidth', 2, 'Color', cGreen, 'Label', 'Apogee (v=0)', 'LabelVerticalAlignment', 'bottom');
title('Altitude & Velocity Profile'); legend('Location', 'best');

% PANEL 2: ACCELERATION
ax2 = subplot(2, 3, 2); hold on; grid on; grid minor; set(ax2, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);
px = [time_vec; flipud(time_vec)]; py = [accel_g;  zeros(num_steps+1, 1)];
patch(ax2, px, py, cYellow, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
plot(ax2, time_vec, accel_g, 'LineWidth', 2.5, 'Color', cYellow);
xlabel('Time (s)'); ylabel('Acceleration (G)'); yline(ax2, 0, '-', 'LineWidth', 1, 'Color', cGray);
title('Acceleration Profile (Notice the Staging Spike)');

% PANEL 3: THRUST & DRAG
ax3 = subplot(2, 3, 3); hold on; grid on; grid minor; set(ax3, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);
plot(ax3, time_vec, thrust_vec/1000, 'LineWidth', 2.5, 'Color', cPurple, 'DisplayName', 'Thrust');
plot(ax3, time_vec, drag_vec/1000, 'LineWidth', 2.5, 'Color', cGreen, 'DisplayName', 'Drag');
xlabel('Time (s)'); ylabel('Force (kN)'); title('Thrust & Drag Forces'); legend('Location', 'best');

% PANEL 4: MASS & WEIGHT
ax4 = subplot(2, 3, 4); hold on; grid on; grid minor; set(ax4, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);
yyaxis left; plot(ax4, time_vec, mass/1000, 'LineWidth', 2.5, 'Color', cCyan); ylabel('Mass (Metric Tons)'); ax4.YAxis(1).Color = cCyan;
yyaxis right; plot(ax4, time_vec, weight_vec/1000, '-', 'LineWidth', 2.5, 'Color', cRed); ylabel('Weight (kN)'); ax4.YAxis(2).Color = cRed;
xlabel('Time (s)'); title('Vehicle Mass & Weight vs Time');

% PANEL 5: COMBUSTION PRESSURE
ax5 = subplot(2, 3, 5); hold on; grid on; grid minor; set(ax5, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);
plot(ax5, time_vec, P_comb_vec/1e6, 'LineWidth', 2.5, 'Color', [0.85 0.33 0.1]);
xlabel('Time (s)'); ylabel('Chamber Pressure (MPa)'); title('Combustion Pressure vs Time');

% PANEL 6: COMBUSTION TEMPERATURE
ax6 = subplot(2, 3, 6); hold on; grid on; grid minor; set(ax6, 'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.5);
plot(ax6, time_vec, T_comb_vec, 'LineWidth', 2.5, 'Color', [0.93 0.69 0.13]);
xlabel('Time (s)'); ylabel('Chamber Temp (K)'); title('Combustion Temp vs Time');

sgtitle('Complete Suborbital Vehicle Dynamics', 'FontSize', 16, 'FontWeight', 'bold');

%% ========== EXPORT DATA TO CSV ==========
data_table = table(time_vec, altitude, velocity, mach_vec, accel_g, mass, weight_vec, ...
    thrust_vec, drag_vec, P_comb_vec, T_comb_vec, ...
    'VariableNames', {'Time_s', 'Altitude_m', 'Velocity_ms', 'Mach_Number', 'Acceleration_G', ...
    'Mass_kg', 'Weight_N', 'Thrust_N', 'Drag_N', 'Combustion_Pressure_Pa', 'Combustion_Temp_K'});
writetable(data_table, 'complete_rocket_dynamics.csv');

%% ============================================================================
% 2-STAGE LOCAL SIMULATION FUNCTION
%% ============================================================================
function [alt, vel, mas, acc, thr, drg, mach, vel_hw_cum, weight, P_comb, T_comb] = simulate_2stage_flight(...
    m_p1, m_p2, m_s1, GLOW, F_thrust1, F_thrust2, mdot1, mdot2, ...
    P0_1, T0_1, P0_2, T0_2, g0, RE, rho0, Hs, Cd, Aref, gamma, R, dt, N)
    
    alt = zeros(N+1,1);  vel = zeros(N+1,1); mas = zeros(N+1,1);
    acc = zeros(N+1,1);  thr = zeros(N+1,1); drg = zeros(N+1,1);
    mach = zeros(N+1,1); vel_hw_cum = zeros(N+1,1);
    weight = zeros(N+1,1); P_comb = zeros(N+1,1); T_comb = zeros(N+1,1);
    
    mas(1) = GLOW;  
    prop_left1 = m_p1; prop_left2 = m_p2;
    stage1_dropped = false; v_hw_current = 0; 
    
    for k = 1:N
        h = alt(k);  v = vel(k);  m = mas(k);
        rho = rho0 * exp(-h/Hs);
        g_loc = g0 * (RE/(RE+h))^2;
        D = 0.5 * rho * v^2 * Cd * Aref;
        drg(k) = D; mach(k) = v / speed_of_sound(h, gamma, R);
        weight(k) = m * g_loc; % Local dynamic weight tracking
        
        % --- STAGING & COMBUSTION LOGIC ---
        dropped_mass = 0;
        
        if prop_left1 > 0
            dm = min(mdot1*dt, prop_left1);
            thrust = F_thrust1 * (dm/(mdot1*dt));
            prop_left1 = prop_left1 - dm;
            current_mdot = mdot1; V_e = F_thrust1 / mdot1;
            P_c = P0_1; T_c = T0_1; % Stage 1 Chamber State
            
        elseif stage1_dropped == false
            dropped_mass = m_s1; stage1_dropped = true;
            thrust = 0; dm = 0; current_mdot = 0; V_e = F_thrust2 / mdot2; 
            P_c = 0; T_c = 0; 
            
        elseif prop_left2 > 0
            dm = min(mdot2*dt, prop_left2);
            thrust = F_thrust2 * (dm/(mdot2*dt));
            prop_left2 = prop_left2 - dm;
            current_mdot = mdot2; V_e = F_thrust2 / mdot2;
            P_c = P0_2; T_c = T0_2; % Stage 2 Chamber State
            
        else
            thrust = 0; dm = 0; current_mdot = 0; V_e = F_thrust2 / mdot2;
            P_c = 0; T_c = 0; % Coasting state
        end
        
        thr(k) = thrust; P_comb(k) = P_c; T_comb(k) = T_c;
        
        % --- HANDWRITTEN EQUATION ---
        if current_mdot > 0
            rho_e_Ae = current_mdot / V_e; A_hw = rho_e_Ae - 0.5 * Cd * rho * Aref;
            B_hw = 2 * rho_e_Ae * V_e + current_mdot - (m / dt);
            C_hw = rho_e_Ae * V_e^2 + current_mdot * g_loc * dt - m * g_loc;
            if A_hw ~= 0
                disc = B_hw^2 - 4 * A_hw * C_hw;
                if disc >= 0
                    root1 = (-B_hw + sqrt(disc)) / (2 * A_hw); root2 = (-B_hw - sqrt(disc)) / (2 * A_hw);
                    if abs(root1) < abs(root2), V_r_step = root1; else, V_r_step = root2; end
                else, V_r_step = 0; end
            else, V_r_step = -C_hw / B_hw; end
            v_hw_current = v_hw_current + V_r_step;
        else
            v_hw_current = v_hw_current - g_loc*dt - sign(v_hw_current)*(D/m)*dt; 
        end
        vel_hw_cum(k+1) = v_hw_current;
        
        % --- STANDARD EULER INTEGRATION ---
        a = thrust/m - g_loc - sign(v)*(D/m);
        acc(k) = a; vel(k+1) = v + a*dt; alt(k+1) = h + v*dt;
        mas(k+1) = m - dm - dropped_mass;
        
        if alt(k+1) < 0 && k > 10
            alt(k+1:end) = 0; vel(k+1:end) = 0; break;
        end
    end
    
    % Final step closing conditions
    acc(N+1) = 0; thr(N+1) = 0; drg(N+1) = drg(N); P_comb(N+1) = 0; T_comb(N+1) = 0;
    weight(N+1) = mas(N+1) * (g0 * (RE/(RE+alt(N+1)))^2);
    mach(N+1) = vel(N+1) / speed_of_sound(alt(N+1), gamma, R);
end

function a = speed_of_sound(h, gamma, R)
    if h < 11000, T = 288.15 - 0.0065 * h;
    elseif h < 20000, T = 216.65;
    elseif h < 32000, T = 216.65 + 0.001 * (h - 20000);
    elseif h < 47000, T = 228.65 + 0.0028 * (h - 32000);
    elseif h < 51000, T = 270.65;
    elseif h < 71000, T = 270.65 - 0.0028 * (h - 51000);
    else, T = 214.65; end
    a = sqrt(gamma * R * T);
end