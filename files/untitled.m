% =========================================================================
%   PROFESSIONAL ROCKET TRAJECTORY SIMULATOR (1D VERTICAL FLIGHT)
%   Synchronized to Python Validation Script 
%   Features: 0.01s CSV Export, Exact Python Aero/Thermodynamic Parity
% =========================================================================
clear; clc; close all;

%% 1. PLANETARY & PHYSICAL CONSTANTS (Exact Python Match)
g0        = 9.80665;        % Sea-level gravity (m/s^2) 
R_Earth   = 6356766.0;      % Earth volumetric radius (m) 
R_air     = 287.053;        % Specific gas constant of air (J/kg*K) 
gamma_air = 1.4;            % Ratio of specific heats (ambient air) 
gamma_gas = 1.26;           % Ratio of specific heats (exhaust gas) 
P_atm0    = 101325.0;       % Sea-level atmospheric pressure (Pa) 
P_e       = 101325.0;       % Nozzle exit design pressure (Pa) 
PI        = 3.1415926535;   % Precise PI 

%% 2. OPTIMIZED ROCKET HARDWARE PARAMETERS (From Python Output)
m_payload = 10000.0;        % Fixed payload mass (kg) 
m_s       = 9456.3;         % Structure mass (kg) 
m_p_init  = 22449.3;        % Propellant mass (kg) 
D_rocket  = 2.707;          % Rocket diameter (m) 
L_rocket  = 35.36;          % Hardcoded reference length from Python script
A_ref     = PI * (D_rocket/2)^2; % Frontal cross-section 
N_noz     = 3;              % Number of nozzles 

% High-Performance Engine Thermodynamics
T_s0      = 3492;           % Stagnation temperature (K) 
R_gas     = 636.52;         % Exhaust gas constant (J/kg*K) 
D_noz     = 0.418;          % Nozzle exit diameter (m) 
A_e       = PI * (D_noz/2)^2; % Nozzle exit area 
P_s0      = 7800000;        % Chamber stagnation pressure (Pa) 

%% 3. ISENTROPIC NOZZLE SOLVER (Python Logic)
M_e = sqrt( ( ((P_s0/P_e)^((gamma_gas-1)/gamma_gas)) - 1 ) * (2/(gamma_gas-1)) ); 
T_e = T_s0 / (1 + ((gamma_gas-1)/2) * M_e^2); 
rho_s = P_s0 / (R_gas * T_s0); 
rho_e = rho_s * (1 / (1 + ((gamma_gas-1)/2) * M_e^2))^(1/(gamma_gas-1)); 
V_e = M_e * sqrt(gamma_gas * R_gas * T_e); 
m_dot_total = rho_e * V_e * A_e * N_noz; 

%% 4. SIMULATION SETUP & PREALLOCATION
dt    = 0.01;               % Fixed RK4 time-step (s) for CSV high-resolution
t_max = 300.0;              % Target time (s) 
time  = 0:dt:t_max;
N     = length(time);

% Preallocate Telemetry Arrays
h_arr = zeros(1, N);        v_arr = zeros(1, N);        
a_arr = zeros(1, N);        T_arr = zeros(1, N);        
D_arr = zeros(1, N);        F_in  = zeros(1, N);        
m_arr = zeros(1, N);

% Initial States
h = 0; v = 0; 
m_prop = m_p_init; 

%% 5. RK4 KINEMATIC INTEGRATOR 
for i = 1:N
    t = time(i);
    m_curr = m_s + m_payload + m_prop;
    
    % Python Hard Cutoff logic: Only use mass flow if enough propellant exists for the step
    if m_prop >= m_dot_total * dt && m_dot_total > 0
        active_mdot = m_dot_total;
    else
        active_mdot = 0;
    end
    
    [accel, thrust, drag] = flight_dynamics(h, v, m_curr, active_mdot, V_e, P_e, A_e, N_noz, A_ref, L_rocket);
    
    if h <= 0 && accel < 0 && t < 1.0
        accel = 0; v = 0;
    end
    
    h_arr(i) = h;
    v_arr(i) = v;
    a_arr(i) = accel;
    T_arr(i) = thrust;
    D_arr(i) = drag;
    F_in(i)  = m_curr * accel; 
    m_arr(i) = m_curr;
    
    % --- RK4 Sub-steps ---
    [a1, ~, ~] = flight_dynamics(h, v, m_curr, active_mdot, V_e, P_e, A_e, N_noz, A_ref, L_rocket);
    k1_v = a1;  k1_h = v;
    
    m_mid = m_curr - (active_mdot * dt/2); 
    v2 = v + 0.5 * dt * k1_v; h2 = h + 0.5 * dt * k1_h;
    [a2, ~, ~] = flight_dynamics(h2, v2, m_mid, active_mdot, V_e, P_e, A_e, N_noz, A_ref, L_rocket);
    k2_v = a2;  k2_h = v2;
    
    v3 = v + 0.5 * dt * k2_v; h3 = h + 0.5 * dt * k2_h;
    [a3, ~, ~] = flight_dynamics(h3, v3, m_mid, active_mdot, V_e, P_e, A_e, N_noz, A_ref, L_rocket);
    k3_v = a3;  k3_h = v3;
    
    m_full = m_curr - (active_mdot * dt); 
    v4 = v + dt * k3_v; h4 = h + dt * k3_h;
    [a4, ~, ~] = flight_dynamics(h4, v4, m_full, active_mdot, V_e, P_e, A_e, N_noz, A_ref, L_rocket);
    k4_v = a4;  k4_h = v4;
    
    % Advance State 
    v = v + (dt/6) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
    h = h + (dt/6) * (k1_h + 2*k2_h + 2*k3_h + k4_h);
    
    m_prop = m_prop - active_mdot * dt;
    
    if h < 0 && t > 10.0
        stop_idx = i; break;
    end
    stop_idx = i;
end

%% 6. DATA TRIMMING
time = time(1:stop_idx);   h_arr = h_arr(1:stop_idx);
v_arr = v_arr(1:stop_idx); a_arr = a_arr(1:stop_idx);
T_arr = T_arr(1:stop_idx); D_arr = D_arr(1:stop_idx);
F_in  = F_in(1:stop_idx);  m_arr = m_arr(1:stop_idx);

%% 7. CONSOLE DASHBOARD (10-Second Intervals)
idx_samples_console = 1:round(10/dt):length(time); 
disp('=============================================================================================================');
disp('                                   ROCKET FLIGHT TELEMETRY LOG (10s Summaries)                               ');
disp('=============================================================================================================');
Console_Table = table(round(time(idx_samples_console)',1), round(h_arr(idx_samples_console)',1), ...
    round(T_arr(idx_samples_console)',1), round(D_arr(idx_samples_console)',1), ...
    round(F_in(idx_samples_console)',1), round(a_arr(idx_samples_console)',2), round(v_arr(idx_samples_console)',1), ...
    'VariableNames', {'Time_s', 'Height_m', 'Thrust_N', 'Drag_N', 'Inertial_Force_N', 'Accel_ms2', 'Velocity_ms'});
disp(Console_Table);
disp('=============================================================================================================');

%% 8. SIX-PANEL DASHBOARD PLOTTING
figure('Name', 'Ascent Trajectory Dynamics', 'Color', 'w', 'Position', [100, 50, 1400, 900]);
subplot(2,3,1); plot(time, h_arr./1000, 'k', 'LineWidth', 2);
title('1. Geometric Altitude', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Altitude (km)'); grid on;
subplot(2,3,2); plot(time, T_arr./1000, 'r', 'LineWidth', 2);
title('2. Engine Thrust', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Thrust (kN)'); grid on;
subplot(2,3,3); plot(time, D_arr./1000, 'b', 'LineWidth', 2);
title('3. Aerodynamic Drag', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Drag Force (kN)'); grid on;
subplot(2,3,4); plot(time, F_in./1000, 'm', 'LineWidth', 2);
title('4. Inertial Force (m \cdot a)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Force (kN)'); grid on;
subplot(2,3,5); plot(time, a_arr, 'g', 'LineWidth', 2);
title('5. Net Acceleration', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); grid on;
subplot(2,3,6); plot(time, v_arr, 'c', 'LineWidth', 2); hold on;
yline(0, 'k--', 'LineWidth', 1); 
title('6. Vertical Velocity', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;

%% 9. HIGH-RESOLUTION CSV EXPORT SCRIPT (0.01s Intervals)
Full_Telemetry_Table = table(round(time',2), round(h_arr',1), ...
    round(T_arr',1), round(D_arr',1), ...
    round(F_in',1), round(a_arr',2), round(v_arr',1), ...
    'VariableNames', {'Time_s', 'Height_m', 'Thrust_N', 'Drag_N', 'Inertial_Force_N', 'Accel_ms2', 'Velocity_ms'});

csv_filename = 'Rocket_Telemetry_Data_0.01s.csv';
writetable(Full_Telemetry_Table, csv_filename);
disp(['SUCCESS: High-resolution telemetry (0.01s steps) exported to -> ', pwd, filesep, csv_filename]);

%% ================= LOCAL PHYSICS FUNCTIONS ================= %%
function [accel, thrust, drag] = flight_dynamics(h, v, m, active_mdot, V_e, P_e, A_e, N_noz, A_ref, L)
    g0 = 9.80665; R_Earth = 6356766.0; R_air = 287.053; gamma_air = 1.4;
    
    H_geo = (R_Earth * h) / (R_Earth + h);
    g_loc = g0 * (R_Earth / (R_Earth + h))^2; 
    
    % Temperature (Exact Python Mapping)
    if H_geo <= 11000
        T_atm = 288.15 - 0.0065 * H_geo; 
    elseif H_geo <= 20000
        T_atm = 216.65; 
    elseif H_geo <= 32000
        T_atm = 216.65 + 0.001 * (H_geo - 20000);
    elseif H_geo <= 47000
        T_atm = 228.65 + 0.0028 * (H_geo - 32000);
    elseif H_geo <= 51000
        T_atm = 270.65;
    elseif H_geo <= 71000
        T_atm = 270.65 - 0.0028 * (H_geo - 51000);
    elseif H_geo <= 84852
        T_atm = 214.65 - 0.002 * (H_geo - 71000);
    else
        if h <= 91000
            T_atm = 186.8673;
        elseif h <= 110000
            delta = (h - 91000) / -19942.9;
            T_atm = 263.1905 - 76.3232 * sqrt(1 - delta^2); 
        elseif h <= 120000
            T_atm = 240 + 0.012 * (h - 110000);
        else
            exp_arg = -0.00001875 * (h - 120000) * (R_Earth + 120000) / (R_Earth + h);
            T_atm = 1000 - 640 * exp(exp_arg);
        end
    end
    
    % Pressure (Exact Python Specifics)
    if H_geo <= 11000
        P_atm = 101325.0 * (288.15 / (288.15 - 0.0065 * H_geo)) ^ (g0 / (R_air * -0.0065));
    elseif H_geo <= 20000
        P_atm = 22632.0554587517 * exp(-g0 * (H_geo - 11000) / (R_air * 216.65));
    elseif H_geo <= 32000
        P_atm = 5474.88465973091 * (216.65 / (216.65 + 0.001 * (H_geo - 20000))) ^ (g0 / (R_air * 0.001));
    elseif H_geo <= 47000
        P_atm = 868.017647755643 * (228.65 / (228.65 + 0.0028 * (H_geo - 32000))) ^ (g0 / (R_air * 0.0028));
    elseif H_geo <= 51000
        P_atm = 110.906115784329 * exp(-g0 * (H_geo - 47000) / (R_air * 270.65));
    elseif H_geo <= 71000
        P_atm = 66.9387500974118 * (270.65 / (270.65 - 0.0028 * (H_geo - 51000))) ^ (g0 / (R_air * -0.0028));
    elseif H_geo <= 84852
        P_atm = 3.95641034818858 * (214.65 / (214.65 - 0.002 * (H_geo - 71000))) ^ (g0 / (R_air * -0.002));
    else
        P_atm = 0.0;
    end
    
    rho = P_atm / (R_air * T_atm); 
    a_sound = sqrt(gamma_air * R_air * T_atm); 
    
    % Thrust 
    if active_mdot > 0
        thrust = (active_mdot * V_e) + (P_e - P_atm) * A_e * N_noz; 
    else
        thrust = 0; 
    end
    
    % Aerodynamics 
    v_abs = abs(v);
    M_ext = v_abs / a_sound; 
    
    dynamic_viscosity = 1.716e-5 * (T_atm / 273.15)^1.5 * (273.15 + 110.4) / (T_atm + 110.4); 
    reynolds_number = rho * v_abs * L / dynamic_viscosity; 
    
    if reynolds_number == 0
        C_f = 0; 
    elseif reynolds_number > 5e5
        C_f = 0.031 / (reynolds_number^(1/7)); 
    else
        C_f = 1.328 / sqrt(reynolds_number); 
    end
    
    if M_ext == 0
        Cd_max = 1;
    elseif M_ext < 1
        term = (1 + ((gamma_air-1)/2)*M_ext^2)^(gamma_air/(gamma_air-1));
        Cd_max = (2 / (gamma_air * M_ext^2)) * (term - 1);
    else
        term1 = ((gamma_air + 1) / 2 * M_ext^2) ^ (gamma_air / (gamma_air - 1));
        term2 = ((gamma_air + 1) / (2 * gamma_air * M_ext^2 - (gamma_air - 1))) ^ (1 / (gamma_air - 1));
        Cd_max = (2 / (gamma_air * M_ext^2)) * (term1 * term2 - 1);
    end
    
    C_d_tot = C_f + (Cd_max * sin(15 * pi / 180)^2); 
    drag = 0.5 * rho * v_abs^2 * C_d_tot * A_ref; 
    drag = sign(v) * drag; 
    
    accel = (thrust - drag - (m * g_loc)) / m;
end
