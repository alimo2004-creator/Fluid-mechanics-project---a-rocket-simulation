%==========================================================================
% COLD-FLOW NOZZLE TEST TRANSIENT SIMULATION
% Suborbital Rocket Project | Dynamic Similarity Analysis
% Senior Aerospace Flight Dynamics Engineer | MATLAB Numerical Solver
%==========================================================================
% Physics: 1D Adiabatic Tank Blowdown through Convergent-Divergent Nozzle
% Fluid: Compressed Air (Ideal Gas, k=1.4, R=287.05 J/kg·K)
%==========================================================================
clear all; close all; clc;

%==========================================================================
% SECTION 1: THERMODYNAMIC CONSTANTS & INITIAL CONDITIONS
%==========================================================================
k_cold = 1.4;           % Specific heat ratio (cold air)
k_hot = 1.2;            % Specific heat ratio (hot gas design reference)
R = 287.05;             % Gas constant (air) [J/kg·K]
P_atm = 101325;         % Atmospheric pressure [Pa]
V_tank = 2.0;           % Tank volume [m^3]
P0_init = 10e6;         % Initial tank stagnation pressure [Pa]
T0_init = 300;          % Initial tank stagnation temperature [K]
A_throat = 0.01;        % Nozzle throat area [m^2]
M_e_target = 3.0;       % Target exit Mach number for dynamic similarity

%==========================================================================
% SECTION 2: DYNAMIC SIMILARITY - AREA-MACH RELATION
%==========================================================================
% For hot gas design (k=1.2, M_e=3.0), calculate reference A_e/A*_hot
M = M_e_target;
term_hot = (2/(k_hot+1)) * (1 + ((k_hot-1)/2)*M^2);
AeAstar_hot = (1/M) * (term_hot)^((k_hot+1)/(2*(k_hot-1)));

% For cold air (k=1.4, M_e=3.0), calculate required A_e/A*_cold
term_cold = (2/(k_cold+1)) * (1 + ((k_cold-1)/2)*M^2);
AeAstar_cold = (1/M) * (term_cold)^((k_cold+1)/(2*(k_cold-1)));

% Exit area required for dynamic similarity
A_exit = AeAstar_cold * A_throat;

%==========================================================================
% SECTION 3: TIME INTEGRATION SETUP
%==========================================================================
dt = 0.01;              % Time step [s]
t_max = 30;             % Maximum simulation time [s]
P_unchoke = 2*P_atm;    % Pressure threshold for nozzle unchoke [Pa]

% Initialize arrays for storage
time_vec = [];
P0_vec = [];
mdot_vec = [];
Thrust_vec = [];
m_tank_vec = [];

% Initial state
P0_current = P0_init;
T0_current = T0_init;
m_tank_current = (P0_init * V_tank) / (R * T0_init);
t_current = 0;

%==========================================================================
% SECTION 4: TRANSIENT SIMULATION LOOP (EULER METHOD)
%==========================================================================
iteration = 0;
max_iterations = ceil(t_max / dt);

while t_current <= t_max && P0_current >= P_unchoke && iteration < max_iterations
    iteration = iteration + 1;
    
    % ===== CHOKED MASS FLOW RATE (Nozzle remains choked) =====
    % ṁ = A* * P0 / sqrt(T0) * sqrt(k/R) * (2/(k+1))^((k+1)/(2(k-1)))
    choke_factor = sqrt(k_cold/R) * (2/(k_cold+1))^((k_cold+1)/(2*(k_cold-1)));
    mdot = A_throat * (P0_current / sqrt(T0_current)) * choke_factor;
    
    % ===== ISENTROPIC STAGNATION TEMPERATURE TO STATIC TEMPERATURE =====
    % Temperature ratio for choked flow (M=1 at throat)
    T_throat = T0_current / (1 + (k_cold-1)/2 * 1^2);
    
    % Exit temperature via isentropic relation
    % T_e / T0 = 1 / (1 + (k-1)/2 * M_e^2)
    T_exit = T0_current / (1 + (k_cold-1)/2 * M_e_target^2);
    
    % ===== EXIT VELOCITY =====
    % V_e = M_e * sqrt(k * R * T_e)
    V_exit = M_e_target * sqrt(k_cold * R * T_exit);
    
    % ===== EXIT STATIC PRESSURE (Isentropic) =====
    % P_e / P0 = (1 + (k-1)/2 * M_e^2)^(-k/(k-1))
    P_exit = P0_current * (1 + (k_cold-1)/2 * M_e_target^2)^(-k_cold/(k_cold-1));
    
    % ===== THRUST CALCULATION =====
    % F = ṁ * V_e + (P_e - P_atm) * A_e
    F_momentum = mdot * V_exit;
    F_pressure = (P_exit - P_atm) * A_exit;
    F_total = F_momentum + F_pressure;
    
    % ===== ADIABATIC TANK BLOWDOWN EQUATIONS =====
    % dP0/dt = -(k * R * T0 / V_tank) * ṁ
    dP0_dt = -(k_cold * R * T0_current / V_tank) * mdot;
    
    % dT0/dt = -(k-1) * T0 * (ṁ / m_tank)
    dT0_dt = -(k_cold-1) * T0_current * (mdot / m_tank_current);
    
    % ===== TANK MASS EVOLUTION =====
    dm_dt = -mdot;
    
    % ===== EULER STEP =====
    P0_next = P0_current + dP0_dt * dt;
    T0_next = T0_current + dT0_dt * dt;
    m_tank_next = m_tank_current + dm_dt * dt;
    
    % ===== STATE UPDATE =====
    P0_current = P0_next;
    T0_current = T0_next;
    m_tank_current = m_tank_next;
    t_current = t_current + dt;
    
    % ===== STORAGE =====
    time_vec = [time_vec; t_current];
    P0_vec = [P0_vec; P0_current];
    mdot_vec = [mdot_vec; mdot];
    Thrust_vec = [Thrust_vec; F_total];
    m_tank_vec = [m_tank_vec; m_tank_current];
end

%==========================================================================
% SECTION 5: PROFESSIONAL VISUALIZATION (4-PANEL SUBPLOT)
%==========================================================================
figure('Name', 'Cold-Flow Nozzle Transient Dynamics', ...
       'NumberTitle', 'off', ...
       'Position', [100 100 1400 900], ...
       'Color', [0.98 0.98 0.98]);

% Define color scheme
color_p0 = [0.1 0.3 0.6];       % Deep blue
color_mdot = [0.8 0.2 0.1];     % Deep red
color_thrust = [0.2 0.7 0.3];   % Forest green
color_mass = [0.6 0.4 0.1];     % Brown

%----------- PANEL 1: STAGNATION PRESSURE vs TIME -----------
subplot(2,2,1);
plot(time_vec, P0_vec/1e6, 'Color', color_p0, 'LineWidth', 2.5);
hold on;
yline(P_atm/1e6, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'Label', '1 atm');
yline(P_unchoke/1e6, '-.', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5, 'Label', '2 atm (Unchoke)');
grid on; grid minor;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Stagnation Pressure P_0 (MPa)', 'FontSize', 11, 'FontWeight', 'bold');
title('Tank Stagnation Pressure Transient', 'FontSize', 12, 'FontWeight', 'bold');
legend('P_0(t)', 'Atmospheric', 'Unchoke Threshold', 'Location', 'northeast', ...
       'FontSize', 9, 'FontWeight', 'bold');
xlim([0 max(time_vec)]);
ylim([0 max(P0_vec/1e6)*1.1]);

%----------- PANEL 2: MASS FLOW RATE vs TIME -----------
subplot(2,2,2);
plot(time_vec, mdot_vec, 'Color', color_mdot, 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Mass Flow Rate $\dot{m}$ (kg/s)', 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'latex');
title('Choked Mass Flow Rate Through Nozzle', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time_vec)]);
ylim([0 max(mdot_vec)*1.1]);

%----------- PANEL 3: THRUST vs TIME -----------
subplot(2,2,3);
plot(time_vec, Thrust_vec/1000, 'Color', color_thrust, 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Instantaneous Thrust F (kN)', 'FontSize', 11, 'FontWeight', 'bold');
title('Thrust Profile (Momentum + Pressure Terms)', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time_vec)]);
ylim([0 max(Thrust_vec/1000)*1.1]);

%----------- PANEL 4: TANK MASS vs TIME -----------
subplot(2,2,4);
plot(time_vec, m_tank_vec, 'Color', color_mass, 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tank Mass m_{tank} (kg)', 'FontSize', 11, 'FontWeight', 'bold');
title('Tank Mass Depletion During Blowdown', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time_vec)]);
ylim([0 max(m_tank_vec)*1.1]);

% Overall title
sgtitle('COLD-FLOW NOZZLE TEST | Transient Adiabatic Tank Blowdown Simulation', ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

%==========================================================================
% SECTION 6: CONSOLE OUTPUT & DYNAMIC SIMILARITY SUMMARY
%==========================================================================
fprintf('\n%s\n', repmat('=', 1, 90));
fprintf('COLD-FLOW NOZZLE TEST | DYNAMIC SIMILARITY ANALYSIS\n');
fprintf('%s\n\n', repmat('=', 1, 90));

fprintf('DESIGN REFERENCE (HOT GAS):\n');
fprintf('  - Specific heat ratio (γ_hot)      : %.2f\n', k_hot);
fprintf('  - Target exit Mach number (M_e)   : %.2f\n', M_e_target);
fprintf('  - Area-Mach ratio (A_e/A*_hot)    : %.6f\n\n', AeAstar_hot);

fprintf('COLD-FLOW TEST (DYNAMIC SIMILARITY):\n');
fprintf('  - Specific heat ratio (γ_cold)    : %.2f\n', k_cold);
fprintf('  - Gas constant (R)                 : %.2f J/(kg·K)\n', R);
fprintf('  - Target exit Mach number (M_e)   : %.2f\n', M_e_target);
fprintf('  - Throat area (A*)                 : %.5f m²\n', A_throat);
fprintf('  - REQUIRED exit area (A_e)         : %.6f m²\n', A_exit);
fprintf('  - REQUIRED area ratio (A_e/A*)     : %.6f\n\n', AeAstar_cold);

fprintf('EXPANSION RATIO CORRECTION:\n');
fprintf('  - Hot gas design ratio             : %.6f\n', AeAstar_hot);
fprintf('  - Cold air required ratio          : %.6f\n', AeAstar_cold);
fprintf('  - Ratio change factor              : %.6f\n\n', AeAstar_cold / AeAstar_hot);

fprintf('INITIAL CONDITIONS:\n');
fprintf('  - Tank volume (V)                  : %.2f m³\n', V_tank);
fprintf('  - Initial stagnation pressure (P₀): %.2e Pa (%.2f MPa)\n', P0_init, P0_init/1e6);
fprintf('  - Initial stagnation temperature   : %.2f K\n', T0_init);
fprintf('  - Initial tank mass                : %.4f kg\n', m_tank_current);
fprintf('  - Atmospheric pressure (P_atm)    : %.2e Pa (%.4f atm)\n\n', P_atm, P_atm/101325);

fprintf('NUMERICAL INTEGRATION:\n');
fprintf('  - Time step (dt)                   : %.4f s\n', dt);
fprintf('  - Simulation duration              : %.2f s\n', t_current);
fprintf('  - Total iterations executed        : %d\n', iteration);
fprintf('  - Final tank pressure              : %.2e Pa (%.4f atm)\n', P0_current, P0_current/P_atm);
fprintf('  - Final tank temperature           : %.2f K\n', T0_current);
fprintf('  - Final tank mass                  : %.4f kg\n', m_tank_current);
fprintf('  - Mass expelled from tank          : %.4f kg (%.2f%% of initial)\n\n', ...
        m_tank_vec(1) - m_tank_current, ...
        100*(m_tank_vec(1) - m_tank_current)/m_tank_vec(1));

fprintf('PEAK PERFORMANCE METRICS:\n');
fprintf('  - Maximum mass flow rate           : %.4f kg/s\n', max(mdot_vec));
fprintf('  - Maximum thrust                   : %.2f N (%.2f kN)\n', max(Thrust_vec), max(Thrust_vec)/1000);
fprintf('  - Total impulse (numerical)        : %.2e N·s\n\n', trapz(time_vec, Thrust_vec));

fprintf('%s\n', repmat('=', 1, 90));
fprintf('SIMULATION COMPLETE | Data exported to workspace\n');
fprintf('%s\n\n', repmat('=', 1, 90));

%==========================================================================
% SECTION 7: DATA EXPORT TO WORKSPACE
%==========================================================================
assignin('base', 'time_data', time_vec);
assignin('base', 'pressure_data', P0_vec);
assignin('base', 'mdot_data', mdot_vec);
assignin('base', 'thrust_data', Thrust_vec);
assignin('base', 'mass_data', m_tank_vec);
assignin('base', 'A_exit_required', A_exit);
assignin('base', 'AeAstar_cold', AeAstar_cold);

%==========================================================================
