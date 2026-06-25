
%%The Core Workspace Variables
% --- DELTA IV SIMULINK INITIALIZATION SCRIPT ---

% 1. Environment & Planet Constraints
g_0     = 9.81;        % Standard gravity (m/s^2)
R_E     = 6.371e6;     % Earth radius (m)
rho_0   = 1.225;       % Sea-level density (kg/m^3)
H_scale = 8500;        % Atmospheric scale height (m)
gamma_air = 1.4;       % Specific heat ratio (air)
R_air     = 287;       % Gas constant (air)

% 2. Vehicle Hardware (Fixed Masses)
m_d  = 10000;          % Payload mass (kg)
m_s1 = 26000;          % Stage 1 dry mass (kg)
m_s2 = 2850;           % Stage 2 dry mass (kg)
A_ref = pi * (4.0^2) / 4; % Frontal area (m^2)

% 3. Engine Specifications
F_thrust1 = 3116000;   % S1 Thrust (N)
mdot1     = F_thrust1 / (365 * 9.81); % S1 Mass Flow (kg/s)
F_thrust2 = 110000;    % S2 Thrust (N)
mdot2     = F_thrust2 / (462 * 9.81); % S2 Mass Flow (kg/s)

% 4. Initial Mass States (Assuming optimal fuel split)
% Note: You will replace these with the exact outputs of your optimization loop
m_p1_init = 100000;    % Placeholder: S1 Propellant (kg) 
m_p2_init = 18000;     % Placeholder: S2 Propellant (kg)
m0 = m_d + m_s1 + m_s2 + m_p1_init + m_p2_init; % Liftoff mass

%%The Aerodynamics Lookup Table

% 5. Aerodynamic Arrays for Simulink Lookup Table Block
Mach_breaks = [0, 0.3, 0.6, 0.8, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2, 1.4, 1.6, 2.0, 2.5, 3.0, 4.0, 5.0, 7.0, 10.0];
Cd_breaks   = [0.18, 0.19, 0.22, 0.28, 0.35, 0.42, 0.60, 0.68, 0.72, 0.70, 0.62, 0.55, 0.44, 0.38, 0.33, 0.28, 0.25, 0.22, 0.20];

%%The Environment Function

function [a, rho] = atmosphere_model(h, gamma, R, rho_0, H_scale)
    % Temperature logic
    if     h < 11000, T=288.15-0.0065*h;
    elseif h < 20000, T=216.65;
    elseif h < 32000, T=216.65+0.001*(h-20000);
    elseif h < 47000, T=228.65+0.0028*(h-32000);
    elseif h < 51000, T=270.65;
    elseif h < 71000, T=270.65-0.0028*(h-51000);
    else,             T=214.65;
    end
    
    a = sqrt(gamma * R * T);
    rho = rho_0 * exp(-h / H_scale);
end

% Delta IV Animated Trajectory Visualizer
figure('Name', 'Delta IV Orbital Insertion', 'Color', 'k');
hold on; grid on;

% Format the dark-mode space background
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', '#333333');
title('Delta IV Flight Path', 'Color', 'w', 'FontSize', 14);
xlabel('Downrange Distance (m)');
ylabel('Altitude (m)');

% Extract the array data from Simulink output
X = out.X_data; 
Z = out.Z_data;

% Set axis limits slightly larger than your max values
axis([0 max(X)*1.1 0 max(Z)*1.1]);

% Create the animated objects (The trail and the rocket)
trail = plot(X(1), Z(1), 'w-', 'LineWidth', 2); % White smoke trail
rocket = plot(X(1), Z(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % Red rocket

% Animation Loop (Skips frames to play at a reasonable speed)
for i = 1:50:length(X)
    % Update the trail line to the current point
    set(trail, 'XData', X(1:i), 'YData', Z(1:i));
    
    % Move the rocket dot to the current point
    set(rocket, 'XData', X(i), 'YData', Z(i));
    
    % Pause briefly to create the animation effect
    pause(0.01);
end