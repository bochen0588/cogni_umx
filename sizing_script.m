% =========================================================================
% Constraint Diagram for a Small Drone Aircraft
% Plots Thrust-to-Weight (T/W) vs. Wing Loading (W/S)
% =========================================================================
clear; clc; close all;

%% 1. Parameterized Variables
% Environmental Conditions
rho = 1.225;            % Air density at sea level (kg/m^3)
g = 9.81;               % Gravity (m/s^2)

% Aerodynamic Parameters
CD0 = 0.03;             % Zero-lift drag coefficient
e = 0.8;                % Oswald efficiency factor
AR = 8.0;               % Aspect ratio
CL_max = 1.3;           % Maximum lift coefficient (flaps down if applicable)

% Performance Requirements (Drone Scale)
V_stall = 12;           % Maximum allowable stall speed (m/s)
V_max = 30;             % Target maximum cruise speed (m/s)
ROC = 6;                % Desired rate of climb (m/s)
V_climb = 15;           % Forward speed during climb (m/s)
S_TO = 15;              % Maximum allowable takeoff distance (m)

%% 2. Wing Loading (W/S) Array
% Define the domain for Wing Loading (N/m^2)
WS = linspace(10, 150, 300); % W/S from 10 to 150 N/m^2

%% 3. Constraint Equations

% A. Stall Speed Constraint (Vertical Line)
% W/S <= 0.5 * rho * V_stall^2 * CL_max
WS_max_stall = 0.5 * rho * V_stall^2 * CL_max;

% B. Maximum Speed Constraint
% T/W >= q_max * CD0 / (W/S) + (W/S) / (q_max * pi * e * AR)
q_max = 0.5 * rho * V_max^2;
TW_maxspeed = (q_max * CD0) ./ WS + WS ./ (q_max * pi * e * AR);

% C. Rate of Climb Constraint
% T/W >= ROC/V_climb + q_climb * CD0 / (W/S) + (W/S) / (q_climb * pi * e * AR)
q_climb = 0.5 * rho * V_climb^2;
TW_roc = (ROC / V_climb) + ((q_climb * CD0) ./ WS) + (WS ./ (q_climb * pi * e * AR));

% D. Takeoff Distance Constraint (Simplified Ground Roll)
% T/W >= 1.21 * (W/S) / (g * rho * CL_max * S_TO)
TW_takeoff = (1.21 * WS) / (g * rho * CL_max * S_TO);

%% 4. Define the Feasible Design Space
% The design must meet the highest T/W requirement among all constraints
TW_required = max([TW_maxspeed; TW_roc; TW_takeoff]);

% Filter out the region that violates the stall constraint
valid_idx = find(WS <= WS_max_stall);
WS_valid = WS(valid_idx);
TW_valid = TW_required(valid_idx);

% Create polygon for shading (up to a max T/W of 1.5 for the plot boundary)
TW_plot_ceiling = 1.5; 
fill_x = [WS_valid, fliplr(WS_valid)];
fill_y = [TW_valid, TW_plot_ceiling * ones(1, length(TW_valid))];

%% 5. Plotting
figure('Name', 'Drone Constraint Diagram', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on;

% Shade the feasible design space
fill(fill_x, fill_y, [0.8 0.95 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Plot the constraint curves
p1 = plot(WS, TW_maxspeed, 'b-', 'LineWidth', 2);
p2 = plot(WS, TW_roc, 'r--', 'LineWidth', 2);
p3 = plot(WS, TW_takeoff, 'k-.', 'LineWidth', 2);
p4 = xline(WS_max_stall, 'm-', 'LineWidth', 2);

% Formatting the plot
axis([10 150 0 TW_plot_ceiling]);
xlabel('Wing Loading, W/S (N/m^2)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Thrust-to-Weight Ratio, T/W', 'FontSize', 12, 'FontWeight', 'bold');
title('Design Space Constraint Diagram for Small Drone', 'FontSize', 14);

% Add optimal design point marker (lowest T/W and highest W/S in feasible space)
opt_WS = WS_max_stall;
opt_TW = interp1(WS_valid, TW_valid, opt_WS);
plot(opt_WS, opt_TW, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Add Legends
legend([p1, p2, p3, p4], ...
    sprintf('Max Speed (V_{max} = %d m/s)', V_max), ...
    sprintf('Rate of Climb (ROC = %d m/s)', ROC), ...
    sprintf('Takeoff Distance (S_{TO} = %d m)', S_TO), ...
    sprintf('Stall Speed Limit (V_{stall} = %d m/s)', V_stall), ...
    'Location', 'northwest', 'FontSize', 11);

% Add text for Design Point
text(opt_WS - 5, opt_TW + 0.05, 'Optimal Design Point', ...
    'HorizontalAlignment', 'right', 'FontSize', 10, 'FontWeight', 'bold');

hold off;