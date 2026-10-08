% =========================================================================
% Constraint Diagram for (MAV)
% Plots Thrust-to-Weight (T/W) vs. Wing Loading (W/S)
% =========================================================================
clear; clc; close all;

%% 1. Parameterized Variables (Micro Drone Scale)
% Environmental Conditions
rho = 1.225;            % Air density at sea level (kg/m^3)
g = 9.81;               % Gravity (m/s^2)

% Aerodynamic Parameters (Low Reynolds Number Regime)
CD0 = 0.045;            % Zero-lift drag coefficient (typically higher for MAVs)
e = 0.75;               % Oswald efficiency factor
AR = 4.52;               % Aspect ratio
CL_max = 1.1;           % Maximum lift coefficient (MAV airfoils have lower CL_max)
alpha_stall = 15;       % Estimated angle of attack at stall (degrees)

% Performance Requirements (60g Prototype Scale)
V_stall = 2.0;          % Target stall/minimum speed (m/s)
V_max = 6.0;           % Target maximum cruise speed (m/s)
ROC = 1.0;              % Desired rate of climb (m/s)
V_climb = 3.0;          % Forward speed during climb (m/s)
S_TO = 3.0;             % Maximum allowable takeoff distance (m)

% Takeoff Specific Parameters
mu = 0.05;              % Rolling friction coefficient (grass/rough surface)
CL_TO = 0.8;            % Lift coefficient during ground roll
CD_TO = CD0 + (CL_TO^2)/(pi * e * AR); % Drag coefficient during ground roll

%% 2. Wing Loading (W/S) Array
% Adjusted range for a 60g drone with a prototype at W/S = 8
WS = linspace(1, 16, 500); % W/S from 2 to 16 N/m^2

%% Current prototype iteration (W/S = 8, T/W = 0.7)
proto_WS = 6.768;
proto_TW = 0.9;

%% 3. Detailed Constraint Equations (Fewer Assumptions)
% A. Stall Speed Constraint (Thrust-Lift Coupled)
% Equation: L + T*sin(alpha) = W  => T/W = (1 - L/W) / sin(alpha)
% At stall, if W/S is too high for pure aerodynamic lift, thrust must make up the difference.
q_stall = 0.5 * rho * V_stall^2;
TW_stall = (1 ./ sind(alpha_stall)) .* (1 - (q_stall * CL_max) ./ WS);
TW_stall(TW_stall < 0) = 0; % Cap at 0 (if wings generate enough lift, no thrust needed to stay aloft)

% B. Maximum Speed Constraint
q_max = 0.5 * rho * V_max^2;
TW_maxspeed = (q_max * CD0) ./ WS + WS ./ (q_max * pi * e * AR);

% C. Rate of Climb Constraint (Exact Flight Path Angle)
% Removing the small angle assumption (cos(gamma) = 1)
gamma = asin(ROC / V_climb); % Flight path angle in radians
q_climb = 0.5 * rho * V_climb^2;
TW_roc = (q_climb * CD0) ./ WS + (WS .* cos(gamma)^2) ./ (q_climb * pi * e * AR) + sin(gamma);

% D. Takeoff Distance Constraint (Detailed Ground Roll)
% Accounts for average acceleration, rolling resistance (mu), and drag
TW_takeoff = mu + (0.72 * (CD_TO - mu * CL_TO) / CL_max) + (1.44 * WS) ./ (g * rho * CL_max * S_TO);

%% 4. Define the Feasible Design Space
% The design must meet the highest T/W requirement among ALL constraints
TW_required = max([TW_stall; TW_maxspeed; TW_roc; TW_takeoff]);

% Create polygon for shading
TW_plot_ceiling = 1.5; % Max T/W on plot
fill_x = [WS, fliplr(WS)];
fill_y = [TW_required, TW_plot_ceiling * ones(1, length(TW_required))];

%% 5. Plotting
figure('Name', 'MAV Constraint Diagram', 'Color', 'w', 'Position', [100, 100, 850, 650]);
hold on; grid on;

% Shade the feasible design space
fill(fill_x, fill_y, [0.8 0.95 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Plot the constraint curves
p1 = plot(WS, TW_maxspeed, 'b-', 'LineWidth', 2);
p2 = plot(WS, TW_roc, 'r--', 'LineWidth', 2);
p3 = plot(WS, TW_takeoff, 'k-.', 'LineWidth', 2);
p4 = plot(WS, TW_stall, 'm-', 'LineWidth', 2);

% Plot current prototype iteration (W/S = 8, T/W = 0.7)
p_proto = plot(proto_WS, proto_TW, 'p', 'MarkerSize', 14, 'MarkerFaceColor', '#D95319', 'MarkerEdgeColor', 'k');

% Formatting the plot
axis([1 16 0 TW_plot_ceiling]);
xlabel('Wing Loading, W/S (N/m^2)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Thrust-to-Weight Ratio, T/W', 'FontSize', 12, 'FontWeight', 'bold');
title('Constraint Diagram', 'FontSize', 14);

% Add Legends
legend([p1, p2, p3, p4, p_proto], ...
    sprintf('Max Speed (V_{max} = %.1f m/s)', V_max), ...
    sprintf('ROC (ROC = %.1f m/s, \\gamma = %.1f^\\circ)', ROC, rad2deg(gamma)), ...
    sprintf('Takeoff (S_{TO} = %.1f m, \\mu = %.2f)', S_TO, mu), ...
    sprintf('Stall (V = %.1f m/s)', V_stall), ...
    sprintf('Prototype (W/S = %.1f, T/W = %.2f)', proto_WS, proto_TW), ...
    'Location', 'northeast', 'FontSize', 11);

% Annotate the prototype point
text(proto_WS + 0.3, proto_TW, 'Current Prototype', ...
    'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold');

hold off;