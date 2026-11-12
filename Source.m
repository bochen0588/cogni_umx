% =========================================================================
% MAV Constraint Diagram + Sizing
%   1. Parameters            (everything you would edit lives here)
%   2. Constraint diagram calculations
%   3. Constraint diagram plot
%   4. Sizing                (mass / wing-area iteration, results, sensitivity)
% =========================================================================
clear; clc; close all;

%% 1. PARAMETERS ==========================================================

% --- Environment ---------------------------------------------------------
rho = 1.225;            % Air density at sea level (kg/m^3)
g   = 9.81;             % Gravity (m/s^2)

% --- Aerodynamics (low Reynolds number regime) ---------------------------
CD0         = 0.045;    % Zero-lift drag coefficient (typically higher for MAVs)
e           = 0.75;     % Oswald efficiency factor
AR          = 2.5;     % Aspect ratio
CL_max      = 0.748;     % Maximum lift coefficient
alpha_stall = 15;       % Estimated angle of attack at stall (degrees)

% --- Performance requirements --------------------------------------------
V_stall = 2.3;          % Target stall/minimum speed (m/s)
V_max   = 6.0;          % Target maximum cruise speed (m/s)
ROC     = 1.0;          % Desired rate of climb (m/s)
V_climb = 3.0;          % Forward speed during climb (m/s)
S_TO    = 3.0;          % Maximum allowable takeoff distance (m)

% --- Takeoff -------------------------------------------------------------
mu    = 0.05;           % Rolling friction coefficient (grass/rough surface)
CL_TO = 0.85;            % Lift coefficient during ground roll

% --- Pylon turn ----------------------------------------------------------
V_turn = V_max;         % Speed in the turn (m/s)
R_turn = 6;             % Turn radius (m)

% --- Diagram range -------------------------------------------------------
WS_min          = 1;    % Lowest wing loading plotted (N/m^2)
WS_max          = 16;   % Highest wing loading plotted (N/m^2)
N_WS            = 1000; % Number of W/S points
TW_plot_ceiling = 1.5;  % Max T/W on plot

% --- Design point selection ----------------------------------------------
WS_force = [];          % [] = auto-pick the W/S with the lowest required T/W
                        % Set a number (e.g. 4.0) to force that wing loading instead

% --- Current prototype iteration (plotted as a marker) -------------------
proto_WS = 6.768;       % Prototype wing loading (N/m^2)
proto_TW = 0.9;         % Prototype thrust-to-weight

% --- Component masses (g), everything except the wing --------------------
m_motor_prop = 3;
m_front_gear = 2;
m_back_gear  = 2;       % Total for both back gear
m_bracket    = 2;
m_tail       = 1;
m_boom       = 1;
m_pcb        = 6;
m_battery    = 4;

% --- Wing (current design) -----------------------------------------------
m_wing_now     = 12;        % Current wing mass (g)
S_now          = 0.0724638; % Current wing area (m^2)
rho_A_override = [];        % [] = use the current wing's area-density (m_wing_now/S_now)
                            % Set a number in g/m^2 (e.g. 100) if you expect a lighter wing

% --- Comparison aircraft -------------------------------------------------
m_nightvapor = 26;      % Night Vapor total mass (g)
b_nightvapor = 0.37;    % Night Vapor wingspan (m)
c_nightvapor = 0.15;    % Night Vapor chord (m)

m_sportscub  = 56;      % Sports Cub total mass (g)
b_sportscub  = 0.61;    % Sports Cub wingspan (m)
c_sportscub  = 0.10;    % Sports Cub chord (m)

% --- Sizing loop ---------------------------------------------------------
m_guess0    = 40;       % Initial total-mass guess (g)
tol         = 1e-6;     % Convergence tolerance (g)
maxIter     = 100;      % Iteration cap
rho_A_sweep = [60 80 100 120 140]; % Wing area-densities for the sensitivity table (g/m^2)

%% 2. CONSTRAINT DIAGRAM CALCULATIONS =====================================
% Required T/W at each wing loading W/S (N/m^2)

WS    = linspace(WS_min, WS_max, N_WS);
K     = 1/(pi*e*AR);                    % Induced drag factor
CD_TO = CD0 + K*CL_TO^2;                % Drag coefficient during ground roll

% A. Stall speed (thrust-lift coupled)
% L + T*sin(alpha) = W  =>  T/W = (1 - L/W) / sin(alpha)
% If W/S is too high for pure aerodynamic lift at V_stall, thrust makes up the difference.
q_stall  = 0.5*rho*V_stall^2;
TW_stall = max((1 - q_stall*CL_max./WS)/sind(alpha_stall), 0); % 0 = wing alone is enough

% B. Maximum speed
q_max       = 0.5*rho*V_max^2;
TW_maxspeed = q_max*CD0./WS + K*WS/q_max;

% C. Rate of climb (exact flight path angle, no small-angle assumption)
gamma   = asin(ROC/V_climb);            % Flight path angle (rad)
q_climb = 0.5*rho*V_climb^2;
TW_roc  = q_climb*CD0./WS + K*cos(gamma)^2*WS/q_climb + sin(gamma);

% D. Takeoff distance (ground roll with rolling resistance and drag)
TW_takeoff = mu + 0.72*(CD_TO - mu*CL_TO)/CL_max + 1.44*WS/(g*rho*CL_max*S_TO);

% E. Pylon turn (sustained level turn at load factor n)
n_turn  = sqrt(1 + (V_turn^2/(g*R_turn))^2);
q_turn  = 0.5*rho*V_turn^2;
TW_turn = q_turn*CD0./WS + K*n_turn^2*WS/q_turn;

% Feasible region: the design must meet the HIGHEST T/W of all constraints at each W/S
TW_required = max([TW_stall; TW_maxspeed; TW_roc; TW_takeoff; TW_turn], [], 1);

% Design point: lowest required T/W = smallest motor (unless W/S is forced)
if isempty(WS_force)
    [TW_design, idx] = min(TW_required);
    WS_design = WS(idx);
else
    WS_design = WS_force;
    TW_design = interp1(WS, TW_required, WS_design);
end

% Reference wing loadings shown on the diagram
m_fixed = m_motor_prop + m_front_gear + m_back_gear + m_bracket ...
        + m_tail + m_boom + m_pcb + m_battery;               % everything except the wing (g)
m_now   = m_fixed + m_wing_now;                              % current design from component masses (g)
WS_now  = m_now/1000*g/S_now;                                % current design W/S (N/m^2)

S_nightvapor  = b_nightvapor*c_nightvapor;                   % wing area (m^2)
WS_nightvapor = (m_nightvapor/1000*g)/S_nightvapor;          % W/S (N/m^2)
S_sportscub   = b_sportscub*c_sportscub;                     % wing area (m^2)
WS_sportscub  = (m_sportscub/1000*g)/S_sportscub;            % W/S (N/m^2)

%% 3. CONSTRAINT DIAGRAM PLOT =============================================
figure('Name', 'MAV Constraint Diagram', 'Color', 'w', 'Position', [100, 100, 850, 650]);
ax = gca; hold on; grid on;

% Shade the feasible design space
fill([WS, fliplr(WS)], [TW_required, TW_plot_ceiling*ones(size(WS))], ...
    [0.8 0.95 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Constraint curves
plot(WS, TW_maxspeed, 'b-',  'LineWidth', 2, 'DisplayName', sprintf('Max Speed (V_{max} = %.1f m/s)', V_max));
plot(WS, TW_roc,      'r--', 'LineWidth', 2, 'DisplayName', sprintf('ROC (ROC = %.1f m/s, \\gamma = %.1f^\\circ)', ROC, rad2deg(gamma)));
plot(WS, TW_takeoff,  'k-.', 'LineWidth', 2, 'DisplayName', sprintf('Takeoff (S_{TO} = %.1f m, \\mu = %.2f)', S_TO, mu));
plot(WS, TW_stall,    'm-',  'LineWidth', 2, 'DisplayName', sprintf('Stall (V = %.1f m/s)', V_stall));
plot(WS, TW_turn,     'c-',  'LineWidth', 2, 'DisplayName', sprintf('Pylon Turn (V = %.1f m/s, R = %.1f m)', V_turn, R_turn));

% Design point and current prototype
plot(WS_design, TW_design, 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', ...
    'DisplayName', sprintf('Design Point (W/S = %.2f, T/W = %.2f)', WS_design, TW_design));
plot(proto_WS, proto_TW, 'p', 'MarkerSize', 14, 'MarkerFaceColor', '#D95319', 'MarkerEdgeColor', 'k', ...
    'DisplayName', sprintf('Prototype (W/S = %.2f, T/W = %.2f)', proto_WS, proto_TW));
text(proto_WS + 0.3, proto_TW, 'Current Prototype', ...
    'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold');

% Reference wing loadings
plot([WS_now WS_now], [0 TW_plot_ceiling], 'k:', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Current Design, Component Masses (W/S = %.2f)', WS_now));
plot([WS_nightvapor WS_nightvapor], [0 TW_plot_ceiling], 'g--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Night Vapor (W/S = %.2f)', WS_nightvapor));
plot([WS_sportscub WS_sportscub], [0 TW_plot_ceiling], 'm--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Sports Cub (W/S = %.2f)', WS_sportscub));

% Formatting
axis([WS_min WS_max 0 TW_plot_ceiling]);
xlabel('Wing Loading, W/S (N/m^2)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Thrust-to-Weight Ratio, T/W', 'FontSize', 12, 'FontWeight', 'bold');
title('Constraint Diagram', 'FontSize', 14);   % sized mass is appended after Section 4
legend('Location', 'northeast', 'FontSize', 10);
hold off;

%% 4. SIZING ==============================================================
% Wing mass scales with wing area: m_wing = rho_A * S, and S = W / (W/S).
% Iterate the total mass until guess = calculated.

% Wing area-density (g/m^2)
rho_A_now = m_wing_now/S_now;
if isempty(rho_A_override)
    rho_A = rho_A_now;
else
    rho_A = rho_A_override;
end

wing_wt_per_area = rho_A/1000*g;        % N of wing per m^2 of wing area
if wing_wt_per_area >= WS_design
    error('Wing alone weighs %.2f N/m^2, more than W/S = %.2f. No solution: lower rho_A or raise W/S.', ...
        wing_wt_per_area, WS_design);
end

% --- Sizing loop ---
m_guess = m_guess0;
fprintf('Iter  guess[g]  S[m^2]   wing[g]  calc[g]\n');
for k = 1:maxIter
    W_guess = m_guess/1000*g;            % N
    S       = W_guess/WS_design;         % wing area from the guess
    m_wing  = rho_A*S;                   % wing mass from area-density
    m_calc  = m_fixed + m_wing;          % total from the parts
    fprintf('%3d   %7.3f  %7.4f  %7.3f  %7.3f\n', k, m_guess, S, m_wing, m_calc);
    if abs(m_calc - m_guess) < tol
        break;
    end
    m_guess = m_calc;                    % feed the result back in
end
m_total = m_calc;
W = m_total/1000*g;
S = W/WS_design;

% Closed-form check: m = m_fixed/(1 - rho_A*g/(1000*W/S)) should match the loop
m_check = m_fixed/(1 - wing_wt_per_area/WS_design);

% --- Results ---
b = sqrt(AR*S);
c = S/b;
Vs_aero = sqrt(2*WS_design/(rho*CL_max));    % stall with NO thrust help
fprintf('\nDesign point      : W/S = %.2f N/m^2, T/W = %.2f\n', WS_design, TW_design);
fprintf('Total mass        : %.1f g  (loop) vs %.1f g (closed form)\n', m_total, m_check);
fprintf('Wing              : S = %.4f m^2, span = %.2f m, chord = %.3f m, wing = %.1f g\n', S, b, c, rho_A*S);
fprintf('Thrust needed     : %.1f gf  (T/W x weight)\n', TW_design*m_total);
fprintf('Stall (no thrust) : %.2f m/s  (target %.1f m/s)\n', Vs_aero, V_stall);
fprintf('Current design    : %.1f g, W/S = %.2f N/m^2\n', m_now, WS_now);

% --- Sensitivity: how the answer moves with wing area-density ---
fprintf('\nrho_A[g/m^2]  mass[g]  S[m^2]  span[m]\n');
for rA = [rho_A_sweep rho_A_now]
    wpa = rA/1000*g;
    if wpa >= WS_design
        continue;
    end
    mm = m_fixed/(1 - wpa/WS_design);
    SS = mm/1000*g/WS_design;
    fprintf('  %6.1f      %6.1f   %.4f   %.2f\n', rA, mm, SS, sqrt(AR*SS));
end

% Put the sized mass in the diagram title now that it is known
title(ax, sprintf('Constraint Diagram (sized mass = %.1f g)', m_total), 'FontSize', 14);
