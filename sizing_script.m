%{ 
This script plots a constraint diagram. Where the area above the
different lines represent the acceptable area. This determines a range of
usable thrust-to-weight and wing_loading. 

The formulas used here can be found here:
https://doi.org/10.1177/0954410014540621 
%}

% Clear previous variables and graphs
clc;
clear;
close all; 

% Assumed constants
density = 1.2071;

% Editable parameters
c_drag_0 = 
oswald_e = 
ar = 
aoi = ;

cruise_v = 

climb_v = 
climb_dynam_p = 
climb_aoa = aoi + 40;

turn_v = 
turn_rad = 

takeoff_v = 

wing_loading = linspace();

% Constraint diagram plots
tw_cruise = (1/aoi) * ...
            (...
                ( 1 / (pi*oswald_e*ar) ) * ...
                ( 2 / (density*cruise_v*2) ) * ...
                ( wing_loading ) + ...
                () ...
            );

tw_climb

tw_turn

tw_accel

% Plotting constraint diagram
