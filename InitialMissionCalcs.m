%% INITIAL FLIGHT MISSION CALCULATIONS - FOR OUTDOOR
clear all; clc; close all;
%% UNIT CONVERSIONS
kmh2ms  = 1000 / 3600; % 
min2sec = 60; 
% speedms = speedkmh * kmh2ms;

%% FLIGHT AREA & CAMERA SPECS
alt_drone           = 5; % [m] - desired drone flight altitude
camera_ratio_altfov = 1; % assumed that the camera from object is the fov edges
nmr_drones          = 2; % number of drones used in the mission so that the research area is assigned
fightTime_limit     = 7 * min2sec; % [sec] - the time limit is obtained from ecalc and assumed that there is uncertainty -2 mins from mixed flight
area_width          = 50; % [m]
area_length         = 80; % [m]
camera_fov_length   = alt_drone / camera_ratio_altfov; % [m]
camera_fov_width    = alt_drone / camera_ratio_altfov; % [m]
mnr_of_search       = 3; % number of how mmay times the drone will cover the searvhing area

%% Mission Calculations
nmr_strips          = area_length / camera_fov_length; % number of search paths strips in total
strip_per_drone     = nmr_strips / nmr_drones; % allocated number of stirps tp each drone
dist_per_drone      = strip_per_drone * area_width * mnr_of_search; % [m] - total distance that a drone should spend
vel_min             = dist_per_drone / fightTime_limit; % [m/sec] - the minimum flight speed to cover all search area

fprintf("The number of the drones are %4.2f ", nmr_drones)
fprintf("\n")
fprintf("The minimum flight velocity to complete the search %4.2f m/s per drone", vel_min)