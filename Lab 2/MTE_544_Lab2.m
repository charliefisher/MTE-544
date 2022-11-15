close all; clear; clc;

%% Load Map %%

MapResolution = 1/0.03;
MapOrigin = [-1.94 -8.63];
MapOccupiedThresh = 0.65;
MapFreeThresh = 0.25;

map_image = imread('map_mazes/map_maze_1.pgm');
map_image = double(map_image)/255;
map_image_occupancy = 1 - map_image;

map = occupancyMap(map_image_occupancy, MapResolution);
map.FreeThreshold = MapFreeThresh;
map.OccupiedThreshold = MapOccupiedThresh;
map.GridLocationInWorld = MapOrigin;

figure;
show(map)

binary_map = binaryOccupancyMap(checkOccupancy(map));

%% Likelihood Field %%

pyenv('Version', '/Users/charliefisher/.pyenv/shims/python');
ros2genmsg('custom_ros2_msgs');

bagreader = ros2bag('bag/t2');

lf = likelihoodFieldSensorModel;
lf.Map = binary_map;
lf.SensorLimits = [0.15 12];
% lf.NumBeams = ;
lf.ExpectedMeasurementWeight = 0.99;
% lf.RandomMeasurementWeight = ; % measure as variance of data?

% 
N_particles = 50;


% -1 is unknown
% 0  is unoccupied
% 1 is occupied