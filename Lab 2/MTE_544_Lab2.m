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
show(map);

binary_map = binaryOccupancyMap(checkOccupancy(map));

% -1 is unknown
% 0  is unoccupied
% 1 is occupied

binary_map_data = getOccupancy(binary_map);
save('binary_occupancy_map.mat','binary_map_data');
