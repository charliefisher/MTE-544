close all; clear; clc;

%% Load Map %%

% given map parameters
MapResolution = 1/0.03;
MapOrigin = [-1.94 -8.63];
MapOccupiedThresh = 0.65;
MapFreeThresh = 0.25;

% load map image
map_image = imread('map_mazes/map_maze_1.pgm');
map_image = double(map_image)/255;
map_image_occupancy = 1 - map_image;

% create occupancy map
map = occupancyMap(map_image_occupancy, MapResolution);
map.FreeThreshold = MapFreeThresh;
map.OccupiedThreshold = MapOccupiedThresh;
map.GridLocationInWorld = MapOrigin;

figure;
show(map);

% update free threshold (based on observation)
map.FreeThreshold = 0.18; 

% export trinary occupancy map
map_occupancy = getOccupancy(map);
trinary_map_data = -1*ones(size(map_occupancy));
trinary_map_data(map_occupancy <= map.FreeThreshold) = 0;
trinary_map_data(map_occupancy >= map.OccupiedThreshold) = 1;

% -1 is unknown
% 0  is unoccupied
% 1 is occupied

writematrix(trinary_map_data, 'trinary_occupancy_map.csv');
