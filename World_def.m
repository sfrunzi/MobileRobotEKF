% Sara Frunzi, ECE 609, Winter 2024
% Extended Kalman Filter Homework 2-1
% World Object

classdef World_def < handle
    properties
        x_range % [x_min, x_max] (m)
        y_range % [y_min, y_max] (m)
        map_res % (m)
        gridMap
        occMap
        waypoints % List of (x, y)
        landmarks % List of (x, y)
    end

    methods
        % Constructor-- define world object
        function world = World_def(x_min, x_max, y_min, y_max, res, wpts, lmks)
            world.x_range = [x_min, x_max];
            world.y_range = [y_min, y_max];
            world.map_res = res;
            world.waypoints = wpts;
            world.landmarks = lmks;
            
            % Empty grid map
            x_grid = x_min : map_res : x_max;
            y_grid = y_min : map_res : y_max;
            world.gridMap = zeros(x_grid, y_grid);
            world.occMap = world.gridMap;

            % True occupancy map generation
            for n = 1:length(world.waypoints)
                occupied = 1;
                x_occ = world.waypoints(n, 1);
                y_occ = world.waypoints(n, 2);
                world.occMap(x_occ, y_occ) = occupied;
            end
        end
        
        % Add Landmarks
        function lmk = addLandmark(world, x, y)
            world.landmarks = [world.landmarks; [x, y]];
            lmk = world.landmarks;
        end
        
        % Add Waypoints
        function wpt = addWaypoint(world, x, y)
            world.waypoints = [world.waypoints; [x, y]];
            wpt = world.waypoints;
        end
    end
end