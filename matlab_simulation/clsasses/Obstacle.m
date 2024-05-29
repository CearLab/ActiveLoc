%% Obstacle class
% file: Obstacle.m
% author: Ido Sherf 
% date: 22/01/2024
% description: hndle class handling the creation of an obstacle on a Map 
% (see map class)
classdef Obstacle < handle

    % class properties
    properties

        % obstacle type (what other that polygon?)
        type

        % obstcle geometry
        geometry

    end

    % class methods
    methods

        % assign type and geometry to the obstacle
        function obj = Obstacle(type,geometry)

            obj.type = type;
            obj.geometry = geometry;

        end

        % draw on an instance of Map class
        function drawOnMap(obj)

            % different cases: rn only polygon
            switch obj.type

                % call class method
                case 'polygon'
                    Obstacle.addPolygonOnMap(obj.geometry)
            end

        end

    end

    % class methods (static)
    methods(Static)

        % add a polygon to a Map class
        function addPolygonOnMap(geometry)

            % get all Xs of the polygon
            x = geometry(1,:);
            y = geometry(2,:);

            % fill with red the area enclosed within the polygon
            fill(x,y,'r');

        end

    end

end

