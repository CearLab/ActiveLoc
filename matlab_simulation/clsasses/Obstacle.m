classdef Obstacle < handle
    properties
        type
        geometry
    end

    methods
        function obj = Obstacle(type,geometry)
            obj.type = type;
            obj.geometry = geometry;
        end
        function draw_on_map(obj)
            switch obj.type
                case 'polygon'
                    Obstacle.add_polygon_on_map(obj.geometry)
            end

        end
    end
    methods(Static)
        function add_polygon_on_map(geometry)
            x = geometry(1,:);
            y = geometry(2,:);
            fill(x,y,'r');
        end
    end
end

