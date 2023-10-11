classdef FireExtingusher


    properties
        handle;
    end
    
    methods
        function self = FireExtingusher(position)
            % read ply file for vertex data
            [f,v,data] = plyread('fireExtinguisherElevated.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Then plot 
            self.handle = trisurf(f ...
                , v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
end