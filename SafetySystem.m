classdef SafetySystem
    properties (Constant)
        defaultFencePosition = eye(4);
        defaultEStopPositions = transl(1.2, -1, 1); 
        defaultFireExtinguisherPosition = transl(1, 1, 0);
        defaultLightCurtainPositions = transl(4.1/2, 2.07/2, 1.14/2);
        defaultSirenPosition = transl(4.1/2, -2.07/2, 1.14 + 0.135);
        defaultAlarmPosition = transl(-4.1/2, -2.07/2, 1.14);
        defaultSignPosition = transl(1.5, 1.5, 0);
    end

    properties
        handle;
    end

    methods (Static)
        function ActivateAll()
            hold on;
            axis equal;
            SafetySystem.Fence();
            SafetySystem.EStop();
            SafetySystem.EStop(transl(1.2, 1.15, 1));
            SafetySystem.FireExtingusher();
            SafetySystem.LightCurtain();
            SafetySystem.LightCurtain(transl(-4.1/2, 2.07/2, 1.14/2));
            SafetySystem.LightCurtain(transl(-4.1/2, -2.07/2, 1.14/2));
            SafetySystem.LightCurtain(transl(4.1/2, -2.07/2, 1.14/2));
            SafetySystem.Siren();
            SafetySystem.Alarm();
            SafetySystem.Sign(); 
        end

        function Fence(position)
            if nargin < 1
                position = SafetySystem.defaultFencePosition;
            end
            [f,v,data] = plyread('fenceAssemblyGreenRectangleModified.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp('Fence activated');
        end

        function EStop(position)
            if nargin < 1
                position = SafetySystem.defaultEStopPositions;
            end
            [f,v,data] = plyread('emergencyStopWallMounted.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp(['Emergency Stop at position: ', mat2str(position(1:3, 4)')]);
        end

        function FireExtingusher(position)
            if nargin < 1
                position = SafetySystem.defaultFireExtinguisherPosition;
            end
            [f,v,data] = plyread('fireExtinguisherElevated.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp(['Fire Extinguisher at position: ', mat2str(position(1:3, 4)')]);
        end

        function LightCurtain(position)
            if nargin < 1
                position = SafetySystem.defaultLightCurtainPositions;
            end
            [f,v,data] = plyread('LightCurtain.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp(['Light Curtain at position: ', mat2str(position(1:3, 4)')]);
        end

        function Siren(position)
            if nargin < 1
                position = SafetySystem.defaultSirenPosition;
            end
            [f,v,data] = plyread('Siren.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp(['Siren at position: ', mat2str(position(1:3, 4)')]);
        end

        function Alarm(position)
            if nargin < 1
                position = SafetySystem.defaultAlarmPosition;
            end
            [f,v,data] = plyread('FireAlarm.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp(['Alarm at position: ', mat2str(position(1:3, 4)')]);
        end

        function Sign(position)
            if nargin < 1
                position = SafetySystem.defaultSignPosition;
            end
            [f,v,data] = plyread('Sign.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            handle = trisurf(f, v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4),...
                'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            %disp(['Sign at position: ', mat2str(position(1:3, 4)')]);
        end
    end
end
