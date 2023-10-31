classdef teaUR3 < RobotBaseClassEdited
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'teaUR3';
    end

    methods
        %% Constructor
        function self = teaUR3(baseTr)
            close all
            % note: no tools passed in. gripper is separate because it is two 3-link serial robot
            if nargin < 1
                baseTr = eye(4); % If there are no input arguments, the base is assumed to be at the origin
            end

            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            %self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',-pi/2);
            link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', -pi/2);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end

        function [centerX, centerY, centerZ,radiusX,radiusY, radiusZ, workspaceRadius] = PointCloud(self)
            stepRads = deg2rad(120);
            qlim = self.model.qlim;

            pointCloudeSize_Vol = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            pointCloud_Vol = zeros(pointCloudeSize_Vol,3);

            counter = 1;

            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                q6 = 0;
                                q = [q1, q2, q3, q4, q5, q6];
                                tr = self.model.fkine(q).T;
                                pointCloud_Vol(counter,:) = tr(1:3,4)';
                                counter = counter + 1;
                            end
                        end
                    end
                end
            end
            plot3(pointCloud_Vol(:,1),pointCloud_Vol(:,2),pointCloud_Vol(:,3),'r.');

            minX = min(pointCloud_Vol(:,1));
            maxX = max(pointCloud_Vol(:,1));
            minY = min(pointCloud_Vol(:,2));
            maxY = max(pointCloud_Vol(:,2));
            minZ = min(pointCloud_Vol(:,3));
            maxZ = max(pointCloud_Vol(:,3));

            % Calculate the center of the workspace
            centerX = (minX + maxX) / 2;
            centerY = (minY + maxY) / 2;
            centerZ = (minZ + maxZ) / 2;

            % Calculate the radius
            radiusX = (maxX - minX) / 2;
            radiusY = (maxY - minY) / 2;
            radiusZ = (maxZ - minZ) / 2;

            % The overall radius is the maximum of the radii along each axis
           workspaceRadius = max([radiusX, radiusY, radiusZ]);

        end

    end
end
