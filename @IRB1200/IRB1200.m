classdef IRB1200 < RobotBaseClassEdited
    %% IRB 1200 industrial robot created by a student

    properties(Access = public)
        plyFileNameStem = 'IRB1200';
    end

    methods


        %% Define robot Function
        function self = IRB1200(baseTr)
            self.CreateModel(); % Creates model

            if nargin < 1
                baseTr = eye(4); % If there are no input arguments, the base is assumed to be at the origin
            end

            self.model.base = self.model.base.T * baseTr; % Edit this line to change the location of the robot

            self.PlotAndColourRobot(); % Plotting the robot
            %self.model.teach;
            

        end

        %% Create the robot model
        function CreateModel(self)

            % DH parameters for IRB 1200
            link(1) = Link('d',(399.1-219)/1000,'a',0,'alpha',pi/2, 'qlim', deg2rad([-170 170]), 'offset',0);
            link(2) = Link('d',0,'a',-(350+42)/1000,'alpha',0,'qlim', deg2rad([-100 135]), 'offset',-pi/2);
            link(3) = Link('d',0,'a',42/1000,'alpha',pi/2,'qlim', deg2rad([-110 160]), 'offset', -pi);
            link(4) = Link('d',(351+(530-351-82))/1000,'a',0,'alpha',-pi/2,'qlim',deg2rad([-270 270]),'offset', 0);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-130 130]), 'offset',0);
            link(6) = Link('d',102/1000,'a',0,'alpha',0,'qlim',deg2rad([-400 400]), 'offset', 0);

            % Creating the serial link
            self.model = SerialLink(link,'name',self.name);

        end

        function [centerX, centerY, centerZ,radiusX,radiusY, radiusZ, workspaceRadius]= PointCloud(self)
            stepRads = deg2rad(60);
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