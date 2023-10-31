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
            % self.model.teach;

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
    end
end