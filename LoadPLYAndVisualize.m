function [vertex, face, faceNormals] = LoadPLYAndVisualize(translationVector,plotOptions,~)
    if nargin < 4
        
        if nargin < 3
            axis_h = gca;
            if nargin < 2
                plotOptions.plotVerts = false;
                plotOptions.plotEdges = false;
                plotOptions.plotFaces = false;
            end
        end
    end

    % Load ply file
    ptCloud = pcread('Models/Obstruction.ply');
    vertex = double(ptCloud.Location + translationVector);  % Apply translation

    % Calculate ConnectivityList using delaunayTriangulation
    tri = delaunayTriangulation(vertex);

    % Assuming triangular faces
    face = tri.ConnectivityList;

   if 2 < nargout    
    faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
   end
   
    if isfield(plotOptions, 'plotVerts') && plotOptions.plotVerts
        for i = 1:size(vertex, 1)
            plot3(vertex(i, 1), vertex(i, 2), vertex(i, 3), 'r*');
            text(vertex(i, 1), vertex(i, 2), vertex(i, 3), num2str(i));
        end
    end

    if isfield(plotOptions, 'plotEdges') && plotOptions.plotEdges
        % Code for plotting edges
        % (similar to your existing code for plotting edges)
    end

    if isfield(plotOptions, 'plotFaces') && plotOptions.plotFaces
        tcolor = [0.2, 0.2, 0.8];
        patch('Faces', face, 'Vertices', vertex, 'FaceVertexCData', tcolor, 'FaceColor', 'flat', 'LineStyle', 'none');
    end


end
