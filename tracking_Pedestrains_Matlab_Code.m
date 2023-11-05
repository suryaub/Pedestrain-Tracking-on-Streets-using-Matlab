%% Tracking Pedestrians on Streets from a Moving Car 

function PedestrianTrackingFromMovingCameraExample()

% Create system objects
videoFile       = 'inputVideo.mp4';
scaleDataFile   = 'pedScaleTable.mat';

obj = setupSystemObjects(videoFile, scaleDataFile);

% Create an empty array of tracks.
tracks = initializeTracks(); 

% ID of the next track.
nextId = 1; 

% Set the global parameters.
option.ROI                  = [40 95 400 140]; 
option.scThresh             = 0.3;              
option.gatingThresh         = 0.9;              
option.gatingCost           = 100;              
option.costOfNonAssignment  = 10;               
option.timeWindowSize       = 16;               
option.confidenceThresh     = 2;                
option.ageThresh            = 8;                
option.visThresh            = 0.6;              

% Detect people and track them across video frames.
cont = ~isDone(obj.reader);
while cont    
    frame   = readFrame();
 
    [centroids, bboxes, scores] = detectPeople();
    
    predictNewLocationsOfTracks();    
    
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();
    
    updateAssignedTracks();    
    updateUnassignedTracks();    
    deleteLostTracks();    
    createNewTracks();
    
    displayTrackingResults();

    % Exit the loop if the video player figure is closed by user.     
    cont = ~isDone(obj.reader) && isOpen(obj.videoPlayer);
end

%% Create System Objects for the Tracking System Initialization 

    function obj = setupSystemObjects(videoFile,scaleDataFile)
	
        % Initialize Video I/O
        obj.reader = vision.VideoFileReader(videoFile, 'VideoOutputDataType', 'uint8');
        
        % Create a video player.
        obj.videoPlayer = vision.VideoPlayer('Position', [29, 597, 643, 386]);                
        
        % Load the scale data file                                        
        ld = load(scaleDataFile, 'pedScaleTable');
        obj.pedScaleTable = ld.pedScaleTable;
    end


%% Initialize Tracks

    function tracks = initializeTracks()
        % Create an empty array of tracks
        tracks = struct(...
            'id', {}, ...
            'color', {}, ...
            'bboxes', {}, ...
            'scores', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'confidence', {}, ...            
            'predPosition', {});
    end

%% Read a Video Frame

    function frame = readFrame()
        frame = step(obj.reader);
    end

%% Detect People

    function [centroids, bboxes, scores] = detectPeople()

        resizeRatio = 1.5;
        frame = imresize(frame, resizeRatio, 'Antialiasing',false);
        
        [bboxes, scores] = detectPeopleACF(frame, option.ROI, ...
            'Model','caltech',...
            'WindowStride', 2,...
            'NumScaleLevels', 4, ...
            'SelectStrongest', false);
			
        height = bboxes(:, 4) / resizeRatio;
        y = (bboxes(:,2)-1) / resizeRatio + 1;        
        yfoot = min(length(obj.pedScaleTable), round(y + height));
        estHeight = obj.pedScaleTable(yfoot); 
        
        invalid = abs(estHeight-height)>estHeight*option.scThresh;        
        bboxes(invalid, :) = [];
        scores(invalid, :) = [];

        [bboxes, scores] = selectStrongestBbox(bboxes, scores, ...
                            'RatioType', 'Min', 'OverlapThreshold', 0.6);                               
        
        if isempty(bboxes)
            centroids = [];
        else
            centroids = [(bboxes(:, 1) + bboxes(:, 3) / 2), ...
                (bboxes(:, 2) + bboxes(:, 4) / 2)];
        end
    end

%% Predict New Locations of Existing Tracks

    function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bboxes(end, :);
            
            predictedCentroid = predict(tracks(i).kalmanFilter);
            
            tracks(i).predPosition = [predictedCentroid - bbox(3:4)/2, bbox(3:4)];
        end
    end

%% Assign Detections to Tracks

    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()
        
        predBboxes = reshape([tracks(:).predPosition], 4, [])';
        cost = 1 - bboxOverlapRatio(predBboxes, bboxes);
        cost(cost > option.gatingThresh) = 1 + option.gatingCost;

        % Solve the assignment problem.
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, option.costOfNonAssignment);
    end

%% Update Assigned Tracks

    function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);

            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);
            
            correct(tracks(trackIdx).kalmanFilter, centroid);
           
            T = min(size(tracks(trackIdx).bboxes,1), 4);
            w = mean([tracks(trackIdx).bboxes(end-T+1:end, 3); bbox(3)]);
            h = mean([tracks(trackIdx).bboxes(end-T+1:end, 4); bbox(4)]);
            tracks(trackIdx).bboxes(end+1, :) = [centroid - [w, h]/2, w, h];
            
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
           
            tracks(trackIdx).scores = [tracks(trackIdx).scores; scores(detectionIdx)];
            
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
           
            T = min(option.timeWindowSize, length(tracks(trackIdx).scores));
            score = tracks(trackIdx).scores(end-T+1:end);
            tracks(trackIdx).confidence = [max(score), mean(score)];
        end
    end

%% Update Unassigned Tracks

    function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            idx = unassignedTracks(i);
            tracks(idx).age = tracks(idx).age + 1;
            tracks(idx).bboxes = [tracks(idx).bboxes; tracks(idx).predPosition];
            tracks(idx).scores = [tracks(idx).scores; 0];
           
            T = min(option.timeWindowSize, length(tracks(idx).scores));
            score = tracks(idx).scores(end-T+1:end);
            tracks(idx).confidence = [max(score), mean(score)];
        end
    end

%% Delete Lost Tracks

    function deleteLostTracks()
        if isempty(tracks)
            return;
        end        
        
        ages = [tracks(:).age]';
        totalVisibleCounts = [tracks(:).totalVisibleCount]';
        visibility = totalVisibleCounts ./ ages;
        
        % Check the maximum detection confidence score.
        confidence = reshape([tracks(:).confidence], 2, [])';
        maxConfidence = confidence(:, 1);

        % Find the indices of 'lost' tracks.
        lostInds = (ages <= option.ageThresh & visibility <= option.visThresh) | ...
             (maxConfidence <= option.confidenceThresh);

        % Delete lost tracks.
        tracks = tracks(~lostInds);
    end

%% Create New Tracks

    function createNewTracks()
        unassignedCentroids = centroids(unassignedDetections, :);
        unassignedBboxes = bboxes(unassignedDetections, :);
        unassignedScores = scores(unassignedDetections);
        
        for i = 1:size(unassignedBboxes, 1)            
            centroid = unassignedCentroids(i,:);
            bbox = unassignedBboxes(i, :);
            score = unassignedScores(i);
            
            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [2, 1], [5, 5], 100);
            
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'color', 255*rand(1,3), ...
                'bboxes', bbox, ...
                'scores', score, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'confidence', [score, score], ...
                'predPosition', bbox);
            
            % Add it to the array of tracks.
            tracks(end + 1) = newTrack; %#ok
            
            % Increment the next id.
            nextId = nextId + 1;
        end
    end

%% Display Tracking Results
    
    function displayTrackingResults()

        displayRatio = 4/3;