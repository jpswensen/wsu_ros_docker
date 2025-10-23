% CentroidTrackingDemo
% Visual servoing demo that points the camera at the centroid of detected
% spheres/points in the image using timer-based control.
%
% This implements the same algorithm as camera_demo.py:
%   1) Find bright and dark regions using adaptive thresholding
%   2) Filter connected components by area and aspect ratio
%   3) Detect circular blobs using SimpleBlobDetector equivalent
%   4) Compute geometric centroid of all detections
%   5) Command pitch/yaw to center the centroid in the image
%
% Control law:
%   - Yaw (about +y): proportional to horizontal pixel error
%   - Pitch (about +x): proportional to vertical pixel error

% Optionally ensure MATLAB uses your ROS 2 Python (adjust if needed):
% pyenv('Version','/usr/bin/python3');

%% Parameters (tunable at runtime)
params = struct();

% Detection thresholds
params.cc_min_area_frac = 2e-5;   % Minimum area as fraction of image
params.cc_min_area_px = 12;       % Minimum area in pixels (absolute floor)
params.cc_aspect_min = 0.3;       % Minimum aspect ratio for connected components
params.cc_aspect_max = 2.5;       % Maximum aspect ratio for connected components
params.merge_tol_px = 10.0;       % Merge tolerance in pixels

% Blob detector parameters (disabled by default for performance)
params.blob_min_area = 50.0;      % Minimum blob area in pixels^2 (increased to avoid warnings)
params.blob_circularity = 0.6;    % Minimum circularity (0-1)
params.blob_inertia = 0.2;        % Minimum inertia ratio

% Control gains
params.k_yaw = -0.2;    % Maps horizontal pixel error to yaw about +y
params.k_pitch = 0.2;   % Maps vertical pixel error to pitch about +x

% Display options
params.show_overlays = true;      % Draw detection overlays on image
params.overlay_rate = 3;          % Show overlays every N control iterations (for performance)

%% Create and start the camera client
cam = FloatingCamera();
cam.start();

% Ensure cleanup on exit or error
cleanupObj = onCleanup(@() cleanupCamera(cam));

% Wait a moment for connection
pause(0.5);

% Set initial pose (matching Python demo)
% Position: x=0, y=1, z=-7
% Orientation: identity quaternion (0, 0, 0, 1) for zero roll/pitch/yaw
fprintf('Setting initial camera pose...\n');
cam.setPose(2.0, 1.0, -7.0, 0.0, 0.0, 0.0, 1.0);

% Wait for first image
pause(1.0);

%% Define the visual servoing control callback
% Create a function handle that captures params
controlFcn = @(cam, t) centroidControlLoop(cam, t, params);

%% Start unified timer (single scheduler for control and display)
% Prepare figure and initialize detection cache used by display callback
fh = cam.getFigureHandle();
setappdata(fh, 'dets', struct('component_centers', [], 'blob_centers', [], 'target_center', [], 't', 0));

% Define display callback (draws using cached detections)
displayFcn = @(camObj, img) displayLoopWithOverlays(camObj, img, params);

% Start a single timer that services both:
% - control at 10 Hz (compute detections + publish twist)
% - display at ~30 Hz (draw overlays only)
cam.startUnifiedTimer(0.033, 0.1, displayFcn, controlFcn);

fprintf('Centroid tracking active. Camera will servo to point at detected objects.\n');
fprintf('Press Ctrl+C to stop.\n');

%% Run indefinitely (or until user interrupts)
try
    % Idle loop until figure closed; Ctrl+C interrupts cleanly
    while ishghandle(fh)
        pause(0.1);
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:KeyboardInterrupt')
        rethrow(ME);
    end
end

%% Cleanup
fprintf('\nStopping...\n');
% Stop any timers (unified or legacy) and clear MATLAB timers to ensure clean state
cam.stopUnifiedTimer();
cam.stopControlTimer();
cam.stopDisplayTimer();
cam.publishTwist(0,0,0, 0,0,0);  % Stop motion
cam.stop();
% Aggressively delete any leftover MATLAB timers (visible or hidden)
try
    delete(timerfindall);
catch
end

fprintf('Demo complete!\n');

%% Helper Functions

function centroidControlLoop(cam, t, params)
    % Visual servoing control loop - finds centroid and commands camera
    persistent iteration_count;
    if isempty(iteration_count)
        iteration_count = 0;
    end
    iteration_count = iteration_count + 1;
    
    % 1) Get the current image
    img = cam.getImage();
    if isempty(img)
        % No image yet; stop motion
        cam.publishTwist(0,0,0, 0,0,0);
        return;
    end
    
    [h, w, ~] = size(img);
    cx0 = w / 2.0;
    cy0 = h / 2.0;
    
    % 2) Find the centroid of tracked points (and cache for the display loop)
    [target_center, component_centers, blob_centers] = findPointsCentroid(img, params);
    % Cache detections for the display loop to avoid recomputing there
    try
        fh = cam.getFigureHandle();
        if ishghandle(fh)
            dets = struct('component_centers', component_centers, ...
                          'blob_centers', blob_centers, ...
                          'target_center', target_center, ...
                          't', t);
            setappdata(fh, 'dets', dets);
        end
    catch
        % Ignore if figure not available
    end
    
    % 3) Compute control commands
    cmd_pitch = 0.0;  % angular.x (pitch about +x)
    cmd_yaw = 0.0;    % angular.y (yaw about +y)
    
    if ~isempty(target_center)
        % Compute pixel errors (normalized by image center)
        ex = (cx0 - target_center(1)) / cx0;  % Horizontal error
        ey = (cy0 - target_center(2)) / cy0;  % Vertical error
        
        % Proportional control law
        cmd_yaw = params.k_yaw * ex;      % Yaw about +y
        cmd_pitch = params.k_pitch * ey;  % Pitch about +x
        
        % Print feedback every ~1 second
        if mod(floor(t), 1) == 0 && mod(t, 1) < 0.15
            fprintf('t=%.1fs: Centroid at (%.0f, %.0f), Errors: ex=%.3f, ey=%.3f, Cmd: pitch=%.3f, yaw=%.3f\n', ...
                t, target_center(1), target_center(2), ex, ey, cmd_pitch, cmd_yaw);
        end
    else
        % No target found; stop motion
        if mod(floor(t), 1) == 0 && mod(t, 1) < 0.15
            fprintf('t=%.1fs: No target detected\n', t);
        end
    end
    
    % 4) Publish twist command
    % Linear velocities remain zero; only orientation changes (pitch/yaw)
    cam.publishTwist(0, 0, 0, cmd_pitch, cmd_yaw, 0);
    
    % 5) Overlays are handled in the display timer callback for smoother visuals
end

% (Removed keystroke handler to restore natural Ctrl+C behavior)

function displayLoopWithOverlays(cam, img, params)
        % Display callback: draw overlays using cached detections from control loop
    % If cache not yet available, just show the raw image; keep display light-weight
    try
        fh = cam.getFigureHandle();
    catch
        fh = [];
    end
    component_centers = [];
    blob_centers = [];
    target_center = [];
    if ~isempty(fh) && ishghandle(fh) && isappdata(fh, 'dets')
        dets = getappdata(fh, 'dets');
        if ~isempty(dets)
            if isfield(dets,'component_centers'), component_centers = dets.component_centers; end
            if isfield(dets,'blob_centers'), blob_centers = dets.blob_centers; end
            if isfield(dets,'target_center'), target_center = dets.target_center; end
        end
    end

    if isempty(component_centers) && isempty(blob_centers) && isempty(target_center)
        % No detections yet - show the raw image
        cam.show(img);
    else
        showDetectionOverlays(cam, img, component_centers, blob_centers, target_center);
    end
end

function [target_center, component_centers, blob_centers] = findPointsCentroid(img, params)
    % Fast centroid finder for bright (or dark) dots on uniform background
    %
    % Returns:
    %   target_center: [x, y] centroid of all detections, or [] if none
    %   component_centers: Nx2 array of connected component centers
    %   blob_centers: [] (not used in fast path)

    % 1) Grayscale
    if size(img,3) == 3
        gray = rgb2gray(img);
    else
        gray = img;
    end

    % 2) Global threshold (Otsu). If foreground dominates, invert.
    level = graythresh(gray);
    mask = gray > level * 255;       % assume bright dots on dark background
    if nnz(mask) > numel(mask)/2
        mask = ~mask;                 % invert if mostly foreground
    end

    % 3) Remove small speckles and very large regions
    min_area_px = params.cc_min_area_px;
    mask = bwareaopen(mask, max(1, round(min_area_px)));

    % 4) Connected components and centroids
    cc = bwconncomp(mask, 8);
    if cc.NumObjects == 0
        target_center = [];
        component_centers = [];
        blob_centers = [];
        return;
    end

    stats = regionprops(cc, 'Area', 'Centroid');

    % Filter by area bounds relative to image
    [h, w] = size(mask);
    min_area_frac = params.cc_min_area_frac;
    min_area = max(min_area_px, min_area_frac * w * h);
    max_area = 0.25 * w * h;

    component_centers = [];
    for i = 1:numel(stats)
        a = stats(i).Area;
        if a < min_area || a > max_area
            continue;
        end
        component_centers = [component_centers; stats(i).Centroid]; %#ok<AGROW>
    end

    % 5) Geometric centroid across accepted components
    if isempty(component_centers)
        target_center = [];
    else
        target_center = mean(component_centers, 1);
    end

    blob_centers = [];
end

function blob_centers = detectCircularBlobs(gray, params)
    % Detect circular blobs in grayscale image
    % Simplified approach - use imfindcircles only if min radius is reasonable
    
    blob_centers = [];
    
    % Calculate minimum radius from area
    min_radius = round(sqrt(params.blob_min_area / pi));
    
    % Only use imfindcircles if radius is >= 6 (to avoid warnings and poor accuracy)
    if min_radius < 6
        % For very small objects, skip blob detection and rely on connected components
        return;
    end
    
    max_radius = 100;  % Adjust based on expected object sizes
    
    try
        % Try to find bright circles - handle variable number of outputs
        [centers, radii] = imfindcircles(gray, [min_radius, max_radius], ...
            'ObjectPolarity', 'bright', 'Sensitivity', 0.85);
        
        if ~isempty(centers)
            blob_centers = [blob_centers; centers];
        end
        
        % Try to find dark circles
        [centers_dark, radii_dark] = imfindcircles(gray, [min_radius, max_radius], ...
            'ObjectPolarity', 'dark', 'Sensitivity', 0.85);
        
        if ~isempty(centers_dark)
            blob_centers = [blob_centers; centers_dark];
        end
    catch ME
        % If imfindcircles fails (e.g., toolbox not available), silently fall back
        % to connected components only
    end
end

function showDetectionOverlays(cam, img, component_centers, blob_centers, target_center)
    % Draw detection overlays on the image for visualization
    % Matches the Python demo visualization style
    % Uses basic drawing without Computer Vision Toolbox dependencies
    
    [h, w, ~] = size(img);
    
    % Draw component centers (green crosses) - matching Python MARKER_CROSS
    for i = 1:size(component_centers, 1)
        x = round(component_centers(i, 1));
        y = round(component_centers(i, 2));
        img = drawCross(img, x, y, [0, 255, 0], 12);
    end
    
    % Draw blob centers (cyan circles) - matching Python cv2.circle with outline
    for i = 1:size(blob_centers, 1)
        x = round(blob_centers(i, 1));
        y = round(blob_centers(i, 2));
        img = drawCircle(img, x, y, 6, [255, 255, 0], false);
    end
    
    % Draw target center (RED FILLED CIRCLE - matches Python cv2.circle with -1 fill)
    if ~isempty(target_center)
        x = round(target_center(1));
        y = round(target_center(2));
        img = drawCircle(img, x, y, 8, [255, 0, 0], true);
    end
    
    % Draw image center (purple/magenta TILTED CROSS - matches Python MARKER_TILTED_CROSS)
    cx = round(w / 2);
    cy = round(h / 2);
    img = drawCross(img, cx, cy, [100, 100, 255], 12);
    
    % Update the display with overlays
    cam.show(img);
end

function img = drawCross(img, x, y, color, markerSize)
    % Draw a cross marker at (x, y)
    [h, w, ~] = size(img);
    halfsize = round(markerSize / 2);
    thickness = 2;
    
    % Bounds check for center point
    if x < 1 || x > w || y < 1 || y > h
        return;
    end
    
    % Horizontal line
    x1 = max(1, x - halfsize);
    x2 = min(w, x + halfsize);
    for t = -thickness:thickness
        yy = y + t;
        if yy >= 1 && yy <= h && x1 <= x2
            img(yy, x1:x2, 1) = color(1);
            img(yy, x1:x2, 2) = color(2);
            img(yy, x1:x2, 3) = color(3);
        end
    end
    
    % Vertical line
    y1 = max(1, y - halfsize);
    y2 = min(h, y + halfsize);
    for t = -thickness:thickness
        xx = x + t;
        if xx >= 1 && xx <= w && y1 <= y2
            img(y1:y2, xx, 1) = color(1);
            img(y1:y2, xx, 2) = color(2);
            img(y1:y2, xx, 3) = color(3);
        end
    end
end

function img = drawCircle(img, cx, cy, radius, color, filled)
    % Draw a circle at (cx, cy) with given radius
    [h, w, ~] = size(img);
    
    % Bounds check for center point
    if cx < 1 || cx > w || cy < 1 || cy > h
        return;
    end
    
    if filled
        % Filled circle - optimized bounds
        y_min = max(1, cy - radius);
        y_max = min(h, cy + radius);
        x_min = max(1, cx - radius);
        x_max = min(w, cx + radius);
        
        for y = y_min:y_max
            for x = x_min:x_max
                if (x - cx)^2 + (y - cy)^2 <= radius^2
                    img(y, x, 1) = color(1);
                    img(y, x, 2) = color(2);
                    img(y, x, 3) = color(3);
                end
            end
        end
    else
        % Circle outline
        thickness = 2;
        y_min = max(1, cy - radius - thickness);
        y_max = min(h, cy + radius + thickness);
        x_min = max(1, cx - radius - thickness);
        x_max = min(w, cx + radius + thickness);
        
        for y = y_min:y_max
            for x = x_min:x_max
                dist = sqrt((x - cx)^2 + (y - cy)^2);
                if dist >= radius - thickness && dist <= radius + thickness
                    img(y, x, 1) = color(1);
                    img(y, x, 2) = color(2);
                    img(y, x, 3) = color(3);
                end
            end
        end
    end
end

function cleanupCamera(cam)
    % Cleanup function to ensure proper shutdown
    try
        if isvalid(cam)
            cam.stopUnifiedTimer();
            cam.stopControlTimer();
            cam.stopDisplayTimer();
            cam.publishTwist(0,0,0, 0,0,0);  % Stop motion
            cam.stop();
        end
        % Also delete any stray timers that might block clear classes
        try
            delete(timerfindall);
        catch
        end
    catch err
        warning('Error during cleanup: %s', err.message);
    end
end
