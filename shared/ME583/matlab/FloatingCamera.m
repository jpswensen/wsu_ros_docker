classdef FloatingCamera < handle
    % FloatingCamera
    % MATLAB wrapper that controls a Python ROS 2 camera node via py.*
    %
    % Summary
    %   - Imports a small Python module (floating_camera_py.py)
    %   - Starts a ROS 2 node and executor spinning in a Python thread
    %   - Publishes geometry_msgs/Twist commands (publishTwist)
    %   - Retrieves the latest RGB image as a MATLAB uint8 array (getImage)
    %   - Retrieves the latest pose as a MATLAB struct (getPose)
    %   - Displays images efficiently using handle reuse (show)
    %
    % Quick start
    %   cam = FloatingCamera();
    %   cam.start();
    %   img = cam.getImage();
    %   cam.show(img);     % or cam.show(); to fetch internally
    %   cam.publishTwist(0,0,0, 0,0,0.2);  % optional control
    %   cam.stop();

    properties (Access = private)
        pyModuleName = 'floating_camera_py'
        pyModulePath  % folder containing the python file
        pyWrapper     % instance of Python CameraDemoWrapper
        hFig          % MATLAB figure handle (for fast plotting)
        hAx           % MATLAB axes handle
        hImg          % MATLAB image handle
        frameCount = 0
        displayTimer  % MATLAB timer for periodic display updates
        controlTimer  % MATLAB timer for periodic control updates
        unifiedTimer  % Single scheduler timer for both display and control
        controlCallback  % user-provided control function handle
        displayCallback  % user-provided display function handle: @(cam, img)
        displayPeriodSec % desired display period (seconds) when using unified timer
        controlPeriodSec % desired control period (seconds) when using unified timer
        nextDisplayDue   % next scheduled display time (s since start)
        nextControlDue   % next scheduled control time (s since start)
    end

    methods
    function obj = FloatingCamera(pyDir, opts)
            % Options (reserved for future use)
            if nargin < 2 || isempty(opts)
                opts = struct();
            end

        %
        % High-level MATLAB wrapper that talks to a Python ROS 2 node using
        % MATLAB's "py.*" interoperability. Think of this class as the "bridge"
        % between MATLAB and the ROS 2 ecosystem.
        %
        % What this class does:
        % - Writes/loads a small Python module (floating_camera_py.py)
        % - Starts a Python thread that spins a ROS 2 executor (non-blocking)
        % - Lets you publish Twist messages to move a simulated/physical camera
        % - Pulls the latest camera image into MATLAB as a uint8 array
        % - Displays frames efficiently by reusing figure/image handles
        %
        % Typical usage:
        %   cam = FloatingCamera();
        %   cam.start();                         % start the Python ROS 2 node
        %   cam.publishTwist(0,0,0, 0,0,0.2);    % command a small yaw rate
        %   cam.show();                          % display the latest image
        %   cam.stop();                          % clean shutdown
            % Optionally specify a directory to store the python module.
            if nargin < 1 || isempty(pyDir)
                % Prefer the folder containing an existing Python module if present
                repoDir = fileparts(mfilename('fullpath'));
                repoPy = fullfile(repoDir, [obj.pyModuleName '.py']);
                if isfile(repoPy)
                    pyDir = repoDir;  % use the bundled python file directly
                else
                    pyDir = tempname; % fallback to a temp dir and write code there
                end
            end
            obj.pyModulePath = pyDir;
            if ~isfolder(obj.pyModulePath)
                % Constructor
                %
                % pyDir (optional): where to read/write the Python module.
                %   By default we prefer the repo folder if the .py exists there;
                %   otherwise a temp folder is created.
                % opts (optional): reserved for future options like topic names.
                mkdir(obj.pyModulePath);
            end

            % Ensure the Python module exists at the chosen location
            pyFile = fullfile(obj.pyModulePath, [obj.pyModuleName '.py']);
            if ~isfile(pyFile)
                code = obj.localPythonCode();
                fid = fopen(pyFile, 'w');
                assert(fid ~= -1, 'Could not create Python file');
                fwrite(fid, code);
                fclose(fid);
            end

            % Make sure Python can import from that folder
            try
                pth = py.sys.path; % Python list object
                % Convert Python list to MATLAB cell of char for comparison
                entries = cell(pth);
                for i = 1:numel(entries)
                    try
                        entries{i} = char(entries{i});
                    catch
                        entries{i} = char(string(entries{i}));
                    end
                end
                if ~any(strcmp(entries, obj.pyModulePath))
                    pth.insert(int32(0), obj.pyModulePath);
                end
            catch outerErr
                % Fallback: best effort insert without prior membership check
                try
                    pth = py.sys.path;
                    pth.insert(int32(0), obj.pyModulePath);
                catch innerErr
                    error('Failed to modify Python sys.path: %s', innerErr.message);
                end
            end
            py.importlib.invalidate_caches();

            % Import the module and build the wrapper (no args)
            pymod = py.importlib.import_module(obj.pyModuleName);
            % Ensure we pick up recent edits (important during development)
            try
                pymod = py.importlib.reload(pymod);
            catch
            end
            obj.pyWrapper = pymod.FloatingCameraWrapper();

            % Initialize plotting handles
            obj.hFig = [];
            obj.hAx = [];
            obj.hImg = [];
            obj.frameCount = 0;
            obj.displayTimer = [];
            obj.controlTimer = [];
            obj.unifiedTimer = [];
            obj.controlCallback = [];
            obj.displayCallback = [];
            obj.displayPeriodSec = [];
            obj.controlPeriodSec = [];
            obj.nextDisplayDue = [];
            obj.nextControlDue = [];

            % Quick sanity check for dependencies (optional)
            fprintf('[FloatingCamera] Python module loaded from: %s\n', obj.pyModulePath);
        end

        function start(obj)
            % Start spinning (non-blocking, Python thread)
            obj.pyWrapper.start();
            fprintf('[FloatingCamera] started.\n');
        end

        function stop(obj)
            % Clean shutdown
            obj.pyWrapper.stop();
            fprintf('[FloatingCamera] stopped.\n');
        end

        function tf = isRunning(obj)
            tf = logical(obj.pyWrapper.is_running());
        end

        function publishTwist(obj, lx, ly, lz, ax, ay, az)
            % Publish a manual twist command to the camera platform
            if nargin < 7, az = 0; end
            if nargin < 6, ay = 0; end
            if nargin < 5, ax = 0; end
            if nargin < 4, lz = 0; end
            if nargin < 3, ly = 0; end
            if nargin < 2, lx = 0; end
            obj.pyWrapper.publish_twist(lx, ly, lz, ax, ay, az);
        end

        function setPose(obj, x, y, z, qx, qy, qz, qw)
            % Set the camera pose directly
            % x, y, z: position
            % qx, qy, qz, qw: orientation quaternion (defaults to identity)
            if nargin < 8, qw = 1.0; end
            if nargin < 7, qz = 0.0; end
            if nargin < 6, qy = 0.0; end
            if nargin < 5, qx = 0.0; end
            if nargin < 4, z = 0.0; end
            if nargin < 3, y = 0.0; end
            if nargin < 2, x = 0.0; end
            obj.pyWrapper.set_pose(x, y, z, qx, qy, qz, qw);
        end

        function img = getImage(obj)
            % Get the current RGB image as a HxWx3 uint8 MATLAB array
            % Returns [] if no image has been received yet.
            tup = obj.pyWrapper.get_current_image_rgb_bytes();
            % Handle Python None or empty
            if isa(tup, 'py.NoneType') || isempty(tup)
                img = [];
                return;
            end
            % Convert Python tuple to MATLAB cell for indexing
            tcell = cell(tup);
            h = double(tcell{1}); w = double(tcell{2}); c = double(tcell{3});
            % Convert Python bytes to uint8
            bytes = uint8(py.array.array('B', tcell{4}));
            img = reshape(bytes, [h, w, c]);
        end

        function pose = getPose(obj)
            % Get the latest pose as a struct with position, orientation, stamp, frame_id
            % Returns [] if no pose has been received yet.
            ptuple = obj.pyWrapper.get_pose();
            if isa(ptuple,'py.NoneType') || isempty(ptuple)
                pose = [];
                return;
            end
            tc = cell(ptuple);
            pose.position = struct('x', double(tc{1}), 'y', double(tc{2}), 'z', double(tc{3}));
            pose.orientation = struct('x', double(tc{4}), 'y', double(tc{5}), 'z', double(tc{6}), 'w', double(tc{7}));
            pose.stamp = struct('sec', double(tc{8}), 'nanosec', double(tc{9}));
            % frame_id may be Python str; convert safely
            try
                pose.frame_id = char(tc{10});
            catch
                pose.frame_id = char(string(tc{10}));
            end
        end

    function show(obj, img)
            % Display an image using fast handle reuse.
            % Usage:
            %   show()     -> fetch latest image via getImage()
            %   show(img)  -> display provided image if non-empty
            %   show([])   -> fetch latest image via getImage()
            if nargin < 2 || isempty(img)
                img = obj.getImage();
            end

            if ~isprop(obj, 'hImg')
                % Fallback for stale class definitions: simple display path
                if isempty(img)
                    img = zeros(720,1280,3,'uint8');
                end
                figure('Name','Floating Camera View (MATLAB)');
                imshow(img);
                drawnow;
                return;
            end

            if isempty(img)
                % Show a placeholder only on first initialization
                if isempty(obj.hImg) || ~ishandle(obj.hImg)
                    placeholder = zeros(720,1280,3,'uint8');
                    obj.ensureFigure();
                    obj.hImg = imshow(placeholder, 'Parent', obj.hAx, ...
                        'InitialMagnification','fit','Border','tight');
                    title(obj.hAx, 'Waiting for camera images...');
                end
                obj.safeDrawnow();
                return;
            end

            obj.ensureFigure();
            if isempty(obj.hImg) || ~ishandle(obj.hImg)
                obj.hImg = imshow(img, 'Parent', obj.hAx, ...
                    'InitialMagnification','fit','Border','tight');
                title(obj.hAx, 'Floating Camera View (MATLAB)');
            else
                % Update only pixel data for speed
                set(obj.hImg, 'CData', img);
            end
            obj.frameCount = obj.frameCount + 1;
            % Avoid frequent title redraws; update every ~30 frames
            if mod(obj.frameCount, 30) == 1
                title(obj.hAx, 'Floating Camera View (MATLAB)');
            end
            obj.safeDrawnow();
        end

        function ensureFigure(obj)
            % Ensure a reusable figure/axes exist
            if isempty(obj.hFig) || ~ishandle(obj.hFig)
                obj.hFig = figure('Name','Floating Camera View (MATLAB)');
                obj.hAx = axes('Parent', obj.hFig);
                axis(obj.hAx, 'image'); axis(obj.hAx, 'off');
                set(obj.hAx,'Units','normalized','Position',[0 0 1 1],'LooseInset',[0 0 0 0]);
                try
                    set(obj.hFig,'SizeChangedFcn',@(s,e) set(obj.hAx,'Position',[0 0 1 1]));
                    set(obj.hFig,'CloseRequestFcn',@(src,evt) obj.onFigureClose(src,evt));
                catch
                end
            elseif isempty(obj.hAx) || ~ishandle(obj.hAx)
                obj.hAx = axes('Parent', obj.hFig);
                axis(obj.hAx, 'image'); axis(obj.hAx, 'off');
                set(obj.hAx,'Units','normalized','Position',[0 0 1 1],'LooseInset',[0 0 0 0]);
            else
                % Ensure axes still fills figure (e.g., after resize)
                set(obj.hAx,'Units','normalized','Position',[0 0 1 1]);
            end
        end

        function onFigureClose(obj, src, ~)
            % Gracefully stop timers and close the figure on user request
            try
                obj.stopDisplayTimer();
            catch
            end
            try
                obj.stopControlTimer();
            catch
            end
            try
                if isvalid(src)
                    delete(src);
                end
            catch
            end
            obj.hFig = [];
            obj.hAx = [];
            obj.hImg = [];
        end

        function fh = getFigureHandle(obj)
            % Public accessor to ensure and retrieve the figure handle
            obj.ensureFigure();
            fh = obj.hFig;
        end

        function stopAll(obj)
            % Stop timers and ROS node in one call
            try, obj.stopDisplayTimer(); catch, end
            try, obj.stopControlTimer(); catch, end
            try, obj.stopUnifiedTimer(); catch, end
            try, obj.stop(); catch, end
        end

        function safeDrawnow(~)
            % Use drawnow with rate limiting when available; swallow Ctrl-C interruptions
            try
                drawnow limitrate nocallbacks;
            catch
                try
                    drawnow;
                catch
                    % Ignore draw interruptions (e.g., Operation terminated by user)
                end
            end
        end

        function delete(obj)
            % Ensure cleanup if object is cleared
            try
                obj.stopDisplayTimer();
                obj.stopControlTimer();
                obj.stopUnifiedTimer();
            catch
            end
            try
                if ~isempty(obj.pyWrapper)
                    obj.stop();
                end
            catch
            end
        end

        function startUnifiedTimer(obj, displayPeriod, controlPeriod, displayCb, controlCb)
            % Start a single scheduler timer that services both display and control
            % displayPeriod, controlPeriod: seconds (set [] or 0 to disable one side)
            % displayCb: optional @(cam, img)
            % controlCb: optional @(cam, t)
            if nargin < 2, displayPeriod = []; end
            if nargin < 3, controlPeriod = []; end

            % Stop any existing timers to avoid conflicts BEFORE setting callbacks
            % to avoid clearing them in stopDisplayTimer
            obj.stopUnifiedTimer();
            obj.stopDisplayTimer();
            obj.stopControlTimer();

            if nargin >= 4 && ~isempty(displayCb)
                obj.displayCallback = displayCb;
            end
            if nargin >= 5 && ~isempty(controlCb)
                obj.controlCallback = controlCb;
            end

            % Normalize periods
            if isempty(displayPeriod) || displayPeriod <= 0
                obj.displayPeriodSec = [];
            else
                obj.displayPeriodSec = displayPeriod;
            end
            if isempty(controlPeriod) || controlPeriod <= 0
                obj.controlPeriodSec = [];
            else
                obj.controlPeriodSec = controlPeriod;
            end

            % (Timers already stopped above)

            % Determine scheduler period (fastest of the enabled ones)
            candidates = [];
            if ~isempty(obj.displayPeriodSec), candidates(end+1) = obj.displayPeriodSec; end %#ok<AGROW>
            if ~isempty(obj.controlPeriodSec), candidates(end+1) = obj.controlPeriodSec; end %#ok<AGROW>
            if isempty(candidates)
                error('At least one of displayPeriod or controlPeriod must be provided and > 0');
            end
            schedulerPeriod = min(candidates);

            % Initialize next-due times to their own periods to avoid cadence skew
            if ~isempty(obj.displayPeriodSec)
                obj.nextDisplayDue = obj.displayPeriodSec;
            else
                obj.nextDisplayDue = [];
            end
            if ~isempty(obj.controlPeriodSec)
                obj.nextControlDue = obj.controlPeriodSec;
            else
                obj.nextControlDue = [];
            end

            % Create and start the unified timer
            obj.unifiedTimer = timer('ExecutionMode','fixedRate', ...
                                     'Period', schedulerPeriod, ...
                                     'BusyMode','drop', ...
                                     'StartDelay', schedulerPeriod, ...
                                     'UserData', struct('t0', tic), ...
                                     'TimerFcn', @(tmr,~) obj.executeUnified(tmr));
            try
                start(obj.unifiedTimer);
                fprintf('[FloatingCamera] Unified timer started at %.1f Hz (control: %s, display: %s)\n', ...
                    1/schedulerPeriod, ...
                    obj.formatRate(obj.controlPeriodSec), obj.formatRate(obj.displayPeriodSec));
            catch err
                if contains(err.message, 'Operation terminated by user')
                    return;
                end
                rethrow(err);
            end
        end

        function stopUnifiedTimer(obj)
            % Stop and delete unified timer
            if ~isempty(obj.unifiedTimer) && isvalid(obj.unifiedTimer)
                stop(obj.unifiedTimer);
                delete(obj.unifiedTimer);
            end
            obj.unifiedTimer = [];
            obj.nextDisplayDue = [];
            obj.nextControlDue = [];
        end

    function startDisplayTimer(obj, period, callback)
            % Start a timer for periodic display updates at the specified rate
            % period: Timer period in seconds (e.g., 0.033 for ~30 Hz)
            % callback (optional): function handle @(cam, img) to render/show
            if nargin < 2 || isempty(period)
                period = 0.033; % default ~30 Hz
            end
            % Only update the callback if explicitly provided; otherwise
            % preserve any previously set display callback.
            if nargin >= 3 && ~isempty(callback)
                obj.displayCallback = callback;
            end
            
            obj.stopDisplayTimer(); % stop any existing timer
            
            obj.displayTimer = timer('ExecutionMode', 'fixedRate', ...
                                     'Period', period, ...
                                     'BusyMode', 'drop', ...
                                     'StartDelay', period, ...
                                     'TimerFcn', @(~,~)obj.updateDisplay());
            try
                start(obj.displayTimer);
                if isempty(obj.displayCallback)
                    fprintf('[FloatingCamera] Display timer started at %.1f Hz (no display callback set)\n', 1/period);
                else
                    fprintf('[FloatingCamera] Display timer started at %.1f Hz (with display callback)\n', 1/period);
                end
            catch err
                if contains(err.message, 'Operation terminated by user')
                    % Swallow user interrupt gracefully
                    return;
                end
                rethrow(err);
            end
        end

        function stopDisplayTimer(obj)
            % Stop and delete the display timer
            if ~isempty(obj.displayTimer) && isvalid(obj.displayTimer)
                stop(obj.displayTimer);
                delete(obj.displayTimer);
                obj.displayTimer = [];
            end
            % Preserve displayCallback; it may be managed by unified scheduler or reused later
        end

        function setDisplayCallback(obj, callback)
            % Set a custom display callback: function handle @(cam, img)
            if nargin < 2 || isempty(callback)
                obj.displayCallback = [];
            else
                validateattributes(callback, {'function_handle'}, {}, mfilename, 'callback');
                obj.displayCallback = callback;
            end
        end

        function cb = getDisplayCallback(obj)
            % Get the current display callback or [] if none
            cb = obj.displayCallback;
        end

        function startControlTimer(obj, period, callback)
            % Start a timer for periodic control loop execution
            % period: Timer period in seconds (e.g., 0.033 for ~30 Hz)
            % callback: Function handle that takes (obj, elapsedTime) as arguments
            %           Example: @(cam, t) cam.publishTwist(0,0,0, 0,0,sin(t))
            if nargin < 2 || isempty(period)
                period = 0.033; % default ~30 Hz
            end
            if nargin < 3 || isempty(callback)
                error('Control callback function is required');
            end
            
            obj.stopControlTimer(); % stop any existing timer
            obj.controlCallback = callback;
            
            % Store start time for elapsed time calculation
            obj.controlTimer = timer('ExecutionMode', 'fixedRate', ...
                                     'Period', period, ...
                                     'BusyMode', 'drop', ...
                                     'StartDelay', 0, ...
                                     'UserData', struct('t0', tic), ...
                                     'TimerFcn', @(tmr,~)obj.executeControl(tmr));
            try
                start(obj.controlTimer);
                fprintf('[FloatingCamera] Control timer started at %.1f Hz\n', 1/period);
            catch err
                if contains(err.message, 'Operation terminated by user')
                    % Swallow user interrupt gracefully
                    return;
                end
                rethrow(err);
            end
        end

        function s = formatRate(~, period)
            if isempty(period) || period <= 0
                s = 'off';
            else
                s = sprintf('%.1f Hz', 1/period);
            end
        end

        function stopControlTimer(obj)
            % Stop and delete the control timer
            if ~isempty(obj.controlTimer) && isvalid(obj.controlTimer)
                stop(obj.controlTimer);
                delete(obj.controlTimer);
                obj.controlTimer = [];
                obj.controlCallback = [];
            end
        end

        function updateDisplay(obj)
            % Timer callback: fetch and display the latest image
            img = obj.getImage();
            if isempty(img)
                return;
            end
            % Debug heartbeat (prints ~1/sec) and state of callback
            persistent lastPrint
            if isempty(lastPrint), lastPrint = tic; end
            if toc(lastPrint) > 1.0
                if isempty(obj.displayCallback)
                    fprintf('[FloatingCamera] display tick (no callback)\n');
                else
                    fprintf('[FloatingCamera] display tick (with callback)\n');
                end
                lastPrint = tic;
            end
            if ~isempty(obj.displayCallback)
                try
                    % Callback is responsible for calling obj.show() itself,
                    % possibly after drawing overlays on the image.
                    obj.displayCallback(obj, img);
                    return;
                catch err
                    % Swallow user-interrupt or drawing-related errors silently
                    if contains(err.message, 'Operation terminated by user')
                        return;
                    end
                    warning('Error in display callback: %s', err.message);
                end
            end
            % Fallback: show raw image
            try
                obj.show(img);
            catch err
                if contains(err.message, 'Operation terminated by user')
                    return;
                end
                warning('Error showing image: %s', err.message);
            end
        end

        function executeControl(obj, tmr)
            % Timer callback: execute user's control function
            if ~isempty(obj.controlCallback)
                try
                    t = toc(tmr.UserData.t0);
                    obj.controlCallback(obj, t);
                catch err
                    warning('Error in control callback: %s', err.message);
                end
            end
        end

        function executeUnified(obj, tmr)
            % Unified scheduler: call control/display callbacks at their own cadence
            % Determine elapsed time
            t = toc(tmr.UserData.t0);

            % Initialize next-due times on first tick relative to t
            if ~isempty(obj.displayPeriodSec) && (isempty(obj.nextDisplayDue) || obj.nextDisplayDue == 0)
                obj.nextDisplayDue = obj.displayPeriodSec; % first due at its own period
            end
            if ~isempty(obj.controlPeriodSec) && (isempty(obj.nextControlDue) || obj.nextControlDue == 0)
                obj.nextControlDue = obj.controlPeriodSec;
            end

            % Decide which callbacks are due (allow both in one tick)
            callControl = ~isempty(obj.controlCallback) && ~isempty(obj.controlPeriodSec) && (t + eps >= obj.nextControlDue);
            callDisplay = ~isempty(obj.displayCallback) && ~isempty(obj.displayPeriodSec) && (t + eps >= obj.nextDisplayDue);

            % Run control first so display can render most recent results in same tick
            if callControl
                try
                    obj.controlCallback(obj, t);
                catch err
                    warning('Error in control callback: %s', err.message);
                end
                % advance nextControlDue to the next boundary beyond t
                while obj.nextControlDue <= t
                    obj.nextControlDue = obj.nextControlDue + obj.controlPeriodSec;
                end
            end

            if callDisplay
                % Fetch image only when needed
                img = obj.getImage();
                if ~isempty(img)
                    try
                        obj.displayCallback(obj, img);
                    catch err
                        if ~contains(err.message, 'Operation terminated by user')
                            warning('Error in display callback: %s', err.message);
                        end
                    end
                    % Low-rate heartbeat for diagnostics (~1 Hz)
                    persistent lastDispPrint
                    if isempty(lastDispPrint), lastDispPrint = tic; end
                    if toc(lastDispPrint) > 1.0
                        fprintf('[FloatingCamera] unified display tick (with callback)\n');
                        lastDispPrint = tic;
                    end
                end
                while obj.nextDisplayDue <= t
                    obj.nextDisplayDue = obj.nextDisplayDue + obj.displayPeriodSec;
                end
            end
        end
    end

    methods (Access = private)
        function code = localPythonCode(~)
            % Return the exact Python source as a char vector (matches file above)
            % Try sibling .py first (kept in repo), then .pycode for inline copy
            thisDir = fileparts(mfilename('fullpath'));
            pyPath = fullfile(thisDir, 'floating_camera_py.py');
            if isfile(pyPath)
                code = fileread(pyPath);
                return;
            end

            pycodePath = mfilename('fullpath') + ".pycode";
            if isfile(pycodePath)
                code = fileread(pycodePath);
                return;
            end

         error(['No Python source found. Expected either "floating_camera_py.py" ' ...
             'beside this file or an inline "FloatingCamera.m.pycode" copy.']);
        end
    end
end

