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

        function safeDrawnow(~)
            % Use drawnow with rate limiting when available
            try
                drawnow limitrate nocallbacks;
            catch
                drawnow;
            end
        end

        function delete(obj)
            % Ensure cleanup if object is cleared
            try
                if ~isempty(obj.pyWrapper)
                    obj.stop();
                end
            catch
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

