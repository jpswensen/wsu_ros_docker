% FloatingCameraExample
% Example script demonstrating the capture-first control loop with the
% FloatingCamera MATLAB wrapper (no MATLAB ROS Toolbox required).
%
% Loop structure each iteration:
%   1) Read the latest pose:    pose = cam.getPose();
%   2) Read the latest image:   img  = cam.getImage();
%   3) Compute and publish control (e.g., yaw rate): cam.publishTwist(...)
%   4) Visualize the image:     cam.show(img);

% Optionally ensure MATLAB uses your ROS 2 Python (adjust if needed):
% pyenv('Version','/usr/bin/python3');

% Create and start the camera client. This will import the Python module,
% create a ROS 2 node and executor, and start spinning in a background thread.
cam = FloatingCamera();
cam.start();

cleanupObj = onCleanup(@() tryStop(cam));

% Move in a gentle yaw while plotting frames for ~10 seconds
T = 10; dt = 1/30.0; t0 = tic;
while toc(t0) < T
    t = toc(t0);

    % 1) Get the current pose (if available)
    pose = cam.getPose();
    if ~isempty(pose)
        fprintf('t=%.2fs pos=(%.2f, %.2f, %.2f) orient=(%.2f, %.2f, %.2f, %.2f)\n', ...
            t, pose.position.x, pose.position.y, pose.position.z, ...
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    end

    % 2) Get the current image
    img = cam.getImage();

    % 3) Compute a control input (example: gentle sinusoidal yaw rate)
    yaw_rate = 0.5 * sin(2*pi*0.1*t);
    cam.publishTwist(0,0,0, 0,0,yaw_rate);

    % 4) Show the image (if empty, show() will fetch the latest itself)
    cam.show(img);
    
    % Alternatively, to speed things up, save images to disk instead of displaying
    %img_fn = sprintf('imgs/frame_%04d.png', round(t/dt));
    %if ~isempty(img)
    %    imwrite(img, img_fn);
    %    %fprintf('Saved image to %s\n', img_fn);
    %end

    pause(dt);
end


% Stop the rotation
%cam.publishTwist(0,0,0, 0,0,0);

% Stop the node
cam.stop();

function tryStop(cam)
try
    if isvalid(cam)
        cam.stop();
    end
catch
end
end
