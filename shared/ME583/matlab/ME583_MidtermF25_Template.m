% ME583 Midterm F25 - Template (MATLAB version)
% Uses MATLAB-Python interop to talk to the Floating Camera via
% ME583_Helpers_F25.FloatingCamClient, and does the rest purely in MATLAB.
%% 
% - Read/set pose and velocities via Python client
% - Read images from the simulator via Python client
% - Feature matching (by color), essential matrix, pose recovery, triangulation,
%   interaction matrix control loop, and drawing are implemented in MATLAB

clear all
close all
terminate(pyenv)
%pyenv("ExecutionMode","OutOfProcess","Version","/usr/bin/python3");


% Ensure Ctrl-C responsiveness and guaranteed cleanup via onCleanup
stopRequested = false;
cleanupObj = [];
try


  % 0) Ensure Python environment and import helper
  helper = ensure_python_and_import();

  % 1) Initialize ROS2 once and start the client executor thread
  if ~logical(py.rclpy.ok())
    py.rclpy.init(pyargs('args', py.None));
  end
  node = helper.FloatingCamClient();
  node.start();
  cleanupObj = onCleanup(@() local_cleanup(node));
  responsive_pause(0.5);

  % 2) Acquire goal image at the nominal origin pose
  origin_pos = [0.0, 0.0, -7.0];
  origin_rpy = [0.0, 0.0, 0.0];
  node.publish_pose_rpy(py.tuple(num2cell(origin_pos)), py.tuple(num2cell(origin_rpy)));
  responsive_pause(1.0);
  imgG = poll_image(node, 8.0); % non-blocking polling for responsiveness
  g_oc_goal = node.get_pose();

  % 3) Move to an initial offset pose and acquire current image
  initial_pos = [-1.0, -1.0, -10.0];
  % initial_rpy = [-pi/28, pi/20, 0.0];
  initial_rpy = [-pi/28, 0.0, pi/20];
  node.publish_pose_rpy(py.tuple(num2cell(initial_pos)), py.tuple(num2cell(initial_rpy)));
  responsive_pause(1.0);
  img = poll_image(node, 8.0);
  g_oc_initial = node.get_pose();

  % Save reference images for sanity checks
  imwrite(bgr2rgb(imgG), 'GoalImage.png');
  imwrite(bgr2rgb(img),  'CurrentImage.png');
  fprintf('Saved images: GoalImage.png, CurrentImage.png\n');

  % 4) Build camera intrinsics (match Python solution)
  [H, W, ~] = size(imgG);
  fprintf('Image size W x H = %d x %d\n', W, H);
  f = 0.035;               % meters
  sensor_width  = 0.032;   % meters
  sensor_height = sensor_width * 9.0/16.0;
  sx = W / sensor_width;
  sy = H / sensor_height;
  st = 0.0;
  ox = W/2.0; oy = H/2.0;
  K = K_intrinsic(f, sx, sy, st, ox, oy);
  Kinv = inv(K);

  % 5) Initial feature match and initial triangulation
  [xg0, x0] = findMatchedPointsByColor_pair(imgG, img);
  xG_pix0 = to_homog_xy(xg0).'; % 3xN
  x_pix0  = to_homog_xy(x0).';  % 3xN
  xG_cam0 = Kinv * xG_pix0;
  x_cam0  = Kinv * x_pix0;

  % Use the actual pose to triangulate
  g_co_goal = g_oc_goal.inv;
  g_goal_camera = py.operator.matmul(g_co_goal,g_oc_initial);

  g_oc_goal
  g_oc_initial
  g_goal_camera

  triangulationMethod = 'min';
  [pntsG, pnts] = triangulate_points_choice(xG_cam0, x_cam0, double(g_goal_camera.R.copy()), double(g_goal_camera.p.copy()), triangulationMethod);
  pntsG'
    
  % Prepare plotting
  %fig = figure('Name', 'ME583 Midterm P1 - Visual Servoing', 'Color','w');
  %tiledlayout(fig,2,2, 'Padding','compact','TileSpacing','compact');

  % 6) Main control loop
  maxIters = 300;
  convThresh = 1e-3;
  for k = 1:maxIters
    % Read latest image
    img = poll_image(node, 8.0);
    
    % Compute matched points between goal and current (by color)
    [xg, x] = findMatchedPointsByColor_pair(imgG, img);
    if isempty(xg) || isempty(x) || size(xg,1) ~= size(x,1)
      warning('No matched points found at iter %d; zeroing velocity briefly.', k);
      node.set_camera_velocity(py.tuple({0.0,0.0,0.0}), py.tuple({0.0,0.0,0.0}));
      pause(0.05);
      continue;
    end

    % Homogeneous pixel coords and normalized rays
    xG_pix = to_homog_xy(xg).'; % 3xN
    x_pix  = to_homog_xy(x).';  % 3xN
    xG_cam = Kinv * xG_pix;
    x_cam  = Kinv * x_pix;

    g_oc_current = node.get_pose();
    g_co_goal = g_oc_goal.inv;
    g_goal_camera = py.operator.matmul(g_co_goal, g_oc_current);
    
    % Triangulate 3D points (in camera-1, i.e., goal frame)
    % HINT: Use double() to convert from python numpy array to matlab array
    [pntsG, pnts] = triangulate_points_choice(xG_cam, x_cam, double(g_goal_camera.R.copy()), double(g_goal_camera.p.copy()), triangulationMethod);

    % TODO: Implement the controller
    Vb = zeros([6,1])
    
    

    % Send velocity for a short duration (velocity mode is continuous)
    node.set_camera_velocity(py.tuple(single(Vb(1:3))), ...
                             py.tuple(single(Vb(4:6))));
    % Let it integrate briefly and then stop
    % NOTE: In both Matlab and Python, this worked best for machines where
    % it couldn't keep up with a fixed velocity while processing the next
    % frame
    responsive_pause(0.05);
    node.set_camera_velocity(py.tuple({0.0,0.0,0.0}), py.tuple({0.0,0.0,0.0}));
  end

  % 7) Stop motion and capture the end image (for comparison to the Goal
  % image)
  node.set_camera_velocity(py.tuple({0.0,0.0,0.0}), py.tuple({0.0,0.0,0.0}));
  endimg = poll_image(node, 8.0);
  imwrite(bgr2rgb(endimg), 'EndImage.png');

  % 8) Cleanup happens via onCleanup

catch ME
  % On Ctrl-C or errors, ensure cleanup and rethrow
  try
    if ~isempty(cleanupObj)
      clear cleanupObj; %#ok<CLSCR>
    end
  catch
  end
  rethrow(ME);
end


% ----------------- Helper: ensure Python and import module -----------------
function helper = ensure_python_and_import()
  % Select the module directory (this file's folder)
  thisDir = fileparts(mfilename('fullpath'));
  % Ensure Python can import from this directory
  sys = py.importlib.import_module('sys');
  sysPath = cell(py.list(sys.path));
  if ~any(strcmp(sysPath, thisDir))
    sys.path.insert(int32(0), thisDir);
  end
  % Import the helper module
  helper = py.importlib.import_module('ME583_Helpers_F25');
end

% Non-blocking image polling that remains responsive to Ctrl-C/Stop
function img = poll_image(node, timeout)
  t0 = tic;
  img = [];
  while toc(t0) < timeout
    pyImg = node.try_get_image();
    if ~strcmp(class(pyImg), 'py.NoneType')
      img = pyimg_to_mat(pyImg);
      return;
    end
    drawnow limitrate;  % allow UI/Ctrl-C processing
    pause(0.02);
  end
  error('Timeout waiting for image');
end

% ----------------- Helper: convert Python/Numpy image to MATLAB ------------
function img = pyimg_to_mat(img_py)
  % Expect img_py as numpy ndarray uint8 HxWx3 in BGR order
  try
    % Modern MATLAB converts py.numpy.ndarray to MATLAB automatically
    img = uint8(img_py);
  catch
    % Fallback: convert through list (slower)
    np = py.importlib.import_module('numpy'); %#ok<NASGU>
    shape = cellfun(@int64, cell(py.tuple(img_py.shape)));
    flat = uint8(py.array.array('B', py.memoryview(img_py.tobytes())));
    img = reshape(flat, [shape(2), shape(1), max(1, prod(shape)/double(shape(1)*shape(2)))]);
    img = permute(img, [2,1,3]);
  end
  % Ensure 3 channels
  if size(img,3) == 1
    img = repmat(img, [1,1,3]);
  end
end

function rgb = bgr2rgb(bgr)
  rgb = bgr(:,:,[3,2,1]);
end

% Responsive pause that continues processing UI events and allows interrupts
function responsive_pause(dt)
  if nargin < 1 || isempty(dt), dt = 0; end
  t0 = tic;
  % Use small sleeps and drawnow to keep MATLAB responsive
  while toc(t0) < dt
    drawnow limitrate;
    pause(0.02);
  end
end

function local_cleanup(node)
  % Best-effort cleanup: stop motion, stop node, shutdown rclpy
  try
    node.set_camera_velocity(py.tuple({0.0,0.0,0.0}), py.tuple({0.0,0.0,0.0}));
  catch
  end
  try
    node.stop();
    node.destroy_node();
  catch
  end
  try
    if logical(py.rclpy.ok())
      py.rclpy.shutdown();
    end
  catch
  end
end

% ----------------- MATLAB implementation of Python utilities --------------

function K = K_intrinsic(f, sx, sy, st, ox, oy)
  Ks = [sx, st, ox; 0.0, sy, oy; 0.0, 0.0, 1.0];
  Kf = [f, 0.0, 0.0; 0.0, f, 0.0; 0.0, 0.0, 1.0];
  K = Ks * Kf;
end

function X = to_homog_xy(centers)
  % centers: Nx2 -> Nx3 homogeneous (x,y,1)
  if isempty(centers)
    X = zeros(0,3);
  else
    X = [centers, ones(size(centers,1),1)];
  end
end

% Feature detection by color matching (Connected Components + centroid color)
function [xg] = findMatchedPointsByColor_single(img)
  gray = rgb2gray(bgr2rgb(img));
  % Dual Otsu-like mask (bright and dark)
  bw1 = imbinarize(gray, 'global');
  bw2 = ~imbinarize(imcomplement(gray), 'global');
  bw = bw1 | bw2;
  bw = bwareaopen(bw, 12);
  bw = imclose(bw, strel('disk',2));
  bw = 1-bw;
  CC = bwconncomp(bw, 8);
  S = regionprops(CC, 'Centroid', 'Area', 'BoundingBox');
  % Filter by area/aspect roughly
  H = size(img,1); W = size(img,2);
  minArea = max(12, round(2e-5 * W * H));
  maxArea = round(0.25 * W * H);
  xg_list = [];
  for i = 1:numel(S)
    a = S(i).Area;
    if a < minArea || a > maxArea, continue; end
    bb = S(i).BoundingBox; asp = bb(3) / max(1,bb(4));
    if asp < 0.3 || asp > 2.5, continue; end
    xg_list(end+1,:) = S(i).Centroid; %#ok<AGROW>
  end
  % Sort by (y then x) for deterministic ordering
  if isempty(xg_list)
    xg = zeros(0,2);
  else
    [~, idx] = sortrows([xg_list(:,2), xg_list(:,1)]);
    xg = xg_list(idx, :);
  end
end

function [xgM, xM] = findMatchedPointsByColor_pair(imgG, img, xgHint)
  % Detect blobs in both, then match by centroid color proximity.
  if nargin < 3 || isempty(xgHint)
    xgM = findMatchedPointsByColor_single(imgG);
  else
    xgM = xgHint;
  end
  xM_candidates = findMatchedPointsByColor_single(img);
  if isempty(xgM) || isempty(xM_candidates)
    xM = zeros(0,2);
    return;
  end

  % Sample BGR at centroids
  colorsG = sample_bgr(imgG, xgM);
  colors  = sample_bgr(img,  xM_candidates);

  % Greedy nearest neighbor in color space
  xM = zeros(size(xgM));
  for i = 1:size(colors,1)
    d = vecnorm(single(colorsG) - single(colors(i,:)), 2, 2);
    [~, idx] = min(d);
    xM(idx, :) = xM_candidates(i, :);
  end
end

function C = sample_bgr(img, pts)
  % pts Nx2 (x,y) -> colors Nx3 BGR
  H = size(img,1); W = size(img,2);
  xy = round(pts);
  xy(:,1) = min(max(xy(:,1),1), W);
  xy(:,2) = min(max(xy(:,2),1), H);
  ind = sub2ind([H,W], xy(:,2), xy(:,1));
  B = img(:,:,1); G = img(:,:,2); R = img(:,:,3);
  C = [B(ind), G(ind), R(ind)];
end

% Algebraic Essential matrix from normalized points (3xN each)
function E = essential_algebraic(x1, x2)
  N = size(x1,2);
  A = zeros(N, 9);
  for i = 1:N
    a = kron(x1(:,i).', x2(:,i).');
    A(i,:) = a;
  end
  [~,~,V] = svd(A, 0);
  Ea = reshape(V(:,end), [3,3]);
  [U,~,Vt] = svd(Ea);
  D = diag([1,1,0]);
  E = U * D * Vt';
end

% Recover motion from E with cheirality test
function [R12, T12] = motion_from_E(E, x1, x2)
  [U,~,V] = svd(E);
  if det(U) < 0, U(:,3) = -U(:,3); end
  if det(V) < 0, V(:,3) = -V(:,3); end
  W = [0 -1 0; 1 0 0; 0 0 1];
  R1 = U*W*V'; if det(R1) < 0, R1 = -R1; end
  R2 = U*W'*V'; if det(R2) < 0, R2 = -R2; end
  u3 = U(:,3);
  candidates = {
    R1,  u3;
    R1, -u3;
    R2,  u3;
    R2, -u3;
  };
  bestScore = -inf; R12 = eye(3); T12 = [1;0;0];
  for i = 1:size(candidates,1)
    Rc = candidates{i,1}; tc = candidates{i,2};
    try
      [X1, X2] = triangulate_points_algebraic(x1, x2, Rc, tc);
      ok1 = X1(3,:) > 0; ok2 = X2(3,:) > 0; score = sum(ok1 & ok2);
      if score > bestScore
        bestScore = score; R12 = Rc; T12 = tc;
      end
    catch
      % continue
    end
  end
end

% Linear-DLT triangulation per point (camera-1 frame)
function [X1, X2] = triangulate_points_algebraic(x1, x2, R12, T12)
  N = size(x1,2);
  X1 = zeros(3, N);
  P1 = [eye(3), zeros(3,1)];
  P2 = [R12, T12(:)];
  for i = 1:N
    u1 = x1(1,i); v1 = x1(2,i); w1 = x1(3,i);
    u2 = x2(1,i); v2 = x2(2,i); w2 = x2(3,i); %#ok<NASGU>
    A = zeros(4,4);
    A(1,:) = u1*P1(3,:) - P1(1,:);
    A(2,:) = v1*P1(3,:) - P1(2,:);
    A(3,:) = u2*P2(3,:) - P2(1,:);
    A(4,:) = v2*P2(3,:) - P2(2,:);
    [~,~,V] = svd(A);
    Xh = V(:,end);
    X1(:,i) = Xh(1:3) ./ Xh(4);
  end
  R21 = R12.'; T21 = -R12.' * T12(:);
  X2 = R21 * (X1 - T12(:));
end

% Interaction matrix for a set of 3D points (each column X = [X,Y,Z])
function L = Lmat(X)
  N = size(X,2);
  L = zeros(2*N, 6);
  for i = 1:N
    x = X(1,i)/X(3,i);
    y = X(2,i)/X(3,i);
    Z = X(3,i);
    Li = [ -1/Z,    0,  x/Z,  x*y, -(1+x^2),   y;
             0,  -1/Z,  y/Z, 1+y^2,   -x*y,  -x];
    L(2*i-1:2*i, :) = Li;
  end
end

% ----------------- Triangulation via minimization (fminsearch) -------------

function [X1, X2] = triangulate_points_min(x1m, x2m, R12, T12, X0)
  % x1m, x2m: 3xN normalized image coordinates (last row ~ 1)
  % R12, T12: transform from cam1 to cam2 (X2 = R12*X1 + T12)
  % Returns X1, X2 as 3xN points in cam1 and cam2 frames respectively.
  N = size(x1m, 2);
  if nargin < 5 || isempty(X0)
    % Initialize along rays with unit depth
    X0 = zeros(3*N,1);
    for i = 1:N
      d = x1m(:, i);
      if abs(d(3)) < 1e-9
        d(3) = 1.0;
      end
      di = d ./ d(3);
      X0(3*i-2:3*i) = di;
    end
  else
    X0 = X0(:);
  end

  % Cost function
  cost = @(x) triangulation_cost_min(x, x1m, x2m, R12, T12);
  opts = optimset('Display','off', 'MaxIter', 1000, 'MaxFunEvals', 5000, 'TolX',1e-6, 'TolFun',1e-8);
  xopt = fminsearch(cost, X0, opts);

  X1 = reshape(xopt, [3, N]);
  R21 = R12.';
  X2 = R21 * (X1 - T12(:));
end

function f = triangulation_cost_min(Xflat, x1m, x2m, R12, T12)
  % Sum of squared reprojection errors in normalized coordinates for both cameras
  N = size(x1m, 2);
  X1 = reshape(Xflat, [3, N]);
  % Project in cam1 and cam2
  xp1 = X1 ./ X1(3, :);
  R21 = R12.';
  X2 = R21 * (X1 - T12(:));
  xp2 = X2 ./ X2(3, :);

  r1 = x1m - xp1;
  r2 = x2m - xp2;
  f = sum(r1(:).^2) + sum(r2(:).^2);

  % Soft penalties to enforce positive depth
  z1 = X1(3, :);
  z2 = X2(3, :);
  pen = 0.0;
  pen = pen + sum((z1 <= 1e-4) .* (1e3 + (1e-2 - max(0, z1)).^2));
  pen = pen + sum((z2 <= 1e-4) .* (1e3 + (1e-2 - max(0, z2)).^2));
  f = f + pen;
end

% Convenience selector: method = 'algebraic' or 'min'
function [X1, X2] = triangulate_points_choice(x1m, x2m, R12, T12, method)
  if nargin < 5 || isempty(method)
    method = 'algebraic';
  end
  switch lower(string(method))
    case "min"
      [X1, X2] = triangulate_points_min(x1m, x2m, R12, T12);
    otherwise
      [X1, X2] = triangulate_points_algebraic(x1m, x2m, R12, T12);
  end
end
