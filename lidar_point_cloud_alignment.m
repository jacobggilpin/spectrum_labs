function results = lidar_point_cloud_alignment(varargin)
%LIDAR_POINT_CLOUD_ALIGNMENT Multi-Camera Intel RealSense L515 3D Point Cloud
%Calibration, Registration, and Merging Pipeline.
%
%   Physical Hardware Geometry:
%   ---------------------------
%   - Frame Structure : Unfilled wooden equilateral triangular prism
%   - Frame Height    : ~6.0 ft (1.83 m) vertical legs
%   - Triangle Side   : ~3.5 ft (1.07 m) equilateral triangle (circumradius ~0.62 m)
%   - Camera Spacing  : 3 Intel RealSense L515 LiDAR cameras mounted 120 deg apart
%                       (Camera 1: 0 deg, Camera 2: +120 deg, Camera 3: +240 / -120 deg)
%
%   Pipeline Modes:
%   ---------------
%   1. 'calibrate' : Loads 3 calibration point clouds of a simple reference target
%                    (e.g., dense_box_angle_0/120/240.ply or example_box_angle_0/120/240.ply),
%                    levels the ground plane and shifts the bottom floor surface to Z = 0,
%                    crops with an expanded ROI ([-0.85, 0.85] m in X/Y so rotated corners at
%                    +/-0.683 m are never clipped), applies inverse 120-deg frame rotations,
%                    segments the calibration target using an adaptive Z-height threshold,
%                    computes coarse-to-fine ICP + Rigid CPD transformations, and saves
%                    the calibration struct to 'calibration_data.mat'.
%   2. 'align'     : Loads saved calibration transformations ('calibration_data.mat'),
%                    applies each camera's specific base calibration adjustments to a
%                    new 3-camera scan (e.g., f1.ply, f2.ply, f3.ply), isolates the target
%                    object, performs secondary fine ICP alignment to refine the model,
%                    and merges all three views into a high-density 3D point cloud.
%   3. 'both'      : Runs 'calibrate' followed immediately by 'align' (Default).
%   4. 'capture'   : Captures live point clouds from 3 connected Intel RealSense L515
%                    cameras via the librealsense MATLAB wrapper, then runs alignment.

    %% 1. Parse Configuration Parameters
    cfg = parseInputs(varargin{:});
    fprintf('====================================================================\n');
    fprintf('  Intel RealSense L515 3-Camera Point Cloud Registration Pipeline\n');
    fprintf('  Operating Mode : %s\n', upper(cfg.Mode));
    fprintf('====================================================================\n');

    results = struct();

    %% 2. Optional Live Hardware Capture Mode
    if strcmpi(cfg.Mode, 'capture')
        fprintf('\n[Step 0] Capturing live point clouds from Intel RealSense L515 cameras...\n');
        capturedFiles = capture_point_clouds(cfg);
        cfg.TargetFiles = capturedFiles;
    end

    %% 3. Execute Calibration Mode
    if any(strcmpi(cfg.Mode, {'calibrate', 'both'}))
        fprintf('\n[Phase 1: CALIBRATION MODE] Computing base spatial transformations...\n');
        calibrationData = runCalibrationMode(cfg);
        save(cfg.CalibrationFile, 'calibrationData');
        fprintf('  -> Saved calibration parameters to: %s\n', cfg.CalibrationFile);
        results.calibrationData = calibrationData;
    end

    %% 4. Execute Alignment Mode
    if any(strcmpi(cfg.Mode, {'align', 'both', 'capture'}))
        fprintf('\n[Phase 2: ALIGNMENT MODE] Applying base calibration + secondary ICP...\n');
        if ~isfield(results, 'calibrationData')
            if isfile(cfg.CalibrationFile)
                loaded = load(cfg.CalibrationFile, 'calibrationData');
                calibrationData = loaded.calibrationData;
                fprintf('  -> Loaded existing calibration file: %s\n', cfg.CalibrationFile);
            else
                error('Calibration file "%s" not found. Run in ''calibrate'' or ''both'' mode first.', ...
                    cfg.CalibrationFile);
            end
        end

        alignmentResults = runAlignmentMode(cfg, calibrationData);
        results.alignmentResults = alignmentResults;

        if ~isempty(cfg.OutputMergedPly)
            pcwrite(alignmentResults.mergedCloudRefined, cfg.OutputMergedPly, 'Encoding', 'binary');
            fprintf('  -> Exported final merged 3D model to: %s (%d points)\n', ...
                cfg.OutputMergedPly, alignmentResults.mergedCloudRefined.Count);
        end
    end

    fprintf('\nPipeline execution complete.\n');
end

%% ========================================================================
%  CALIBRATION MODE PIPELINE
%  ========================================================================
function calibrationData = runCalibrationMode(cfg)
    numCams = 3;
    rawCalibClouds     = cell(1, numCams);
    tformGround        = cell(1, numCams);
    groundLeveled      = cell(1, numCams);
    croppedClouds      = cell(1, numCams);
    tformNominal       = cell(1, numCams);
    nominalRotated     = cell(1, numCams);
    groundFiltered     = cell(1, numCams);
    largestClusters    = cell(1, numCams);
    zThresholdsUsed    = zeros(1, numCams);

    groundRef = createReferenceGroundPlane(cfg.GroundXLim, cfg.GroundYLim, cfg.GroundGridSize);

    for i = 1:numCams
        if ~isfile(cfg.CalibFiles{i})
            error('Calibration file not found: %s', cfg.CalibFiles{i});
        end
        rawCalibClouds{i} = pcread(cfg.CalibFiles{i});
        fprintf('  [Calib Cam %d] Loaded "%s" (%d points)\n', ...
            i, cfg.CalibFiles{i}, rawCalibClouds{i}.Count);

        if isempty(rawCalibClouds{i}.Normal) && rawCalibClouds{i}.Count >= 10
            rawCalibClouds{i}.Normal = pcnormals(rawCalibClouds{i}, min(15, rawCalibClouds{i}.Count));
        end

        % 2. Ground Plane Alignment + Floor Z-Shift to Z = 0
        tformGround{i} = computeGroundTransformation(rawCalibClouds{i}, groundRef, cfg);
        groundLeveled{i} = pctransform(rawCalibClouds{i}, tformGround{i});

        % 3. Region-of-Interest (ROI) Spatial Cropping ([-0.85, 0.85] preserves rotated corners)
        croppedClouds{i} = processPtCld(groundLeveled{i}, cfg.CropROI);

        % 4. Nominal Azimuth Rotation (0 deg, -120 deg, +120 deg around Z-axis)
        rotAnglesDeg = [0, 0, cfg.NominalYawAnglesDeg(i)];
        tformNominal{i} = rigidtform3d(rotAnglesDeg, [0, 0, 0]);
        nominalRotated{i} = pctransform(croppedClouds{i}, tformNominal{i});

        % 5. Adaptive Ground Removal Relative to Z = 0 Floor (maxZ / X)
        [groundFiltered{i}, zThresholdsUsed(i)] = removeGroundAdaptive( ...
            nominalRotated{i}, cfg.GroundHeightDivisor, cfg.MinGroundThreshold);

        % 6. Largest Cluster Segmentation (Isolating Calibration Object)
        largestClusters{i} = extractLargestCluster(groundFiltered{i}, cfg.ClusterMaxDistance);
        fprintf('  [Calib Cam %d] Segmented calibration cluster: %d points (Z-threshold = %.4f m)\n', ...
            i, largestClusters{i}.Count, zThresholdsUsed(i));
    end

    % 7. Multi-View Coarse ICP (planeToPlane) + Fine Rigid CPD Registration to Camera 1
    tformCalibICP = cell(1, numCams);
    tformCalibCPD = cell(1, numCams);
    rmseCalibICP  = zeros(1, numCams);
    alignedClustersICP = cell(1, numCams);
    alignedClustersCPD = cell(1, numCams);

    tformCalibICP{1} = rigidtform3d([0, 0, 0], [0, 0, 0]);
    tformCalibCPD{1} = rigidtform3d([0, 0, 0], [0, 0, 0]);
    alignedClustersICP{1} = largestClusters{1};
    alignedClustersCPD{1} = largestClusters{1};

    refCluster = prepareCloudForICP(largestClusters{1});

    for i = 2:numCams
        movingCluster = prepareCloudForICP(largestClusters{i});

        [tformICP, movingRegICP, rmseICP] = pcregistericp( ...
            movingCluster, refCluster, ...
            'Metric', cfg.ICPMetric, ...
            'InlierRatio', cfg.CalibICPInlierRatio, ...
            'MaxIterations', cfg.CalibICPMaxIterations);

        tformCalibICP{i} = tformICP;
        rmseCalibICP(i)  = rmseICP;
        alignedClustersICP{i} = pctransform(largestClusters{i}, tformICP);
        fprintf('  [Calib Cam %d -> Cam 1] Coarse ICP (%s) RMSE: %.5f m\n', ...
            i, cfg.ICPMetric, rmseICP);

        if cfg.UseCPDInCalibration
            movingForCPD = downsampleIfLarge(movingRegICP, cfg.CPDMaxPoints);
            refForCPD    = downsampleIfLarge(refCluster, cfg.CPDMaxPoints);
            tformCPD = pcregistercpd(movingForCPD, refForCPD, ...
                'Transform', 'Rigid', ...
                'MaxIterations', cfg.CPDMaxIterations);
            tformCalibCPD{i} = tformCPD;
            alignedClustersCPD{i} = pctransform(alignedClustersICP{i}, tformCPD);
            fprintf('  [Calib Cam %d -> Cam 1] Completed Rigid CPD refinement.\n', i);
        else
            tformCalibCPD{i} = rigidtform3d([0, 0, 0], [0, 0, 0]);
            alignedClustersCPD{i} = alignedClustersICP{i};
        end
    end

    mergedCalib12  = pcmerge(alignedClustersCPD{1}, alignedClustersCPD{2}, cfg.MergeGridStep);
    mergedCalib123 = pcmerge(mergedCalib12, alignedClustersCPD{3}, cfg.MergeGridStep);

    calibrationData = struct();
    calibrationData.timestamp          = datetime('now');
    calibrationData.calibFiles         = cfg.CalibFiles;
    calibrationData.nominalYawAngles   = cfg.NominalYawAnglesDeg;
    calibrationData.cropROI            = cfg.CropROI;
    calibrationData.groundHeightDivisor= cfg.GroundHeightDivisor;
    calibrationData.zThresholdsUsed    = zThresholdsUsed;
    calibrationData.tformGround        = tformGround;
    calibrationData.tformNominal       = tformNominal;
    calibrationData.tformCalibICP      = tformCalibICP;
    calibrationData.tformCalibCPD      = tformCalibCPD;
    calibrationData.rmseCalibICP       = rmseCalibICP;
    calibrationData.largestClusters    = largestClusters;
    calibrationData.alignedClusters    = alignedClustersCPD;
    calibrationData.mergedCalibCloud   = mergedCalib123;

    if cfg.ShowPlots
        plotCalibrationDiagnostics(calibrationData);
    end
end

%% ========================================================================
%  ALIGNMENT MODE PIPELINE (BASE CALIBRATION + FURTHER ICP REFINEMENT)
%  ========================================================================
function alignmentResults = runAlignmentMode(cfg, calibrationData)
    numCams = 3;
    rawTargetClouds   = cell(1, numCams);
    baseAlignedFull   = cell(1, numCams);
    baseAlignedObject = cell(1, numCams);
    fineAlignedObject = cell(1, numCams);
    tformFineICP      = cell(1, numCams);
    rmseFineICP       = zeros(1, numCams);

    for i = 1:numCams
        if ~isfile(cfg.TargetFiles{i})
            error('Target scan file not found: %s', cfg.TargetFiles{i});
        end
        rawTargetClouds{i} = pcread(cfg.TargetFiles{i});
        fprintf('  [Align Cam %d] Loaded target scan "%s" (%d points)\n', ...
            i, cfg.TargetFiles{i}, rawTargetClouds{i}.Count);

        if cfg.RecomputeGroundInAlignMode
            groundRef = createReferenceGroundPlane(cfg.GroundXLim, cfg.GroundYLim, cfg.GroundGridSize);
            tGround = computeGroundTransformation(rawTargetClouds{i}, groundRef, cfg);
        else
            tGround = calibrationData.tformGround{i};
        end
        pcGroundLeveled = pctransform(rawTargetClouds{i}, tGround);

        pcCropped = processPtCld(pcGroundLeveled, cfg.CropROI);
        pcNominal = pctransform(pcCropped, calibrationData.tformNominal{i});

        pcAfterCalibICP = pctransform(pcNominal, calibrationData.tformCalibICP{i});
        pcAfterCalibCPD = pctransform(pcAfterCalibICP, calibrationData.tformCalibCPD{i});
        baseAlignedFull{i} = pcAfterCalibCPD;

        [pcNoGround, zThresh] = removeGroundAdaptive( ...
            pcAfterCalibCPD, cfg.GroundHeightDivisor, cfg.MinGroundThreshold);
        baseAlignedObject{i} = extractLargestCluster(pcNoGround, cfg.ClusterMaxDistance);
        fprintf('  [Align Cam %d] Base-aligned object segmented: %d points (Z-thresh = %.4f m)\n', ...
            i, baseAlignedObject{i}.Count, zThresh);
    end

    tformFineICP{1}      = rigidtform3d([0, 0, 0], [0, 0, 0]);
    fineAlignedObject{1} = baseAlignedObject{1};
    refTarget = prepareCloudForICP(baseAlignedObject{1});

    for i = 2:numCams
        movingTarget = prepareCloudForICP(baseAlignedObject{i});

        [tFine, ~, rmseFine] = pcregistericp( ...
            movingTarget, refTarget, ...
            'Metric', cfg.ICPMetric, ...
            'InlierRatio', cfg.AlignICPInlierRatio, ...
            'MaxIterations', cfg.AlignICPMaxIterations, ...
            'Tolerance', [1e-4, 1e-4]);

        tformFineICP{i}      = tFine;
        rmseFineICP(i)       = rmseFine;
        fineAlignedObject{i} = pctransform(baseAlignedObject{i}, tFine);
        fprintf('  [Align Cam %d -> Cam 1] Secondary Fine ICP RMSE: %.5f m\n', i, rmseFine);
    end

    mergedBase12  = pcmerge(baseAlignedObject{1}, baseAlignedObject{2}, cfg.MergeGridStep);
    mergedBase123 = pcmerge(mergedBase12, baseAlignedObject{3}, cfg.MergeGridStep);

    mergedFine12  = pcmerge(fineAlignedObject{1}, fineAlignedObject{2}, cfg.MergeGridStep);
    mergedFine123 = pcmerge(mergedFine12, fineAlignedObject{3}, cfg.MergeGridStep);

    alignmentResults = struct();
    alignmentResults.targetFiles        = cfg.TargetFiles;
    alignmentResults.baseAlignedFull    = baseAlignedFull;
    alignmentResults.baseAlignedObject  = baseAlignedObject;
    alignmentResults.fineAlignedObject  = fineAlignedObject;
    alignmentResults.tformFineICP       = tformFineICP;
    alignmentResults.rmseFineICP       = rmseFineICP;
    alignmentResults.mergedCloudBase    = mergedBase123;
    alignmentResults.mergedCloudRefined = mergedFine123;

    if cfg.ShowPlots
        plotAlignmentDiagnostics(alignmentResults);
    end
end

%% ========================================================================
%  INTEL REALSENSE L515 MULTI-CAMERA CAPTURE MODULE
%  ========================================================================
function savedFilenames = capture_point_clouds(cfg)
    ctx = realsense.context();
    devices = ctx.query_devices();
    num_cameras = length(devices);

    if num_cameras < 3
        warning('Expected 3 Intel RealSense L515 cameras, but detected %d device(s).', num_cameras);
    end

    pipelines      = cell(1, num_cameras);
    configs        = cell(1, num_cameras);
    serials        = cell(1, num_cameras);
    savedFilenames = cell(1, num_cameras);

    w = cfg.StreamWidth;
    h = cfg.StreamHeight;
    framerate = cfg.StreamFPS;
    value_min = cfg.DepthMinMeters;
    value_max = cfg.DepthMaxMeters;

    colorizer = realsense.colorizer();
    colorizer.set_option(realsense.option.visual_preset, 1);
    colorizer.set_option(realsense.option.min_distance, value_min);
    colorizer.set_option(realsense.option.max_distance, value_max);

    threshold_filter = realsense.threshold_filter(value_min, value_max);
    spatial_filter   = realsense.spatial_filter(0.4, 40.0, 4.0, 1.0);
    temporal_filter  = realsense.temporal_filter(0.1, 80.0, 6);

    for i = 1:num_cameras
        pipelines{i} = realsense.pipeline(ctx);
        configs{i}   = realsense.config();
        serials{i}   = devices{i}.get_info(realsense.camera_info.serial_number);
        configs{i}.enable_device(serials{i});
        configs{i}.enable_stream(realsense.stream.depth, w, h, realsense.format.z16, framerate);
        configs{i}.enable_stream(realsense.stream.color, w, h, realsense.format.bgr8, framerate);
        pipelines{i}.start(configs{i});
        fprintf('  Started Intel RealSense L515 Camera %d (S/N: %s)\n', i, serials{i});
    end

    numWarmupFrames = 15;
    for i = 1:num_cameras
        fprintf('  Capturing & filtering frames from Camera %d...\n', i);
        for f = 1:numWarmupFrames
            frameset = pipelines{i}.wait_for_frames();
            depth_frame = frameset.get_depth_frame();
            filtered_frame = threshold_filter.process(depth_frame);
            filtered_frame = spatial_filter.process(filtered_frame);
            filtered_frame = temporal_filter.process(filtered_frame);
        end
        color_frame = frameset.get_color_frame();

        pointcloud = realsense.pointcloud();
        if color_frame.logical()
            pointcloud.map_to(color_frame);
            points = pointcloud.calculate(filtered_frame);
            filename = sprintf('camera%d.ply', i);
            points.export_to_ply(filename, color_frame);
        else
            filename = sprintf('camera%d.ply', i);
            pointcloud.export_to_ply(filename, filtered_frame);
        end
        savedFilenames{i} = filename;
        fprintf('  Saved %s\n', filename);
    end

    for i = 1:num_cameras
        pipelines{i}.stop();
    end
    fprintf('  All %d RealSense L515 cameras stopped cleanly.\n', num_cameras);
end

%% ========================================================================
%  GEOMETRIC & POINT CLOUD PROCESSING HELPER FUNCTIONS
%  ========================================================================
function ptCldOut = processPtCld(ptCldIn, roiLimits)
    if ptCldIn.Count == 0
        ptCldOut = ptCldIn;
        return;
    end
    inlierIdx = findPointsInROI(ptCldIn, roiLimits);
    if isempty(inlierIdx)
        warning('ROI crop returned 0 points; retaining uncropped point cloud.');
        ptCldOut = ptCldIn;
    else
        ptCldOut = select(ptCldIn, inlierIdx);
    end
end

function ground = createReferenceGroundPlane(xLim, yLim, gridSize)
    [x, y] = meshgrid(xLim(1):gridSize:xLim(2), yLim(1):gridSize:yLim(2));
    z = zeros(size(x));
    pts = single([x(:), y(:), z(:)]);
    normals = repmat(single([0, 0, 1]), size(pts, 1), 1);
    ground = pointCloud(pts, 'Normal', normals);
end

function tformGround = computeGroundTransformation(ptCloudIn, groundRef, cfg)
    if strcmpi(cfg.GroundAlignmentMethod, 'ransacPlane')
        [planeModel, inlierIdx] = pcfitplane(ptCloudIn, 0.02, [0, 0, 1], 45);
        n = double(planeModel.Normal(:)');
        if n(3) < 0
            n = -n;
        end
        targetN = [0, 0, 1];
        v = cross(n, targetN);
        s = norm(v);
        c = dot(n, targetN);
        if s < 1e-7
            R = eye(3);
        else
            vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
            R = eye(3) + vx + (vx * vx) * ((1 - c) / (s^2));
        end
        inlierCloud = select(ptCloudIn, inlierIdx);
        centroid = double(mean(inlierCloud.Location, 1));
        t = -centroid * R';
        tformGround = rigidtform3d(R, t);
    else
        downsampled = pcdownsample(ptCloudIn, 'gridAverage', cfg.GroundGridSize);
        if isempty(downsampled.Normal) && downsampled.Count >= 10
            downsampled.Normal = pcnormals(downsampled, min(15, downsampled.Count));
        end
        tformICP = pcregistericp(downsampled, groundRef, 'Metric', 'pointToPlane');
        pcTemp = pctransform(ptCloudIn, tformICP);

        % Shift lowest surface to Z = 0 so floor removal never slices mid-object
        zFloor = double(min(pcTemp.Location(:, 3)));
        T_shift = eye(4);
        T_shift(3, 4) = -zFloor;
        tformGround = rigidtform3d(T_shift * tformICP.A);
    end
end

function [ptCloudOut, threshold] = removeGroundAdaptive(ptCloudIn, heightDivisor, minThreshold)
    if ptCloudIn.Count == 0
        ptCloudOut = ptCloudIn;
        threshold = minThreshold;
        return;
    end

    zCoords = ptCloudIn.Location(:, 3);
    zFloor  = double(min(zCoords));
    maxZRel = double(max(zCoords)) - zFloor;
    threshold = max(minThreshold, maxZRel / heightDivisor);

    aboveGroundIdx = find((zCoords - zFloor) > threshold);
    if isempty(aboveGroundIdx)
        warning('Adaptive ground threshold (%.4f m) removed all points; retaining cloud.', threshold);
        ptCloudOut = ptCloudIn;
    else
        ptCloudOut = select(ptCloudIn, aboveGroundIdx);
    end
end

function largestClusterCloud = extractLargestCluster(ptCloudIn, maxDistance)
    if ptCloudIn.Count == 0
        largestClusterCloud = ptCloudIn;
        return;
    end

    [labels, numClusters] = pcsegdist(ptCloudIn, maxDistance);
    validMask = labels > 0;
    if ~any(validMask) || numClusters == 0
        largestClusterCloud = ptCloudIn;
        return;
    end

    counts = histcounts(labels(validMask), 1:(numClusters + 1));
    [~, maxClusterIdx] = max(counts);
    clusterIndices = find(labels == maxClusterIdx);
    largestClusterCloud = select(ptCloudIn, clusterIndices);
end

function ptCloudReady = prepareCloudForICP(ptCloudIn)
    ptCloudReady = ptCloudIn;
    if isempty(ptCloudReady.Normal) && ptCloudReady.Count >= 10
        ptCloudReady.Normal = pcnormals(ptCloudReady, min(15, ptCloudReady.Count));
    end
end

function ptCloudDown = downsampleIfLarge(ptCloudIn, maxPoints)
    if ptCloudIn.Count > maxPoints
        ratio = maxPoints / ptCloudIn.Count;
        ptCloudDown = pcdownsample(ptCloudIn, 'random', ratio);
    else
        ptCloudDown = ptCloudIn;
    end
end

%% ========================================================================
%  DIAGNOSTIC VISUALIZATION FUNCTIONS
%  ========================================================================
function plotCalibrationDiagnostics(calibrationData)
    figure('Name', 'Calibration Mode: Segmented & Aligned Reference Clusters', 'Color', 'k');
    for i = 1:3
        subplot(2, 3, i);
        pcshow(calibrationData.largestClusters{i});
        title(sprintf('Calib Cam %d Cluster (%d deg)', i, calibrationData.nominalYawAngles(i)), 'Color', 'w');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([0 1.0]);
    end
    subplot(2, 3, 4);
    pcshowpair(calibrationData.alignedClusters{1}, calibrationData.alignedClusters{2});
    title('Calib Pair: Cam 1 vs Cam 2', 'Color', 'w');
    xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([0 1.0]);

    subplot(2, 3, 5);
    pcshowpair(calibrationData.alignedClusters{1}, calibrationData.alignedClusters{3});
    title('Calib Pair: Cam 1 vs Cam 3', 'Color', 'w');
    xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([0 1.0]);

    subplot(2, 3, 6);
    pcshow(calibrationData.mergedCalibCloud);
    title('Merged Calibrated Reference Model', 'Color', 'w');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([0 1.0]);
end

function plotAlignmentDiagnostics(alignmentResults)
    figure('Name', 'Alignment Mode: Base Calibration vs. Fine ICP Refinement', 'Color', 'k');
    subplot(2, 2, 1);
    pcshowpair(alignmentResults.baseAlignedObject{1}, alignmentResults.baseAlignedObject{2});
    title('Base Calib Only: Cam 1 vs Cam 2', 'Color', 'w');

    subplot(2, 2, 2);
    pcshowpair(alignmentResults.fineAlignedObject{1}, alignmentResults.fineAlignedObject{2});
    title(sprintf('After Secondary ICP: Cam 1 vs Cam 2 (RMSE: %.4f m)', alignmentResults.rmseFineICP(2)), 'Color', 'w');

    subplot(2, 2, 3);
    pcshow(alignmentResults.mergedCloudBase);
    title('Merged Target (Base Calibration Only)', 'Color', 'w');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

    subplot(2, 2, 4);
    pcshow(alignmentResults.mergedCloudRefined);
    title('Final Merged 3D Model (Base + Secondary ICP)', 'Color', 'w');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
end

%% ========================================================================
%  INPUT PARSER & DEFAULT CONFIGURATION
%  ========================================================================
function cfg = parseInputs(varargin)
    p = inputParser();
    p.FunctionName = 'lidar_point_cloud_alignment';

    addParameter(p, 'Mode', 'both', @(x) ischar(x) || isstring(x));
    addParameter(p, 'CalibFiles', { ...
        'dense_box_angle_0.ply', ...
        'dense_box_angle_120.ply', ...
        'dense_box_angle_240.ply'}, @iscell);
    addParameter(p, 'TargetFiles', { ...
        'f1.ply', ...
        'f2.ply', ...
        'f3.ply'}, @iscell);
    addParameter(p, 'CalibrationFile', 'calibration_data.mat', @(x) ischar(x) || isstring(x));
    addParameter(p, 'OutputMergedPly', 'merged_aligned_model.ply', @(x) ischar(x) || isstring(x));

    % Expanded CropROI [-0.85, 0.85, -0.85, 0.85, -0.05, 1.25] prevents clipping rotated box corners
    addParameter(p, 'NominalYawAnglesDeg', [0, -120, 120], @isnumeric);
    addParameter(p, 'CropROI', [-0.85, 0.85, -0.85, 0.85, -0.05, 1.25], @isnumeric);

    addParameter(p, 'GroundAlignmentMethod', 'groundGridICP', @(x) ischar(x) || isstring(x));
    addParameter(p, 'GroundXLim', [-1.0, 1.0], @isnumeric);
    addParameter(p, 'GroundYLim', [-1.0, 1.0], @isnumeric);
    addParameter(p, 'GroundGridSize', 0.02, @isnumeric);
    addParameter(p, 'GroundHeightDivisor', 10.0, @isnumeric);
    addParameter(p, 'MinGroundThreshold', 0.04, @isnumeric);
    addParameter(p, 'RecomputeGroundInAlignMode', false, @islogical);

    addParameter(p, 'ClusterMaxDistance', 0.05, @isnumeric);
    addParameter(p, 'ICPMetric', 'planeToPlane', @(x) ischar(x) || isstring(x));
    addParameter(p, 'CalibICPInlierRatio', 0.95, @isnumeric);
    addParameter(p, 'CalibICPMaxIterations', 100, @isnumeric);
    addParameter(p, 'UseCPDInCalibration', true, @islogical);
    addParameter(p, 'CPDMaxIterations', 50, @isnumeric);
    addParameter(p, 'CPDMaxPoints', 4000, @isnumeric);
    addParameter(p, 'AlignICPInlierRatio', 0.85, @isnumeric);
    addParameter(p, 'AlignICPMaxIterations', 100, @isnumeric);
    addParameter(p, 'MergeGridStep', 0.005, @isnumeric);
    addParameter(p, 'ShowPlots', true, @islogical);

    addParameter(p, 'StreamWidth', 640, @isnumeric);
    addParameter(p, 'StreamHeight', 480, @isnumeric);
    addParameter(p, 'StreamFPS', 30, @isnumeric);
    addParameter(p, 'DepthMinMeters', 0.1, @isnumeric);
    addParameter(p, 'DepthMaxMeters', 3.0, @isnumeric);

    parse(p, varargin{:});
    cfg = p.Results;
end
