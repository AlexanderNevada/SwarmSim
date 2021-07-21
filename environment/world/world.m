classdef world
    %WORLD simulated environment with multiple robots and obstacles
    
    properties
        env        % multi robot environment object
        numRobots  % number of robot in the system
        numSensors % number of sensors equiped on each robot
        sensorRange% maximum range of sensing
        poses      % current poses of each robot
        sensors    % sensor information of each robot
        detectors  % robot detector of each robot
        cameras    % landmark sensor of each robot
        landmarks  % land marks in the environment
    end
    
    methods
        function obj = world(swarmInfo,simConst)
            %WORLD constructor
            numRobots = swarmInfo.numRobots;
            showTraj = swarmInfo.showTraj;
            poses = swarmInfo.poses;
            obj.numRobots = numRobots;
            obj.env = MultiRobotEnv(obj.numRobots);
            obj.env.hasWaypoints = true;
            obj.env.showTrajectory = showTraj;
            obj.env.robotRadius = simConst.length;
            obj.env.mapName = 'map';
            obj.sensors = cell(1,obj.numRobots);
            obj.detectors = cell(1,obj.numRobots);
            obj.cameras = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                robotInfo = swarmInfo.infos{i};
                % associate range finders for each robot
                lidar = MultiRobotLidarSensor;
                lidar.robotIdx = i;
                lidar.scanAngles = linspace(-simConst.lidar_range,simConst.lidar_range,robotInfo.numSensors); %% range !
                lidar.maxRange = robotInfo.sensorRange;
                attachLidarSensor(obj.env, lidar); % associate lidar with map
                obj.sensors{i} = lidar;
                % associate robot detector of each robot
                detector = RobotDetector(obj.env,i);
                detector.sensorOffset = [0 0];
                detector.sensorAngle = 0;
                detector.fieldOfView = simConst.detect_fieldOfView;  % full range robot detection
                detector.maxRange = robotInfo.sensorRange;
                detector.maxDetections = obj.numRobots-1; % maximum number of detections
                obj.detectors{i} = detector;
                % associate landmark detector of each robot
                camera = ObjectDetector;
                camera.fieldOfView = simConst.cam_fieldOfView;
                attachObjectDetector(obj.env,i,camera);
                obj.cameras{i} = camera;
            end
            obj.poses = poses; % initial poses
        end
        
        function obj = update_poses(obj,new_poses)
            % return the current poses
            obj.poses = new_poses;
        end
        
        function obj = update_vels(obj,new_vels)
            % return the current poses
            obj.vels = new_vels;
        end
        
        function poses = get_poses(obj)
            % update current poses
            poses = obj.poses;
        end
        
        function obj = addLandmarks(obj,data)
            % add landmarks into the world
            obj.landmarks = data;
            numLandmarks = size(data,1);
            color = rand(numLandmarks,3);
            %color = [1 0 0;0 1 0;0 0 1];
            obj.env.objectColors = color;
        end
        
        function ranges = readSensors(obj)
            % read sensors at current poses
            ranges = cell(1,obj.numRobots); % empty sensor readings
            obj.env.Poses = obj.poses;
            for i = 1:obj.numRobots
                lidar = obj.sensors{i};
                % scans = lidar(obj.poses(:,i));
                scans = lidar();
                ranges{i} = scans;
            end
        end
        
        function detections = readDetections(obj)
            % read robot detection result [numDetections, 3] [range angle idx]
            detections = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                detector = obj.detectors{i};
                detection = step(detector);
                detections{i} = detection;
            end
        end
        
        function landmarks = readCameras(obj)
            % read landmark detection results
            landmarks = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                camera = obj.cameras{i};
                landmark = camera(obj.poses(:,i),obj.landmarks);
                disp("observed landmarks")
                disp(landmark);
                landmarks{i} = landmark;
            end
        end
        
        function visualize(obj,ranges)
            % visualize sensor readings at current poses
            %obj.env(1:obj.numRobots,obj.poses,ranges,obj.landmarks);
            for i = 1:obj.numRobots
                obj.env(i,obj.poses(:,i),[2 2;12 12],ranges{i},obj.landmarks);
            end
        end
    end
end

