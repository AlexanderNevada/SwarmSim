%% a test script for simulator class
clear all;
close all;
% generate map for the simulation
size = 15;
resolution = 10;
numObstacles = 2;
space = 5;
%p = zeros(size*resolution);
map_gen = MapGenerate(size,size,space,resolution);
[p,map_gen] = map_gen.addBounds(2); % boundary width
for i = 1:numObstacles
    [p,map_gen] = map_gen.addRandomObstacle(1.0,0.5); %max_size, min_size
end
map = binaryOccupancyMap(p,resolution);

%% specify some parameters
form = LineFormation(); %DiamondFormation(); %VShapeFormation();  %DiamondFormation();
numRobots = form.numRobots;
numSensors = 5;
sensorRange = 2.5;
showTraj = false;
initial_poses = 8*(rand(3,numRobots).*[0.5;0.5;0]) + [0.5;0.5;0];
robotInfos = cell(1,numRobots);
for i = 1:numRobots
    t = "DiffDrive"; % differential drive dynamics
    R = 0.1; 
    L = 0.5;
    s = numSensors;
    r = sensorRange;
    show = showTraj;
    robotInfo = RobotInfo(t,R,L,s,r);
    robotInfos{i} = robotInfo;
end
swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);
%% leader-follower simulation
sim = LeaderFollowerSimulation(map,swarmInfo,form);
for i = 1:1000
    sim = sim.step();
    axis([0 size 0 size])
    pause(0.02);
end
