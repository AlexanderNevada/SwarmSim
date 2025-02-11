function const = SimulationConst()
% 
% Define the constants used in the simulation. 
%
% Class:
% Advanced topics in control
% Spring 2021
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Qi Ma qimaqi@student.ethz.ch

%% The binary map and obstacles
const.Map_size = 15;
const.resolution = 10;
const.numObstacles = 0;
const.space = 5;

%% The agent parameters
const.numRobots = 4;
const.numSensors = 25;
const.sensorRange = 3;
const.showTraj = false;
const.radius = 0.1; 
const.length = 0.3;
const.values = randsrc(const.numRobots,1,[1,2,3;0.2,0.5,0.3]);
const.lidar_range = pi/3;
const.detect_fieldOfView = pi*2/3;
const.cam_fieldOfView = pi*2/3;
%% Control && Simulation parameters
const.sampleTime = 0.05;
const.goal = [12 12];

%% Optimization:
const.c = 1.0;
const.lambda = 0.00001 ;
const.d = 2.0 ;
const.protection = 1.0;
const.target_weight = 0.1;
const.max_v_l = 1;
const.min_v_l = 0;
const.max_a_l = 1;
const.min_a_l = -1;
const.max_v_r = 2;
const.min_v_r = -2;
const.max_a_r = 2;
const.min_a_r = -2;

%% The wall contour - (x,y) coordinates of corners as in Table 1
const.contour = [0.50, 0.00;
                 2.50, 0.00;
                 2.50, 1.50;
                 3.00, 2.00;
                 2.00, 3.00;
                 1.25, 2.25;
                 1.00, 2.50;
                 0.00, 2.50;
                 0.00, 0.50;
                 0.50, 0.50];
             
%% Initialization
const.pA = [1.1,0.6]; % Center point pA of the initial position distribution
const.pB = [1.8,2.0]; % Center point pB of the initial position distribution
const.d = 0.2;  % Radius of shaded regions for initialization

const.phi_0 = pi/4; % Initial heading is uniformly distr. in [-phi_0,phi_0]

const.l = 0.2;  % Uniform distribution parameter of points p9 and p10

%% Noise properties
% process noise
const.sigma_phi = 0.05; % Parameter for process noise v_phi
const.sigma_f = 0.01; % Parameter for process noise v_f

% measurement noise
const.epsilon = 0.01; % Parameter for measurement noise w

%% Times
% Number of samples (discrete time steps) of the simulation.
const.N = 500;
