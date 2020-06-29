addpath(genpath('../../lib'))
addpath('functions')

% Clear workspace to increase repeatability
clc
clear

% Reset the Matlab profiler
profile clear

% Seed the pseudo random number generator. This is required if we want
% reproducible simulation, e. g. for profiling the code.
rng(0);

% Flag to enable exporting a video from the simulation results
saveVideo = false;

%% Network parameters
agentCount = 25;    % Number of agents in the network
dimension  = 2;     % Dimension of the space the agents move in
dT         = 0.05;  % Size of the simulation time steps
range      = 7*1.2; % Range of the radio communication
pTransmit  = 0.95;  % Probability of successful transmission
steps      = 1000;  % Simulation time steps

%% Field parameters
no_centers=11;
fcenter=50;
frange=100;
fvar=1e2;
fscale=50;
%% Field Sensor parameters
N               =10;    % Number of measurements around the agent pos
sensor_range    =5;     % sensor range around the agent pos
noise_bound     =20;    % apriori known bound on measurement noise
C_reg           =0.01;  % L1 regularization tuning.(0-> no regularization)
%% Initialize the external potential field
conc_Field=InvGaussiansField(dimension,no_centers,fcenter,frange,fvar,fscale);
%% Initialize the Interaction field between agents
interac_field=OS_InteractionField();

%% Initialize the Field sensor for each agent
field_sensor=AgentFieldSensor(sensor_range,N,noise_bound,conc_Field,C_reg);

%% Initialize the network
Network = IdealNetwork(agentCount, dT, dimension, range);
% Network = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);

% To avoid Matlab initializing the array of agents without including the
% required constructor arguments, we first construct a cell array of agents
% and later convert this to a standard Matlab array.
Agents = cell(agentCount, 1);
for i = 1:length(Agents)
    % Randomly place the agents in the square [0,100]^2
    pos = 50 + 100 * (rand(dimension, 1) - 0.5);
    % Randomly assign the agent velocites in [-5,5]^2
    vel = 1 * (rand(dimension, 1) - 0.5);
    
    % Initiallize an agent with the generated initial conditions
    Agents{i} = FlockingAgent2(Network.getId(), dT, pos, vel, conc_Field, interac_field,field_sensor);
end
Agents = [Agents{:}];

%% Perform simulation
pos_history = zeros(steps+1, dimension, agentCount);
pos_history(1,:,:) = [Agents.position];

% Data structure for Energy analysis
Energy=struct;
Energy.KE=zeros(1,steps);
Energy.Vfield=zeros(1,steps);
Energy.V=zeros(1,steps);



tic
% profile on
for k = 1:steps
    t = (k-1) * dT;
    
    % Perform a single time step for each agent. This includes evaluating
    % the dynamic equations and the flocking protocol as well as sending
    % its own position and velocity to the other agents as a message.
    for agent = Agents        
        agent.step()
        Network.updateAgent(agent)
    end
    
    % Process sent messages in the network
    recvMessages = Network.process();
    
    % Save current position of all agents
    pos_history(k+1,:,:) = [Agents.position];
    
    for agent = Agents
        % Distribute transmitted messages among the agents
        agent.receive(t, recvMessages{agent.id});
        
        % Calculate kinetic and source field energies
        Energy.KE(1,k)=Energy.KE(1,k)+0.5*norm(agent.velocity,2)^2;
        Energy.Vfield(1,k)=Energy.Vfield(1,k)+conc_Field.get_field_value_at(agent.position);        
        
        % Calculate the distance from the agent to all other agents
        dist = vecnorm(agent.position - squeeze(pos_history(k+1,:,:))); 
        Energy.V(1,k)=Energy.V(1,k)+0.5*sum(interac_field.look_up_psi_a(dist'));        
    end
    % Print status periodically
    if rem(k, steps/100) == 0
        fprintf('Simulation %3.5g%% finished\n', 100*k/steps)
    end
end
% profile viewer
toc

%% Analyze energy in the system
post_process(dT,steps,Energy)
%% Animate simulation results
figure()

% Compute boundary
x_pos  = squeeze(pos_history(:,1,:));
y_pos  = squeeze(pos_history(:,2,:));
bounds = @(x) [min(min(x)), max(max(x))];

% Compute contour lines
lim     = [ bounds(x_pos); bounds(y_pos) ];
[X,Y,Z] = data_for_contour(lim,conc_Field);

% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('source_seeking', 'MPEG-4');
    video.FrameRate = 50;
    open(video)
else
    video = [];
end

for k = 1:steps+1
    contour(X,Y,Z)
    hold on
    pos = squeeze(pos_history(k,:,:));
    scatter(pos(1,:), pos(2,:))
    hold off
    
    xlim(bounds(x_pos));
    ylim(bounds(y_pos));     
    drawnow limitrate
    
    if ~isempty(video)
        % Save figure as video frame
        frame = getframe(gcf);
        writeVideo(video, frame);
    end
end

if ~isempty(video)
    close(video)
    video = [];
end