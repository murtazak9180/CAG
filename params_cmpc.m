%% Predator params
params.predator = 0;
params.pFactor = 1.40;
params.pred_radius = 6;
params.wp = 500;

%% Target seeking
params.target = [50,50];

%% Params
params.controller_type = 'SS'; %use 'SS' or 'MS'
params.steps = 300;
params.n = 10; %5
params.h = 5; %10
params.dt = 0.1;
params.ct = 0.3;
params.amax = 1.5; %1.5
params.vmax = 2;%2

params.minP = -15;
params.maxP = 15;
params.minV = 0;
params.maxV = 1;
params.Dmax = 100; % neighbor def
params.dmin = 2.0;
params.Ds = params.dmin; % safe dist

params.Dc = 0.8;%0.6; % connectivity dist
params.gamma = 0.05; %1e0; %nominal case
% params.rs = sensing_radius(params);
params.rs = 8;

%% for fmincon
% params.ws = 200; %1000
% params.wc = 1;
% params.wo = 400;
% params.wt = 10;
% params.wv = 5;

%% scaled ipopt
params.ws = 0.1; %1000
params.wc = 0.8;
params.wo = 400;
params.wt = 0.1;
params.wv = 5;


%% flags
params.use_mex = true;
