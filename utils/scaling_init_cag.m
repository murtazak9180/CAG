function scale = scaling_init_cag(params)
%% scaling_init_cag - Defines normalization factors for the swarm

scale.L = params.rs;           % Length scale
scale.V = params.vmax;         % Velocity scale
scale.A = params.amax;         % Acceleration scale

  
% Distance Constants 
internal_dmin = params.dmin * 1.0001;  %buffer
scale.dmin = internal_dmin / scale.L;
scale.dmin2 = (internal_dmin / scale.L)^2;
scale.rs   = params.rs / scale.L; 

% Target Scaling
scale.target = params.target / scale.L;

end