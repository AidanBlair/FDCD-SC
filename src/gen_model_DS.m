function model= gen_model_DS(range, theta)
% basic parameters
model.x_dim= 5;   %dimension of state vector
model.z_dim= 2;   %dimension of observation vector
model.v_dim= 3;   %dimension of process noise
model.w_dim= 2;   %dimension of observation noise

% dynamical model parameters (CT model)
% state transformation given by gen_newstate_fn, transition matrix is N/A in non-linear case
model.T= 1;                         %sampling period
model.sigma_vel= 15;
model.sigma_turn= pi/180;   %std. of turn rate variation (rad/s)
model.bt= model.sigma_vel*[ (model.T^2)/2; model.T ];
model.B2= [ model.bt zeros(2,2); zeros(2,1) model.bt zeros(2,1); zeros(1,2) model.T*model.sigma_turn ];
model.B= eye(model.v_dim);
model.Q= model.B*model.B';



% survival/death parameters
model.P_S= .99;
model.Q_S= 1-model.P_S;

%===here is the parameter for the birth target states
model.T_birth= 15;         %no. of Gaussian birth terms

% observation model parameters (noisy r/theta only)
% measurement transformation given by gen_observation_fn, observation matrix is N/A in non-linear case
model.D= diag([ 2; 2]);      %std for angle and range noise  (x and y noise now)
model.R= model.D*model.D';              %covariance for observation noise
model.C_posn = [1 0 0 0 0; 0 0 1 0 0];

% detection parameters
model.P_D= .99;   %probability of detection in measurements
model.Q_D= 1-model.P_D; %probability of missed detection in measurements

% clutter parameters
model.lambda_c=5;                             %poisson average rate of uniform clutter (per scan)
model.range_c= [ -2000 2000; -2000 2000 ];         %uniform clutter on r/theta
model.pdf_c= 1/prod(model.range_c(:,2)-model.range_c(:,1)); %uniform clutter density

% Sector FOV parameters
model.Heading = pi / 2;  % looking up, 2 * pi * rand()
model.range = range;
model.FOV = theta;  % plus-minus angle from camera heading that is within FOV, pi/2 = 180deg FOV
 
