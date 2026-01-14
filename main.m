clear; close all; clc

% Get folder containing this script
projectRoot = fileparts(mfilename('fullpath'));

% Add src and all its subfolders
addpath(genpath(fullfile(projectRoot, 'src')));
% Add results and all its subfolders
addpath(genpath(fullfile(projectRoot, 'results')));

FloodingOrConsensus = "flooding";

rng(1, 'twister');

prompt = 'Scenario? (1 or 2) ';
Scenario = input(prompt);

prompt = 'Optimization Method? (Fixed = 1, Individual = 2, Distributed Coordinate Descent = 3, Fully Distributed Coordinate Descent = 4) ';
OptimizationMethod = input(prompt);

prompt = 'Number of iterations:';
NumIt = input(prompt);

prompt='Generate Videos? (yes=1 or no=0) ';
Video_flag=input(prompt);

prompt = 'Which method (CS = 1 or GCI = 2 or CF = 3)? ';
flag = input(prompt);

prompt = 'Distributed or Centralized? (Dist=1, Cent=0)? ';
distributed = input(prompt);

if distributed==1
    prompt = 'How many Consensus iterations? ';
    L=input(prompt);
end

model_flag =2;
plotting = 1;

global Num_Sen colorarray
if Scenario==1
    Num_Sen = 6;
    TIME_STEPS = 50;
    valid_actions = 1;  % Security Cameras
    COMMUNICATION_RANGE = 800;
    RHO_BUFFER = 0;
    THETA_BUFFER = pi/45;
    model_fov = pi / 4; % 90 deg
    model_range = 500;
elseif Scenario==2
    Num_Sen = 8;
    TIME_STEPS = 100;
    valid_actions = 2;  % 6 Actions Ortho-Diagonal
    COMMUNICATION_RANGE = 300;
    RHO_BUFFER = 8;
    THETA_BUFFER = 0;
    model_fov = pi; % 360 deg
    model_range = 100;
end
DCD = false;
Fixed = false;
SingleSensor = false;
FDCD = false;
if OptimizationMethod == 1
    Fixed = true;
elseif OptimizationMethod == 2
    SingleSensor = true;
elseif OptimizationMethod == 3
    DCD = true;
elseif OptimizationMethod == 4
    FDCD = true;
end
handles_ = cell(Num_Sen * NumIt, 1);
times = zeros(TIME_STEPS * NumIt, 1);
EXCLUSION_RADIUS = 0;
PSI_THRESHOLD = 0.90;
ETA_THRESHOLD = 50;
COMMUNICATION_BUFFER = 15;
Orientations_ = cell(NumIt, 1);
Positions_ = cell(NumIt, 1);
stationarySensors=[1, 2, 3, 4, 5, 6];
egoSensors=[];

for run = 1:NumIt
    Orientations_{run} = 2 *pi * rand(1, Num_Sen) - pi;
    invalid_positions = true;
    while invalid_positions
        invalid_positions = false;
        collision_check = false;
        Positions_{run} = 1600 * rand(2, Num_Sen) - 800;
        for i = 1:Num_Sen
            neighbor_counter = 0;
            for j = 1:Num_Sen
                if (sqrt((Positions_{run}(1,i) - Positions_{run}(1, j))^2 + (Positions_{run}(2,i) - Positions_{run}(2,j))^2) <= COMMUNICATION_RANGE) && (i ~= j)
                    neighbor_counter = neighbor_counter + 1;
                end
                if (sqrt((Positions_{run}(1,i) - Positions_{run}(1, j))^2 + (Positions_{run}(2,i) - Positions_{run}(2,j))^2) <= ETA_THRESHOLD) && (i ~= j)
                    collision_check = true;
                end
            end
            if neighbor_counter == 0 || collision_check
                invalid_positions = true;
                disp("Invalid Positions")
                break
            end
        end
    end
    %
end

%% Filter
for run = 1:NumIt
    if run >= 0 && Video_flag
        mov = VideoWriter(string(datetime("now", "Format", "yyyy-MM-dd_HH_mm_ss"))+"_run_"+run+".avi");
        mov.FrameRate = 1;
        open(mov);
    end
    tEnd=zeros(NumIt,1);
    et=zeros(TIME_STEPS,NumIt); %%index 1 = num of time steps
    
    tStart = tic; %pair 2 tic
    close all;
    clc

    disp("Run")
    disp(run)

    Sen_Locations = zeros(TIME_STEPS, 2, Num_Sen);
    Sen_Headings = zeros(TIME_STEPS, Num_Sen);
    switch distributed
        case 0
            model= gen_model_DS(sensorLocations,model_flag,FoV,truth,BirthRegions,noiseErrors,stats);
            meas= gen_meas_cartesian(Ego1,Ego2,Ego3,Ego4,Ego5,Ego6,Ego7,Ego8,Ego9,truth,SensorLocations,model);
        case 1
            for S=1:Num_Sen %%generate a model for each sensor
                model{S}= gen_model_DS(model_range, model_fov);
            end
            if Scenario == 1
                distributedBirthLocationsScenario1;
            elseif Scenario == 2
                distributedBirthLocationsScenario2;
            end
    end
    if Scenario == 1
        load("Scenario1.mat")
    elseif Scenario == 2
        load("Scenario2.mat")
    end

    Orientations_{run} = Headings_;
    Positions_{run} = Pos_;

    SensorLocations{1}.positions = Positions_{run};
    SensorLocations{1}.actorIDs = 1:Num_Sen;
    SensorHeadings{1} = Orientations_{run};
    model{1}.SenLoc = SensorLocations{1}.positions;

    id=4; %%which sensor to look at

    %output variables
    
    switch distributed
        case 0
            est.X= cell(meas{1}.K,1);
            est.N= zeros(meas{1}.K,1);
            est.L= cell(meas{1}.K,1);
        case 1
            est = cell(size(SensorLocations{1}.positions, 2), 1);
            for S=1:size(SensorLocations{1}.positions,2)
                est{S}.X= cell(truth.K,1);
                est{S}.N= zeros(truth.K,1);
                est{S}.L= cell(truth.K,1);
            end
    end
    

    %filter parameters
    filter.T_max= 100;%100                  %maximum number of tracks
    filter.track_threshold= 0.25; %1e-3      %threshold to prune tracks

    filter.H_bth= 4;                    %requested number of birth components/hypotheses (for LMB to GLMB casting before update)
    filter.H_sur= 50;%100                  %requested number of surviving components/hypotheses (for LMB to GLMB casting before update)
    filter.H_upd= 50;%100                  %requested number of updated components/hypotheses (for GLMB update)
    filter.H_max= 100;                  %cap on number of posterior components/hypotheses (not used yet)
    filter.hyp_threshold= 1e-7;         %pruning threshold for components/hypotheses (not1 used yet) 1e-7
    filter.prediction_threshold=0.25;         %pruning threshold for estimates 

    filter.npt= 1000;%1000                   %number of particles per track
    filter.nth= 100;%100                    %threshold on effective number of particles before resampling (not used here, resampling is forced at every step, otherwise number of particles per track grows)

    filter.run_flag= 'disp';            %'disp' or 'silence' for on the fly output
    filter.merging_threshold=0.1;%15;%0.75;
    filter.correction_threshold=1.5;

    %=== Filtering

    %initial prior
    switch distributed
        case 0
            tt_lmb_update = [];
            tt_lmb_birth=[];
            tt_lmb_survive=[];
        case 1
            tt_lmb_update=cell(size(model{1}.SenLoc,2),1);
            tt_lmb_birth=cell(size(model{1}.SenLoc,2),1);
            tt_lmb_survive=cell(size(model{1}.SenLoc,2),1);
            for i=1:size(model{1}.SenLoc,2)	
                tt_lmb_survive{i}=cell(15,1);	
                tt_lmb_birth{i}=cell(8,1);		
            end
            S=cell(size(model{1}.SenLoc,2),1);
            S_temp=zeros(1,size(model{1}.SenLoc,2));
            NodeWeights=cell(size(model{1}.SenLoc,2),1);
            tempweights=cell(size(model{1}.SenLoc,2),1);
    end

    sums = [];
    for k = 1:truth.K
        for s = 1:Num_Sen
            meas{s}.Z{k,1} = [];
        end
    end
    %% recursive filtering
    for k=1:truth.K
        if k > 1
            model = AdaptiveBirth2(model, meas, est, k-1, truth);
        end
        switch distributed
            case 0
                model.SenLoc=SensorLocations{max(k-1,1)}.positions;
            case 1
                model{1}.SenLoc=SensorLocations{max(k-1,1)}.positions;
                NodeWeights=cell(size(model{1}.SenLoc,2),1);
                %get set of all neighbors of sensor S_(n,k)
                [S,s,t] = GenerateGraph(model{1},SensorLocations{max(k-1,1)}, Num_Sen, COMMUNICATION_RANGE);
                [NodeWeights] = GenerateNodeWeights(S,model{1});
                A=sparse(s,t,1);
                G=graph(A,'omitselfloops','upper');
        end

        %prediction
        switch distributed
            case 0 %centralized
                [tt_lmb_birth,tt_lmb_survive]= lmbpredict(tt_lmb_update,model,filter,k);
                T_predict= length(tt_lmb_birth)+length(tt_lmb_survive);
                glmb_predict= castlmbpred(tt_lmb_birth,tt_lmb_survive,filter);
                tt_lmb_update = cell(20,1);
            case 1 %distributed
                for SenNum=1:size(model{1}.SenLoc,2) %%do everything for each independent sensor
                    [tt_lmb_birth{SenNum},tt_lmb_survive{SenNum}]= lmbpredict_DS(tt_lmb_update{SenNum},model{SenNum},filter,k,SenNum);
                    T_predict{SenNum}= length(tt_lmb_birth{SenNum})+length(tt_lmb_survive{SenNum});
                    glmb_predict{SenNum}= castlmbpred(tt_lmb_birth{SenNum},tt_lmb_survive{SenNum},filter);
                    temp_tt_lmb_update{SenNum} = cell(20,1);
                    tt_lmb_update{SenNum} = cell(20,1);
                end
        end

        %% Prediction Estimation
        for SenNum=1:size(model{1}.SenLoc,2) %generate on a per sensor basis
            tt_lmb_predict{SenNum}= glmb2lmb2_dist(glmb_predict{SenNum}); %%Tharindu's preallocated version
        end
        for Sen=1:size(model{1}.SenLoc,2)
            [est_pred{Sen}.X{k},est_pred{Sen}.N(k),est_pred{Sen}.L{k}]= extract_estimates_DS2(tt_lmb_predict{Sen},model{Sen},filter);
        end

        %
        if k > 1
            if valid_actions == 1
                % proper PIMS
                num_controls = 0;
                
                % motion
                ang_steps = 1;
                ang_incr = pi / 8 / ang_steps;
                step_size = 0;
    
                pims_models = cell(Num_Sen, 3);
                for SenNum = 1:Num_Sen
                    pims_models{SenNum, 1} = model{SenNum};
                end
                
                for i=-1:ang_steps
                    num_controls = num_controls + 1;
                    angle = i*ang_incr;
                    for SenNum = 1:Num_Sen
                        pims_models{SenNum, num_controls} = model{SenNum};
                        new_angle = pims_models{SenNum, num_controls}.Heading + angle;
                        if new_angle < -pi
                            new_angle = 2 * pi + new_angle;
                        elseif new_angle > pi
                            new_angle = new_angle - 2 * pi;
                        end
                        pims_models{SenNum, num_controls}.Heading = new_angle;
                    end
                end
            elseif valid_actions == 2
                % proper PIMS
                num_controls = 0;
                
                % motion
                step_size = 15;
                directions = 8;
    
                pims_models = cell(Num_Sen, 3);
                
                % Stationary
                num_controls = num_controls + 1;
                for SenNum = 1:Num_Sen
                    pims_models{SenNum, num_controls} = model{SenNum};
                end

                % Translations
                for i = 1:directions
                    num_controls = num_controls + 1;
                    angle = (num_controls - 2) * ((2 * pi) / 8);
                    next_x = step_size * cos(angle);
                    next_y = step_size * sin(angle);
                    for SenNum = 1:Num_Sen
                        pims_models{SenNum, num_controls} = model{SenNum};
                        pims_models{SenNum, num_controls}.SenLoc(1, SenNum) = pims_models{SenNum, num_controls}.SenLoc(1, SenNum) + next_x;
                        pims_models{SenNum, num_controls}.SenLoc(2, SenNum) = pims_models{SenNum, num_controls}.SenLoc(2, SenNum) + next_y;
                    end
                end
            end
    
            % extract tracks for PIMS
            THRESH = filter.prediction_threshold;
            pims_meas_temp = cell(Num_Sen, 1);
            for SenNum = 1:Num_Sen
                for i = 1:size(glmb_predict{SenNum}.tt, 1)
                    if glmb_predict{SenNum}.tt{i}.r > THRESH
                        pims_meas_temp{SenNum} = [pims_meas_temp{SenNum}, mean(glmb_predict{SenNum}.tt{i}.x, 2)];
                    end
                end
            end
    
            

            %% Control
            % Make lists of neighbours
            neighbours = cell(Num_Sen, 1);
            for i = 1:size(G.Edges, 1)
                if ~ismember(G.Edges{i, 1}(2), neighbours{G.Edges{i, 1}(1)})
                    neighbours{G.Edges{i, 1}(1)} = [neighbours{G.Edges{i, 1}(1)}, G.Edges{i, 1}(2)];
                end
                if ~ismember(G.Edges{i, 1}(1), neighbours{G.Edges{i, 1}(2)})
                    neighbours{G.Edges{i, 1}(2)} = [neighbours{G.Edges{i, 1}(2)}, G.Edges{i, 1}(1)];
                end
            end
    
            neighbor_indices = cell(Num_Sen, 1);
            for i = 1:Num_Sen
                neighbor_indices{i} = [neighbor_indices{i}, i];
                for j = 1:size(neighbours{i}, 2)
                    neighbor_indices{i} = [neighbor_indices{i}, neighbours{i}(j)];
                end
            end

            tic;
            %% Single Sensor
            if SingleSensor == true
                pims_meas = cell(Num_Sen, num_controls);
                pims_glmb_update = cell(Num_Sen, num_controls);
                pims_tt_lmb_update = cell(Num_Sen, num_controls);
                pims_T_posterior = zeros(Num_Sen, num_controls);
                for SenNum = 1:Num_Sen
                    for u = 1:num_controls
                        pims_meas{SenNum, u}.Z{k, 1} = [];
                        for me = 1:size(pims_meas_temp{SenNum}, 2)
                            pD_ = compute_pD(pims_models{SenNum, u}, pims_meas_temp{SenNum}(:, me), SenNum, true);
                            if rand() < pD_
                                pims_meas{SenNum, u}.Z{k, 1} = [pims_meas{SenNum, u}.Z{k, 1}, [pims_meas_temp{SenNum}(1, me) + pims_meas_temp{SenNum}(2, me); pims_meas_temp{SenNum}(3, me) + pims_meas_temp{SenNum}(4, me)]];
                            end
                        end
                        pims_glmb_update{SenNum, u} = [];
                        pims_glmb_update{SenNum, u}= up_date(glmb_predict{SenNum},pims_models{SenNum, u},filter,pims_meas{SenNum, u},k,SenNum);
                        pims_tt_lmb_update{SenNum, u}= glmb2lmb2_dist(pims_glmb_update{SenNum, u}); %%Tharindu's preallocated version
                        pims_T_posterior(SenNum, u)= length(pims_tt_lmb_update{SenNum, u});
                    end
                end
                Costs = cell(Num_Sen, 1);
                Actions = cell(Num_Sen, 1);

                for SenNum = 1:Num_Sen
                    disp("Sensor: " + SenNum)
                    % For each possible actions given others stationary
                    for i = 1:num_controls
                        u = i;
                        BREAK_DIST = COMMUNICATION_RANGE - COMMUNICATION_BUFFER;
                        DC_PEN = 100;
                        DC_THRESH = 2;
                        dc_counter = 0;
                        it_count = 0;
                        in_range_counter = 0;
                        min_d = inf;
                        for it = 1:size(neighbor_indices{SenNum}, 2)
                            it_count = it_count + 1;
                            if neighbor_indices{SenNum}(it) ~= SenNum
                                d = Calculate_Distance(model{neighbor_indices{SenNum}(1)}.SenLoc(:,neighbor_indices{SenNum}(1)), model{neighbor_indices{SenNum}(it)}.SenLoc(:,neighbor_indices{SenNum}(it)), u, 0);
                                if d < min_d
                                    min_d = d;
                                end
                                if d <= BREAK_DIST
                                    in_range_counter = in_range_counter + 1;
                                end
                            end
                        end
                        zeta = (in_range_counter > 1);
                        psi = void_probability(model{SenNum}.SenLoc(:, SenNum), i, step_size, EXCLUSION_RADIUS, pims_tt_lmb_update{SenNum, u}) > PSI_THRESHOLD;
                        eta = min_d > ETA_THRESHOLD;
                        combined_penalty = ((1-psi) + (1-eta) + (1-zeta)) * DC_PEN;
                        Costs{SenNum}(i) = D_KL_final(pims_tt_lmb_update{SenNum, u}, tt_lmb_predict{SenNum}, filter) - combined_penalty;
                        Actions{SenNum}(i) = i;
                    end
                end

                U_max = zeros(Num_Sen, 1);

                for i = 1:Num_Sen
                    max_idxs = (Costs{i} == max(Costs{i}));
                    max_actions = Actions{i}(max_idxs);
                    final_idx = randi(length(max_actions), 1);
                    U_max(i) = max_actions(final_idx);
                end
            end


            %% Fully Distributed Coordinate Descent
            if FDCD == true
                % Aidan Blair 2025
                % Each sensor first calculates optimal single-sensor
                % control command, then iteratively, each sensor in the
                % network calculates optimal single-sensor control
                % command given the other sensors are following their
                % previously selected control commands. When a sensor
                % has the same inputs from all sensors as a previous
                % iteration, stop.
                NUM_IT = 1;
                U_i = cell(1, NUM_IT);
                dcs_i = cell(Num_Sen, NUM_IT);
                max_dcs = -inf;
                for it_f = 1:NUM_IT
                    % Step 1
                    % Create u_i^*'s and pi_i^*'s
                    % Note: Each u_i^* is created assuming no movement for
                    % other neighbouring sensors, and initialize
                    % U = 0_{12x1}.
                    pims_meas = cell(Num_Sen, num_controls);
                    pims_glmb_update = cell(Num_Sen, num_controls);
                    pims_tt_lmb_update = cell(Num_Sen, num_controls);
                    pims_T_posterior = zeros(Num_Sen, num_controls);
                    for SenNum = 1:Num_Sen
                        for u = 1:num_controls
                            pims_meas{SenNum, u}.Z{k, 1} = [];
                            for me = 1:size(pims_meas_temp{SenNum}, 2)
                                pD_ = compute_pD(pims_models{SenNum, u}, pims_meas_temp{SenNum}(:, me), SenNum, true);
                                if rand() < pD_
                                    pims_meas{SenNum, u}.Z{k, 1} = [pims_meas{SenNum, u}.Z{k, 1}, [pims_meas_temp{SenNum}(1, me); pims_meas_temp{SenNum}(3, me)]];
                                end
                            end
                            pims_glmb_update{SenNum, u} = [];
                            pims_glmb_update{SenNum, u}= up_date(glmb_predict{SenNum},pims_models{SenNum, u},filter,pims_meas{SenNum, u},k,SenNum);
                            pims_tt_lmb_update{SenNum, u}= glmb2lmb2_dist(pims_glmb_update{SenNum, u}); %%Tharindu's preallocated version
                            pims_T_posterior(SenNum, u)= length(pims_tt_lmb_update{SenNum, u});
                        end
                    end
                    Costs = cell(Num_Sen, 1);
                    Actions = cell(Num_Sen, 1);
                    Constraints = cell(Num_Sen, 1);

                    for SenNum = 1:Num_Sen
                        % For each possible actions given others stationary
                        for i = 1:num_controls
                            u = i;
                            BREAK_DIST = COMMUNICATION_RANGE - COMMUNICATION_BUFFER;
                            DC_PEN = 50;
                            min_d = inf;
                            for it = 1:size(neighbor_indices{SenNum}, 2)
                                if neighbor_indices{SenNum}(it) ~= SenNum
                                    d = Calculate_Distance_New(pims_models{neighbor_indices{SenNum}(1), u}.SenLoc(:,neighbor_indices{SenNum}(1)), model{neighbor_indices{SenNum}(it)}.SenLoc(:,neighbor_indices{SenNum}(it)));
                                    if d < min_d
                                        min_d = d;
                                    end
                                end
                            end
                            psi = void_probability(model{SenNum}.SenLoc(:, SenNum), i, step_size, EXCLUSION_RADIUS, pims_tt_lmb_update{SenNum, u}) > PSI_THRESHOLD;
                            eta = min_d > ETA_THRESHOLD;
                            combined_penalty = ((1-psi) + (1-eta)) * DC_PEN;
                            Costs{SenNum}(i) = D_KL_final(pims_tt_lmb_update{SenNum, u}, tt_lmb_predict{SenNum}, filter) - combined_penalty;
                            Actions{SenNum}(i) = i;
                        end
                    end

                    U_i{1, it_f} = zeros(Num_Sen, 1);
                    dcs_old = zeros(Num_Sen, 1);
                    for i = 1:Num_Sen
                        max_idxs = (Costs{i} == max(Costs{i}));
                        max_actions = Actions{i}(max_idxs);
                        final_idx = randi(length(max_actions), 1);
                        U_i{1, it_f}(i) = max_actions(final_idx);
                        dcs_i{i, it_f} = max(Costs{i});
                        dcs_old(i, 1) = dcs_i{i, it_f};
                    end

                    U_old = cell(1, 1);
                    U_old{1} = U_i{1, it_f};
                    dcs_i_old = cell(Num_Sen, 1);
                    for i = 1:Num_Sen
                        dcs_i_old{i} = zeros(size(U_i, 1));
                    end

                    saved_pis = cell(Num_Sen, 1);
                    saved_Actions = cell(Num_Sen, 1);
                    saved_counter = cell(Num_Sen, 1);
                    for s = 1:Num_Sen
                        saved_pis{s} = cell(100, 1);
                        saved_Actions{s} = zeros(Num_Sen, 100);
                        saved_counter{s} = 0;
                    end
                    flood_counter = 1;
                    stop_flag = 0;
                    while ~stop_flag
                        fprintf("iteration: %d\n", flood_counter)
                        % Step 6
                        % At each node, fuse pi_i^*'s of self and neighbours
                        % for all possible actions, assuming that neighbours
                        % have already moved according to their current
                        % u_i^*'s, and decide on best action -> update u_i^*
                        % and pi_i^* for that node.

                        final_fused_pims_dist_tt_lmb_update = cell(Num_Sen, 1);
                        dcs = cell(Num_Sen, 1);
                        for SenNum = 1:Num_Sen
                            for u_i = 1:num_controls
                                BREAK_DIST = COMMUNICATION_RANGE - COMMUNICATION_BUFFER;
                                DC_PEN = 50;
                                min_d = inf;
                                L_pims = L;
        
                                % Combine set of neighbouring lmbs
                                set_pims_tt_lmb_update = cell(Num_Sen, 1);
                                current_Action = zeros(Num_Sen, 1);
                                detection_probabilities_future = zeros(Num_Sen, num_controls);
                                for j = 1:Num_Sen
                                    if j == SenNum
                                        u_j = u_i;
                                    else
                                        u_j = U_i{end}(j);
                                    end
                                    current_Action(j) = u_j;
                                    set_pims_tt_lmb_update{SenNum}{j, 1} = pims_tt_lmb_update{j, u_j};
                                    for it = size(set_pims_tt_lmb_update{SenNum}{j, 1}, 1):-1:1
                                        if size(set_pims_tt_lmb_update{SenNum}{j, 1}{it}.x, 2) == 0
                                            set_pims_tt_lmb_update{SenNum}{j, 1}(it) = [];
                                        end
                                    end
                                end

                                idx = find(all(saved_Actions{SenNum} == current_Action));
                                if idx
                                    final_fused_pims_dist_tt_lmb_update{SenNum, u_i} = saved_pis{SenNum}{idx};
                                else
                                    if flag == 3
                                        % Complementary Fusion                    
                                        fused_pims_dist_tt_lmb_update=cell(max(L_pims+1,1),1); %create cell consiting of L+1 iterations
                                        for l=1:L_pims+1
                                            fused_pims_dist_tt_lmb_update{l}.S=cell(Num_Sen,1); %%initialise cell structure
                                        end
                                        P_concat=[];
                                        tempVar=[];

                                        zero_lambdas = cell(Num_Sen, 1);

                                        SensorLocations_pims = SensorLocations{max(k-1,1)};
    
                                        for j = 1:Num_Sen
                                            if j == SenNum
                                                u_j = u_i;
                                            else
                                                u_j = U_i{end}(j);
                                            end
                                            SensorLocations_pims.positions(:,j) = pims_models{j,u_j}.SenLoc(:,j);
                                            fused_pims_dist_tt_lmb_update{1}.S{j} = pims_tt_lmb_update{j, u_j};
                                            for it = 1:size(pims_tt_lmb_update{j, u_j}, 1)
                                                prev_est = false;
                                                for index = 1:length(tt_lmb_predict{j})
                                                    if all(tt_lmb_predict{j}{index}.l == pims_tt_lmb_update{j, u_j}{it}.l)
                                                        pred_pos = mean(tt_lmb_predict{j}{index}.x, 2);
                                                        pims_pos = mean(pims_tt_lmb_update{j, u_j}{it}.x, 2);
                                                        if compute_pD(pims_models{j, u_j}, pims_pos, j, true, RHO_BUFFER, THETA_BUFFER) == 1% || compute_pD(pims_models{j, u_j}, pred_pos, j, true, RHO_BUFFER) == 1
                                                            prev_est = true;
                                                            break
                                                        end
                                                    end
                                                end

                                                if prev_est
                                                    fused_pims_dist_tt_lmb_update{1}.S{j}{it}.lambda = 1.0;
                                                else
                                                    fused_pims_dist_tt_lmb_update{1}.S{j}{it}.lambda = 0;
                                                end
                                            end
                                            zero_lambdas{j} = cellfun(@(s) s.lambda == 0, fused_pims_dist_tt_lmb_update{1}.S{j});
                                        end
                                        
                                        for j = 1:Num_Sen
                                            zero_lambdas{j} = cellfun(@(s) s.lambda == 0, fused_pims_dist_tt_lmb_update{1}.S{j});
                                            fused_pims_dist_tt_lmb_update{1}.S{j}(zero_lambdas{j}) = [];
                                        end

                                        [S_pims,blank,blank2] = GenerateGraph(pims_models{1},SensorLocations_pims, Num_Sen, COMMUNICATION_RANGE);

                                        for ell=2:L_pims+1 %%ell 1 is equivelant to p_0 (initial posterior before consensus)
                                            for Sen=1:Num_Sen%actorcount%size(model{1}.senLoc,2)
                                                if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %ismember(Sen,egoSensors)%not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                                                    idx=S_pims{Sen}; %get indexes of all neigbouring sensors
                                                    P_concat=cell(length(idx),1);
                                                    for i_=1:length(idx)
                                                        P_concat(i_)=fused_pims_dist_tt_lmb_update{ell-1}.S(idx(i_)); %dist_tt_lmb_update{ell-1}.S(idx(i)) %%concat all posteriors of neighbouring sensors
                                                    end
                                                    P_concat2=cell(sum(cellfun(@length,P_concat)),1);
                                                    indx=1;
                                                    for s_n=1:size(S_pims{Sen},2)  %%for all sensors, concatenate data if not empty
                                                        if ~isempty(P_concat{s_n})
                                                            P_concat2(indx:indx+length(P_concat{s_n})-1)=P_concat{s_n};
                                                        end
                                                        indx=indx+length(P_concat{s_n});
                                                    end
                                                    fused_pims_dist_tt_lmb_update{ell}.S{Sen}=vertcat(fused_pims_dist_tt_lmb_update{ell}.S{Sen},P_concat2);%EuclidianMerge2(fusiontemp3,filter{Sen});
                                                    fused_pims_dist_tt_lmb_update{ell}.S{Sen}=RemoveIntersection(fused_pims_dist_tt_lmb_update{ell}.S{Sen},filter);
                                                end
                                            end
                                            fused_pims_dist_tt_lmb_update{ell-1}=[];
                                        end
                                        for Sen=1:Num_Sen
                                            if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                                                fusiontemp2=cmpl_fusion_dist_flooding(fused_pims_dist_tt_lmb_update{ell}.S{Sen}, filter); %fuse neighbouring sensor information
                                                fused_pims_dist_tt_lmb_update{L+1}.S{Sen}=EuclidianMerge2(fusiontemp2,filter, Scenario);
                                            end
                                        end
                                        final_fused_pims_dist_tt_lmb_update{SenNum, u_i} = fused_pims_dist_tt_lmb_update{L+1}.S{SenNum};
                                        saved_counter{SenNum} = saved_counter{SenNum} + 1;
                                        saved_pis{SenNum}{saved_counter{SenNum}} = final_fused_pims_dist_tt_lmb_update{SenNum, u_i};
                                        saved_Actions{SenNum}(:, saved_counter{SenNum}) = current_Action;
                                    end
                                end
        
                                for it = 1:Num_Sen
                                    if it ~= SenNum
                                        d = Calculate_Distance_New(pims_models{SenNum, u_i}.SenLoc(:,SenNum), pims_models{it, U_i{end}(it)}.SenLoc(:,it));
                                        if d < min_d
                                            min_d = d;
                                        end
                                    end
                                end
                                psi = void_probability(model{SenNum}.SenLoc(:, SenNum), u_i, step_size, EXCLUSION_RADIUS, final_fused_pims_dist_tt_lmb_update{SenNum, u_i}) > PSI_THRESHOLD;
                                eta = min_d > ETA_THRESHOLD;
                                combined_penalty = ((1-psi) + (1-eta)) * DC_PEN;
                                dcs{SenNum}(u_i) = D_KL_final(final_fused_pims_dist_tt_lmb_update{SenNum, u_i}, tt_lmb_predict{SenNum}, filter) - combined_penalty;
                                Actions{SenNum}(u_i) = u_i;
                            end

                            actions = 1:num_controls;
                            max_idxs = (dcs{SenNum} == max(dcs{SenNum}));
                            max_actions = actions(max_idxs);
                            final_idx = randi(length(max_actions), 1);
                            % Keep action the same, even if multiple
                            % have same KLD, to allow for termination
                            if any(ismember(max_actions, U_old{end}(SenNum)))
                                U_i{1, it_f}(SenNum) = U_old{end}(SenNum);
                            else
                                U_i{1, it_f}(SenNum) = max_actions(final_idx);
                            end
                            dcs_i{SenNum, it_f} = max(dcs{SenNum});

                            disp("U (" + SenNum + "): " + U_i{1, it_f})

                            if SenNum == Num_Sen
                                if flood_counter >= 100
                                    stop_flag = 1;
                                    stopped_sensor_idx = Num_Sen;
                                    break;
                                end
                                if any(cellfun(@(c) isequal(c, U_i{1, it_f}), U_old))
                                    stop_flag = 1;
                                    stopped_sensor_idx = Num_Sen;
                                    break;
                                else
                                    U_old{end+1} = U_i{1, it_f};
                                    dcs_i_old{i} = dcs_i{i, it_f};
                                    stopped_sensor_idx = i;
                                end
                                flood_counter = flood_counter + 1;
                            end
                        end
                    end
    
                    disp(U_i{1, it_f})
                    disp("FDCD counter: " + flood_counter)
                    U_max = U_i{1, it_f};
                end
            end

            %% Distributed Coordinate Descent
            if DCD == true
                % Aidan Blair 2025
                % Iterate for m_max times, choose best option
                m_max = 10;
                C_final = cell(Num_Sen, 1);
                U_final = cell(Num_Sen, 1);
                saved_pis = cell(Num_Sen, 1);
                saved_Actions = cell(Num_Sen, 1);
                saved_counter = cell(Num_Sen, 1);
                for s = 1:Num_Sen
                    saved_pis{s} = cell(100, 1);
                    saved_Actions{s} = zeros(size(neighbor_indices{s}, 2), 100);
                    saved_counter{s} = 0;
                end
                for m = 1:m_max
                    for SenNum = 1:size(model{1}.SenLoc, 2)
                        for u = 1:num_controls
                            pims_meas{SenNum, u}.Z{k, 1} = [];
                            for me = 1:size(pims_meas_temp{SenNum}, 2)
                                pD_ = compute_pD(pims_models{SenNum, u}, pims_meas_temp{SenNum}(:, me), SenNum, true);
                                if rand() < pD_
                                    pims_meas{SenNum, u}.Z{k, 1} = [pims_meas{SenNum, u}.Z{k, 1}, [pims_meas_temp{SenNum}(1, me) + pims_meas_temp{SenNum}(2, me); pims_meas_temp{SenNum}(3, me) + pims_meas_temp{SenNum}(4, me)]];
                                end
                            end
                            pims_glmb_update{SenNum, u} = [];
                            pims_glmb_update{SenNum, u}= up_date(glmb_predict{SenNum},pims_models{SenNum, u},filter,pims_meas{SenNum, u},k,SenNum);
                            pims_tt_lmb_update{SenNum, u}= glmb2lmb2_dist(pims_glmb_update{SenNum, u}); %%Tharindu's preallocated version
                            pims_T_posterior(SenNum, u)= length(pims_tt_lmb_update{SenNum, u});
                        end
                    end
                    % Initialize random actions
                    U = cell(Num_Sen, 1);
                    U_old = cell(Num_Sen, 1);
                    % Uncomment to have same rng each time
                    for i = 1:Num_Sen
                        U{i} = randi([1, (num_controls - 1)], 1, size(neighbor_indices{i}, 2));
                        U_old{i} = zeros(size(U{i}));
                    end
                    
                    % For each local sensor
                    dcs = cell(Num_Sen, 1);
                    [SelectedNodes, temp_neighbor_indices, currentNode] = DistributedNodeSelection(neighbor_indices);
                    temp_neighbor_indices = neighbor_indices;
                    SelectedNodes = 1:Num_Sen;

                    % Counter just in case it goes on forever
                    u_count = 0;
                    u_max = 10;
                    counter_max = 10;
                    L_pims = 1;
                    
                    for sen_counter = 1:length(SelectedNodes)
                        SenNum = SelectedNodes(sen_counter);
                        counter = 0;
                        if sen_counter == 1
                            u_count = u_count + 1;
                        end
                        if u_count > u_max
                            break
                        end

                        while any(U{SenNum} ~= U_old{SenNum}) && (counter < counter_max)
                            % Save U_old
                            U_old{SenNum} = U{SenNum};
                            % For each sensor in the local neighbourhood
                            for n = 1:size(temp_neighbor_indices{SenNum}, 2)
                                final_fused_pims_dist_tt_lmb_update = cell(size(temp_neighbor_indices{SenNum}, 2), 1);
                                % For each possible action
                                for u = 1:num_controls
                                    BREAK_DIST = COMMUNICATION_RANGE - COMMUNICATION_BUFFER;
                                    DC_PEN = 50;
                                    min_d = inf;

                                    % Combine set of neighbouring lmbs
                                    current_Action = zeros(size(temp_neighbor_indices{SenNum}, 2), 1);
                                    for j = 1:size(temp_neighbor_indices{SenNum}, 2)
                                        if j == n
                                            u_j = u;
                                        else
                                            u_j = U{SenNum}(j);
                                        end
                                        current_Action(j) = u_j;
                                    end

                                    idx = find(all(saved_Actions{SenNum} == current_Action));
                                    if idx
                                        final_fused_pims_dist_tt_lmb_update{SenNum} = saved_pis{SenNum}{idx};
                                    else
                                        if flag == 3
                                            % Complementary Fusion                    
                                            fused_pims_dist_tt_lmb_update=cell(max(L_pims+1,1),1); %create cell consiting of L+1 iterations
                                            for l=1:L_pims+1
                                                fused_pims_dist_tt_lmb_update{l}.S=cell(Num_Sen,1); %%initialise cell structure
                                            end
                                            P_concat=[];
                                            tempVar=[];

                                            zero_lambdas = cell(size(temp_neighbor_indices{SenNum}, 2), 1);

                                            SensorLocations_pims = SensorLocations{max(k-1,1)};
        
                                            for j = 1:size(temp_neighbor_indices{SenNum}, 2)
                                                Sen = temp_neighbor_indices{SenNum}(j);
                                                if j == n
                                                    u_j = u;
                                                else
                                                    u_j = U{SenNum}(j);
                                                end
                                                SensorLocations_pims.positions(:,Sen) = pims_models{Sen,u_j}.SenLoc(:,Sen);
                                                fused_pims_dist_tt_lmb_update{1}.S{Sen} = pims_tt_lmb_update{Sen, u_j};
                                                for it = 1:size(pims_tt_lmb_update{Sen, u_j}, 1)
                                                    prev_est = false;
                                                    for index = 1:length(tt_lmb_predict{Sen})
                                                        if all(tt_lmb_predict{Sen}{index}.l == pims_tt_lmb_update{Sen, u_j}{it}.l)
                                                            pred_pos = mean(tt_lmb_predict{Sen}{index}.x, 2);
                                                            pims_pos = mean(pims_tt_lmb_update{Sen, u_j}{it}.x, 2);
                                                            if compute_pD(pims_models{Sen, u_j}, pims_pos, Sen, true, RHO_BUFFER, THETA_BUFFER) == 1 || compute_pD(pims_models{Sen, u_j}, pred_pos, Sen, true, RHO_BUFFER, THETA_BUFFER) == 1
                                                                prev_est = true;
                                                                break
                                                            end
                                                        end
                                                    end

                                                    if prev_est
                                                        fused_pims_dist_tt_lmb_update{1}.S{Sen}{it}.lambda = 1.0;
                                                    else
                                                        fused_pims_dist_tt_lmb_update{1}.S{Sen}{it}.lambda = 0;
                                                    end
                                                end
                                                zero_lambdas{j} = cellfun(@(s) s.lambda == 0, fused_pims_dist_tt_lmb_update{1}.S{Sen});
                                            end
                                            
                                            for j = 1:size(temp_neighbor_indices{SenNum}, 2)
                                                Sen = temp_neighbor_indices{SenNum}(j);
                                                zero_lambdas{Sen} = cellfun(@(s) s.lambda == 0, fused_pims_dist_tt_lmb_update{1}.S{Sen});
                                                fused_pims_dist_tt_lmb_update{1}.S{Sen}(zero_lambdas{Sen}) = [];
                                            end

                                            [S_pims,blank,blank2] = GenerateGraph(pims_models{Sen,1},SensorLocations_pims,Num_Sen,COMMUNICATION_RANGE);

                                            for ell=2:L_pims+1 %%ell 1 is equivelant to p_0 (initial posterior before consensus)
                                                for it = 1:size(temp_neighbor_indices{SenNum}, 2)
                                                    Sen = temp_neighbor_indices{SenNum}(it);
                                                    if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %ismember(Sen,egoSensors)%not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                                                        idx=S_pims{Sen}; %get indexes of all neigbouring sensors
                                                        P_concat=cell(length(idx),1);
                                                        for i_=1:length(idx)
                                                            P_concat(i_)=fused_pims_dist_tt_lmb_update{ell-1}.S(idx(i_)); %dist_tt_lmb_update{ell-1}.S(idx(i)) %%concat all posteriors of neighbouring sensors
                                                        end
                                                        P_concat2=cell(sum(cellfun(@length,P_concat)),1);
                                                        indx=1;
                                                        for s_n=1:size(S_pims{Sen},2)  %%for all sensors, concatenate data if not empty
                                                            if ~isempty(P_concat{s_n})
                                                                P_concat2(indx:indx+length(P_concat{s_n})-1)=P_concat{s_n};
                                                            end
                                                            indx=indx+length(P_concat{s_n});
                                                        end
                                                        fused_pims_dist_tt_lmb_update{ell}.S{Sen}=vertcat(fused_pims_dist_tt_lmb_update{ell}.S{Sen},P_concat2);%EuclidianMerge2(fusiontemp3,filter{Sen});
                                                        fused_pims_dist_tt_lmb_update{ell}.S{Sen}=RemoveIntersection(fused_pims_dist_tt_lmb_update{ell}.S{Sen},filter);
                                                    end
                                                end
                                                fused_pims_dist_tt_lmb_update{ell-1}=[];
                                            end
                                            for it = 1:size(temp_neighbor_indices{SenNum}, 2)
                                                Sen = temp_neighbor_indices{SenNum}(it);
                                                if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                                                    fusiontemp2=cmpl_fusion_dist_flooding(fused_pims_dist_tt_lmb_update{ell}.S{Sen}, filter); %fuse neighbouring sensor information
                                                    fused_pims_dist_tt_lmb_update{L+1}.S{Sen}=EuclidianMerge2(fusiontemp2,filter, Scenario);
                                                end
                                            end
                                            final_fused_pims_dist_tt_lmb_update{SenNum} = fused_pims_dist_tt_lmb_update{L+1}.S{SenNum};
                                            saved_counter{SenNum} = saved_counter{SenNum} + 1;
                                            saved_pis{SenNum}{saved_counter{SenNum}} = final_fused_pims_dist_tt_lmb_update{SenNum};
                                            saved_Actions{SenNum}(:, saved_counter{SenNum}) = current_Action;
                                        end
                                    end
            
                                    for it = 1:size(temp_neighbor_indices{SenNum}, 2)
                                        Sen = temp_neighbor_indices{SenNum}(it);
                                        if Sen ~= SenNum
                                            d = Calculate_Distance_New(pims_models{SenNum, u}.SenLoc(:,SenNum), pims_models{Sen, U{SenNum}(it)}.SenLoc(:,Sen));
                                            if d < min_d
                                                min_d = d;
                                            end
                                        end
                                    end
                                    psi = void_probability(model{SenNum}.SenLoc(:, SenNum), u, step_size, EXCLUSION_RADIUS, final_fused_pims_dist_tt_lmb_update{SenNum}) > PSI_THRESHOLD;
                                    eta = min_d > ETA_THRESHOLD;
                                    combined_penalty = ((1-psi) + (1-eta)) * DC_PEN;
                                    dcs{SenNum}(n, u) = D_KL_final(final_fused_pims_dist_tt_lmb_update{SenNum}, tt_lmb_predict{SenNum}, filter) - combined_penalty;
                                    counter = counter + 1;
                                end
                                % Coordinate descent
                                u_idx = find(dcs{SenNum}(n, :) == max(dcs{SenNum}(n, :)));
                                if (max(dcs{SenNum}(n, :)) == dcs{SenNum}(n,U{SenNum}(n)))
                                    % If min value is the same, and multiple actions
                                    % have it (e.g. 0), use same action
                                    U{SenNum}(n) = U{SenNum}(n);
                                else
                                    U{SenNum}(n) = u_idx(randi(length(u_idx), 1));
                                end
                            end
                        end
                        U_final{SenNum}(m, :) = U{SenNum};
                        C_final{SenNum}(m) = dcs{SenNum}(end, U{SenNum}(end));
                        disp(num2str(SenNum) + ": " + num2str(m) + ", " + num2str(u_count))
                    end
                end

                U_final_ = cell(Num_Sen, 1);
                C_final_ = cell(Num_Sen, 1);
                for i = 1:Num_Sen
                    max_C_idxs = find(C_final{i} == max(C_final{i}));
                    if size(max_C_idxs, 2) > 1
                        max_C = max_C_idxs(randi(length(max_C_idxs), 1));
                    else
                        max_C = max_C_idxs;
                    end
                    U_final_{i} = U_final{i}(max_C, :);
                    C_final_{i} = C_final{i}(max_C);
                end
                U_max = zeros(Num_Sen, 1);
                U_final_dist = cell(Num_Sen, 1);
                C_final_dist = cell(Num_Sen, 1);
                for i = 1:Num_Sen
                    for n = 1:size(neighbor_indices{i}, 2)
                        if any(ismember(SelectedNodes, neighbor_indices{i}(n)))
                            U_final_dist{i} = [U_final_dist{i}, U_final_{neighbor_indices{i}(n)}(find(neighbor_indices{neighbor_indices{i}(n)}(:) == i))];
                            C_final_dist{i} = [C_final_dist{i}, C_final_{neighbor_indices{i}(n)}];
                        end
                    end
                    C_max = max(C_final_dist{i});
                    C_max_idx = find(C_final_dist{i} == C_max);
                    if length(C_max_idx) > 1
                        C_max_idx = C_max_idx(randi(length(C_max_idx), 1)); 
                    end
                    U_max(i) = U_final_dist{i}(C_max_idx);

                end
                disp("U max:")
                disp(U_max)
            end
            times((run-1)*truth.K + k) = toc;
            for i = 1:Num_Sen
                if Fixed == true
                    if valid_actions == 1
                        U_max(i) = 2;
                    else
                        U_max(i) = 1;
                    end
                end

                if valid_actions == 1
                    if U_max(i) <= 3
                        angle = (U_max(i) - 2) * ang_incr;
                        x = 0;
                        y = 0;
                    else
                        angle = 0;
                        x = step_size * cos((U_max(i) - 4) * (2 * pi / directions));
                        y = step_size * sin((U_max(i) - 4) * (2 * pi / directions));
                    end
                elseif valid_actions == 2
                    if U_max(i) == 1
                        angle = 0;
                        x = 0;
                        y = 0;
                    else
                        angle = 0;
                        x = step_size * cos((U_max(i) - 2) * (2 * pi / 8));
                        y = step_size * sin((U_max(i) - 2) * (2 * pi / 8));
                    end
                end
                Sen_Locations(k, 1, i) = Sen_Locations(k-1, 1, i) + x;
                Sen_Locations(k, 2, i) = Sen_Locations(k-1, 2, i) + y;
                Sen_Headings(k, i) = Sen_Headings(k-1, i) + angle;
            end
            SensorLocations{k}.positions = reshape(Sen_Locations(k, :, :), [2, Num_Sen]);
            SensorHeadings{k} = Sen_Headings(k, :);
            for i = 1:Num_Sen
                model{i}.SenLoc = SensorLocations{k}.positions;
                model{i}.Heading = SensorHeadings{k}(i);
            end
            for i = 1:Num_Sen
                near_counter = 0;
                for j = 1:Num_Sen
                    if j ~= i
                        if Calculate_Distance(Sen_Locations(k, :, i), Sen_Locations(k, :, j), 0, 0) <= COMMUNICATION_RANGE
                            near_counter = near_counter + 1;
                        end
                    end
                end
                if near_counter == 0
                    disp("near_counter")
                end
            end
        else
            Sen_Locations(k, :, :) = Positions_{run};
            Sen_Headings(k, :) = Orientations_{run};
        end
        %% Measurements
        if k == 1
            SensorLocations{k}.positions = Positions_{run};
            SensorHeadings{k} = Orientations_{run};
        else
            SensorLocations{k}.positions = reshape(Sen_Locations(k, :, :), 2, Num_Sen);
            SensorHeadings{k} = Sen_Headings(k, :);
        end
        SensorLocations{k}.actorIDs = [1, 2, 3, 4, 5, 6];

        for i = 1:Num_Sen
            model{i}.SenLoc = SensorLocations{k}.positions;
            model{i}.Heading = SensorHeadings{k}(i);
        end
        for s = 1:Num_Sen
            pD = [];
            if s == 1
                [meas{s}.Z{k,1}, sums] = gen_observation(model{s}, truth.X{k}, SensorLocations{k}.positions(:, s), sums);
            else
                [meas{s}.Z{k,1}, blank] = gen_observation(model{s}, truth.X{k}, SensorLocations{k}.positions(:, s), sums);
            end
            meas{s}.K = k;
            if size(meas{s}.Z{k, 1}, 2) > 0
                temp_meas = [meas{s}.Z{k, 1}(1, :); zeros(size(meas{s}.Z{k, 1}(1, :))); meas{s}.Z{k, 1}(2, :); zeros(size(meas{s}.Z{k, 1}(1, :))); zeros(size(meas{s}.Z{k, 1}(1, :)))];
                pD = compute_pD(model{s}, temp_meas, s, false);
            end
            meas_temp = [];
            for i = 1:size(meas{s}.Z{k, 1}, 2)
                r = rand();
                if (r < pD(i)) || (k==1)
                    meas_temp = [meas_temp, meas{s}.Z{k, 1}(:, i)];
                end
            end
            meas{s}.Z{k, 1} = meas_temp;
        end
        for s = 1:Num_Sen
            N_c= poissrnd(model{s}.lambda_c);    %no. of clutter points
            C= repmat(model{s}.range_c(:,1),[1 N_c])+ diag(model{s}.range_c*[ -1; 1 ])*rand(model{s}.z_dim,N_c);  %clutter generation
            if size(C, 2) > 0
                C = C + model{s}.SenLoc(:, s);
                C_ = [];
                for c = 1:size(C, 2)
                    range_x = C(1, c) - model{s}.SenLoc(1, s);
                    range_y = C(2, c) - model{s}.SenLoc(2, s);
                    theta = atan2(range_y, range_x);
                    if theta < -pi
                        theta = 2 * pi + theta;
                    end
                    if theta > pi
                        theta = theta - 2 * pi;
                    end
                    min_angle = model{s}.Heading - model{s}.FOV;
                    max_angle = model{s}.Heading + model{s}.FOV;
                    if theta < min_angle
                        theta = theta + 2 * pi;
                    end
                    if theta > max_angle
                        theta = theta - 2 * pi;
                    end
                    if (theta >= min_angle) && (theta <= max_angle)
                        C_ = [C_, C(:, c)];
                    end
                end
            else
                C_ = [];
            end
            meas{s}.Z{k} = [ meas{s}.Z{k} C_ ];
        end
        %% update

        tic;
        switch distributed
            case 0 %centralized
                for SenNum=1:size(model.SenLoc,2)
                    glmb_update = [];
                    glmb_update= up_date(glmb_predict,model,filter,meas{1,SenNum},k,SenNum);
                    tt_lmb_update{SenNum}= glmb2lmb2_dist(glmb_update); %%Tharindu's preallocated version
                    T_posterior(SenNum)= length(tt_lmb_update);
                end
            case 1 %distributed
                for SenNum=1:size(model{1}.SenLoc,2) %generate on a per sensor basis
                    glmb_update{SenNum} = [];
                    glmb_update{SenNum}= up_date(glmb_predict{SenNum},model{SenNum},filter,meas{1,SenNum},k,SenNum);
                    tt_lmb_update{SenNum}= glmb2lmb2_dist(glmb_update{SenNum}); %%Tharindu's preallocated version
                    T_posterior(SenNum)= length(tt_lmb_update{SenNum});

                    temp_tt_lmb_update=[];
                    temp_tt_lmb_update2=[];
                end
        end
        if ~isreal(tt_lmb_update{1}{1}.w)
            pause(5);
        end

        before_tt_lmb_update = tt_lmb_update;

        et(k,run)=toc;
        %% History
        track_history = cell(Num_Sen, 1);
        for Sen = 1:Num_Sen
            for it = 1:length(tt_lmb_update{Sen})
                if tt_lmb_update{Sen}{it}.r > filter.track_threshold
                    track_history{Sen} = [track_history{Sen}, tt_lmb_update{Sen}{it}.l];
                end
            end
        end
        %% fusion method
        if flag == 1
            switch distributed
                case 0 %centralized case
                    [tt_lmb_update]=CS5_fusion(tt_lmb_update,size(model.SenLoc,2));
                case 1 %distributed case
                    dist_tt_lmb_update=cell(max(L+1,1),1); %create cell consiting of L+1 iterations
                    for l=1:L+1
                        dist_tt_lmb_update{l}.S=cell(Num_Sen,1); %%initialise cell structure
                    end
                    P_concat=[];
                    tempVar=[];
                    for Sen=1:size(model{1}.SenLoc,2)
                        dist_tt_lmb_update{1}.S{Sen}=tt_lmb_update{Sen}; %first consensus iteration needs information from all unfused, neighbouriung sensors
                    end

                    for ell=2:L+1 %%ell 1 is equivelant to p_0 (initial posterior before consensus)
                        if ismember(Sen,truth.L{k})||ismember(Sen,[6,7,8,9]) %not all sensors are in the scenario at time k
                            for Sen=1:size(model{1}.SenLoc,2)
                                idx=S{Sen}; %get indexes of all neigbouring sensors
                                for i=1:length(idx)
                                    P_concat=vertcat(P_concat,dist_tt_lmb_update{ell-1}.S(idx(i))); %dist_tt_lmb_update{ell-1}.S(idx(i)) %%concat all posteriors of neighbouring sensors
                                end

                                fusiontemp=SupportUnification(P_concat,size(S{Sen},2),filter);

                                fusiontemp2=CS5_fusion(fusiontemp,size(S{Sen},2)); %fuse neighbouring sensor information
                                dist_tt_lmb_update{ell}.S{Sen}=EuclidianMerge2(fusiontemp2, filter, Scenario);
                                dist_tt_lmb_update{L+1}.S{Sen} = clean_lmb(dist_tt_lmb_update{L+1}.S{Sen},filter);
                                P_concat=[];
                                tempVar=[];
                                fusiontemp=[];
                                fusiontemp2=[];
                                fusiontemp3=[];
                            end
                            dist_tt_lmb_update{ell-1}=[];
                        end
                    end
            end
        elseif flag==2
            switch distributed
            case 0 %centralized case
                for Sen=1:actorcount
                    idxLocation3=[];
                    idxLocation3=find(Sen==sensorsinscene);
                    if ismember(Sen,egoSensors)||ismember(Sen,stationarySensors)
                        if ~(ismember(sensorsinscene(idxLocation3),truth.L{k})||ismember(sensorsinscene(idxLocation3),stationarySensors)) %%incorrect to fuse sensors which no longer exist in scene
                            tt_lmb_update{idxLocation3}=[];
                        end
                    end
                end
                [tt_lmb_update]=weighted_GCI_fusion(tt_lmb_update,length(sensorsinscene),sensorLocations(:,~cellfun(@isempty,tt_lmb_update)),egoSensors,stationarySensors,model);
            case 1 %distributed case
                dist_tt_lmb=cell(max(L+1,1),1); %create cell consiting of L+1 iterations
                for l=1:L+1
                    dist_tt_lmb{l}.S=cell(30,1); %%initialise cell structure
                end
                dist_tt_lmb_update=cell(max(L+1,1),1); %create cell consiting of L+1 iterations
                for l=1:L+1
                    dist_tt_lmb_update{l}.S=cell(Num_Sen,1); %%initialise cell structure
                end
                P_concat=[];P_concat2=[];P_concat3=[];
                tempVar=[];
                for Sen=1:Num_Sen%size(model{1}.senLoc,2)
                    dist_tt_lmb{1}.S{Sen}=tt_lmb_update{Sen}; %first consensus iteration needs information from all unfused, neighbouriung sensors
                    for element=1:length(dist_tt_lmb{1}.S{Sen})
                        dist_tt_lmb{1}.S{Sen}{element}.first_communicated_sensor=Sen;
                        authority = false;
                        for index = 1:length(tt_lmb_predict{Sen})
                            if tt_lmb_predict{Sen}{index}.l == tt_lmb_update{Sen}{element}.l
                                pos = mean(tt_lmb_update{Sen}{element}.x, 2);
                                if (compute_pD(model{Sen}, pos, Sen, true) == 1) || ((tt_lmb_update{Sen}{element}.r - tt_lmb_predict{Sen}{index}.r < 0) && (tt_lmb_predict{Sen}{index}.r > filter.track_threshold))
                                    authority = true;
                                    break
                                end
                            end
                        end
                        if authority
                            dist_tt_lmb_update{1}.S{Sen}{element}.lambda = 1;
                        else
                            dist_tt_lmb_update{1}.S{Sen}{element}.lambda = 1e-5;
                        end
                    end
                end

                for ell=2:L+1 %%ell 1 is equivelant to p_0 (initial posterior before consensus)
                    for Sen=1:Num_Sen%actorcount%size(model{1}.senLoc,2)
                        if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %ismember(Sen,egoSensors)%not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                            idx=S{Sen}; %get indexes of all neigbouring sensors
                            P_concat=cell(length(idx),1);
                            for i=1:length(idx)
                                P_concat(i)=dist_tt_lmb{ell-1}.S(idx(i)); %dist_tt_lmb_update{ell-1}.S(idx(i)) %%concat all posteriors of neighbouring sensors
                            end
                            P_concat2=cell(sum(cellfun(@length,P_concat)),1);
                            indx=1;
                            for s_n=1:size(S{Sen},2)  %%for all sensors, concatenate data if not empty
                                if ~isempty(P_concat{s_n})
                                    P_concat2(indx:indx+length(P_concat{s_n})-1)=P_concat{s_n};
                                end
                                indx=indx+length(P_concat{s_n});
                            end
                            dist_tt_lmb{ell}.S{Sen}=vertcat(dist_tt_lmb{ell}.S{Sen},P_concat2);%EuclidianMerge2(fusiontemp3,filter{Sen});
                            dist_tt_lmb{ell}.S{Sen}=RemoveIntersection(dist_tt_lmb{ell}.S{Sen},filter);
                        end
                    end
                    dist_tt_lmb{ell-1}=[];
                end
                for Sen=1:Num_Sen%size(model{1}.senLoc,2)
                    if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                        fusiontemp=[]; fusiontemp2=[];
                        fusiontemp=LabelCorrection_James(dist_tt_lmb{L+1}.S{Sen},filter,k);
                        fusiontemp2=GCI_fusion_dist(fusiontemp,meas,SensorLocations{k}.positions,SensorHeadings{k},egoSensors,stationarySensors,model{Sen},k,filter); % Sen,weightMatrix(idxLocation,:) sensorLocations(:,~cellfun(@isempty,tt_lmb_update)) (:,~cellfun(@isempty,tt_lmb_update)) SensorLocations{k}.positions 
                        dist_tt_lmb_update{L+1}.S{Sen}=EuclidianMerge2_James(fusiontemp2,filter.merging_threshold,k);
                    end
                end
        end
        elseif flag==3
            switch distributed
                case 0
                    [tt_lmb_update]=cmpl_fusion(tt_lmb_update,size(model.SenLoc,2));
                    fusiontemp=[];
                case 1
                    if strcmp(FloodingOrConsensus, 'consensus')
                        dist_tt_lmb_update=cell(max(L+1,1),1); %create cell consiting of L+1 iterations
                        for l=1:L+1
                            dist_tt_lmb_update{l}.S=cell(Num_Sen,1); %%initialise cell structure
                        end
                        P_concat=[];
                        tempVar=[];
                        for Sen=1:size(model{1}.SenLoc,2)
                            dist_tt_lmb_update{1}.S{Sen}=tt_lmb_update{Sen}; %first consensus iteration needs information from all unfused, neighbouriung sensors
                            non_zero = false;

                            for it = 1:size(tt_lmb_update{Sen}, 1)
                                authority = false;
                                for j = 1:Num_Sen
                                    for index = 1:length(tt_lmb_predict{Sen})
                                        if tt_lmb_predict{Sen}{index}.l == tt_lmb_update{Sen}{it}.l
                                            pos = mean(tt_lmb_update{Sen}{it}.x, 2);
                                            if (compute_pD(model{Sen}, pos, Sen, true) == 1) || ((tt_lmb_update{Sen}{it}.r - tt_lmb_predict{Sen}{index}.r < 0) && (tt_lmb_predict{Sen}{index}.r > filter.track_threshold))
                                                authority = true;
                                                break
                                            end
                                        end
                                    end
                                end
                                if authority
                                    dist_tt_lmb_update{1}.S{Sen}{it}.lambda = 1;
                                    non_zero = true;
                                else
                                    dist_tt_lmb_update{1}.S{Sen}{it}.lambda = 0;
                                end
                            end
                            zero_lambda = cellfun(@(s) s.lambda == 0, dist_tt_lmb_update{1}.S{Sen});
                            dist_tt_lmb_update{1}.S{Sen}(zero_lambda) = [];
                        end
    
                        for ell=2:L+1 %%ell 1 is equivelant to p_0 (initial posterior before consensus)
                            for Sen=1:size(model{1}.SenLoc,2)
                                if ismember(Sen,truth.L{k})||ismember(Sen,1:50) %not all sensors are in the scenario at time k
                                    idx=S{Sen}; %get indexes of all neigbouring sensors
                                    for i=1:length(idx)
                                        P_concat=vertcat(P_concat,dist_tt_lmb_update{ell-1}.S(idx(i))); %dist_tt_lmb_update{ell-1}.S(idx(i)) %%concat all posteriors of neighbouring sensors
                                    end

                                    fusiontemp2 = cmpl_fusion_dist(P_concat, size(S{Sen}, 2), filter); %fuse neighbouring sensor information
                                    dist_tt_lmb_update{ell}.S{Sen}=EuclidianMerge2(fusiontemp2,filter, Scenario);

                                    for it = 1:size(dist_tt_lmb_update{ell}.S{Sen}, 1)
                                        authority = false;
                                        for index = 1:length(tt_lmb_predict{Sen})
                                            if tt_lmb_predict{Sen}{index}.l == dist_tt_lmb_update{ell}.S{Sen}{it}.l
                                                pos = mean(dist_tt_lmb_update{ell}.S{Sen}{it}.x, 2);
                                                if (compute_pD(model{Sen}, pos, Sen, true) == 1) || ((dist_tt_lmb_update{ell}.S{Sen}{it}.r - tt_lmb_predict{Sen}{index}.r < 0) && (tt_lmb_predict{Sen}{index}.r > filter.track_threshold))
                                                    authority = true;
                                                    break
                                                end
                                            end
                                        end
                                        if authority
                                            dist_tt_lmb_update{ell}.S{Sen}{it}.lambda = 1.0;
                                        else
                                            dist_tt_lmb_update{ell}.S{Sen}{it}.lambda = 0;
                                        end
                                    end
                                    zero_lambda = cellfun(@(s) s.lambda == 0, dist_tt_lmb_update{ell}.S{Sen});
                                    dist_tt_lmb_update{ell}.S{Sen}(zero_lambda) = [];
    
                                    P_concat=[];
                                    tempVar=[];
                                    fusiontemp=[];
                                    fusiontemp2=[];
                                end
                            end
                            dist_tt_lmb_update{ell-1}=[];
                        end
                    elseif strcmp(FloodingOrConsensus, 'flooding')
                        dist_tt_lmb_update=cell(max(L+1,1),1); %create cell consiting of L+1 iterations
                        for l=1:L+1
                            dist_tt_lmb_update{l}.S=cell(Num_Sen,1); %%initialise cell structure
                        end
                        P_concat=[];
                        tempVar=[];
                        for Sen=1:size(model{1}.SenLoc,2)
                            dist_tt_lmb_update{1}.S{Sen}=tt_lmb_update{Sen}; %first consensus iteration needs information from all unfused, neighbouriung sensors
                            non_zero = false;

                            for it = 1:size(tt_lmb_update{Sen}, 1)
                                authority = false;
                                for index = 1:length(tt_lmb_predict{Sen})
                                    if all(tt_lmb_predict{Sen}{index}.l == tt_lmb_update{Sen}{it}.l)
                                        pred_pos = mean(tt_lmb_predict{Sen}{index}.x, 2);
                                        pims_pos = mean(tt_lmb_update{Sen}{it}.x, 2);
                                        if compute_pD(model{Sen}, pims_pos, Sen, true) == 1 || compute_pD(model{Sen}, pred_pos, Sen, true) == 1
                                            authority = true;
                                            break
                                        end
                                    end
                                end

                                if authority
                                    dist_tt_lmb_update{1}.S{Sen}{it}.lambda = 1;
                                    non_zero = true;
                                else
                                    dist_tt_lmb_update{1}.S{Sen}{it}.lambda = 0;
                                end
                            end

                            for it = 1:length(tt_lmb_update{1})
                                at_least_one = false;
                                l = tt_lmb_update{1}{it}.l;
                                for j = 1:Num_Sen
                                    for it2 = 1:length(dist_tt_lmb_update{1}.S{j})
                                        if all(dist_tt_lmb_update{1}.S{j}{it2}.l == l)
                                            if dist_tt_lmb_update{1}.S{j}{it2}.lambda == 1.0
                                                at_least_one = true;
                                            end
                                            break
                                        end
                                    end
                                end
                                if ~at_least_one
                                    for j = 1:Num_Sen
                                        for it2 = 1:length(dist_tt_lmb_update{1}.S{j})
                                            if all(dist_tt_lmb_update{1}.S{j}{it2}.l == l)
                                                dist_tt_lmb_update{1}.S{j}{it2}.lambda = 1.0;
                                                break
                                            end
                                        end
                                    end
                                end
                            end

                            zero_lambda = cellfun(@(s) s.lambda == 0, dist_tt_lmb_update{1}.S{Sen});
                            dist_tt_lmb_update{1}.S{Sen}(zero_lambda) = [];
                        end

                        for ell=2:L+1 %%ell 1 is equivelant to p_0 (initial posterior before consensus)
                            for Sen=1:Num_Sen%actorcount%size(model{1}.senLoc,2)
                                if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %ismember(Sen,egoSensors)%not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                                    idx=S{Sen}; %get indexes of all neigbouring sensors
                                    P_concat=cell(length(idx),1);
                                    for i=1:length(idx)
                                        P_concat(i)=dist_tt_lmb_update{ell-1}.S(idx(i)); %dist_tt_lmb_update{ell-1}.S(idx(i)) %%concat all posteriors of neighbouring sensors
                                    end
                                    P_concat2=cell(sum(cellfun(@length,P_concat)),1);
                                    indx=1;
                                    for s_n=1:size(S{Sen},2)  %%for all sensors, concatenate data if not empty
                                        if ~isempty(P_concat{s_n})
                                            P_concat2(indx:indx+length(P_concat{s_n})-1)=P_concat{s_n};
                                        end
                                        indx=indx+length(P_concat{s_n});
                                    end
                                    dist_tt_lmb_update{ell}.S{Sen}=vertcat(dist_tt_lmb_update{ell}.S{Sen},P_concat2);%EuclidianMerge2(fusiontemp3,filter{Sen});
                                    dist_tt_lmb_update{ell}.S{Sen}=RemoveIntersection(dist_tt_lmb_update{ell}.S{Sen},filter);
                                end
                            end
                            dist_tt_lmb_update{ell-1}=[];
                        end
                        for Sen=1:Num_Sen%size(model{1}.senLoc,2)
                            if ismember(Sen,truth.L{k})||ismember(Sen,stationarySensors) %not all sensors are in the scenario at time k [6,7,8,9][16,17,18,19,20]
                                fusiontemp2 = cmpl_fusion_dist_flooding(dist_tt_lmb_update{ell}.S{Sen}, filter); %fuse neighbouring sensor information
                                dist_tt_lmb_update{ell}.S{Sen}=EuclidianMerge2(fusiontemp2,filter, Scenario);
                            end
                        end
                    end
                    
            end
        end

        %% pruning, truncation and track cleanup
        unclean_tt_lmb_update = tt_lmb_update;
        switch distributed
            case 0
                tt_lmb_update= clean_lmb(tt_lmb_update,filter);
                tt_lmb_update_tem{k} = tt_lmb_update;
                T_clean= length(tt_lmb_update);
            case 1
                labels_to_delete = [];
                var_cutoff = 50;
                for Senidx=1:size(model{1}.SenLoc,2)
                    dist_tt_lmb_update{L+1}.S{Senidx} = clean_lmb(dist_tt_lmb_update{L+1}.S{Senidx},filter);
                    tt_lmb_update{Senidx}=dist_tt_lmb_update{L+1}.S{Senidx};

                    for tar = size(tt_lmb_update{Senidx}, 1):-1:1
                        if (std(tt_lmb_update{Senidx}{tar}.x(1, :)) > var_cutoff) && (std(tt_lmb_update{Senidx}{tar}.x(3, :)) > var_cutoff)
                            labels_to_delete = [labels_to_delete, tt_lmb_update{Senidx}{tar}.l];
                             tt_lmb_update{Senidx}{tar}.r = 0;
                             tt_lmb_update{Senidx}(tar) = [];
                             dist_tt_lmb_update{L+1}.S{Senidx}{tar}.r = 0;
                        end
                    end
                end
                tt_lmb_update_tem{k} = dist_tt_lmb_update{L+1}.S{id};
                T_clean= length(dist_tt_lmb_update{L+1}.S{id});
        end

        %% state estimation

        switch distributed
            case 0
                PX = [];
                [est.X{k},est.N(k),est.L{k}]= extract_estimates(tt_lmb_update,model);

                if flag==3
                    PX(1,:) = est.X{k}(1,:);
                    PX(2,:) = est.X{k}(3,:);
                    DIST = dist(PX);
                    UpTri = triu(DIST);
                    IdxDist = find(UpTri<5 & UpTri>0); %7.5 good
                    IdxDist = ceil(IdxDist/size(PX(1,:),2));

                    if ~isempty(IdxDist)
                        lable_remove = est.L{k}(:,IdxDist);
                        for idx_up_tt_lmb = 1:size(tt_lmb_update,1)
                            if sum(tt_lmb_update{idx_up_tt_lmb,1}.l == est.L{k}(:,IdxDist)) == 2
                                tt_lmb_update{idx_up_tt_lmb,1} = [];
                                disp('Happened!')
                            end
                        end
                        tt_lmb_update = tt_lmb_update(~cellfun('isempty', tt_lmb_update'));
                    end
                else
                end

                display_diaginfo(tt_lmb_update,k,est,filter,T_predict,T_posterior,T_clean);
            case 1
                for Sen=1:size(model{1}.SenLoc,2)
                    PX = [];
                    [est{Sen}.X{k},est{Sen}.N(k),est{Sen}.L{k}]= extract_estimates_DS2(dist_tt_lmb_update{L+1}.S{Sen},model{Sen},filter);
                end

                display_diaginfo(tt_lmb_update{id},k,est{id},filter,T_predict{id},T_posterior(id),T_clean);
        end
        % Make video
        num_targets = 0;
        if run >= 0
            Sensor_Locations = Sen_Locations;
            senNo = size(model{1}.SenLoc, 2);
            [X_track,k_birth,k_death]= extract_tracks(truth.X,truth.track_list,truth.total_tracks);
            labelcount= countestlabels2(meas{id}, est{id});
            colorarray= makecolorarray(labelcount);
            
            [X_track,k_birth,k_death]= extract_tracks(truth.X,truth.track_list,truth.total_tracks);
            
            %plot ground truths
            figure; truths= gcf; hold on;
            for i=1:truth.total_tracks
                Zt= gen_observation_fn( model{id}, X_track(:,k_birth(i):1:k_death(i),i),'noiseless',senNo);
                if k >= k_birth(i) && k <= k_death(i)
                    plot(Zt(1, k-k_birth(i)+1), Zt(2, k-k_birth(i)+1), 'rx');
                end
            end
            for i = 1:senNo
                plot(Sensor_Locations(k, 1, i), Sensor_Locations(k, 2, i), 'bo');
                text(Sensor_Locations(k, 1, i), Sensor_Locations(k, 2, i), num2str(i));
                th = linspace(Sen_Headings(k, i) - model{i}.FOV, Sen_Headings(k, i) + model{i}.FOV, 100);
                th_full = linspace(0, 2 * pi, 100);
                for j = 1:Num_Sen
                    if i ~= j && sqrt((Sensor_Locations(k, 1, i) - Sensor_Locations(k, 1, j))^2 + (Sensor_Locations(k, 2, i) - Sensor_Locations(k, 2, j))^2) <= COMMUNICATION_RANGE
                        plot([Sensor_Locations(k, 1, i), Sensor_Locations(k, 1, j)], [Sensor_Locations(k, 2, i), Sensor_Locations(k, 2, j)], 'k:');
                    end
                end
                if i == id
                    if ~isempty(tt_lmb_update{id})
                        num_targets = 0;
                        for tar = 1:size(tt_lmb_update{id}, 1)
                            if tt_lmb_update{id}{tar}.r >= filter.prediction_threshold
                                mean_x = mean(tt_lmb_update{id}{tar}.x(1, :));
                                mean_y = mean(tt_lmb_update{id}{tar}.x(3, :));
                                plot(mean_x, mean_y, "kx")
                                x_std = std(tt_lmb_update{id}{tar}.x(1, :));
                                y_std = std(tt_lmb_update{id}{tar}.x(3, :));
                                x = 2 * x_std * cos(th_full) + mean_x;
                                y = 2 * y_std * sin(th_full) + mean_y;
                                plot(x, y, 'k:')
                                num_targets = num_targets + 1;
                            end
                        end
                    end
                end
                %
                R = 500;
                x = R * cos(th) + Sensor_Locations(k, 1, i);
                y = R * sin(th) + Sensor_Locations(k, 2, i);
                plot(x, y, "b");
                R_0 = 500;
                x_0 = R_0 * cos(th) + Sensor_Locations(k, 1, i);
                y_0 = R_0 * sin(th) + Sensor_Locations(k, 2, i);
                plot(x_0, y_0, "b");
                x_2 = [Sensor_Locations(k, 1, i), x(1)];
                y_2 = [Sensor_Locations(k, 2, i), y(1)];
                plot(x_2, y_2, "b");
                x_3 = [Sensor_Locations(k, 1, i), x(end)];
                y_3 = [Sensor_Locations(k, 2, i), y(end)];
                plot(x_3, y_3, "b");
                arrow_length = 250;
                ear_length = 50;
                ear_angle = pi / 8;
                x_arrow = [Sensor_Locations(k, 1, i), Sensor_Locations(k, 1, i) + arrow_length * cos(Sen_Headings(k, i))];
                y_arrow = [Sensor_Locations(k, 2, i), Sensor_Locations(k, 2, i) + arrow_length * sin(Sen_Headings(k, i))];
                x_ear_1 = [Sensor_Locations(k, 1, i) + arrow_length * cos(Sen_Headings(k, i)), Sensor_Locations(k, 1, i) + arrow_length * cos(Sen_Headings(k, i)) - ear_length * cos(Sen_Headings(k, i) + ear_angle)];
                y_ear_1 = [Sensor_Locations(k, 2, i) + arrow_length * sin(Sen_Headings(k, i)), Sensor_Locations(k, 2, i) + arrow_length * sin(Sen_Headings(k, i)) - ear_length * sin(Sen_Headings(k, i) + ear_angle)];
                x_ear_2 = [Sensor_Locations(k, 1, i) + arrow_length * cos(Sen_Headings(k, i)), Sensor_Locations(k, 1, i) + arrow_length * cos(Sen_Headings(k, i)) - ear_length * cos(Sen_Headings(k, i) - ear_angle)];
                y_ear_2 = [Sensor_Locations(k, 2, i) + arrow_length * sin(Sen_Headings(k, i)), Sensor_Locations(k, 2, i) + arrow_length * sin(Sen_Headings(k, i)) - ear_length * sin(Sen_Headings(k, i) - ear_angle)];
            end
            axis equal; axis 'auto'; title('Ground Truths'); %axis([-model.range_c(2,2) model.range_c(2,2) 0 model.range_c(2,2)])
            title("k = "+k)
            if Scenario == 1
                xlim([-700, 700])
                ylim([-1100, 1100])
            elseif Scenario == 2
                xlim([-500, 500])
                ylim([-250, 900])
            end
            
            if num_targets ~= truth.N(k)
                disp("Cardinality error: check before_tt_lmb_update.")
                fprintf("truth: %d\n", truth.N(k))
                fprintf("estimate: %d\n", num_targets)
                disp("")
            end
            hold off
            if Video_flag
                frame = getframe(gcf);
                writeVideo(mov, frame);
            end
            close
        end 

    end
    switch distributed
        case 0
            handles= plot_results(model,truth,meas,est,size(model.SenLoc,2));
        case 1
            [ospa_vals, ospa2_cell, handles] = plot_results3(model{1}, truth, meas, est{1}, k_birth);
            ospa2_cells = cell(Num_Sen, 1);
            [X_track,k_birth,k_death]= extract_tracks(truth.X,truth.track_list,truth.total_tracks);
            for id_ = 1:Num_Sen
                ospa_vals= zeros(truth.K,3);
                ospa_c= 100;
                ospa_p= 1;
                for k=1:meas{1}.K
                    [ospa_vals(k,1), ospa_vals(k,2), ospa_vals(k,3)]= ospa_dist(get_comps(truth.X{k},[1 3]),get_comps(est{id_}.X{k},[1 3]),ospa_c,ospa_p);
                end
                handles_{(run - 1) * Num_Sen + id_} = ospa_vals;

                order = 1;
                cutoff = 100;
                win_len = 10;
                ospa2_cells{id_} = cell(1,length(win_len));
                labelcount = countestlabels2(meas{id}, est{id});
                colorarray = makecolorarray(labelcount);
                est{id_}.total_tracks= labelcount;
                est{id_}.track_list= cell(truth.K,1);
                for k=1:truth.K
                    for eidx=1:size(est{id_}.X{k},2)
                        [idx_, colorarray] = assigncolor(est{id_}.L{k}(:,eidx), colorarray);
                        est{id_}.track_list{k} = [est{id_}.track_list{k} idx_];
                    end
                end
                [Y_track,l_birth,l_death]= extract_tracks(est{id_}.X,est{id_}.track_list,est{id_}.total_tracks);
                for i = 1:length(win_len)
                    ospa2_cells{id_}{i} = compute_ospa2(X_track([1 3],:,:),Y_track([1 3],:,:),cutoff,order,win_len)';
                end
                handles_{(run - 1) * Num_Sen + id_, 2} = ospa2_cells{id_}{1};
                handles_{(run - 1) * Num_Sen + id_, 3} = est{id_}.N;
            end
    end

    tEnd(run)=toc(tStart); 

    if run >= 0 && Video_flag
        close(mov);
    end

    tEnd_indv=tEnd./meas{1}.K;
    avg_filter_time= mean(tEnd_indv);
    str=['Mean simulation iteration time is ', num2str(avg_filter_time), ' seconds.'];
    disp(str)
    
    et_sen=et./SenNum; %average update time per sensor
    avg_update_time=mean(mean(et_sen,1));
    str2=['Mean filter update per sensor time is ', num2str(avg_update_time), ' seconds.'];
    disp(str2)
    
    dist = zeros(Num_Sen, 1);
    loc = zeros(Num_Sen, 1);
    card = zeros(Num_Sen, 1);
    for i = (run-1)*Num_Sen+1:run*Num_Sen
        dist(i) = mean(handles_{i,1}(:, 1));
        loc(i) = mean(handles_{i,1}(:, 2));
        card(i) = mean(handles_{i,1}(:, 3));
    end
    OSPAdist=mean(dist);
    OSPAloc=mean(loc);
    OSPAcard=mean(card);
end

dist = zeros(NumIt*Num_Sen, 1);
loc = zeros(NumIt*Num_Sen, 1);
card = zeros(NumIt*Num_Sen, 1);
dist2 = zeros(NumIt*Num_Sen, 1);
loc2 = zeros(NumIt*Num_Sen, 1);
card2 = zeros(NumIt*Num_Sen, 1);
num_estimates = zeros(NumIt*Num_Sen*truth.K, 1);
for i = 1:NumIt * Num_Sen
    dist(i) = mean(handles_{i,1}(:, 1));
    loc(i) = mean(handles_{i,1}(:, 2));
    card(i) = mean(handles_{i,1}(:, 3));
    dist2(i) = mean(handles_{i,2}(:, 1));
    loc2(i) = mean(handles_{i,2}(:, 2));
    card2(i) = mean(handles_{i,2}(:, 3));
end
for i = 1:NumIt * Num_Sen
    num_estimates(((i-1)*truth.K + 1):(i*truth.K)) = handles_{i,3};
end
plot_average_cardinality_error(truth, meas, num_estimates);
if Scenario == 1
    plot_6cams_figures_final(truth, meas);
elseif Scenario == 2
    plot_8cams_figures_final(truth, meas);
end
disp(mean(dist))
disp(mean(loc))
disp(mean(card))
disp(mean(dist2))
disp(mean(loc2))
disp(mean(card2))
if FDCD
    disp(mean(times))
else
    disp(mean(times)/Num_Sen)
end
save("results_" + string(datetime("now", "Format", "yyyy-MM-dd_HH_mm_ss")) + ".mat", "dist", "loc", "card", "dist2", "loc2", "card2", "times", "num_estimates")

function [X_track,k_birth,k_death]= extract_tracks(X,track_list,total_tracks)

K= size(X,1);
x_dim= size(X{K},1);
k=K-1; while x_dim==0, x_dim= size(X{k},1); k= k-1; end;
X_track= zeros(x_dim,K,total_tracks);
k_birth= zeros(total_tracks,1);
k_death= zeros(total_tracks,1);

max_idx= 0;
for k=1:K
    if ~isempty(X{k})
        X_track(:,k,track_list{k})= X{k};
    end;
    if max(track_list{k})> max_idx%new target born?
        idx= find(track_list{k}> max_idx);
        k_birth(track_list{k}(idx))= k;
    end;
    if ~isempty(track_list{k}), max_idx= max(track_list{k}); end; 
    k_death(track_list{k})= k;
end;
end

function ca= makecolorarray(nlabels)
        lower= 0.1;
        upper= 0.9;
        rrr= rand(1,nlabels)*(upper-lower)+lower;
        ggg= rand(1,nlabels)*(upper-lower)+lower;
        bbb= rand(1,nlabels)*(upper-lower)+lower;
        ca.rgb= [rrr; ggg; bbb]';
        ca.lab= cell(nlabels,1);
        ca.cnt= 0;
    end

    function [idx, colorarray] = assigncolor(label, colorarray)
        str= sprintf('%i*',label);
        tmp= strcmp(str,colorarray.lab);
        if any(tmp)
            idx= find(tmp);
        else
            colorarray.cnt= colorarray.cnt + 1;
            colorarray.lab{colorarray.cnt}= str;
            idx= colorarray.cnt;
        end
    end

    function count= countestlabels2(meas, est)
        labelstack= [];
        for k=1:meas.K
            labelstack= [labelstack est.L{k}];
        end
        [c,~,~]= unique(labelstack','rows');
        count=size(c,1);
    end

function Xc= get_comps(X,c)

if isempty(X)
    Xc= [];
else
    Xc= X(c,:);
end
end

function [y1, y2]=Crossover(x1,x2,gamma,VarMin,VarMax)
    cutoff = randi([1, size(x1, 2)]);
    y1 = [x1(1:cutoff), x2(cutoff+1:end)];
    y2 = [x2(1:cutoff), x1(cutoff+1:end)];

end

function [y1, y2]=UniformCrossover(x1,x2)

    alpha=randi([0 1],size(x1));
    
    y1=alpha.*x1+(1-alpha).*x2;
    y2=alpha.*x2+(1-alpha).*x1;
    
end

function y=Mutate(x,mu,VarMin,VarMax)
    nVar=numel(x);
    
    nmu=ceil(mu*nVar);
    
    j=randsample(nVar,nmu);
    
    sigma=0.1*(VarMax-VarMin);
    
    y=x;
    y(j) = randi([VarMin, VarMax]);
    
    y=max(y,VarMin);
    y=min(y,VarMax);
end

function i=RouletteWheelSelection(P)
    r=rand;
    
    c=cumsum(P);
    
    i=find(r<=c,1,'first');
end

function i=TournamentSelection(pop,m)

    nPop=numel(pop);

    S=randsample(nPop,m);
    
    spop=pop(S);
    
    scosts=[spop.Cost];
    
    [~, j]=min(scosts);
    
    i=S(j);

end
