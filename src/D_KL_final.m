function d = D_KL_final(pi_1, pi_2, filter)
    %% Kullback-Leibler Divergence between two LMB distributions
    s = 0;
    d = 0;
    %Pi = cell(2, 1);
    %Pi{1} = pi_1;
    %Pi{2} = pi_2;

    %% Support Unification
    % Keep only high probability tracks
    rvect= get_rvals(pi_1);
    idxkeep= find(rvect > filter.track_threshold);
    %idxkeep= find(rvect > 0.98);
    %idxkeep= find(rvect > filter.track_threshold);
    rvect= rvect(idxkeep);
    pi_1 = pi_1(idxkeep);
    
    rvect= get_rvals(pi_2);
    %idxkeep= find(rvect > filter.prediction_threshold);
    idxkeep= find(rvect > filter.prediction_threshold);
    %idxkeep= find(rvect >= 0.9798);
    rvect= rvect(idxkeep);
    pi_2 = pi_2(idxkeep);

    % Keep only tracks within pseudo-update's FoV
    %{
    pi_1_new = cell(1,1);
    pi_2_new = cell(1,1);
    DIST_THRESHOLD = 20;
    for m = 1:size(pims_meas, 2)
        for i = 1:length(pi_1)
            if norm(mean(pi_1{i}.x([1, 3], :), 2) - pims_meas(:, m), 2) < DIST_THRESHOLD
                if isempty(pi_1_new{1})
                    pi_1_new{1} = pi_1{i};
                else
                    pi_1_new{length(pi_1_new)+1} = pi_1{i};
                end
                break
            end
        end
        for i = 1:length(pi_2)
            if norm(mean(pi_2{i}.x([1, 3], :), 2) - pims_meas(:, m), 2) < DIST_THRESHOLD
                if isempty(pi_2_new{1})
                    pi_2_new{1} = pi_2{i};
                else
                    pi_2_new{length(pi_2_new)+1} = pi_2{i};
                end
                break
            end
        end
    end
    %pi_1 = pi_1_new;
    %pi_2 = pi_2_new;
    %}

    if ~(isempty(pi_1) || isempty(pi_2))
    
        pi_1_labels = [];
        pi_2_labels = [];
        
        for i = 1:size(pi_1)
            pi_1_labels = [pi_1_labels, pi_1{i}.l];
        end
        for i = 1:size(pi_2)
            pi_2_labels = [pi_2_labels, pi_2{i}.l];
        end
        
        pi_1_new = cell(size(pi_1_labels, 2), 1);
        pi_2_new = cell(size(pi_2_labels, 2), 1);
        
        count_1 = 1;
        count_2 = 1;
        % For each label in pi_1
        for i = 1:size(pi_1_labels, 2)
            pi_1_idx = i;
            if ~isempty(pi_2_labels)
                pi_2_idx = find(all(pi_2_labels == pi_1_labels(:, i)));
                % If label is in both distributions
                if ~isempty(pi_2_idx)
                    % Keep r
                    pi_1_new{count_1}.r = pi_1{pi_1_idx}.r;
                    pi_2_new{count_2}.r = pi_2{pi_2_idx}.r;
                    pi_1_new{count_1}.w = pi_1{pi_1_idx}.w;
                    pi_2_new{count_2}.w = pi_2{pi_2_idx}.w;

                    %%
                    %{
    
                    %num_particles = 1000;
                    max_size = min(size(pi_1{pi_1_idx}.x, 2), size(pi_2{pi_2_idx}.x, 2));
                    %num_particles = min(max_size, 1000);
                    num_particles = max_size;
                    % Sometimes there are duplicates in pi_2, need to remove
    
                    [P_unique_1, ia, ic] = unique([pi_1{pi_1_idx}.x(1,:); pi_1{pi_1_idx}.x(3,:)]','rows');
                    V_unique_1 = pi_1{pi_1_idx}.w(ia);
                    [P_unique_2, ia, ic] = unique([pi_2{pi_2_idx}.x(1,:); pi_2{pi_2_idx}.x(3,:)]','rows');
                    V_unique_2 = pi_2{pi_2_idx}.w(ia);
                    num_particles = min(num_particles, size(P_unique_2, 1));
                    num_particles = min(num_particles, size(P_unique_1, 1));
    
                    %sample_idxs_1 = randsample(size(pi_1{pi_1_idx}.x, 2), num_particles, true, pi_1{pi_1_idx}.w);
                    %sample_idxs_2 = randsample(size(pi_2{pi_2_idx}.x, 2), num_particles, true, pi_2{pi_2_idx}.w);
                    %sample_idxs_1 = randsample(size(pi_1{pi_1_idx}.x, 2), num_particles);
                    %sample_idxs_2 = randsample(size(unique([pi_2{pi_2_idx}.x(1,:); pi_2{pi_2_idx}.x(3,:)]','rows')', 2), num_particles);
                    sample_idxs_1 = randsample(size(P_unique_1, 1), num_particles);
                    sample_idxs_2 = randsample(size(P_unique_2, 1), num_particles);
            
                    %xs_1 = pi_1{pi_1_idx}.x(:, sample_idxs_1);
                    %xs_2 = pi_2{pi_2_idx}.x(:, sample_idxs_2);
                    %xs_new = [xs_1, xs_2];
                    %ws_1 = pi_1{pi_1_idx}.w(sample_idxs_1)';
                    %ws_2 = pi_2{pi_2_idx}.w(sample_idxs_2)';
                    %ws_1 = ws_1 ./ sum(ws_1);
                    %ws_2 = ws_2 ./ sum(ws_2);
    
                    xs_1 = P_unique_1(sample_idxs_1, :);
                    ws_1 = V_unique_1(sample_idxs_1, :);
                    xs_2 = P_unique_2(sample_idxs_2, :);
                    ws_2 = V_unique_2(sample_idxs_2, :);
                    xs_new = [xs_1', xs_2'];
    
                    %{
                    % Keeps only unique pairings (prevents warning, look into it as there shouldn't be duplicates?)
                    [P_unique_2, ia, ic] = unique([pi_2{pi_2_idx}.x(1,:); pi_2{pi_2_idx}.x(3,:)]','rows');
                    V_unique_2 = pi_2{pi_2_idx}.w(ia);
                    sum_V_unique_2 = sum(V_unique_2);
                    V_unique_2 = V_unique_2 ./ sum_V_unique_2;
                    % number of unique particles from pi_2 can sometimes be
                    % less than num_particles, so will need to adjust
                    % num_particles
                    [P_unique_1, ia, ic] = unique([xs_1(1,:); xs_1(3,:)]','rows');
                    V_unique_1 = pi_1{pi_1_idx}.w(ia);
                    sum_V_unique_1 = sum(V_unique_1);
                    V_unique_1 = V_unique_1 ./ sum_V_unique_1;
                    %}
    
                    %figure
                    %hist3(P_unique_1)
                    %figure
                    %hist3(P_unique_2)
                    %close
                    %close
    
                    %pd_1_x = fitdist(P_unique_1(:, 1), "Normal");
                    %pd_1_y = fitdist(P_unique_1(:, 2), "Normal");
                    %pd_2_x = fitdist(P_unique_2(:, 1), "Normal");
                    %pd_2_y = fitdist(P_unique_2(:, 2), "Normal");
                    
                    %pd_1_x = fitdist(xs_1(:, 1), "Normal");
                    %pd_1_y = fitdist(xs_1(:, 2), "Normal");
                    %pd_2_x = fitdist(xs_2(:, 1), "Normal");
                    %pd_2_y = fitdist(xs_2(:, 2), "Normal");
                    pd_1_x = fitdist(pi_1{pi_1_idx}.x(1, :)', "Normal");
                    pd_1_y = fitdist(pi_1{pi_1_idx}.x(3, :)', "Normal");
                    pd_2_x = fitdist(pi_2{pi_2_idx}.x(1, :)', "Normal");
                    pd_2_y = fitdist(pi_2{pi_2_idx}.x(3, :)', "Normal");
    
                    %{
                    xs_1_x = [min(P_unique_1(:, 1)):0.1:max(P_unique_1(:, 1))];
                    ys_1_x = pdf(pd_1_x, xs_1_x);
                    %figure
                    %hold on
                    histogram(P_unique_1(:, 1), 20)
                    figure
                    plot(xs_1_x, ys_1_x)
                    %hold off
                    close
                    close
                    %}
                    weights_1_x = pdf(pd_1_x, xs_1(:, 1));
                    weights_1_y = pdf(pd_1_y, xs_1(:, 2));
                    weights_2_x = pdf(pd_2_x, xs_2(:, 1));
                    weights_2_y = pdf(pd_2_y, xs_2(:, 2));
    
                    weights_1_comb = weights_1_x .* weights_1_y;
                    weights_1_comb = weights_1_comb / sum(weights_1_comb);
                    weights_2_comb = weights_2_x .* weights_2_y;
                    weights_2_comb = weights_2_comb / sum(weights_2_comb);
                    
                    % Get weights for particles sampled from other distribution
                    %F = scatteredInterpolant(P_unique_1, V_unique_1, "nearest");
                    F = scatteredInterpolant(xs_1, weights_1_comb, "nearest");
                    lastwarn("", "");
                    %ws_1_new = [ws_1, F(xs_new(1,:)', xs_new(3,:)')']';
                    ws_1_new = [weights_1_comb', F(xs_2(:,1)', xs_2(:,2)')]';
                    %ws_1_new = F(xs_new(1,:)', xs_new(3,:)');
                    [warnMsg, warnId] = lastwarn();
                    if ~isempty(warnId)
                        ws_1_new = [ws_1, ws_2]';
                        ws_2_new = [ws_2, ws_1]';
                    end
                    %F = scatteredInterpolant(P_unique_2, V_unique_2, "nearest");
                    F = scatteredInterpolant(xs_2, weights_2_comb, "nearest");
                    %ws_2_new = [ws_2, F(xs_new(1,:)', xs_new(3,:)')']';
                    ws_2_new = [F(xs_1(:,1)', xs_1(:,2)'), weights_2_comb']';
                    %ws_2_new = F(xs_new(1,:)', xs_new(3,:)');
                    [warnMsg, warnId] = lastwarn();
                    if ~isempty(warnId)
                        ws_1_new = [ws_1, ws_2]';
                        ws_2_new = [ws_2, ws_1]';
                    end
            
                    % If below threshold (small chance), set weights to epsilon
                    % (NOT 0, CAUSES DIVIDE BY 0 ERROR)
                    epsilon = 1e-10;
                    if any(isnan(ws_1_new))
                        ws_1_new(isnan(ws_1_new)) = epsilon;
                    end
                    if any(isnan(ws_2_new))
                        ws_2_new(isnan(ws_2_new)) = epsilon;
                    end
                    ws_1_new(ws_1_new <= epsilon) = epsilon;
                    ws_2_new(ws_2_new <= epsilon) = epsilon;
                    if any(isnan(ws_2_new))
                        disp("nan")
                    end
                    ws_1_new = ws_1_new / sum(ws_1_new);
                    ws_2_new = ws_2_new / sum(ws_2_new);
                    if any(isnan(ws_2_new))
                        disp("nan")
                    end
            
                    % New unified particles and weights
                    pi_1_new{count_1}.x = xs_new;
                    pi_1_new{count_1}.w = ws_1_new;
                    pi_2_new{count_2}.x = xs_new;
                    pi_2_new{count_2}.w = ws_2_new;

                    %%
                    %}
            
                    % Same label
                    pi_1_new{count_1}.l = pi_1{pi_1_idx}.l;
                    pi_2_new{count_2}.l = pi_2{pi_2_idx}.l;
            
                    count_1 = count_1 + 1;
                    count_2 = count_2 + 1;
                % if label only in pi_1, just pass the distribution through
                else
                    pi_1_new{count_1} = pi_1{i};
                    count_1 = count_1 + 1;
                end
            % if label only in pi_1, just pass the distribution through
            else
                pi_1_new{count_1} = pi_1{i};
                count_1 = count_1 + 1;
            end
        end
        
        % If label only in pi_2, just pass the distribution through
        for i = 1:size(pi_2_labels, 2)
            if ~isempty(pi_1_labels)
                t = find(all(pi_1_labels == pi_2_labels(:, i)));
                if isempty(t)
                    pi_2_new{count_2} = pi_2{i};
                    count_2 = count_2 + 1;
                end
            else
                pi_2_new{count_2} = pi_2{i};
                count_2 = count_2 + 1;
            end
        end
    
        pi_1 = pi_1_new;
        pi_2 = pi_2_new;
        if ~isempty(pi_1)
            if size(pi_1{1}, 1) ~= 0
                for me = 1:size(pi_1, 1)
                    l = pi_1{me}.l;
                    idx = 0;
                    for i = 1:size(pi_2, 1)
                        if pi_2{i}.l == l
                            idx = i;
                        end
                    end
                    if idx ~= 0
                        r1 = pi_1{me}.r;
                        if r1 == 1
                            r1 = 0.9999;
                        end
                        r2 = pi_2{idx}.r;
                        if r2 == 1
                            r2 = 0.9999;
                        end
                        w1 = pi_1{me}.w;
                        w2 = pi_2{idx}.w;
                        %w1(w1 < 1e-6) = 1e-6;
                        %w1 = 0;
                        %w2 = 0;
                        if any(isnan(w1)) || any(isnan(w2))
                            disp("NaN")
                        end
                        %if size(w1, 1) ~= size(w2, 1)
                        %    disp(size(w1))
                        %    disp(size(w2))
                        %    disp("")
                        %end
                        s = s + 1*D_KL_B(r1, r2, w1, w2);
                        %disp(D_KL_B(r1, r2, w1, w2))
                    else
                        r1 = pi_1{me}.r;
                        r2 = 0;
                        if r1 == 1
                            r1 = 0.9999;
                        end
                        r2 = 0.01;
                        r2 = 1e-4;
                        %s = s + (1 - r1) * log((1 - r1) / (1 - r2));
                        s = s + (1 - r1) * log((1 - r1) / (1 - r2)) + r1 * log(r1 / r2);
                        %disp((1 - r1) * log((1 - r1) / (1 - r2)) + r1 * log(r1 / r2))
                    end
                end
            end
        end
        %
        if ~isempty(pi_2)
            for me = 1:size(pi_2, 1)
                l = pi_2{me}.l;
                idx = 0;
                for i = 1:size(pi_1, 1)
                    if pi_1{i}.l == l
                        idx = i;
                    end
                end
                if idx == 0
                    %disp("label in pi_2 but not pi_1")
                    r1 = 0;
                    r2 = pi_2{me}.r;
                    %s = s - log((1-r2) / (1-r1));
                    %s = s + (1 - r1) * log((1 - r1) / (1 - r2)) - 5 * log((1-r1)/(1-r2));
                    %disp((1 - r1) * log((1 - r1) / (1 - r2)) - 100)
                    %s = s - 10;
                    %s = s - 5*r2;
                    s = s + 100*log(r2);
                end
            end
        end
        %
        d = s;
        disp("KLD: " + d)
    else
        d=-100;
    end
end

function KL = D_KL_B(r1, r2, w1, w2)
    %% Kullback-Leibler Divergence between two Bernoulli distributions
    % Same particles, but different weights, do only need to use w not p
    %KLD = sum(w1 .* log((w1) ./ (w2)));
    KLD = 0;
    KL = (1 - r1) * log((1 - r1) / (1 - r2)) + r1 * log(r1 / r2) + r1 * KLD;
    if isinf(KL)
        disp("Infinte")
    end
end