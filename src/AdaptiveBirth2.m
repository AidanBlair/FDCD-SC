function model = AdaptiveBirth2(model, meas, est, k, truth)
    % Update model's birth regions at each timestep
    % So that there are birth regions at each measurement and one in centre
    for i = 1:size(model, 2)
        range = 1000;
    
        model{i}.L_birth=[];                                                           
        model{i}.r_birth=[];                                                         
        model{i}.w_birth=[];                                               
        model{i}.m_birth=[];                        
        model{i}.B_birth=[];                      
        model{i}.P_birth=[];
        pos = model{1}.SenLoc(:, i);

        count = 1;
        for m = 1:size(meas{i}.Z{k}, 2)
            x = meas{i}.Z{k}(1, m);
            y = meas{i}.Z{k}(2, m);
            if sqrt((model{1}.SenLoc(1, i) - x)^2 + (model{1}.SenLoc(2, i) - y)^2) <= range
                meas_region = 2;
                if size(est{i}.X{max(k-1, 1)}, 2) > 0
                    for j = 1:size(est{i}.X{max(k-1, 1)}, 2)
                        if sqrt((est{i}.X{max(k-1, 1)}(1, j) - x)^2 + (est{i}.X{max(k-1, 1)}(3, j) - y)^2) <= 25
                            meas_region = 0;
                        end
                    end
                end
                model{i}.L_birth(count) = 1;
                model{i}.r_birth(count) = meas_region * 0.02;
                model{i}.w_birth{count}(1,1) = 1;
                model{i}.m_birth{count}(:, 1) = [x, 0, y, 0, 0];                
                %model{i}.B_birth{count}(:, :, 1) = diag([7.5;15;7.5;15;6*pi/180]);
                %model{i}.B_birth{count}(:, :, 1) = diag([5;10;5;10;6*pi/180]);
                %model{i}.B_birth{count}(:, :, 1) = diag([8;5;8;5;6*pi/180]);
                model{i}.B_birth{count}(:, :, 1) = diag([5;5;5;5;10*pi/180]);
                model{i}.P_birth{count}(:, :, 1) = model{i}.B_birth{count}(:,:,1)*model{i}.B_birth{count}(:,:,1)';
                count = count + 1;
            end
        end

        %
        model{i}.L_birth(count) = 1;
        model{i}.r_birth(count) = 0.002;
        model{i}.w_birth{count}(1,1) = 1;
        model{i}.m_birth{count}(:, 1) = [pos(1), 0, pos(2), 0, 0];                
        model{i}.B_birth{count}(:, :, 1) = diag([35;15;35;15;6*pi/180]);
        model{i}.P_birth{count}(:, :, 1) = model{i}.B_birth{count}(:,:,1)*model{i}.B_birth{count}(:,:,1)';
        %
    end

    %{
    clf(f1);
    clf(f2);
    clf(f3);
    clf(f4);
    figure(f1);
    hold on
    plot(model{1}.SenLoc(1, 1), model{1}.SenLoc(2, 1), 'Color', 'k', 'Marker', '.');
    centers = [];
    radii = [];
    for j = 1:size(model{1}.m_birth, 2)
        centers = [centers; model{1}.m_birth{j}(1), model{1}.m_birth{j}(3)];
        radii = [radii; (model{1}.B_birth{j}(1) ^ 2) / 2];
    end
    viscircles(centers, radii);
    for j = 1:size(truth.X{k}, 2)
        plot(truth.X{k}(1, j), truth.X{k}(3, j), 'Color', 'g', 'Marker', 'o');
    end
    hold off
    figure(f2);
    hold on
    plot(model{1}.SenLoc(1, 2), model{1}.SenLoc(2, 2), 'Color', 'k', 'Marker', '.');
    centers = [];
    radii = [];
    for j = 1:size(model{2}.m_birth, 2)
        centers = [centers; model{2}.m_birth{j}(1), model{2}.m_birth{j}(3)];
        radii = [radii; (model{2}.B_birth{j}(1) ^ 2) / 2];
    end
    viscircles(centers, radii);
    for j = 1:size(truth.X{k}, 2)
        plot(truth.X{k}(1, j), truth.X{k}(3, j), 'Color', 'g', 'Marker', 'o');
    end
    hold off
    figure(f3);
    hold on
    plot(model{1}.SenLoc(1, 3), model{1}.SenLoc(2, 3), 'Color', 'k', 'Marker', '.');
    centers = [];
    radii = [];
    for j = 1:size(model{3}.m_birth, 2)
        centers = [centers; model{3}.m_birth{j}(1), model{3}.m_birth{j}(3)];
        radii = [radii; (model{3}.B_birth{j}(1) ^ 2) / 2];
    end
    viscircles(centers, radii);
    for j = 1:size(truth.X{k}, 2)
        plot(truth.X{k}(1, j), truth.X{k}(3, j), 'Color', 'g', 'Marker', 'o');
    end
    hold off
    figure(f4);
    hold on
    plot(model{1}.SenLoc(1, 4), model{1}.SenLoc(2, 4), 'Color', 'k', 'Marker', '.');
    centers = [];
    radii = [];
    for j = 1:size(model{4}.m_birth, 2)
        centers = [centers; model{4}.m_birth{j}(1), model{4}.m_birth{j}(3)];
        radii = [radii; (model{4}.B_birth{j}(1) ^ 2) / 2];
    end
    viscircles(centers, radii);
    for j = 1:size(truth.X{k}, 2)
        plot(truth.X{k}(1, j), truth.X{k}(3, j), 'Color', 'g', 'Marker', 'o');
    end
    hold off
    %}
end