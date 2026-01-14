function [Z, sums]= gen_observation(model,X,SensorLocation, sums)
        % this observation generation function is for coordinate
        % measurements
        
        if isempty(X),
            Z= [];
        else
            [Z, sums]= gen_observation_fn2( model, X, model.D*randn(size(model.D,2),size(X,2)),SensorLocation, sums ); %coordinate extraction
        end;
    end
%%
    function [Z, sums]= gen_observation_fn2(model,X,V,Sen, sums)
        % this observation generation function is for bearing and range
        % measurements
        % global Sen
        if isempty(X),
            Z= [];
        else
            Z = [];
            P= model.C_posn*X; %coordinate extraction
            %Z(2,:) = sqrt((P(1,:)-(Sen(1,:).*(ones(size(P(1,:)))))).^2+ (P(2,:)-(Sen(2,:).*(ones(size(P(2,:)))))).^2);
            %     Z(1,:) = atan2 ((P(2,:)-(Sen(2,:).*(ones(size(P(2,:)))))),(P(1,:)-(Sen(1,:).*(ones(size(P(1,:)))))));
            %Z(1,:) = atan2 ((P(2,:)-(Sen(2,:).*(ones(size(P(2,:)))))),(P(1,:)-(Sen(1,:).*(ones(size(P(1,:)))))));
            
            %Z(1,:) = P(1, :) - Sen(1, :);
            %Z(2,:) = P(2, :) - Sen(2, :);

            range = sqrt((P(1,:)-(Sen(1,:).*(ones(size(P(1,:)))))).^2+ (P(2,:)-(Sen(2,:).*(ones(size(P(2,:)))))).^2);

            range_x = (P(1,:)-(Sen(1,:).*(ones(size(P(1,:))))));
            range_y = (P(2,:)-(Sen(2,:).*(ones(size(P(2,:))))));
            
            count = 1;
            for i = 1:size(range, 2)
                if range(i) < 1000
                    Z(1, count) = P(1, i);
                    Z(2, count) = P(2, i);
        
                    sigma = [1 + 0.00001 * (range_x(i)  .* range_x(i)); 1 + 0.00001 * (range_y(i)  .* range_y(i))];
                    V = sigma * randn;
                    V = sigma .* randn(2,1);
                    %V = randn;
                    %V = [0;0];
                    %Z(:, count)= Z(:, count)+ V(:, count);
                    Z(:, count)= Z(:, count)+ V;
                    %disp("Sigma")
                    %disp(sigma)
                    %disp("V")
                    %disp(V)
                    count = count + 1;
                end
            end
            sums = [sums, sum(range < 1000)];
        end;
    end