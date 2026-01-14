function Z= gen_observation_fn(model,X,W,senNo,truth)

if ~isnumeric(W)
    if strcmp(W,'noise')
        W= model.D*randn(size(model.D,2),size(X,2));
    elseif strcmp(W,'noiseless')
        W= zeros(size(model.D,1),size(X,2));
    end
end

if isempty(X)
    Z= [];
else
    mid= [0;0];%model.senLoc(:,senNo);
    P= X([1 3],:);
    M= size(P,2);
    Z = P - repmat(mid,[1 M]); %P = P - repmat(mid,[1 M]);
    %Z(1,:)= atan2(P(1,:),P(2,:));
    %Z(2,:)= sqrt(sum(P.^2));
    %Z= Z+ W;
    
end
end