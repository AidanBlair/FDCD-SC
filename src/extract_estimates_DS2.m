function [X,N,L]=extract_estimates_DS2(tt_lmb,model,filter)

% function [X,N,L,X0, Y0, r_ellipse_x, r_ellipse_y]=extract_estimates(tt_lmb,model)
%extract estimates via MAP cardinality and corresponding tracks
rvect= get_rvals(tt_lmb);

idxkeep= find(rvect > filter.prediction_threshold);
rvect= rvect(idxkeep);
%cdn= prod(1-rvect)*esf(rvect./(1-rvect));
%[~,mode] = max(cdn);
%N = min(length(rvect),mode-1);
X= zeros(model.x_dim,length(rvect));
L= zeros(3,length(rvect));

[~,idxcmp]= sort(rvect,'descend');
for n=1:length(rvect)
    %     %[~,idxtrk]= max(tt_lmb{idxcmp(n)}.w); %for MAP estimate
    %     try
    X(:,n)= tt_lmb{idxcmp(n)}.x*tt_lmb{idxcmp(n)}.w(:); %EAP estimate
    %     catch
    %         disp('Inner matrix dimensions must agree.')
    %     end
    L(:,n)= tt_lmb{idxcmp(n)}.l;
    % %     [X0(n), Y0(n), r_ellipse_x(:,n), r_ellipse_y(:,n)]=error_ellipse(tt_lmb{idxcmp(n)}.x);
end
N = length(rvect);
end