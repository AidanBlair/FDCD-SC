function [Posterior_no_repeats] = RemoveIntersection(tt_lmb,filter)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
tt_lmb_total=tt_lmb;
%tt_lmb_total2=[];

rvect= get_rvals(tt_lmb_total);
idxkeep= find(rvect > filter.track_threshold);
rvect= rvect(idxkeep);
tt_lmb_total= tt_lmb_total(idxkeep);

% lmat= zeros(3,length(tt_lmb_total),1); %%get a lookup table for labels
% for tabidx= 1:length(tt_lmb_total)
%     lmat(:,tabidx)= tt_lmb_total{tabidx}.l;
% end
% lmat= lmat';
  xmat= zeros(5,length(tt_lmb_total),1); %%get a lookup table for labels
for tabidx= 1:length(tt_lmb_total)
    xmat(:,tabidx)= tt_lmb_total{tabidx}.x(:,1);
end
xmat= xmat';

[cu,~,ic]= unique(xmat,'rows'); %find unique labels in lookup table

cu= cu';
  
 
Posterior_no_repeats= cell(size(cu,2),1);

for i = 1:size(cu,2) %loop over number of targets
    %w_temp = [];
    %P_1_temp = [];
    capidx=[];
    %maxVal=[];
    idx = i;%ic(i);
    capidx = find(ic == idx);

Posterior_no_repeats{i}=tt_lmb_total{capidx(1)};


    %     for j=1:length(capidx)
    %         maxVal=[maxVal;tt_lmb_total{capidx(j)}.r];
    %     end
    %     [~,ValIdx]=max(maxVal);
    %     Posterior_no_repeats{i}=tt_lmb_total{capidx(ValIdx)};
end
    %tic
    %tt_lmb_total=horzcat(tt_lmb_total);
%     tic
%     isUnique = true(size(tt_lmb_total));
%     for ii = 1:length(tt_lmb_total)-1
%         for jj = ii+1:length(tt_lmb_total)
%             if isequal(tt_lmb_total{ii},tt_lmb_total{jj})
%                 isUnique(ii) = false;
%                 break;
%             end
%         end
%     end
%     toc
    %toc
    %tic
    % hash=cell(length(tt_lmb_total),1);
    %
    % for i=1:length(tt_lmb_total)
    %     hash{i}=GetMD5(tt_lmb_total{i},'Array');
    % end
    % [~,idx,~]=unique(hash);
    %
    % Posterior_no_repeats=tt_lmb_total(idx);
    %toc

    %tic
   % Posterior_no_repeats=uniqueStruct(tt_lmb_total)';%uniqueCellGeneralized(A)';
    
   % toc


%     Posterior_no_repeats= tt_lmb_total(isUnique);



    %tt_lmb_total2 = tt_lmb_total(~cellfun(@isempty, tt_lmb_total));



end
 
   