function tt_lmb_out= clean_lmb(tt_lmb_in,filter)
%prune tracks with low existence probabilities
rvect= get_rvals(tt_lmb_in);
idxkeep= find(rvect > filter.track_threshold);
rvect= rvect(idxkeep);
tt_lmb_out= tt_lmb_in(idxkeep);

%enforce cap on maximum number of tracks
if length(tt_lmb_out) > filter.T_max
    [~,idxkeep]= sort(rvect,'descend');
    tt_lmb_out= tt_lmb_out(idxkeep);
end

%cleanup tracks
for tabidx=1:length(tt_lmb_out)
    xtemptemp= tt_lmb_out{tabidx}.x;
    wtemptemp= tt_lmb_out{tabidx}.w;
    try
        rspidx= randsample(length(wtemptemp),filter.npt,true,wtemptemp); %rspidx= resample(wtemptemp,filter.npt);
    catch
        disp('W must contain non-negative values with at least one positive value.')
    end
    try
        tt_lmb_out{tabidx}.x= xtemptemp(:,rspidx);
    catch
        disp('Index exceeds matrix dimensions.')
    end
    tt_lmb_out{tabidx}.w= ones(filter.npt,1)/filter.npt;%wtemptemp(rspidx)/sum(wtemptemp(rspidx));%ones(filter.npt,1)'/filter.npt;
end
end