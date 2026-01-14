function [tt_lmb_birth,tt_lmb_survive]= lmbpredict_DS(tt_lmb_update,model,filter,k,Sen)
%---generate birth tracks
tt_lmb_birth= cell(length(model.r_birth),1);                                                                            %initialize cell array
for tabbidx=1:length(model.r_birth)
    tt_lmb_birth{tabbidx}.r= model.r_birth(tabbidx);                                                                    %birth prob for birth track
    tt_lmb_birth{tabbidx}.x= gen_gms(model.w_birth{tabbidx},model.m_birth{tabbidx},model.P_birth{tabbidx},filter.npt);  %samples for birth track
    tt_lmb_birth{tabbidx}.w= ones(filter.npt,1)/filter.npt;                                                             %weights of samples for birth track
    tt_lmb_birth{tabbidx}.l= [k;Sen;tabbidx];                                                                           %track label
end

%---generate surviving tracks
tt_lmb_survive= cell(length(tt_lmb_update),1);                                                                              %initialize cell array
for tabsidx=1:length(tt_lmb_update)
%     try
     wtemp_predict= compute_pS(model,tt_lmb_update{tabsidx}.x).*tt_lmb_update{tabsidx}.w(:); 
%     catch
%         disp('prediction error')
%     end
    xtemp_predict= gen_newstate_fn(model,tt_lmb_update{tabsidx}.x,'noise');      %particle prediction
    tt_lmb_survive{tabsidx}.r= sum(wtemp_predict)*tt_lmb_update{tabsidx}.r;                                                 %predicted existence probability for surviving track
    tt_lmb_survive{tabsidx}.x= xtemp_predict;                                                                               %samples for surviving track
    tt_lmb_survive{tabsidx}.w= wtemp_predict/sum(wtemp_predict);                                                            %weights of samples for predicted track
    tt_lmb_survive{tabsidx}.l= tt_lmb_update{tabsidx}.l;                                                                    %track label
end

end
