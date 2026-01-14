function display_diaginfo(tt_lmb,k,est,filter,T_predict,T_posterior,T_clean)
rvect= get_rvals(tt_lmb);
cdn= prod(1-rvect)*esf(rvect./(1-rvect));

eap= (0:(length(cdn)-1))*cdn(:);
if isnan(eap)
    disp('NaN')
elseif isinf(eap)
    disp('Inf')
end
var= (0:(length(cdn)-1)).^2*cdn(:)-((0:(length(cdn)-1))*cdn(:))^2;
if ~strcmp(filter.run_flag,'silence')
    disp([' time= ',num2str(k),...
        ' #eap cdn=' num2str(eap),...
        ' #var cdn=' num2str(var,4),...
        ' #est card=' num2str(est.N(k),4),...
        ' #trax pred=' num2str(T_predict,4)]);
 %       ' #trax post=' num2str(T_posterior,4),...
%        ' #trax updt=',num2str(T_clean,4)   ]);
end
end