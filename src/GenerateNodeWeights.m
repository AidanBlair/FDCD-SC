function [NodeWeights] = GenerateNodeWeights(S,model)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
        tempweights=cell(size(model.SenLoc,2),1);
        for i=1:length(S)
            for j=1:length(S{i})
                if i~=S{i}(j)
                    NodeWeights{i}(j)=1/(1+max(length(S{i}),length(S{S{i}(j)})));
                    tempweights{i}(j)=NodeWeights{i}(j);
                else
                    NodeWeights{i}(j)=0;
                end
            end
            for j=1:length(S{i})
                if i==S{i}(j)
                    NodeWeights{i}(j)=1-sum(tempweights{i});
                end
            end
        end
end

