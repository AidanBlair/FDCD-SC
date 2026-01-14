function [S,s,t] = GenerateGraph(model,SensorLocations,targets,com_range)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
s=[];
t=[];

locations = SensorLocations.positions;

for Sen=1:targets
    if true
        S_temp=nan*zeros(2,size(model.SenLoc,2));
       
        for i=1:targets
            if true
                S_temp(:,i)=locations(:,i);
            end
        end
        S_i=locations(:,Sen);    %%[x_traj{S_n}(k);y_traj{S_n}(k)];
        % COMMUNICATION RANGE
        S{Sen}=(sqrt(sum((S_i - S_temp) .^ 2))<=com_range);    %%neighbours = all vehicles within some distance threshold
        S{Sen}=find(S{Sen});

        s=[s,repmat(Sen,1,size(S{Sen},2))];
        t=[t,S{Sen}];
    end
end
end

