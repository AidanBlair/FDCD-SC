function tt_lmb_merged=EuclidianMerge2_James(tt_lmb,merging_threshold,CurrentTime)
%DklMerge calculates dkl between two labelled densities and merges them if dkl is less than some error.
%   The function gets all combinations of BC pairs then calculates the euclidian distance. If
%   the value is less than some threshold, then the two components are merged.

tt_lmb_total=[];
%dthresh=0.5; %euclidiean distance threshold for state of n dimensions (here is 2) %1 for roundabout

%tt_lmb=clean_lmb(tt_lmb,filter);

lab_new = [];
q=[];
X=cell(length(tt_lmb),1);
w=cell(length(tt_lmb),1);
lab=cell(length(tt_lmb),1);

for n=1:length(tt_lmb)  %%for all components, concatenate data if not empty
    if ~isempty(tt_lmb{n})
        tt_lmb_total=vertcat(tt_lmb_total,tt_lmb{n});
    end
end

for ii=1:length(tt_lmb_total) %%get all lmb information for each component and place it in its own array
    q=[q;tt_lmb_total(ii).r];
    X{ii}=tt_lmb_total(ii).x;
    w{ii}=tt_lmb_total(ii).w;
    lab{ii}=tt_lmb_total(ii).l;
end

x_dim= 2;%size(X{1},1); %% just use x and y cartesian data
L= length(q); %%size of q, ie how many componenets
MergeBypass=0;
stillmerge= 1;
while stillmerge %merge all unmerged components
    Xmean= zeros(x_dim,L);
    for i=1:L
        %         repmat(w{i}',[x_dim 1])
        %         X{i}
        %Xmean(:,i)= sum(repmat(w{i}',[x_dim 1]).*X{i}([1,3],:),2); %%get weighted mean of particles using only x and y measurement data
        Xmean(1,i)= sum(bsxfun(@times,w{i}',X{i}(1,:)),2);%sum(repmat(w{i}',[x_dim 1]).*X{i}([1,3],:),2); %%get weighted mean of particles using only x and y measurement data
        Xmean(2,i)= sum(bsxfun(@times,w{i}',X{i}(3,:)),2);
    end

    distmatrixm= 1e10*ones(L,L); %create a matrix of size L
    %pextmatrix= 10*ones(L,L);
    for i=1:L
        for j=i+1:L
            % You may chose to use either Eucleadian distance or infinte
            % norm to compare and find close targets.
            distmatrixm(i,j)= norm(Xmean(:,i)-Xmean(:,j),2); %each element ij of distmatrix corresponds to the difference between two components i and j
            %pextmatrix(i,j)= q(i)+q(j);
        end
    end

    if any(any(triu(distmatrixm<=0,1))) % ensures that any 0 values in the upper triangle are counted in the merging by setting them to a small value
        A=triu(distmatrixm<=0,1); %%ensures that any element where the dist is 0 in the upper triangle will be counted in the fusion
        distmatrixm(A)=0.01;
    end

    cm= ((distmatrixm) <= merging_threshold); %% logic matrix where values of 1 correspond to values less than the threshold
    %cd=(distmatrixm)>0; %%logic matrix for values greater than 0. removes lower triangle which is always 0
    %cq= pextmatrix <= qthresh;
    %cj= cm.*cd; %multiply two matrixes to get a logic matrix where both conditions are true
    cj=cm;
    if any(any(cj)) %if any value in cj is 1, continue merging
        ctemp=cm.*distmatrixm; %multiply cm by distmatrix values
        %         if triu(ctemp<=0,1)
        %             A=triu(ctemp<=0,1); %%ensures that any element where the dist is 0 in the upper triangle will be counted in the fusion
        %             ctemp(A)=0.01;
        %         end
        ctemp((ctemp)<=0)=nan; %removes 0 from the matrix so the find command doesn't fail.
        %ctemp=cj.*pextmatrix;
        %  One pair of particles which are too close and need to be merged
        %  is chosen here. The pair chosen is the one with maximum
        %  cumulative probability of existence.

        [n,m]= find(ctemp/min(min(ctemp))==1,1); %%find the location of the smallest element
        %[n,m]= find(ctemp<=filter.merging_threshold);

        %[n,m]= find(ctemp/max(max(ctemp))==1,1);
        % The merged target has all the particles. Their weights are scaled
        % according to the probabilities of existence, so as to sum to 1.
        % The merged probability of existence is the sum of the
        % probabilities for the two targets, thresholded at 0.999.

            L= L-1; %decrease L value
        %merging
        %if (lab{n}(1)==CurrentTime||lab{m}(1)==CurrentTime)

            if(lab{n}(1) < lab{m}(1)) %if n is older label
                %set the target label of n
                X{n}= [X{n}, X{m}]; %concat states
                w{n}= [q(n)*w{n}; q(m)*w{m}]/(q(n)+q(m)); %merge weights by reweighing them relative to their importance
                q(n)= (q(n)+q(m)-(q(n)*q(m)));%merge probability of existance according to probability rule for addition of independent events
                lab(m)= []; %delete younger label
                q(m)= []; %delete other values in m
                X(m)= [];
                w(m)= [];

            else %(lab{m}(1) < lab{n}(1))
                % Set the target label of m
                X{m}= [X{n}, X{m}]; %concat states
                w{m}= [q(n)*w{n}; q(m)*w{m}]/(q(n)+q(m)); %merge weights by reweighing them relative to their importance
                q(m)= (q(n)+q(m)-(q(n)*q(m)));%merge probability of existance according to probability rule for addition of independent events
                lab(n)= []; %this time n is the younger label
                q(n)= []; %delete other values in m
                X(n)= [];
                w(n)= [];
                %          elseif (lab{m}(1) == lab{n}(1))%if label k is the same
                %              if lab{n}(2)<lab{m}(2) %if sensor index m is bigger than n, delete m
                %                  X{n}= [X{n}, X{m}]; %concat states
                %                  w{n}= [q(n)*w{n}; q(m)*w{m}]/(q(n)+q(m)); %merge weights by reweighing them relative to their importance
                %                  q(n)= min((q(n)+q(m)-(q(n)*q(m))),0.98);%merge probability of existance according to probability rule for addition of independent events
                %                  lab(m)= []; %delete younger label
                %                  q(m)= []; %delete other values in m
                %                  X(m)= [];
                %                  w(m)= [];
                %              elseif lab{m}(2)<lab{n}(2)
                %                  X{m}= [X{n}, X{m}]; %concat states
                %                  w{m}= [q(n)*w{n}; q(m)*w{m}]/(q(n)+q(m)); %merge weights by reweighing them relative to their importance
                %                  q(m)= min((q(n)+q(m)-(q(n)*q(m))),0.98);%merge probability of existance according to probability rule for addition of independent events
                %                  lab(n)= []; %this time n is the younger label
                %                  q(n)= []; %delete other values in m
                %                  X(n)= [];
                %                  w(n)= [];
                %              else %if sen index is the same, check target index
                %                  if lab{n}(3)<lab{m}(3) %delete the label with the larger corresponding sum, implying that the index is larger
                %                      X{n}= [X{n}, X{m}]; %concat states
                %                      w{n}= [q(n)*w{n}; q(m)*w{m}]/(q(n)+q(m)); %merge weights by reweighing them relative to their importance
                %                      q(n)= min((q(n)+q(m)-(q(n)*q(m))),0.98);%merge probability of existance according to probability rule for addition of independent events
                %                      lab(m)= []; %delete younger label
                %                      q(m)= []; %delete other values in m
                %                      X(m)= [];
                %                      w(m)= [];
                %                  else
                %                      X{m}= [X{n}, X{m}]; %concat states
                %                      w{m}= [q(n)*w{n}; q(m)*w{m}]/(q(n)+q(m)); %merge weights by reweighing them relative to their importance
                %                      q(m)= min((q(n)+q(m)-(q(n)*q(m))),0.98);%merge probability of existance according to probability rule for addition of independent events
                %                      lab(n)= []; %this time n is the younger label
                %                      q(n)= []; %delete other values in m
                %                      X(n)= [];
                %                      w(n)= [];
                %                  end
                %              end
            end
        %else
        %MergeBypass=MergeBypass+1;
        %end

        if L==1
            lab_new = lab;
            stillmerge = 0;
        end
        % It is important to note that only two targets have been merged so
        % far, and the computation of mutual distances should be repeated
        % then the above process to be looped for the next best pair of
        % targets to be merged.
    else
        lab_new = lab;
        stillmerge = 0;
    end
end
q_new= q;
X_new= X;
w_new= w;
%L_new= L;

tt_lmb_merged=cell(length(q_new),1);

for i=1:L+MergeBypass %%put merged components back into a lmb
    tt_lmb_merged{i}.r=q_new(i);
    tt_lmb_merged{i}.x=X_new{i};
    tt_lmb_merged{i}.w=w_new{i};
    tt_lmb_merged{i}.l=lab_new{i};
end
end