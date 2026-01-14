function tt_lmb= glmb2lmb2_dist(glmb)
%{
%find unique labels (with different possibly different association histories)
lmat= zeros(3,length(glmb.tt),1); %change to 3 for unique label idea
for tabidx= 1:length(glmb.tt)
    lmat(:,tabidx)= glmb.tt{tabidx}.l;
end
lmat= lmat'; %matrix of labels, each label is a row
%sort(lmat);

[cu,~,ic]= unique(lmat,'rows'); cu= cu'; %cu is a matrix, each label is a column

%initialize LMB struct
tt_lmb= cell(size(cu,2),1);

for tabidx=1:length(tt_lmb)
   tt_lmb{tabidx}.r= 0;
   tt_lmb{tabidx}.x= [];
   tt_lmb{tabidx}.w= []; %particle weights
   tt_lmb{tabidx}.l= cu(:,tabidx);
end
%tt_lmb_temp = tt_lmb;
Num_particles = zeros(1,length(tt_lmb));
Particle_Index = ones(1,length(tt_lmb));

for hidx=1:length(glmb.w) %for each hypothesis  
   for t= 1:glmb.n(hidx)    %for each element in the hypothesis
      trkidx= glmb.I{hidx}(t);  %find the t'th track
      newidx= ic(trkidx);
      Num_particles(newidx)  = Num_particles(newidx) + size(glmb.tt{trkidx}.x,2); %num particles in each new lmb component
   end
end

for tabidx=1:length(tt_lmb)
   tt_lmb{tabidx}.x= zeros(5,Num_particles(tabidx));
   tt_lmb{tabidx}.w= zeros(Num_particles(tabidx),1);
end
%}

% Preallocate lmat properly
n_tracks = length(glmb.tt);
lmat = zeros(n_tracks, 3); % Transposed for better memory access

for tabidx = 1:n_tracks
    lmat(tabidx, :) = glmb.tt{tabidx}.l';
end

% Faster unique on integers
[cu, ~, ic] = unique(lmat, 'rows', 'stable');
cu = cu';

% Preallocate tt_lmb
n_unique = size(cu, 2);
tt_lmb = cell(n_unique, 1);
Num_particles = zeros(1, n_unique);
Particle_Index = ones(1,length(tt_lmb));

% Single pass to count particles
for hidx = 1:length(glmb.w)
    for t = 1:glmb.n(hidx)
        trkidx = glmb.I{hidx}(t);
        newidx = ic(trkidx);
        Num_particles(newidx) = Num_particles(newidx) + size(glmb.tt{trkidx}.x, 2);
    end
end

% Initialize structures
for tabidx = 1:n_unique
    tt_lmb{tabidx}.r = 0;
    tt_lmb{tabidx}.l = cu(:, tabidx);
    tt_lmb{tabidx}.x = zeros(5, Num_particles(tabidx));
    tt_lmb{tabidx}.w = zeros(Num_particles(tabidx), 1);
end


%extract individual tracks
for hidx=1:length(glmb.w) %each hypothesis
   for t= 1:glmb.n(hidx) %each element in a hypothesis
      trkidx= glmb.I{hidx}(t); %element t of hypothesis 
      newidx= ic(trkidx);
      tt_lmb{newidx}.x(:,Particle_Index(newidx):(Particle_Index(newidx)+size(glmb.tt{trkidx}.w,1)-1)) = glmb.tt{trkidx}.x;
      tt_lmb{newidx}.w(Particle_Index(newidx):(Particle_Index(newidx)+size(glmb.tt{trkidx}.w,1)-1),:) = glmb.w(hidx)*glmb.tt{trkidx}.w;
      Particle_Index(newidx) = Particle_Index(newidx) + size(glmb.tt{trkidx}.w,1);
      %tt_lmb{newidx}.x= cat(2,tt_lmb{newidx}.x,glmb.tt{trkidx}.x);
      %tt_lmb{newidx}.w= cat(1,tt_lmb{newidx}.w,glmb.w(hidx)*glmb.tt{trkidx}.w);
   end
end

%extract existence probabilities and normalize track weights
for tabidx=1:length(tt_lmb)
   tt_lmb{tabidx}.r= sum(tt_lmb{tabidx}.w); %added 7/10/21. ensures no potential matlab rounding error to 1
   tt_lmb{tabidx}.w= tt_lmb{tabidx}.w/tt_lmb{tabidx}.r;
end

%
for tabidx=1:length(tt_lmb)
    [xtemp, ~, ic] = unique(tt_lmb{tabidx}.x','rows','stable');  
    tt_lmb{tabidx}.x=xtemp';
    tt_lmb{tabidx}.w= accumarray(ic, tt_lmb{tabidx}.w); tt_lmb{tabidx}.w = tt_lmb{tabidx}.w/sum(tt_lmb{tabidx}.w);
end
%

%{
for tabidx = 1:length(tt_lmb)
    x_data = tt_lmb{tabidx}.x;
    w_data = tt_lmb{tabidx}.w;
    
    if size(x_data, 2) == 0
        continue;
    end
    
    % Round to reduce unique states (adjust precision as needed)
    precision = 1e6; % This gives ~6 decimal places
    x_rounded = round(x_data * precision) / precision;
    
    % Now unique() is much faster on rounded data
    [xtemp, ~, ic] = unique(x_rounded', 'rows', 'stable');
    tt_lmb{tabidx}.x = xtemp';
    tt_lmb{tabidx}.w = accumarray(ic, w_data);
    tt_lmb{tabidx}.w = tt_lmb{tabidx}.w / sum(tt_lmb{tabidx}.w);
end
%}

end