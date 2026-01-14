%% Sen 1
model{1}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{1}.r_birth(1)=0.03;         %0.03                                                  %prob of birth
model{1}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{1}.m_birth{1}(:,1)= [387,19,987,-4,-pi/180];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{1}.B_birth{1}(:,:,1)= diag([5;5;5;5;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]                %std of Gaussians
model{1}.P_birth{1}(:,:,1)= model{1}.B_birth{1}(:,:,1)*model{1}.B_birth{1}(:,:,1)';      %cov of Gaussians

%% Sen 2
model{2}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{2}.r_birth(1)=0.03;         %0.03                                                  %prob of birth
model{2}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{2}.m_birth{1}(:,1)= [-236,20,1014,3,pi/180];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{2}.B_birth{1}(:,:,1)= diag([5;5;5;5;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]                %std of Gaussians
model{2}.P_birth{1}(:,:,1)= model{2}.B_birth{1}(:,:,1)*model{2}.B_birth{1}(:,:,1)';      %cov of Gaussians

%% Sen 3
model{3}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{3}.r_birth(1)=0.03;         %0.03                                                  %prob of birth
model{3}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{3}.m_birth{1}(:,1)= [984,-20,1468,-20,pi/180];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{3}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]                %std of Gaussians
model{3}.P_birth{1}(:,:,1)= model{3}.B_birth{1}(:,:,1)*model{3}.B_birth{1}(:,:,1)';      %cov of Gaussians

%% Sen 4
model{4}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{4}.r_birth(1)=0.03;         %0.03                                                  %prob of birth
model{4}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{4}.m_birth{1}(:,1)= [706,-12,1078,-25,pi/180];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{4}.B_birth{1}(:,:,1)= diag([5;5;5;5;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]                %std of Gaussians
model{4}.P_birth{1}(:,:,1)= model{4}.B_birth{1}(:,:,1)*model{4}.B_birth{1}(:,:,1)';      %cov of Gaussians

for i = 5:8
    model{i}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
    model{i}.r_birth(1)=0.03;         %0.03                                                  %prob of birth
    model{i}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
    model{i}.m_birth{1}(:,1)= [984,-20,1468,-20,pi/180];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
    model{i}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
    model{i}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
end

%% 8 sensors, 20 targets
% Sensor 1
model{1}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{1}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{1}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{1}.m_birth{2}(:,1)= [-296, 4, 0, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{1}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{1}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{1}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{1}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{1}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{1}.m_birth{3}(:,1)= [-277, 4, 59, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{1}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{1}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{1}.L_birth(4)=1;                                                             %no of Gaussians in birth term 1
model{1}.r_birth(4)=0.8;         %0.03                                                  %prob of birth
model{1}.w_birth{4}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{1}.m_birth{4}(:,1)= [-277, 4, -59, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{1}.B_birth{4}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{1}.P_birth{4}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 2
model{2}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{2}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{2}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{2}.m_birth{2}(:,1)= [-227, 4, 95, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{2}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{2}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{2}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{2}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{2}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{2}.m_birth{3}(:,1)= [-165, 4, 95, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{2}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{2}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 3
model{3}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{3}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{3}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{3}.m_birth{2}(:,1)= [-115, 4, 59, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{3}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{3}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{3}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{3}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{3}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{3}.m_birth{3}(:,1)= [-96, 4, 0, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{3}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{3}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{3}.L_birth(4)=1;                                                             %no of Gaussians in birth term 1
model{3}.r_birth(4)=0.8;         %0.03                                                  %prob of birth
model{3}.w_birth{4}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{3}.m_birth{4}(:,1)= [-115, 4, -59, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{3}.B_birth{4}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{3}.P_birth{4}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 4
model{4}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{4}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{4}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{4}.m_birth{2}(:,1)= [-165, 4, -95, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{4}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{4}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{4}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{4}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{4}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{4}.m_birth{3}(:,1)= [-227, 4, -95, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{4}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{4}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 7
model{7}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{7}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{7}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{7}.m_birth{2}(:,1)= [296, -4, 62, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{7}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{7}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{7}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{7}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{7}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{7}.m_birth{3}(:,1)= [277, -4, 121, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{7}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{7}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{7}.L_birth(4)=1;                                                             %no of Gaussians in birth term 1
model{7}.r_birth(4)=0.8;         %0.03                                                  %prob of birth
model{7}.w_birth{4}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{7}.m_birth{4}(:,1)= [277, -4, 3, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{7}.B_birth{4}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{7}.P_birth{4}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 6
model{6}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{6}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{6}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{6}.m_birth{2}(:,1)= [227, -4, 157, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{6}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{6}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{6}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{6}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{6}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{6}.m_birth{3}(:,1)= [165, -4, 157, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{6}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{6}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 5
model{5}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{5}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{5}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{5}.m_birth{2}(:,1)= [115, -4, 121, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{5}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{5}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{5}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{5}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{5}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{5}.m_birth{3}(:,1)= [96, -4, 62, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{5}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{5}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{5}.L_birth(4)=1;                                                             %no of Gaussians in birth term 1
model{5}.r_birth(4)=0.8;         %0.03                                                  %prob of birth
model{5}.w_birth{4}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{5}.m_birth{4}(:,1)= [115, -4, 3, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{5}.B_birth{4}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{5}.P_birth{4}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

% Sensor 8
model{8}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{8}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
model{8}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{8}.m_birth{2}(:,1)= [165, -4, -33, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{8}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{8}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{8}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
model{8}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
model{8}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{8}.m_birth{3}(:,1)= [227, -4, -33, 4, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{8}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{8}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
