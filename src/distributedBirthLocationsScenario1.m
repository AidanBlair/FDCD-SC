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

for i = 5:6
    model{i}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
    model{i}.r_birth(1)=0.03;         %0.03                                                  %prob of birth
    model{i}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
    model{i}.m_birth{1}(:,1)= [984,-20,1468,-20,pi/180];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
    model{i}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
    model{i}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
end
%

%% 6 sensors, 11 targets
%
for i = 1:2
    model{i}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
    model{i}.r_birth(1)=0.8;         %0.03                                                  %prob of birth
    model{i}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
    model{i}.m_birth{1}(:,1)= [0, 0, 295, -5, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
    model{i}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
    model{i}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
    
    model{i}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
    model{i}.r_birth(2)=0.8;         %0.03                                                  %prob of birth
    model{i}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
    model{i}.m_birth{2}(:,1)= [0, 0, 500, 0, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
    model{i}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
    model{i}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
    
    model{i}.L_birth(3)=1;                                                             %no of Gaussians in birth term 1
    model{i}.r_birth(3)=0.8;         %0.03                                                  %prob of birth
    model{i}.w_birth{3}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
    model{i}.m_birth{3}(:,1)= [0, 0, 705, 5, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
    model{i}.B_birth{3}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
    model{i}.P_birth{3}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
end
model{5}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{5}.r_birth(1)=0.9;         %0.03                                                  %prob of birth
model{5}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{5}.m_birth{1}(:,1)= [-205, -5, -503, -3, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{5}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{5}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{3}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{3}.r_birth(1)=0.9;         %0.03                                                  %prob of birth
model{3}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{3}.m_birth{1}(:,1)= [-205, -5, -297, 3, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{3}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{3}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{5}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{5}.r_birth(2)=0.9;         %0.03                                                  %prob of birth
model{5}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{5}.m_birth{2}(:,1)= [-103, -3, -605, -5, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{5}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{5}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{3}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{3}.r_birth(2)=0.9;         %0.03                                                  %prob of birth
model{3}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{3}.m_birth{2}(:,1)= [-103, -3, -195, 5, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{3}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{3}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{6}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{6}.r_birth(1)=0.9;         %0.03                                                  %prob of birth
model{6}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{6}.m_birth{1}(:,1)= [103, 3, -605, -5, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{6}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{6}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{4}.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model{4}.r_birth(1)=0.9;         %0.03                                                  %prob of birth
model{4}.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{4}.m_birth{1}(:,1)= [103, 3, -195, 5, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{4}.B_birth{1}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{4}.P_birth{1}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{6}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{6}.r_birth(2)=0.9;         %0.03                                                  %prob of birth
model{6}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{6}.m_birth{2}(:,1)= [205, 5, -503, -3, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{6}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{6}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians

model{4}.L_birth(2)=1;                                                             %no of Gaussians in birth term 1
model{4}.r_birth(2)=0.9;         %0.03                                                  %prob of birth
model{4}.w_birth{2}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model{4}.m_birth{2}(:,1)= [205, 5, -297, 3, 0];%truth.X{22}(:,1);%[ -1.7; 0; 29.4; 0; 0 ];                                 %mean of Gaussians
model{4}.B_birth{2}(:,:,1)= diag([2.7;3.8;2.7;3.8;6*pi/180]);  %[2.5;1.5;2.5;2.5;pi/180]            %std of Gaussians
model{4}.P_birth{2}(:,:,1)= model{i}.B_birth{1}(:,:,1)*model{i}.B_birth{1}(:,:,1)';      %cov of Gaussians
