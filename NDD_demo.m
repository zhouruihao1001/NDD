%% Acknowledgement
%Thanks for scancontext and M2DP code authors.

%% Parameters 
clear; clc;
addpath(genpath('src'));
data_dir = '.\data\';

range = 80; % meter 
N_s = 60; 
N_r = 20;
%% Visualization
% KITTI08: 1684-209
KITTI1 = [data_dir, '001684.bin'];
KITTI2 = [data_dir, '000209.bin'];
ptcloud1 = readBin(KITTI1);
ptcloud2 = readBin(KITTI2);
%% NDD
tic;
NDD1 = NDD(ptcloud1, N_s, N_r, range);
toc; 
NDD2 = NDD(ptcloud2, N_s, N_r, range);

img_ndd = [NDD2(:,:,1);NDD2(:,:,2)];

% Align. Key
img_col_sum_ndd = sum(img_ndd,1);
% Searching Key
img_row_sum_ndd = sum(img_ndd,2);


h1 = figure(1); clf;
imagesc(img_ndd);
% colorbar;
set(gcf, 'Position', [10 10 800 400]);
axis off;
caxis([0, 6]);

%sim. cal.
tic 
NDD_sim = corr_sim(NDD1, NDD2);
toc
NDD_dist = 1- abs(NDD_sim);
disp([NDD_sim, NDD_dist]);

%% NDD (1-scale)

NDD1_P = NDD_P(ptcloud1, N_s, N_r, range);
NDD2_P = NDD_P(ptcloud2, N_s, N_r, range);

NDD_P_sim = corr_sim_P(NDD1_P, NDD2_P);
% disp([NDD_P_sim]);
