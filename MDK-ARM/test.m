% savefig(fig1,"pos_err.fig");
% 
% % savefig(fig2,"spd_err.fig");
% clear;
close all;

file1 = fullfile("data_go.log");
f1 = fopen(file1);
dt = textscan(f1,'%s');

% close all;

data0 = dt{1,1};
data01 = str2double(data0);
len = size(data01,1);
pos_err_idx0 = 1:2:len;
spd_err_idx0 = 2:2:len;
% time_idx0 = 3:3:len;
pos_err0 = data01(pos_err_idx0);
spd_err0 = data01(spd_err_idx0)/pi*180;
% 
% file2 = fullfile("data1.log");
% f2 = fopen(file2);
% dt2 = textscan(f2,'%s');

% close all;
% 
% data1 = dt2{1,1};
% data11 = str2double(data1);
% len = size(data11,1);
% pos_err_idx1 = 1:3:len;
% spd_err_idx1 = 2:3:len;
% time_idx1 = 3:3:len;
% pos_err1 = data11(pos_err_idx1);
% spd_err1 = data11(spd_err_idx1)/pi*180;

% t1 = data01(time_idx0)/1000;
% t1 = t1-t1(1)*ones(501,1);
t1 = 0:0.010:5;

% t2 = data11(time_idx1)/1000;
% t2 = t2-t2(1)*ones(501,1);

tf = 5;
t0 = 0;
tt = tf -t0;
dt = 0.010;
t_vec = t0:dt:tf;

delta_x = 360;
a3 = 10*delta_x/(tt.^3);
a4 = -15*delta_x/(tt.^4);
a5 = 6*delta_x/(tt.^5);

s = a3*t_vec.^3 + a4*t_vec.^4 + a5 * t_vec.^5; 
ds = 3*a3*t_vec.^2 + 4*a4*t_vec.^3 + 5*a5 * t_vec.^4; 
s = s';

fig1 = figure();
plot(t1,pos_err0);%s(1:501) - 
hold on;
% plot(t2,pos_err1);

% plot(t_vec,s);

% plot(t_vec,s);

% fig2 = figure();
% plot(t,spd_err);
% hold on;
% plot(t_vec,ds);
