close all;clear;clc
% 俯视图矩阵(原图转鸟瞰图矩阵)
hb=[-0.04830713747496683, -0.4373795992324429, 143.9690250293831;
0.04183045591775792, -1.310515413405067, 407.3017489049775;
0.0001219405187325797, -0.004114035756470801, 1];
% 测距矩阵 (原图点转自车距离)
hm=[-0.6172059535628878, 0.151643317385102, 339.6902632022253;
-0.02809492209016167, -0.05975965575762906, -873.0178598361697;
0.0001219405261698892, -0.004114035991148776, 1];
% 图像控制点像素坐标
pxy=[136.059	627.653
296.873	501.451
411.184	413.759
479.116	363.219
1109.53	664.576
947.094	526.368
823.338	429.558
753.371	371.971
];
figure(1)
plot(pxy(:,1),pxy(:,2),'r*')
xlabel('x pixel')
ylabel('y pixel')
title('图像的像素平面坐标')
p2=[0.4112    0.4138]*1000;p7=[0.8233    0.4296]*1000;
hold on
p27=(p2+p7)/2; %27中心点验证点
plot(p27(1),p27(2),'b^')
%% 
pxy1=[pxy,ones(8,1)]
pt27=[p27,1];
% 图像控制点鸟瞰图像素坐标
pxy_bird_view_img=[]
for i=1:8
    pxy_bird_view=hb*pxy1(i,:)';
    pxy_bird_view=pxy_bird_view/pxy_bird_view(3);
    pxy_bird_view_img=[pxy_bird_view_img;pxy_bird_view(1:2)'];
end
ptb=hb *pt27';
ptbxy=ptb(1:2)/ptb(3);
figure(2)
plot(pxy_bird_view_img(:,1),pxy_bird_view_img(:,2),'g*')
hold on
plot(ptbxy(1),ptbxy(2),'b^')
xlabel('x pixel form bird')
ylabel('y pixel form bird')
title('摄像机鸟瞰图的坐标系 绿色为此坐标系下控制点坐标')


%% 
% pxy1=[pxy,ones(8,1)]
% pt27=[p27,1];

%  第一种方式 绿色 hm测矩 
pxy_dist_hm=[]
figure(2)
for i=1:8
    pxy_hm=hm*pxy1(i,:)';
    pxy_hm=pxy_hm/pxy_hm(3);
    pxy_dist_hm=[pxy_dist_hm;pxy_hm(1:2)'];
end
figure(3)
plot(pxy_dist_hm(:,1),pxy_dist_hm(:,2),'g^')
ptm=hm*pt27'% 真实值[-0，1387]
ptm_d=ptm(1:2)/ptm(3);
hold on
plot(ptm_d(1),ptm_d(2),'g*')


%  第二种方式 蓝色 10cm  
pxy_bdist=[110,320].*ones(8,2)-pxy_bird_view_img;
pxy_bird_dist=[-pxy_bdist(:,1),pxy_bdist(:,2)]*10;
plot(pxy_bird_dist(:,1),pxy_bird_dist(:,2),'b*')
hold on
% ptb=hb *pt27';
% ptbxy=ptb(1:2)/ptb(3);
ptbm1=([110,320]-ptbxy')*10; % bird pixel (110,320) 
plot(-ptbm1(1),ptbm1(2),'b^')
xlabel('x距离cm')
ylabel('y距离cm')
title('摄像机地面投影为原点的坐标系 绿色为此坐标系下控制点坐标')
