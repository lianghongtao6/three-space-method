function Three_Space
clear all;close all;clc;
cargodrone=[45 25 25; 30 30 22; 60 50 30; 25 20 25;
    25 20 27; 40 40 25; 32 32 17; 65 75 41];
cargotype=[8 10 14; 24 20 20];
cargoMED=[14 7 5; 5 8 5; 12 7 4];
cargoISO=[231 92 94];
cargo.sum = [cargodrone;cargotype;cargoMED;cargoISO];

volumedrone=[28125 19800 90000 12500 13500 40000 17408 199875];
volumetype=[1120 9600];
volumeMED=[490 200 336];
volumeISO=1997688;
volume.sum = [volumedrone volumetype volumeMED volumeISO];
volume.endsum = 0;

restcargo = [231 92 94];
originPoint = [0 0 0];

% low = 0.7;
% high = 1;
c = 0.90;

    droneB = floor(c * 121);
    droneD = floor(c * 54);
    medSum1 = floor(c * 411*7);
    medSum2 = floor(c * 411*2);
    medSum3 = floor(c * 411*4);
    onemed = [7 2 4];
    Amed = [medSum1-floor(medSum1*0.43)-floor(medSum1*0.17),medSum2-floor(medSum2*0.163),medSum3-floor(medSum3*0.35)];
    Bmed = [floor(medSum1*0.43),floor(medSum2*0.031),floor(medSum3*0.25)];
    Cmed = [floor(medSum1*0.17),floor(medSum2*0.132),floor(medSum3*0.10)];
    
    
    droneB_in_ConB = floor(droneB*0.4)-2;
    droneB_in_ConC = floor(droneB*0.6);
    droneH = 1;
    Type1A = droneD;
    Type1B = droneB_in_ConB;
    Type1C = droneB_in_ConC;
    wamountsumA = [0 0 0 droneD 0 0 0 droneH Type1A 0 Amed+0*onemed];
    wamountsumB = [0 droneB_in_ConB 0 0 0 0 0 droneH Type1B 0 Bmed+6*onemed];
    wamountsumC = [0 droneB_in_ConC 0 0 0 0 0 droneH Type1C 0 Cmed];
    
    cccccc = [231 92 94];
    dddddd = [0 0 0];
    PlotCuboid(dddddd,cccccc);
    [~,endvolumeA,endamountA] = model(cargo,volume,wamountsumA,restcargo,originPoint,0);
    if sum(endamountA) == 0
        name = 'A箱能放下';
        a = c;
    else
        name = 'A箱不能放下';
    end
    disp(name);
    per=endvolumeA/volumeISO;
    str = ['A箱空间利用率为',num2str(per)];
    disp(str);
    str=['A箱存入货物为' num2str(wamountsumA)];
    disp(str);
    str=['A箱剩余货物为' num2str(endamountA)];
    disp(str);
    
    figure;
    cccccc = [231 92 94];
    dddddd = [0 0 0];
    PlotCuboid(dddddd,cccccc);
    [~,endvolumeB,endamountB] = model(cargo,volume,wamountsumB,restcargo,originPoint,0);
    if sum(endamountB) == 0
        name = 'B箱能放下';
        a = c;
    else
        name = 'B箱不能放下';
    end
    disp(name);
    per=endvolumeB/volumeISO;
    str = ['B箱空间利用率为',num2str(per)];
    disp(str);
    str=['B箱存入货物为' num2str(wamountsumB)];
    disp(str);
    str=['B箱剩余货物为' num2str(endamountB)];
    disp(str);
    
    figure;
    cccccc = [231 92 94];
    dddddd = [0 0 0];
    PlotCuboid(dddddd,cccccc);
    [~,endvolumeC,endamountC] = model(cargo,volume,wamountsumC,restcargo,originPoint,0);
    if sum(endamountC) == 0
        name = 'C箱能放下';
    else
        name = 'C箱不能放下';
    end
    disp(name);
    per=endvolumeC/volumeISO;
    str = ['C箱空间利用率为',num2str(per)];
    disp(str);
    str=['C箱存入货物为' num2str(wamountsumC)];
    disp(str);
    str=['C箱剩余货物为' num2str(endamountC)];
    disp(str);
    
%     close all;
%     c = (low + high)/2;
%     if sum([endamountA endamountB endamountC])==0
%         low = c;
%     elseif sum([endamountA endamountB endamountC])>0
%         high = c;
%     end
    
end



function [whether,endvolume,endamount] = model(cargo,volume,amountsum,restcargo,originPoint,volumesum)
V = restcargo(1)*restcargo(2)*restcargo(3);
fper=0;whether = 0;
for i=1:1:size(amountsum,2)
    if amountsum(i)> 0
        Vs = cargo.sum(i,1)*cargo.sum(i,2)*cargo.sum(i,3);
        B11= sort(cargo.sum(i,:));B12 = sort(restcargo);
        if (B11(3)<=B12(3))&&(B11(2)<=B12(2))&&(B11(1)<=B12(1))
            per = Vs/V;
            if (per>fper)&&(per>0)
                fper = per;
                whether = 1;
                k = i;
            end
        end
    end
end
if whether == 1
    endvolume = volumesum + volume.sum(k);
    amountsum(k) = amountsum(k) - 1;
    hold on;
    [B11,I11] = sort(cargo.sum(k,:));
    [~,I1] = sort(restcargo);
    if I1(1) == 1  %剩余空间第一条边最短
        restV(1,1) = restcargo(1)-cargo.sum(k,I11(1));
        originPoint3(1) = originPoint(1)+B11(1);
        restV(2,1) = cargo.sum(k,I11(1));
        restV(3,1) = cargo.sum(k,I11(1));
        if I1(2) == 2  %剩余空间第一条边最短 第二条边中间长度 第三条边最长
            restV(2,2) = restcargo(2)-cargo.sum(k,I11(2));
            originPoint2(2) = originPoint(2)+cargo.sum(k,I11(2));
            restV(3,2) = cargo.sum(k,I11(2));
            restV(3,3) = restcargo(3)-cargo.sum(k,I11(3));
            originPoint1(3) = originPoint(3)+cargo.sum(k,I11(3));
            cuboidSize(1) = B11(1);
            cuboidSize(2) = B11(2);
            cuboidSize(3) = B11(3);
        else  %剩余空间第一条边最短 第二条边最长 第三条边中间长度
            restV(2,2) = restcargo(2)-cargo.sum(k,I11(3));
            originPoint2(2) = originPoint(2)+cargo.sum(k,I11(3));
            restV(3,2) = cargo.sum(k,I11(3));
            restV(3,3) = restcargo(3)-cargo.sum(k,I11(2));
            originPoint1(3) = originPoint(3)+cargo.sum(k,I11(2));
            cuboidSize(1) = B11(1);
            cuboidSize(2) = B11(3);
            cuboidSize(3) = B11(2);
        end
    elseif I1(3) == 1 %剩余空间第一条边最长
        restV(1,1) = restcargo(1)-cargo.sum(k,I11(3));
        originPoint3(1) = originPoint(1)+B11(3);
        restV(2,1) = cargo.sum(k,I11(3));
        restV(3,1) = cargo.sum(k,I11(3));
        if I1(2) == 2%剩余空间第一条边最长 第二条边中间长度 第三条边最短
            restV(2,2) = restcargo(2)-cargo.sum(k,I11(2));
            originPoint2(2) = originPoint(2)+cargo.sum(k,I11(2));
            restV(3,2) = cargo.sum(k,I11(2));
            restV(3,3) = restcargo(3)-cargo.sum(k,I11(1));
            originPoint1(3) = originPoint(3)+cargo.sum(k,I11(1));
            cuboidSize(1) = B11(3);
            cuboidSize(2) = B11(2);
            cuboidSize(3) = B11(1);
        else%剩余空间第一条边最长 第二条边最短 第三条边中间长度
            restV(2,2) = restcargo(2)-cargo.sum(k,I11(1));
            originPoint2(2) = originPoint(2)+cargo.sum(k,I11(1));
            restV(3,2) = cargo.sum(k,I11(1));
            restV(3,3) = restcargo(3)-cargo.sum(k,I11(2));
            originPoint1(3) = originPoint(3)+cargo.sum(k,I11(2));
            cuboidSize(1) = B11(3);
            cuboidSize(2) = B11(1);
            cuboidSize(3) = B11(2);
        end
    else %剩余空间第一条边中间长度 I1(2) == 1
        restV(1,1) = restcargo(1)-cargo.sum(k,I11(2));
        originPoint3(1) = originPoint(1)+B11(2);
        restV(2,1) = cargo.sum(k,I11(2));
        restV(3,1) = cargo.sum(k,I11(2));
        if I1(1) == 2 %剩余空间第一条边中间长度 第二条边最短 第三条边最长
            restV(2,2) = restcargo(2)-cargo.sum(k,I11(1));
            originPoint2(2) = originPoint(2)+cargo.sum(k,I11(1));
            restV(3,2) = cargo.sum(k,I11(1));
            restV(3,3) = restcargo(3)-cargo.sum(k,I11(3));
            originPoint1(3) = originPoint(3)+cargo.sum(k,I11(3));
            cuboidSize(1) = B11(2);
            cuboidSize(2) = B11(1);
            cuboidSize(3) = B11(3);
        else %剩余空间第一条边中间长度 第二条边最长 第三条边最短 I1(1) == 3
            restV(2,2) = restcargo(2)-cargo.sum(k,I11(3));
            originPoint2(2) = originPoint(2)+cargo.sum(k,I11(3));
            restV(3,2) = cargo.sum(k,I11(3));
            restV(3,3) = restcargo(3)-cargo.sum(k,I11(1));
            originPoint1(3) = originPoint(3)+cargo.sum(k,I11(1));
            cuboidSize(1) = B11(2);
            cuboidSize(2) = B11(3);
            cuboidSize(3) = B11(1);
        end
    end
    restV(1,2) =restcargo(2);
    restV(1,3) =restcargo(3);
    restV(2,3) =restcargo(3);
    originPoint3(2) = originPoint(2);
    originPoint3(3) = originPoint(3);
    originPoint2(1) = originPoint(1);
    originPoint2(3) = originPoint(3);
    originPoint1(1) = originPoint(1);
    originPoint1(2) = originPoint(2);
    PlotCuboid(originPoint,cuboidSize)
    
    [~,endvolume1,amount1] = model(cargo,volume,amountsum,restV(3,:),originPoint1,endvolume);
    [~,endvolume2,amount2] = model(cargo,volume,amount1,restV(2,:),originPoint2,endvolume1);
    [~,endvolume,endamount] = model(cargo,volume,amount2,restV(1,:),originPoint3,endvolume2);
    
    
else
    endvolume = volumesum;
    endamount = amountsum;
    return;
end
end

function PlotCuboid(originPoint,cuboidSize)
% 函数功能： 绘制长方体
% 输入：
%       originPoint：长方体的原点,行向量，如[0，0，0];
%       cuboidSize：长方体的长宽高,行向量，如[10，20，30];
% 输出：长方体图形

% 根据原点和尺寸，计算长方体的8个的顶点

vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
vertex=originPoint+vertexIndex.*cuboidSize;

% 定义6个平面分别对应的顶点
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];

% 定义8个顶点的颜色，绘制的平面颜色根据顶点的颜色进行插补
color=[1;2;3;4;5;6;7;8];

% 绘制并展示图像
% patch 对图像进行绘制。
% view(3) 将图像放到三维空间中展示。
% 其余的是设置背景等等
axis equal
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Cuboid');
fig=gcf;
fig.Color=[1 1 1];
fig.NumberTitle='off';
% view(160,30)
hold on
hold on
hold on
end