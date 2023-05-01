clear, clc, close all;
A = importdata('/home/ed/work/gitprojects/Anemometer/Documents/matLab/rawData1.txt');
B = importdata('/home/ed/work/gitprojects/Anemometer/Documents/matLab/refData1.txt');

mm = maxEnv(A, B);
display(mm);