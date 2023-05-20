clear, clc, close all;
A = importdata('rawData1.txt');
B = importdata('refData1.txt');

mm = maxEnv(A, B);
display(mm);
