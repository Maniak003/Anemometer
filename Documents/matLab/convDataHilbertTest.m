clear, clc, close all;
A = importdata('rawData1.txt');
B = importdata('refData1.txt');
mm = maxEnvHilbert(A, B);
display(mm);