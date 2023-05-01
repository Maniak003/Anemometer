clear, clc, close all;
A = importdata('rawData1.txt');
B = importdata('refData1.txt');
C = conv(A, B, 'same');
s = hilbert(C);
%plot(A), grid on
z = abs(s);
m = max(z);
for i= 1:length(z)
   if z(i) == m
      o = i;
      break
   end
end
subplot(2,1,1)
plot(A);
subplot(2,1,2)
plot(C), grid on, hold on
plot(z), hold on
x = linspace(o, o, 2);
y = linspace(-m, m, 2);
line(x, y, 'color', 'red')
