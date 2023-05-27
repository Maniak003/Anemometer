clear, clc, close all;
A = importdata('rawData1.txt');
B = importdata('refData1.txt');
C = conv(A, B, 'same');
z = abs(hilbert(C));
mx1 = 0;
for i = 1:length(z)
   if z(i) > mx1
      mx1 = z(i);
      o = i;
   end
end
n = linspace(o - 4, o + 5, 29);
D = spline(o - 4: o + 5, z(o - 4: o + 5) , n);
mx2 = 0;
for i = 1:length(D)
   if D(i) > mx2
      mx2 = D(i);
      k = i;
   end
end
k = o - 4 + (k - 1) / 3;
subplot(2,1,1)
plot(A);
subplot(2,1,2)
plot(C), grid on, hold on
plot(z, '.','Color', 'red'), hold on
x = linspace(k, k, 1);
y = linspace(mx2, mx2, 1);
plot(k, mx2, '*', 'color', 'green')
