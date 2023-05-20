clear, clc, close all;
A = importdata('rawData2.txt');
B = importdata('refData2.txt');

C = conv(A, B, 'same');
subplot(2,2,1)
plot(A), grid on, hold on
subplot(2,2,2)
plot(C), grid on
N = length(C);
X = fft(C);

fp = 2: N/2 ;
fn = (N)/2 + 2 : N;

S(fp) = X(fp) * 2;
S(fn) = 0;

s = ifft(S);
%plot(A), grid on
z = abs(s);
m = max(z);
for i= 1:length(z)
   if z(i) == m
      o = i;
      break
   end
end

subplot(2,1,2)
plot(C), grid on, hold on
plot(z), hold on
x = linspace(o, o, 2);
y = linspace(-m, m, 2);
line(x, y, 'color', 'red')
