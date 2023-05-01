function mx = maxEnv(A, B)
mx = zeros(1, 1);
C = conv(A, B, 'same');
N = length(C);
X = fft(C);

fp = 2: N/2 ;
fn = (N)/2 + 2 : N;
S = complex(zeros(N, 1));
S(fp) = X(fp) * 2;
S(fn) = 0;
z = abs(ifft(S));
m = max(z);
for i= 1:length(z)
   if z(i) == m
      mx = i;
      break
   end
end
