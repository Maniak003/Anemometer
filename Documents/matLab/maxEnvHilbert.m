function mx = maxEnvHilbert(A, B)
mx = zeros(1, 1);
mx2 = zeros(1, 1);
m = zeros(1, 1);
k = zeros(1, 1);
z = abs(hilbert(conv(A, B, 'same')));
for i = 1:length(z)
    if z(i) > m
        m = z(i);
        mx = i;
    end
end
n = linspace(mx - 4, mx + 5, 29);
D = spline(mx - 4: mx + 5, z(mx - 4: mx + 5) , n);
for i = 1:length(D)
   if D(i) > mx2
      mx2 = D(i);
      k = i;
   end
end
mx = mx - 4 + (k - 1) / 3;
