function mx = maxEnvHilbert(A, B)
mx = zeros(1, 1);
m = zeros(1, 1);
z = abs(hilbert(conv(A, B, 'same')));
for i = 1:length(z)
    if z(i) > m
        m = z(i);
        mx = i;
    end
end
