Fs = 16000;Fs = 16000;  N = 1024;  T = 30;  K = 3;

% record 30 s from laptop mic (download Audio Toolbox if you don't have it)
d = audioDeviceReader("SampleRate",Fs,"SamplesPerFrame",N);
x = zeros(T*Fs,1);
for i = 1:floor(numel(x)/N)
    x((i-1)*N+1:i*N) = d();
end
release(d);
x = x - mean(x);

% spectrogram (freq x time)
[S,F,t] = spectrogram(x, hann(N), 0, N, Fs); % no overlap, N-point FFT
M = 20*log10(abs(S)+1e-12); % log magnitude to display

% top 3 dominant freqs per time slice, consider increasing for sirens
domF = nan(K, numel(t));
for k = 1:numel(t)
    [~,locs] = findpeaks(abs(S(:,k)),'SortStr','descend');
    kk = min(K,numel(locs));
    domF(1:kk,k) = F(locs(1:kk));
end

% plot heatmap + peaks
figure; imagesc(t,F,M); axis xy; ylim([0 3000]); colorbar;
xlabel("Time (s)"); ylabel("Frequency (Hz)");
title("30 s Spectrogram + Top Peaks");

hold on;
plot(t, domF, '.');  % plots all K rows at once
hold off;
legend("Dom #1","Dom #2","Dom #3","Location","northeast");
