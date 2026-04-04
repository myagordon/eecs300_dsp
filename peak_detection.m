clear;
clc;
close all;

fs = 16000;
recordTime = 2; 
frameLen = 512;
minMag = 0.1; %min mag of a peak, tune this based on volume
minPeakSpacingHz = 250; %spacing between peaks in hz, rougly corresponds to harmonics spacing 
peakFactor = 4; %factor greater than avg the peak must be to register as a siren

recObj = audiorecorder(fs, 16, 1);
disp('Recording.')
recordblocking(recObj, recordTime);
disp('Done.')

x = getaudiodata(recObj);

N = frameLen;
minPeakSpacingBins = floor(minPeakSpacingHz * N / fs);

lastMag = [];
lastF = [];
lastPeak1_idx = -1;
lastPeak1_mag = 0;
lastPeak2_idx = -1;
lastPeak2_mag = 0;

for startSample = 1:frameLen:(length(x) - frameLen + 1)

    frame = x(startSample:startSample + frameLen - 1);

    mag = abs(fft(frame));
    mag = mag(1:floor(N/2));
    f = (0:length(mag)-1) * fs / N;

    startIdx = floor(600 * N / fs) + 1;
    endIdx   = floor(3000 * N / fs) + 1;

    bandAvg = mean(mag(startIdx:endIdx));

    peakIdxList = [];
    peakMagList = [];

    lastAcceptedIdx = -1000000;

    for k = startIdx+1:endIdx-1
        if mag(k) > mag(k-1) && mag(k) > mag(k+1) && mag(k) > minMag && mag(k) > peakFactor * bandAvg
            if k - lastAcceptedIdx >= minPeakSpacingBins
                peakIdxList(end+1) = k;
                peakMagList(end+1) = mag(k);
                lastAcceptedIdx = k;
            end
        end
    end

    peak1_idx = -1;
    peak1_mag = 0;
    peak2_idx = -1;
    peak2_mag = 0;

    if ~isempty(peakMagList)
        [sortedMags, order] = sort(peakMagList, 'descend');

        peak1_mag = sortedMags(1);
        peak1_idx = peakIdxList(order(1));

        if length(sortedMags) >= 2
            peak2_mag = sortedMags(2);
            peak2_idx = peakIdxList(order(2));
        end
    end

    if peak1_idx == -1
        peak1_freq = -1;
    else
        peak1_freq = f(peak1_idx);
    end

    if peak2_idx == -1
        peak2_freq = -1;
    else
        peak2_freq = f(peak2_idx);
    end

    disp(['Frame start sample: ', num2str(startSample)])
    disp(['Band average: ', num2str(bandAvg)])
    disp(['Peak 1 freq: ', num2str(peak1_freq), ' Hz'])
    disp(['Peak 1 mag:  ', num2str(peak1_mag)])
    disp(['Peak 2 freq: ', num2str(peak2_freq), ' Hz'])
    disp(['Peak 2 mag:  ', num2str(peak2_mag)])
    disp(' ')

    lastMag = mag;
    lastF = f;
    lastPeak1_idx = peak1_idx;
    lastPeak1_mag = peak1_mag;
    lastPeak2_idx = peak2_idx;
    lastPeak2_mag = peak2_mag;
end

figure;
plot(lastF, lastMag);
hold on;
xlim([0 4000]);
xlabel('Frequency Hz');
ylabel('Magnitude');
title('FFT Mag Spectrum (Last 512-Sample Frame)');

if lastPeak1_idx ~= -1
    plot(lastF(lastPeak1_idx), lastPeak1_mag, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
end

if lastPeak2_idx ~= -1
    plot(lastF(lastPeak2_idx), lastPeak2_mag, 'go', 'MarkerSize', 8, 'LineWidth', 2);
end

grid on;
legend('Spectrum', 'Peak 1', 'Peak 2');
