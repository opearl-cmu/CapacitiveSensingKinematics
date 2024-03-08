function [outputEMG]=ProcessEMGLinearEnvelope(inputEMG, emgTime, SampleFreq, FilterOrder, ArtifactHighPassFreq, ArtifactLowPassFreq, SmoothLowPassFreq, startEMGnormRange, endEMGnormRange, scale)
% ProcessEMGLinearEnvelope Calculate Linear Envelope of EMG Signal
%   The input EMG data is processed with a high pass filter, rectifier and 
%   low pass filter. The filter used is a double pass second order
%   butterworth filter. This function is designed for the Biometric K800
%   amplifier.
% 
% Input:
%    inputEMG: Input EMG Data for Processing
%    SampleFreq: threshold value
%    FilterOrder:
%    ArtifactHighPassFreq: 
%    ArtifactLowPassFreq: 
%    SmoothLowPassFreq:
%
% Output:
%    outputEMG: Output EMG Data 
%
% Usage: ProcessEMGLinearEnvelope( inputEMG, SampleFreq, FilterOrder, ArtifactHighPassFreq, ArtifactLowPassFreq, SmoothLowPassFreq) 
    % Nyquist Freq
    nyqFreq = SampleFreq/2;
    
%     Artifact and Offset Removal High Pass
    [b,a]=butter(2, [ArtifactLowPassFreq/nyqFreq, ArtifactHighPassFreq/nyqFreq]);
    emgSignals=filtfilt(b,a,inputEMG);

    % Full Wave Rectify
    emgSignals = abs(inputEMG);

    % Low Pass Filter
    [b,a]=butter(FilterOrder,SmoothLowPassFreq/nyqFreq,'low');
    emgSignals=filtfilt(b,a,emgSignals);

    % Truncate Any Zeros
    zeroIndices = emgSignals < 0;
    emgSignals(zeroIndices) = 0;
    
    % Normalize to 0-1 range using min max over time range
    normEMGRange = emgSignals(emgTime > startEMGnormRange & emgTime < endEMGnormRange);
    emgSignals = scale.*(emgSignals - min(normEMGRange))./(max(normEMGRange) - min(normEMGRange));

    % Store
    outputEMG = emgSignals;
end

