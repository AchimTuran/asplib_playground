% this script tests the SpectrumVisProcessor module from asplib

%/*
% * Copyright (C) 2014 Achim Turan, Achim.Turan@o2online.de
% * https://github.com/AchimTuran/asplib
% *
% * This file is part of asplib (Achim's Signal Processing LIBrary)
% *
% * asplib (Achim's Signal Processing LIBrary) is free software:
% * you can redistribute it and/or modify it under the terms of the
% * GNU General Public License as published by the Free Software Foundation,
% * either version 3 of the License, or (at your option) any later version.
% *
% * asplib (Achim's Signal Processing LIBrary) is distributed
% * in the hope that it will be useful, but WITHOUT ANY WARRANTY;
% * without even the implied warranty of MERCHANTABILITY or FITNESS
% * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
% *
% * You should have received a copy of the GNU General Public License
% * along with asplib (Achim's Signal Processing LIBrary).
% * If not, see <http://www.gnu.org/licenses/>.
% *
% */

% reset workspace
clear all;
close all;
clc;

% load asplib_CompressorDll
asplib_load_CompressorDll()

%load handel.mat
FrameSize = 128
sampleFrequency = 44100;

t = (0:sampleFrequency*1000)/sampleFrequency;
inputSignal = 0.6*sin(2*pi*200.*t);
inputSignal(3000:5000) = 2*inputSignal(3000:5000);

asplib_createCompressor(FrameSize, sampleFrequency);

while stop <= length(inputSignal)
    buffer = inputSignal(start:stop);
    start = stop + 1;
    stop = start + bufferSize - 1;
    b = [b,buffer];
    
    outputbuffer = asplib_processCompressor(buffer);

    output = [output,outputbuffer];   
end



asplib_destroyCompressorDll()

% unload asplib_MatlabDll
asplib_unload_CompressorDll()

figure
subplot(2,1,1)
plot(b)
title('Originalsignal')
grid on
hold on
plot(1:length(b),ones(1,length(b))*10^(Threshold/20),'r')
xlabel('t in s')
ylabel('|x(t)|')
subplot(2,1,2)
plot(output)
grid on
hold on
plot(1:length(b),ones(1,length(b))*10^(Threshold/20),'r')
title('Komprimiertes Signal')
xlabel('t in s')
ylabel('xc(t)')
ylim([-2,2])

Nfft = 1024;
f = (0:Nfft/2-1)*8000/Nfft;
X = abs(fft(output,Nfft)/Nfft);
Xorg = abs(fft(b,Nfft))/Nfft;

figure
subplot(2,1,1)
plot(f,Xorg(1:Nfft/2))
grid on
xlabel('f in Hz')
ylabel('|X(f)|')
title('Spektrum Originalsignal')
subplot(2,1,2)
plot(f,X(1:Nfft/2));
title('Spektrum komprimiertes Signal')
grid on
xlabel('f in Hz')
ylabel('|Xx(f)|')

