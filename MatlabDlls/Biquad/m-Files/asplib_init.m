% initializes asplib's biquad handle for MATLAB
%   SampleFrequency = sample frequency of the input and output signal
%   MaxChannels     = maximum amount of channels to process
%   MaxFrameSize    = maximum amount of samples per frame


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



function [] = asplib_init(SampleFrequency, MaxChannels, MaxFrameSize)
  if not(libisloaded('BiquadDll'))
    disp('[asplib] BiquadDll was not loaded! Please run asplib_load_BiquadDll.m first!');
    %return;
  end
  
  % ToDo evaluate err
  err = calllib('BiquadDll', 'init_asplib', single(SampleFrequency), uint32(MaxChannels), uint32(MaxFrameSize));
end
