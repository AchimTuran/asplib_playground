% this script processes float samples with the SpectrumVisProcessor module from asplib

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


function [Out] = asplib_processSpectrumVisProcessor(In)
%TEST Summary of this function goes here
%   Detailed explanation goes here
	if not(libisloaded('SpectrumVisProcessorDll'))
    disp('[asplib] SpectrumVisProcessorDll is not loaded! Please run asplib_load_SpectrumVisProcessorDll.m first!');
    return;
  end

	pIn = libpointer('singlePtr', In);
	
	% ToDo evaluate err
	[err, Out] = calllib('SpectrumVisProcessorDll', 'ProcessSpectrumVisProcessor', pIn);
end