% this script unloads the SpectrumVisProcessorDll and releases all requested memory

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



if (libisloaded('SpectrumVisProcessorDll'))
  calllib('SpectrumVisProcessorDll', 'DestroySpectrumVisProcessor')
  unloadlibrary('SpectrumVisProcessorDll');
  disp('[asplib] unloaded SpectrumVisProcessorDll');
else
  disp('[asplib] SpectrumVisProcessorDll was already unloaded');
end
