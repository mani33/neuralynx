function ts = get_timestamps

[fn,pn] = uigetfile('Events.nev','Pick a file','C:\CheetahData\*');
FieldSelectionFlags = [1 0 0 0 1];
     [ts,~] = Nlx2MatEV(fullfile(pn,fn),FieldSelectionFlags,0,1,[]);