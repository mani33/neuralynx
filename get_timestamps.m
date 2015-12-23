function [ts,ids,ttl] = get_timestamps

[fn,pn] = uigetfile('Events.nev','Pick a file','E:\CheetahData\*');
FieldSelectionFlags = [1 1 1 0 0];
     [ts,ids,ttl] = Nlx2MatEV(fullfile(pn,fn),FieldSelectionFlags,0,1,[]);