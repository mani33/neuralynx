function data = random_stim(levels,nRep)
%% Generate random current level for input-output curve
% levels = [25:25:100 150 250 350];
n = length(levels);
data.current_levels = cell(1,nRep);
for i = 1:nRep
    rn = randperm(n);
    data.current_levels{i} = levels(rn)';
    fprintf('set %d: %s\n',i,mat2str(data.current_levels{i}'))
end
% Save current levels to the directory of the latest data collection
% session
data.nBlocks = nRep;

d = dir('C:\CheetahData\2015*');
[~,ind] = sort([d.datenum]);
% file = ['rand_curr_levels_' datestr(now,'yyyy-mm-dd_HH-MM-SS') '.mat'];
file = 'current_levels_io_uamps.mat';
dd = fullfile('C:\CheetahData',d(ind(end)).name,file);
save(dd,'data')
