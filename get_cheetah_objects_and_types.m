function [cheetahObjects, cheetahTypes] = get_cheetah_objects_and_types()
% function [cheetahObjects, cheetahTypes] = NlxGetCheetahObjectsAndTypes()
% Mani Subramaniyan 2015-02-25

[succeeded, cheetahObjects, cheetahTypes] = NlxGetCheetahObjectsAndTypes;
if succeeded == 0
    disp 'FAILED get cheetah objects and types'
else
    disp 'PASSED get cheetah objects and types'
end