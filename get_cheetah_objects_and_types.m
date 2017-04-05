function [cheetahObjects, cheetahTypes] = get_cheetah_objects_and_types()
% function [cheetahObjects, cheetahTypes] = NlxGetCheetahObjectsAndTypes()
% Mani Subramaniyan 2015-02-25

% [succeeded, cheetahObjects, cheetahTypes] = NlxGetCheetahObjectsAndTypes;
[succeeded, cheetahObjects, cheetahTypes] = NlxGetDASObjectsAndTypes;
if succeeded == 0
    disp 'FAILED get cheetah objects and types'
else
    disp 'PASSED get cheetah objects and types'
end

% Remove the AcqSystem1 object because it can't be opened for streaming
tf = ~strcmp(cheetahObjects,'AcqSystem1');
cheetahObjects = cheetahObjects(tf);
cheetahTypes = cheetahTypes(tf);