function succeeded = open_data_and_event_stream(cheetahObjects)
% function open_data_and_event_stream(cheetahObjects)
% Mani Subramaniyan 2015-02-25
N = length(cheetahObjects);
succeeded = false(1,N);
for index = 1:N
    succeeded(index) = NlxOpenStream(cheetahObjects(index));
    if ~succeeded(index)
        sprintf('FAILED to open stream for %s', char(cheetahObjects(index)));
        break;
    end
end;
succeeded = all(succeeded);
if succeeded
    disp 'PASSED open stream for all current objects'
end