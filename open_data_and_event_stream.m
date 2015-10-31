function open_data_and_event_stream(cheetahObjects)
% function open_data_and_event_stream(cheetahObjects)
% Mani Subramaniyan 2015-02-25

for index = 1:length(cheetahObjects)
    succeeded = NlxOpenStream(cheetahObjects(index));
    if succeeded == 0
        disp(sprintf('FAILED to open stream for %s', char(cheetahObjects(index))));
        break;
    end
end;
if succeeded == 1
    disp 'PASSED open stream for all current objects'
end