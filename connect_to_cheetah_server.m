function connect_to_cheetah_server()
% Mani Subramaniyan 2015-02-25

serverName = 'localhost';
disp(sprintf('Connecting to %s...', serverName));
succeeded = NlxConnectToServer(serverName);
if succeeded ~= 1
    disp(sprintf('FAILED connect to %s. Exiting script.', serverName));
    return;
else
    disp(sprintf('Connect successful.'));
end

