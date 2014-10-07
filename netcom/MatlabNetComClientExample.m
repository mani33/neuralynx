%MatlabNetComClient example script.
%Connects to a server, streams some data into plots, then exits.
%
%Note: If you have any questions about any of the functions in the
%MatlabNetComClient package, you can type help <function_name> at the
%Matlab command prompt.
%
%Load NetCom into Matlab, and connect to the NetCom server
%If you are running Matlab on a PC other than the one with the NetCom
%server, you will need to change the server name to the name of the server
%PC.
serverName = 'localhost';
disp(sprintf('Connecting to %s...', serverName));
succeeded = NlxConnectToServer(serverName);
if succeeded ~= 1
    disp(sprintf('FAILED connect to %s. Exiting script.', serverName));
    return;
else
    disp(sprintf('Connect successful.'));
end

serverIP = NlxGetServerIPAddress();
disp(sprintf('Connected to IP address: %s', serverIP));

serverPCName = NlxGetServerPCName();
disp(sprintf('Connected to PC named: %s', serverPCName));

serverApplicationName = NlxGetServerApplicationName();
disp(sprintf('Connected to the NetCom server application: %s', serverApplicationName));

%Identify this program to the server we're connected to.
succeeded = NlxSetApplicationName('My Matlab Script');
if succeeded ~= 1
    disp 'FAILED set the application name'
else
    disp 'PASSED set the application name'
end

%get a list of all objects in Cheetah, along with their types.
[succeeded, cheetahObjects, cheetahTypes] = NlxGetCheetahObjectsAndTypes;
if succeeded == 0
    disp 'FAILED get cheetah objects and types'
else
    disp 'PASSED get cheetah objects and types'
end

%open up a stream for all objects
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

%send out an event so that there is something in the event buffer when
%this script queries the event buffer  You can use NlxSendCommand to send
%any Cheetah command to Cheetah.
[succeeded, cheetahReply] = NlxSendCommand('-PostEvent "Test Event" 10 11');
if succeeded == 0
    disp 'FAILED to send command'
else
    disp 'PASSED send command'
end

%This loop is run to get new data from NetCom.  All open streams must be
%servised regularly, or there will be dropped records.
numberOfPasses = 4;
for pass = 1:numberOfPasses
    for objectIndex = 1:length(cheetahObjects)
        objectToRetrieve = char(cheetahObjects(objectIndex));
        %determine the type of acquisition entity we are currently indexed
        %to and call the appropriate function for that type
        if strcmp('CscAcqEnt', char(cheetahTypes(objectIndex))) == 1
            [succeeded,dataArray, timeStampArray, channelNumberArray, samplingFreqArray, numValidSamplesArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewCSCData(objectToRetrieve);
            if succeeded == 0
                 disp(sprintf('FAILED to get new data for CSC stream %s on pass %d', objectToRetrieve, pass));
                break;
            else
                disp(sprintf('Retrieved %d CSC records for %s with %d dropped.', numRecordsReturned, objectToRetrieve, numRecordsDropped));
                
                %Here is where you'll perform some calculation on any of 
                %the returned values. Make sure any calculations done here
                %don't take too much time, otherwise NetCom will back up 
                %and you'' have dropped records
                plot(dataArray);
            end
        %The test and actions are repeated for each acquisition entity type
        elseif strcmp('SEScAcqEnt', char(cheetahTypes(objectIndex))) == 1
            [succeeded, dataArray, timeStampArray, spikeChannelNumberArray, cellNumberArray, featureArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewSEData(objectToRetrieve);
            if succeeded == 0
                 disp(sprintf('FAILED to get new data for SE stream %s on pass %d', objectToRetrieve, pass));
                break;
            else
                disp(sprintf('Retrieved %d SE records for %s with %d dropped.', numRecordsReturned, objectToRetrieve, numRecordsDropped));
                plot(dataArray);
            end
        elseif strcmp('STScAcqEnt', char(cheetahTypes(objectIndex))) == 1
            [succeeded, dataArray, timeStampArray, spikeChannelNumberArray, cellNumberArray, featureArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewSTData(objectToRetrieve);
            if succeeded == 0
                 disp(sprintf('FAILED to get new data for ST stream %s on pass %d', objectToRetrieve, pass));
                break;
            else
                disp(sprintf('Retrieved %d ST records for %s with %d dropped.', numRecordsReturned, objectToRetrieve, numRecordsDropped));
                plot(dataArray);
            end
         elseif strcmp('TTScAcqEnt', char(cheetahTypes(objectIndex))) == 1
            [succeeded, dataArray, timeStampArray, spikeChannelNumberArray, cellNumberArray, featureArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewTTData(objectToRetrieve);
            if succeeded == 0
                 disp(sprintf('FAILED to get new data for TT stream %s on pass %d', objectToRetrieve, pass));
                break;
            else
                disp(sprintf('Retrieved %d TT records for %s with %d dropped.', numRecordsReturned, objectToRetrieve, numRecordsDropped));
                plot(dataArray);
            end
         elseif strcmp('EventAcqEnt', char(cheetahTypes(objectIndex))) == 1
            [succeeded, timeStampArray, eventIDArray, ttlValueArray, eventStringArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewEventData(objectToRetrieve);
            if succeeded == 0
                 disp(sprintf('FAILED to get new data for event stream %s on pass %d', objectToRetrieve, pass));
                break;
            else
                disp(sprintf('Retrieved %d event records for %s with %d dropped.', numRecordsReturned, objectToRetrieve, numRecordsDropped));
                for recordIndex=1:numRecordsReturned                
                    disp(sprintf('Event String: %s Event ID: %d TTL Value: %d',char(eventStringArray(recordIndex)), eventIDArray(recordIndex), ttlValueArray(recordIndex)));
                end
            end
         elseif strcmp('VTAcqEnt', char(cheetahTypes(objectIndex))) == 1
            [succeeded,  timeStampArray, extractedLocationArray, extractedAngleArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewVTData(objectToRetrieve);
            if succeeded == 0
                 disp(sprintf('FAILED to get new data for VT stream %s on pass %d', objectToRetrieve, pass));
                break;
            else
                disp(sprintf('Retrieved %d VT records for %s with %d dropped.', numRecordsReturned, objectToRetrieve, numRecordsDropped));
               plot(extractedLocationArray);
            end   
        end
    end
   
    if succeeded == 0
        break;
    end
end
if succeeded == 0
    disp 'FAILED to get data consistently for all open streams'
else
    disp 'PASSED get data consistently for all open streams'
end

pause(3);

%close all open streams before disconnecting
for index = 1:length(cheetahObjects)
    succeeded = NlxCloseStream(cheetahObjects(index));
    if succeeded == 0
        disp(sprintf('FAILED to close stream for %s', char(cheetahObjects(index))));
        break;
    end
end;
if succeeded == 1
    disp 'PASSED close stream for all current objects'
end


%Disconnects from the server and shuts down NetCom
succeeded = NlxDisconnectFromServer();
if succeeded ~= 1
    disp 'FAILED disconnect from server'
else
    disp 'PASSED disconnect from server'
end

%remove all vars created in this test script
clear
    