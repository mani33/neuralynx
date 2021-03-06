%NLXCONNECTTOSERVER   Connects to the specified NetCom Server
%   Takes a string containing the computer name or IP address 
%   of the NetCom Server computer, and attempts to connect to it.
%
%   NLXCONNECTTOSERVER(SERVERNAME) attempts to connect to the server
%
%   Example:  NlxConnectToServer('CheetahPC');
%	Connects to a NetCom server running on a computer named 'CheetahPC'
%
%	Returns: 1 means a successful connection was made.
%			 0 means the connection failed
%
%   Class support for input SERVERNAME:
%      string
%

function succeeded = NlxConnectToServer(serverName)  

  %load library if not already loaded
  if ~libisloaded('MatlabNetComClient')
    %load the 64bit DLL if we are running 64bit Matlab
    if(strcmp(mexext(), 'mexw64') == 1) 
      loadlibrary('MatlabNetComClient2_x64', 'MatlabNetComClient2_x64_proto', 'alias', 'MatlabNetComClient');
    else
      loadlibrary('MatlabNetComClient2', 'MatlabNetComClient2_proto', 'alias', 'MatlabNetComClient');
    end
  end
    
  %make sure the library is loaded correctly
  succeeded = libisloaded('MatlabNetComClient');
  
  if succeeded == 1
    succeeded = calllib('MatlabNetComClient', 'ConnectToServer', serverName);
  else
    unloadlibrary('MatlabNetComClient');
  end;
  
end