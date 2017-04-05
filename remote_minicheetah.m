function varargout = remote_minicheetah(varargin)
% REMOTE_MINICHEETAH MATLAB code for remote_minicheetah.fig
%      REMOTE_MINICHEETAH, by itself, creates a new REMOTE_MINICHEETAH or raises the existing
%      singleton*.
%
%      H = REMOTE_MINICHEETAH returns the handle to a new REMOTE_MINICHEETAH or the handle to
%      the existing singleton*.
%
%      REMOTE_MINICHEETAH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in REMOTE_MINICHEETAH.M with the given input arguments.
%
%      REMOTE_MINICHEETAH('Property','Value',...) creates a new REMOTE_MINICHEETAH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before remote_minicheetah_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to remote_minicheetah_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help remote_minicheetah

% Last Modified by GUIDE v2.5 23-Dec-2016 21:20:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @remote_minicheetah_OpeningFcn, ...
    'gui_OutputFcn',  @remote_minicheetah_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before remote_minicheetah is made visible.
function remote_minicheetah_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to remote_minicheetah (see VARARGIN)

% Choose default command line output for remote_minicheetah
handles.output = hObject;
handles.server = '130.91.204.37';

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes remote_minicheetah wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = remote_minicheetah_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in connect.
function connect_Callback(hObject, eventdata, handles)
% hObject    handle to connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
add_path_to_neuralynx_lib
connect_to_cheetah_server(handles.server);
NlxSetApplicationName('MATLAB_MiniCheetah')
[cheetahObjects, cheetahTypes] = get_cheetah_objects_and_types();
% Choose VT1 object which is the video tracker channel
vtobj = 'VT1';
handles.objectToRetrieve = vtobj;
isvt = strcmp(cheetahObjects,vtobj);
assert(any(isvt), 'Video tracking object VT1 not found')
suc = open_data_and_event_stream({vtobj});
if suc
    mess = 'Opened video tracker stream successfully';
else
    mess = 'Failed to open video tracker stream';
end
handles.cheetahObjects = cheetahObjects(isvt);
handles.cheetahTypes = cheetahTypes(isvt);
handles.sel_data = {};
handles.sel_time = {};

[~,dd] =  NlxSendCommand('-GetDataDirectory ');
dd = strtrim(strrep( dd{:},'"',''));
handles.cheetah_data_dir = dd;
set(handles.message_update,'String',mess)
guidata(hObject,handles)

% --- Executes on button press in disconnect.
function disconnect_Callback(hObject, eventdata, handles)
% hObject    handle to disconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject,'UserData',1)
%%close all open streams before disconnecting
cheetahObjects = handles.cheetahObjects;
n = length(cheetahObjects);
succeeded = false(1,n);
for index = 1:n
    succeeded(index) = NlxCloseStream(cheetahObjects(index));
    if succeeded(index) == 0
        sprintf('FAILED to close stream for %s', char(cheetahObjects(index)));
        break;
    end
end;
suc = all(succeeded);
if suc
    mess = 'PASSED close stream for all current objects';
else
    mess = 'FAILED to close stream';
end
set(handles.message_update,'String',mess)

%Disconnects from the server and shuts down NetCom
succeeded = NlxDisconnectFromServer();
if succeeded ~= 1
    disp 'FAILED disconnect from server'
else
    disp 'PASSED disconnect from server'
end


% --- Executes on button press in acq.
function acq_Callback(hObject, eventdata, handles)
% hObject    handle to acq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of acq
if logical(get(hObject,'Value'))
    [succeeded, ~] = NlxSendCommand('-StartAcquisition');
    if succeeded
        mess = 'Started Acquisition';
    else
        mess = 'Failed to start Acquisition';
    end
    
else
    [succeeded, ~] = NlxSendCommand('-StopAcquisition');
    if succeeded
        mess = 'Stopped Acquisition';
    else
        mess = 'Failed to stop Acquisition';
    end
end
set(handles.message_update,'String',mess)

% --- Executes on button press in record.
function record_Callback(hObject, eventdata, handles)
% hObject    handle to record (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of record
if logical(get(hObject,'Value'))
    [succeeded, ~] = NlxSendCommand('-StartRecording');
    if succeeded
        mess = 'Started Recording';
    else
        mess = 'Failed to start Recording';
    end
    
else
    [succeeded, ~] = NlxSendCommand('-StopRecording');
    if succeeded
        mess = 'Stopped Recording';
    else
        mess = 'Failed to stop Recording';
    end
end
set(handles.message_update,'String',mess)


function message_update_Callback(hObject, eventdata, handles)
% hObject    handle to message_update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of message_update as text
%        str2double(get(hObject,'String')) returns contents of message_update as a double


% --- Executes during object creation, after setting all properties.
function message_update_CreateFcn(hObject, eventdata, handles)
% hObject    handle to message_update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% clear_exp_data;
set(handles.stop_watching,'Value',0)
guidata(hObject,handles);
loop_period = 0.05;

% Before start, remove data in existing buffer of Cheetah by simply reading
% them off.
[~,~, ~, ~, ~, ~ ] = NlxGetNewVTData(handles.objectToRetrieve);

% Set buffer size for around 4 seconds by default
buffer_duration = 5; % in sec

buff_size = round(buffer_duration/loop_period);
tic
temp = struct;
temp.data_xy = cell(1,1);
temp.data_t = cell(1,1);
i = 1;
handles.t0 = 0;
t0captured = false;

while ~logical(get(handles.stop_watching,'Value'))
    [suc,  tsArray, xy, ~, ~, ~ ] = NlxGetNewVTData('VT1');
    if suc && ~isempty(tsArray)
        temp.data_xy{i} = double(xy);
        temp.data_t{i} = tsArray;
        ts = double(tsArray);
        if ~t0captured
            handles.t0 = ts(1);
            t0captured = true;
        end
    end
    %      if data_duration >= buffer_duration
    if i == buff_size
        i = buff_size;
        xy = [temp.data_xy{:}]';
        handles.data_xy = [xy(1:2:end) xy(2:2:end)];
        handles.data_t = ((double([temp.data_t{:}]')-handles.t0)*1e-6)/60; % uS to min
        handles = process_data(handles);
        
        % Circular buffer - most recent data at the end
        
        if ~isempty(temp.data_xy)
            temp.data_xy(1) = [];
        end
        if ~isempty(temp.data_t)
            temp.data_t(1) = [];
        end
    else
        i = i + 1;
    end
    pause(loop_period)
end
function handles = process_data(handles)
update_plot(handles);

function update_plot(handles)
xy = handles.data_xy;
t = handles.data_t;
if logical(get(handles.enable_xy_plot,'Value'))
    update_xy_plot(xy,handles)
end

if logical(get(handles.enable_xyt_plot,'Value'))
    update_xyt_plot(t,xy,handles)
end

function update_xy_plot(xy,handles)
axes(handles.mouse_xy_plot)
axis ij
plot(xy(:,1),xy(:,2),'k-')
axis ij
hold on
plot(xy(end,1),xy(end,2),'rO','markerfacecolor','r')
axis ij
axis([0 720 0 480])
hold off

function update_xyt_plot(t,xy,handles)
axes(handles.mouse_xyt_plot)

plot(t,xy(:,1),'r')
hold on
plot(t,xy(:,2),'b')
ylim([0 750])
hold off





% --- Executes on button press in clear_plot.
function clear_plot_Callback(hObject, eventdata, handles)
% hObject    handle to clear_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla

% --- Executes on button press in stop_watching.
function stop_watching_Callback(hObject, eventdata, handles)
% hObject    handle to stop_watching (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of stop_watching
if get(hObject,'Value')
    set(handles.run,'Value',0)
end

% --- Executes on button press in enable_xy_plot.
function enable_xy_plot_Callback(hObject, eventdata, handles)
% hObject    handle to enable_xy_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_xy_plot


% --- Executes on button press in enable_xyt_plot.
function enable_xyt_plot_Callback(hObject, eventdata, handles)
% hObject    handle to enable_xyt_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_xyt_plot
