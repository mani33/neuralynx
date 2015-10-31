function varargout = iocurve(varargin)
% IOCURVE MATLAB code for iocurve.fig
%      IOCURVE, by itself, creates a new IOCURVE or raises the existing
%      singleton*.
%
%      H = IOCURVE returns the handle to a new IOCURVE or the handle to
%      the existing singleton*.
%
%      IOCURVE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IOCURVE.M with the given input arguments.
%
%      IOCURVE('Property','Value',...) creates a new IOCURVE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before iocurve_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to iocurve_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to start (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help iocurve

% Last Modified by GUIDE v2.5 15-Aug-2015 06:44:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @iocurve_OpeningFcn, ...
    'gui_OutputFcn',  @iocurve_OutputFcn, ...
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


% --- Executes just before iocurve is made visible.
function iocurve_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to iocurve (see VARARGIN)

% Choose default command line output for iocurve
handles.output = hObject;
handles.duration = 2;
handles.pre_event_time = 0.5;
handles.post_event_time = 0.5;
handles.overlay_plots = false;
handles.multicolor = false;
handles.xmax = 1; % freq max in KHz for display
set(handles.preEventTime,'String',handles.duration)
set(handles.stop,'UserData',0);
set(handles.postEventTime,'String',handles.post_event_time);
set(handles.preEventTime,'String',handles.pre_event_time);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes iocurve wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = iocurve_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% Connect to server

add_path_to_neuralynx_lib
connect_to_cheetah_server()
[cheetahObjects, cheetahTypes] = get_cheetah_objects_and_types();
set(handles.channel_list,'String',cheetahObjects)
open_data_and_event_stream(cheetahObjects);
handles.cheetahObjects = cheetahObjects;
handles.cheetahTypes = cheetahTypes;
handles.sel_data = {};
handles.sel_time = {};
guidata(hObject,handles)


% --- Executes on selection change in channel_list.
function channel_list_Callback(hObject, eventdata, handles)
% hObject    handle to channel_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns channel_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from channel_list

contents = cellstr(get(handles.channel_list,'String'));
handles.objectToRetrieve =  contents{get(hObject,'Value')};

guidata(hObject,handles)
% update_plot(handles);





% --- Executes during object creation, after setting all properties.
function channel_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(hObject,'UserData',1)
%%close all open streams before disconnecting
cheetahObjects = handles.cheetahObjects;
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



% --- Executes on button press in overlay.
function overlay_Callback(hObject, eventdata, handles)
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.overlay_plots = logical(get(hObject,'Value'));
guidata(hObject,handles)

% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.spectrum)
cla
axes(handles.raw_trace)
cla


function preEventTime_Callback(hObject, eventdata, handles)
% hObject    handle to preEventTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of preEventTime as text
%        str2double(get(hObject,'String')) returns contents of preEventTime as a double


% --- Executes during object creation, after setting all properties.
function preEventTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to preEventTime (see GCBO)
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
set(handles.stop_watching,'Value',0)
guidata(hObject,handles);
loop_period = 0.05;
buff_prev = 0;
event_detected = false;
temp = struct;
buff_size = 0;
i = 1;
kn = 0;

while ~logical(get(handles.stop_watching,'Value'))
    if buff_prev ~= buff_size
        i = 1;
        kn = 0;
        temp = struct;
    end
   
   
    buff_size = round((temp.pre+temp.post)/loop_period);
    handles.pre = str2double(get(handles.preEventTime,'String'));
    handles.post = str2double(get(handles.postEventTime,'String'));
     temp.pre = handles.pre * 2;
    temp.post = handles.post * 2;
    
    kn = kn + 1;
    % Get some basic information
    [~, scale] = NlxSendCommand(['-GetADBitVolts ' handles.objectToRetrieve]);
    scale = str2double(char(scale));
    [~,  event_ts, ~, ttlValueArray, ~, ~, ~ ] = NlxGetNewEventData('Events');
    [~,dataArray, ~, ~, Fs,...
        ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);
    temp.data_uV{i} = double(dataArray) * scale * 10e6;
    
    if ~isempty(Fs)
        temp.Fs = double(Fs(1));
    end
       % Event detected, collect more data
       f = ttlValueArray == 128;
    if any(f)        
        event_detected = true;
        temp.event_ts = event_ts(f);
        kn = 0;
        tic
    end
    etime = toc;
    if (etime >  temp.post) && event_detected  
        % Don't take the first loop data because it is big and it probably includes what was left in the buffer
        handles.data_uV = [temp.data_uV{2:end}];
        handles = process_data(temp,handles);
        guidata(hObject,handles)
        temp.data_uV = cell(1,1);
        event_detected = false;
    end
    % Circular buffer - most recent data at the end
    if i == buff_size
        i = buff_size;
        temp.data_uV(1) = [];
    else
          i = i + 1;
    end

    pause(loop_period)
    buff_prev = buff_size;
end

function handles = process_data(temp,handles)
% Detetct stimulus artifact location
x = 1:length(handles.data_uV);
t = x*1/temp.Fs;
d = handles.data_uV;
b = median(d);
sd = std(d);
th = b + 5*sd;
[~,ind] = find(d>th,1,'first');
t0 = t(ind);
segSel = t >= (t0-handles.pre) & t <= (t0 + handles.post);
handles.sel_data{end+1} = d(segSel);
handles.sel_time{end+1} = t(segSel);
handles.t0 = t0;




function update_plot(temp,handles)
axes(handles.raw_trace)
Fs = temp.Fs(1);

if handles.overlay_plots
    if handles.multicolor
        hold all;
    else
        hold on
    end
else
    hold off
end
x = 1:length(handles.data_uV);
x = x*1/Fs;
plot(x,handles.data_uV)
xlabel('Time (sec)')
ylabel('uV')
title('Raw data')
set(gca,'TickLength',[0.005 0.005])
% xlim([temp.pre temp.post])
% Get the color of the last drawn line
a = get(gca,'Children');
last = a(1);
col = get(last,'Color');

axes(handles.spectrum)
if logical(get(handles.overlay,'Value'))
    hold on
else
    hold off
end
% pwelch(handles.data_uV,[],[],[],Fs);
% xlim([0 handles.xmax])

if logical(get(handles.multi_color,'Value'))
    a = get(gca,'Children');
    if ~isempty(a)
    last = a(1);
    set(last,'Color',col)
    end
end

hold on
plot([0.06 0.06],ylim,'-','Color',[0.75 0.75 0.75])
if ~handles.overlay_plots
    hold off
end



function postEventTime_Callback(hObject, eventdata, handles)
% hObject    handle to postEventTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of postEventTime as text
%        str2double(get(hObject,'String')) returns contents of postEventTime as a double
handles.xmax = str2double(get(hObject,'String'));
axes(handles.spectrum)
xlim([0 handles.xmax])
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function postEventTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to postEventTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
handles.post_event_time =  str2double(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes on button press in multi_color.
function multi_color_Callback(hObject, eventdata, handles)
% hObject    handle to multi_color (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of multi_color
handles.multicolor = logical(get(hObject,'Value'));
guidata(hObject,handles);


% --- Executes on button press in stop_watching.
function stop_watching_Callback(hObject, eventdata, handles)
% hObject    handle to stop_watching (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop_data = logical(get(hObject,'Value'));
guidata(hObject,handles)
