function varargout = channel_ps(varargin)
% CHANNEL_PS MATLAB code for channel_ps.fig
%      CHANNEL_PS, by itself, creates a new CHANNEL_PS or raises the existing
%      singleton*.
%
%      H = CHANNEL_PS returns the handle to a new CHANNEL_PS or the handle to
%      the existing singleton*.
%
%      CHANNEL_PS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CHANNEL_PS.M with the given input arguments.
%
%      CHANNEL_PS('Property','Value',...) creates a new CHANNEL_PS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before channel_ps_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to channel_ps_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to start (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help channel_ps

% Last Modified by GUIDE v2.5 24-Dec-2015 11:00:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @channel_ps_OpeningFcn, ...
    'gui_OutputFcn',  @channel_ps_OutputFcn, ...
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


% --- Executes just before channel_ps is made visible.
function channel_ps_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to channel_ps (see VARARGIN)

% Choose default command line output for channel_ps
handles.output = hObject;
handles.duration = 5;
handles.overlay_plots = false;
handles.multicolor = false;
handles.xmax = 1; % freq max in KHz for display
set(handles.duration_sec,'String',handles.duration)
set(handles.stop,'UserData',0);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes channel_ps wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = channel_ps_OutputFcn(hObject, eventdata, handles)
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

function  update_plot(handles)


tic
start_time = toc;
end_time = start_time+handles.duration;
i = 0;
temp = struct;

while toc < end_time
    i = i + 1;
    % Get some basic information
    [~, scale] = NlxSendCommand(['-GetVoltageConversion ' handles.objectToRetrieve]);
    scale = str2double(char(scale));
    [~,dataArray, ~, ~, Fs,...
        ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);
    temp.data_uV{i} = double(dataArray) * scale * 10e6;
    if ~isempty(Fs)
        temp.Fs = Fs(1);
    end
    pause(0.05)
end

axes(handles.raw_trace)
Fs = double(temp.Fs(1));
% Don't take the first loop data because it is big and it probably includes what was left in the buffer
handles.data_uV = [temp.data_uV{2:end}];
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

% Get the color of the last drawn line
a = get(gca,'Children');
last = a(1);
col = get(last,'Color');

axes(handles.spectrum)
if handles.overlay_plots
    hold on
else
    hold off
end
pwelch(handles.data_uV,[],[],[],Fs);
xlim([0 handles.xmax])

if handles.multicolor
    a = get(gca,'Children');
    last = a(1);
    set(last,'Color',col)
end

hold on
plot([0.06 0.06],ylim,'-','Color',[0.75 0.75 0.75])
if ~handles.overlay_plots
    hold off
end
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


function duration_sec_Callback(hObject, eventdata, handles)
% hObject    handle to duration_sec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of duration_sec as text
%        str2double(get(hObject,'String')) returns contents of duration_sec as a double
handles.duration = str2double(get(hObject,'String'));
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function duration_sec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to duration_sec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in plot.
function plot_Callback(hObject, eventdata, handles)
% hObject    handle to plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
update_plot(handles)



function freq_max_Callback(hObject, eventdata, handles)
% hObject    handle to freq_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_max as text
%        str2double(get(hObject,'String')) returns contents of freq_max as a double
handles.xmax = str2double(get(hObject,'String'));
axes(handles.spectrum)
xlim([0 handles.xmax])
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function freq_max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in multi_color.
function multi_color_Callback(hObject, eventdata, handles)
% hObject    handle to multi_color (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of multi_color
handles.multicolor = logical(get(hObject,'Value'));
guidata(hObject,handles);
