function varargout = stim_view(varargin)
% STIM_VIEW MATLAB code for stim_view.fig
%      STIM_VIEW, by itself, creates a new STIM_VIEW or raises the existing
%      singleton*.
%
%      H = STIM_VIEW returns the handle to a new STIM_VIEW or the handle to
%      the existing singleton*.
%
%      STIM_VIEW('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STIM_VIEW.M with the given input arguments.
%
%      STIM_VIEW('Property','Value',...) creates a new STIM_VIEW or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before stim_view_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to stim_view_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to start (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help stim_view

% Last Modified by GUIDE v2.5 25-Feb-2015 17:06:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @stim_view_OpeningFcn, ...
                   'gui_OutputFcn',  @stim_view_OutputFcn, ...
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


% --- Executes just before stim_view is made visible.
function stim_view_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to stim_view (see VARARGIN)

% Choose default command line output for stim_view
handles.output = hObject;
handles.t_begin = -100;
handles.t_end = 100;
set(handles.stop,'UserData',0);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes stim_view wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = stim_view_OutputFcn(hObject, eventdata, handles) 
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


% % Start listening for stimulation events and plotting data

axes(handles.trace1)
while get(handles.stop,'UserData')~=1
    contents = cellstr(get(handles.channel_list,'String'));
    objectToRetrieve =  contents{get(hObject,'Value')};
    
    % Get some basic information
    [success, scale] = NlxSendCommand(['-GetADBitVolts ' objectToRetrieve]);
 
    scale = str2double(char(scale));
    [succeeded_event, ~, ~, TTL_value, ~, ~, ~ ] = NlxGetNewEventData('Events');
    if succeeded_event && any(TTL_value==128)
        [succeeded_data,dataArray, timeStampArray, channelNumberArray, samplingFreqArray,...
        numValidSamplesArray, numRecordsReturned, numRecordsDropped ] = NlxGetNewCSCData(objectToRetrieve);
        
        data_uV = double(dataArray) * scale * 10e6;
        
        plot(data_uV)
 
        if get(handles.overlay,'Value')==1
            hold on
            
        end
    end
   
   
       pause(0.01)
end

% tic
% goon = true;
% while goon
%     v = get(handles.stop,'UserData');
%     disp(v)
%     pause(1)
%     t = toc;
%     if t > 30
%         goon = false;
%     end
% end


% --- Executes on selection change in channel_list.
function channel_list_Callback(hObject, eventdata, handles)
% hObject    handle to channel_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns channel_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from channel_list


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


% --- Executes on button press in stimulate.
function stimulate_Callback(hObject, eventdata, handles)
% hObject    handle to stimulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[succeeded, ~] = NlxSendCommand('-DigitalIOTtlPulse AcqSystem1_0 0 0 High');
if succeeded == 0
    disp 'FAILED to send TTL pulse to cheetah'
else
    disp 'PASSED TTL pulse'
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


% --- Executes on slider movement.
function begin_time_Callback(hObject, eventdata, handles)
% hObject    handle to begin_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

val = get(hObject,'Value');
set(handles.begin_time_val,'String',num2str(round(val)));
handles.t_begin = val;
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function begin_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to begin_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function end_time_Callback(hObject, eventdata, handles)
% hObject    handle to end_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

val = get(hObject,'Value');
set(handles.end_time_val,'String',num2str(round(val)));
handles.t_end = val;
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function end_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to end_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in overlay.
function overlay_Callback(hObject, eventdata, handles)
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla

function begin_time_val_Callback(hObject, eventdata, handles)
% hObject    handle to begin_time_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of begin_time_val as text
%        str2double(get(hObject,'String')) returns contents of begin_time_val as a double
handles.t_begin = str2double(get(hObject,'String'));
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function begin_time_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to begin_time_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function end_time_val_Callback(hObject, eventdata, handles)
% hObject    handle to end_time_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of end_time_val as text
%        str2double(get(hObject,'String')) returns contents of end_time_val as a double
handles.t_end = str2double(get(hObject,'String'));
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function end_time_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to end_time_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
