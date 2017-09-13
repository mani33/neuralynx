function varargout = iocurve_online_mini(varargin)
% IOCURVE_ONLINE_MINI MATLAB code for iocurve_online_mini.fig
%      IOCURVE_ONLINE_MINI, by itself, creates a new IOCURVE_ONLINE_MINI or raises the existing
%      singleton*.
%
%      H = IOCURVE_ONLINE_MINI returns the handle to a new IOCURVE_ONLINE_MINI or the handle to
%      the existing singleton*.
%
%      IOCURVE_ONLINE_MINI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IOCURVE_ONLINE_MINI.M with the given input arguments.
%
%      IOCURVE_ONLINE_MINI('Property','Value',...) creates a new IOCURVE_ONLINE_MINI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before iocurve_online_mini_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to iocurve_online_mini_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to start (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help iocurve_online_mini

% Last Modified by GUIDE v2.5 24-Apr-2017 15:23:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @iocurve_online_mini_OpeningFcn, ...
    'gui_OutputFcn',  @iocurve_online_mini_OutputFcn, ...
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


% --- Executes just before iocurve_online_mini is made visible.
function iocurve_online_mini_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to iocurve_online_mini (see VARARGIN)

% Choose default command line output for iocurve_online_mini
% Mani Subramaniyan
% Dani Lab. University of Pennsylvania

handles.slope_win = 1; %  ms
handles.output = hObject;
handles.pre_event_time = 5; % ms
handles.post_event_time = 50; %ms
set(handles.stop,'UserData',0);
set(handles.postEventTime,'String',handles.post_event_time);
set(handles.preEventTime,'String',handles.pre_event_time);
% Update handles structure
% Default current levels to be tested
dc = {'1','2','3','4','5','7.5','10','12.5','15','17.5','20','22.5','25','27.5','30','32.5','35','37.5','40','42.5','45','47.5','50','55','60','65','70','75','80','85',...
    '90','95','100','105','110','115','120','125','130','135','140','145',...
    '150','155','160','165','170','175','180','185','190','195','200','205','210','215','220','225','250','275','300','350','400'};
set(handles.trial_curr,'String',dc);
set(handles.cut_percent,'String','40')
handles.temp.curr_slope_start_time = [];
handles.rough = struct;
handles.loaded_trial_slope_data = [];
handles.temp.event_detected = false;
handles.exp = struct;
tempSlopeData = [];
expSlopeData = [];
expPopspikeData = [];
tempPopspikeData = [];
set(handles.ttl,'String','128')
% Create a unique folder name each time this gui opens to keep all temp
% data

rfn = sprintf('%d',round(clock));
handles.tempDir = rfn;
mkdir(rfn)
handles.tempSlopeFn = fullfile(rfn,'tempSlopeData');
handles.tempPopspikeFn = fullfile(rfn,'tempPopspikeData');
handles.expSlopeFn = fullfile(rfn,'expSlopeData');
handles.expPopspikeFn = fullfile(rfn,'expPopspikeData');
handles.avgExpSlopeFn = fullfile(rfn,'avgExpSlopeData');
handles.avgExpPopspikeFn = fullfile(rfn,'avgExpPopSpikeData');




save(handles.tempSlopeFn,'tempSlopeData')
save(handles.tempPopspikeFn,'tempPopspikeData')
save(handles.expSlopeFn,'expSlopeData')
save(handles.expPopspikeFn,'expPopspikeData')

set(handles.message_update,'String','')
set(handles.figure1,'Name','TTL_128')
handles.objectToRetrieve = '';
guidata(hObject, handles);

% UIWAIT makes iocurve_online_mini wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = iocurve_online_mini_OutputFcn(hObject, eventdata, handles)
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
suc = open_data_and_event_stream(cheetahObjects);
if suc
    mess = 'Opened data streams successfully';
else
    mess = 'Failed to open some data stream';
end
NlxSetApplicationName('I/O online')
handles.cheetahObjects = cheetahObjects;
handles.cheetahTypes = cheetahTypes;
handles.sel_data = {};
handles.sel_time = {};

[~,dd] =  NlxSendCommand('-GetDataDirectory ');
dd = strtrim(strrep( dd{:},'"',''));
handles.cheetah_data_dir = dd;
set(handles.message_update,'String',mess)
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
ttl = str2double(get(handles.ttl,'String'));
figname = sprintf('%s_TTL_%u',handles.objectToRetrieve,ttl);
set(handles.figure1,'Name',figname)
guidata(hObject,handles)
% % update_plot(handles);
% disp('-----')
% disp(handles.objectToRetrieve)




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
axes(handles.inst_response)
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
handles.temp.event_detected = false;
temp = struct;
i = 1;
kn = 0;
handles.pre = str2double(get(handles.preEventTime,'String'));
handles.post = str2double(get(handles.postEventTime,'String'));

% Set buffer size for around 4 seconds by default
pre = max(handles.pre/1000,2);
post = max(handles.post/1000,2);
temp.data_uV = cell(1,1);
buff_size = round((pre+post)/loop_period);
tic
% Before start, remove data in existing buffer of Cheetah by simply reading
% them off.

[~,  ~, ~, ~, ~, ~, ~ ] = NlxGetNewEventData('Events');
[~,~, ~, ~, ~, ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);

while ~logical(get(handles.stop_watching,'Value'))
    kn = kn + 1;
    % Get some basic information
    [~, scale] = NlxSendCommand(['-GetVoltageConversion ' handles.objectToRetrieve]);
    scale = str2double(char(scale));
    [~,  event_ts, ~, ttlValueArray, ~, ~, ~ ] = NlxGetNewEventData('Events');
    [~,dataArray, tsArray, ~, Fs,...
        ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);
    temp.data_uV{i} = double(dataArray) * scale * 10e6;
    temp.tsArray{i} = tsArray;
    
    if ~isempty(Fs)
        handles.Fs = double(Fs(1));
    end
    % Event detected, collect more data
%     f = ttlValueArray == 128;
    f = ttlValueArray == str2double(get(handles.ttl,'String'));
    if any(f)
        handles.temp.event_detected = true;
        handles.temp.event_ts = double(event_ts(f));
        kn = 0;
        tic
    end
    etime = toc;
    if (etime >  post) && handles.temp.event_detected
        disp(handles.objectToRetrieve)
        % Don't take the first loop data because it is big and it probably includes what was left in the buffer
        handles.data_uV = [temp.data_uV{1:end}]';
        handles.data_t = double([temp.tsArray{1:end}]');
        handles = process_data(handles);
        guidata(hObject,handles)
        temp.data_uV = cell(1,1);
        temp.tsArray = cell(1,1);
        i = 0;
        handles.temp.event_detected = false;
    end
    % Circular buffer - most recent data at the end
    if i == buff_size
        i = buff_size;
        if ~isempty(temp.data_uV)
            temp.data_uV(1) = [];
        end
        if ~isempty(temp.tsArray)
            temp.tsArray(1) = [];
        end
    else
        i = i + 1;
    end
    
    pause(loop_period)
end
guidata(hObject,handles)
set(handles.run,'Value',false)

function handles = process_data(handles)

% Just use the start time of the block to get time zero
T = 1/handles.Fs;
d = handles.data_uV;
t1 = handles.data_t(1);
nsamples = length(d);
tvec = t1 + (0:nsamples)*T*1e6; % micro sec
t = (tvec - handles.temp.event_ts)/1000; % ms

segSel = t >= (-handles.pre) & t <= handles.post;
y = d(segSel);
t = t(segSel);
try
    % subtract the baseline trace just after the stimulus artifact
    sel = (t > 1.5) & (t < 2.5);
    by = mean(y(sel));
    y = y-by;
catch
    y = handles.temp.uV-mean(handles.temp.uV);
end

handles.temp.uV = y;
handles.temp.t = t;
plot_raw_trace(handles)
plot_trace_for_resp_measure(handles)

% save traces if asked for
if logical(get(handles.save_traces,'Value')) && logical(get(handles.run_io_exp,'Value'))
    % Get some metadata
    data = struct;
%     obj = handles.objectToRetrieve;
%     [~,data.high_cut] =  NlxSendCommand('-GetAnalogHighCutFrequency t1c1');
%     [~,data.low_cut] =  NlxSendCommand(['-GetAnalogLowCutFrequency ' obj]);
    data.Fs = handles.Fs;
    dataPath = fullfile(handles.cheetah_data_dir,'EventTrigTraces');
    if ~exist(dataPath,'dir')
        mkdir(dataPath)
    end
    % create file name based on current time
    pfix = get(handles.trace_file_name_prefix,'String');
    ctime = round(clock);
    ch = handles.objectToRetrieve;
    fnc = sprintf('%s%sh%02dm%02ds%02d_TTL_%u.mat',pfix,ch,ctime(4),ctime(5),ctime(6),str2double(get(handles.ttl,'String')));
    fn = fullfile(dataPath,fnc);
    data.t1_ms = handles.temp.t(1); % ms
    data.uV = handles.temp.uV; % 
    data.event_ts_us = handles.temp.event_ts; 
    data.chan = handles.objectToRetrieve;
    data.curr_uA = handles.exp.curr(end);%#ok
    % Save data
    save(fn,'data')
end

function plot_raw_trace(handles)
axes(handles.raw_trace)
if logical(get(handles.overlay,'Value'));
    if logical(get(handles.multi_color,'Value'))
        hold all;
    else
        hold on
    end
else
    hold off
end

t = handles.temp.t;
y = handles.temp.uV;

plot(t,y)

% plot(handles.temp.t,handles.temp.uV-mean(handles.temp.uV))
xlabel('Time (ms)')
ylabel('uV')
title('Raw data')
set(gca,'TickLength',[0.005 0.005])
box off
xlim([-2 50])

function plot_trace_for_resp_measure(handles)
axes(handles.inst_response)
cla
t = handles.temp.t;
y = handles.temp.uV;

yso = mconv(y,getGausswin(0.5,1000*1/handles.Fs));
plot(t,y,'b')
hold all
plot(t,yso,'k')
dy = diff(yso);
mInd = t>0.5 & t < min(100,t(end)); % ms
dy = max(yso(mInd))*dy/max(dy(mInd));
plot(t(2:end),dy,'r')
xlim([-2 45])

function handles = add_slope_datapoint_to_io_plot(handles)
axes(handles.io_curve_slope)
% Get current level used
load(handles.tempSlopeFn)
d = tempSlopeData;
slope = d(end,2);
plot(d(end,1),slope,'kO','markerfacecolor','k')
hold on
grid on
xlabel('Current (\muA)')
% ylabel('Abs fEPSP slope (V/s)')
title('Slope I/O Curve')
box off

function handles = add_popspike_datapoint_to_io_plot(handles)
axes(handles.io_curve_popspike)
% Get current level used
load(handles.tempPopspikeFn)
d = tempPopspikeData;
ps = abs(d(end,2));
plot(d(end,1),ps,'kO','markerfacecolor','k')
hold on
xlabel('Current (\muA)')
% ylabel('Abs Popspike Amp (V)')
title('PopSpike I/O Curve')
box off
grid on
function postEventTime_Callback(hObject, eventdata, handles)
% hObject    handle to postEventTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of postEventTime as text
%        str2double(get(hObject,'String')) returns contents of postEventTime as a double
handles.post_event_time =  str2double(get(hObject,'String'));
guidata(hObject,handles);


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
set(handles.run,'Value',0)
handles.stop_data = logical(get(hObject,'Value'));
guidata(hObject,handles)


% --- Executes on selection change in trial_curr.
function trial_curr_Callback(hObject, eventdata, handles)
% hObject    handle to trial_curr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns trial_curr contents as cell array
%        contents{get(hObject,'Value')} returns selected item from trial_curr


% --- Executes during object creation, after setting all properties.
function trial_curr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trial_curr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cut_percent_Callback(hObject, eventdata, handles)
% hObject    handle to cut_percent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cut_percent as text
%        str2double(get(hObject,'String')) returns contents of cut_percent as a double


% --- Executes during object creation, after setting all properties.
function cut_percent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cut_percent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cut_slope_Callback(hObject, eventdata, handles)
% hObject    handle to cut_slope (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cut_slope as text
%        str2double(get(hObject,'String')) returns contents of cut_slope as a double


% --- Executes during object creation, after setting all properties.
function cut_slope_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cut_slope (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load_curr_levels_file.
function load_curr_levels_file_Callback(hObject, eventdata, handles)
% hObject    handle to load_curr_levels_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fn,pn] = uigetfile('*.mat','Choose a current levels file','E:\CheetahData\');
fin = fullfile(pn,fn);
load(fin)
handles.exp.nBlocks = data.nBlocks;
handles.exp.current_levels = data.current_levels;
b1 = data.current_levels{1};
set(handles.set_current,'String',[num2str(b1(1)) 'uA'])
set(handles.block_num,'String','1')
guidata(hObject,handles)

function send_stim_pulse
[succeeded, ~] = NlxSendCommand('-DigitalIOTtlPulse AcqSystem1_0 0 0 High');
if succeeded == 0
    disp 'FAILED to send TTL pulse to cheetah'
else
    disp 'PASSED TTL pulse'
end
% --- Executes on button press in run_io_exp.
function run_io_exp_Callback(hObject, eventdata, handles)
% hObject    handle to run_io_exp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clear_exp_data;
set(handles.stop_watching,'Value',0)
guidata(hObject,handles);
loop_period = 0.05;
handles.temp.event_detected = false;
temp = struct;
i = 1;
kn = 0;
handles.pre = str2double(get(handles.preEventTime,'String'));
handles.post = str2double(get(handles.postEventTime,'String'));

% Set buffer size for around 4 seconds by default
pre = max(handles.pre/1000,2);
post = max(handles.post/1000,2);

buff_size = round((pre+post)/loop_period);
tic

clevels = [handles.exp.current_levels{:}];
clevels = clevels(:);
nStim = length(clevels);
nLevels = nStim/handles.exp.nBlocks;
cc = 0;

handles.exp.curr = [];
handles.exp.slope = [];
% Before start, remove data in existing buffer of Cheetah by simply reading
% them off.
[~,  ~, ~, ~, ~, ~, ~ ] = NlxGetNewEventData('Events');
[~,~, ~, ~, ~, ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);

while ~logical(get(handles.stop_watching,'Value'))
    kn = kn + 1;
    % Get some basic information
    [~, scale] = NlxSendCommand(['-GetVoltageConversion ' handles.objectToRetrieve]);
    scale = str2double(char(scale));
    [~,  event_ts, ~, ttlValueArray, ~, ~, ~ ] = NlxGetNewEventData('Events');
    [~,dataArray, tsArray, ~, Fs,...
        ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);
    temp.data_uV{i} = double(dataArray) * scale * 10e6;
    temp.tsArray{i} = tsArray;
    
    if ~isempty(Fs)
        handles.Fs = double(Fs(1));
    end
    % Event detected, collect more data
%     f = ttlValueArray == 128;
    f = ttlValueArray == str2double(get(handles.ttl,'String'));
    if any(f)
        handles.temp.event_detected = true;
        handles.temp.event_ts = double(event_ts(f));
        kn = 0;
        tic
    end
    etime = toc;
    if (etime >  post) && handles.temp.event_detected
        % Don't take the first loop data because it is big and it probably includes what was left in the buffer
        handles.data_uV = [temp.data_uV{2:end}]';
        handles.data_t = double([temp.tsArray{2:end}]');
        cc = cc + 1;
        
        handles.exp.curr(cc) = clevels(cc);
        handles.exp.cc = cc;
        handles.temp.event_detected = false;
        
        handles = process_data(handles);
        if logical(get(handles.slope_measure,'Value'))
            handles = compute_exp_slope(handles);
            handles = plot_exp_slope(handles);
        end
        if logical(get(handles.popspike_measure,'Value'))
            handles = compute_exp_popspike(handles);
            handles = plot_exp_popspike(handles);
        end
        
        
        temp.data_uV = cell(1,1);
        temp.tsArray = cell(1,1);
        i = 0;
        % Update the current level to be set
        
%         bn = ceil(cc/nLevels);
        
        if cc==nStim
%             if (bn ~= bn_prev) || cc==nStim
%             bn_prev = bn;
            update_cut_curr(handles,cc==nStim);
        end
        
        guidata(hObject,handles)
        % Next block?
        if cc < nStim
            nbn = ceil((cc+1)/nLevels);
            set(handles.set_current,'String',[num2str(clevels(cc+1)) 'uA'])
            set(handles.block_num,'String',num2str(nbn))
        end
        if cc==nStim
            % Turn off loop; Experiment is done.
            set(handles.stop_watching,'Value',1)
        end
    end
    
    % Circular buffer - most recent data at the end
    if i == buff_size
        i = buff_size;
        if ~isempty(temp.data_uV)
            temp.data_uV(1) = [];
        end
        if ~isempty(temp.tsArray)
            temp.tsArray(1) = [];
        end
    else
        i = i + 1;
    end
    
    pause(loop_period)
    
end

guidata(hObject,handles)
set(handles.run_io_exp,'Value',false)

function handles = plot_exp_slope(handles)
axes(handles.io_curve_slope)
plot(handles.exp.curr(end),handles.exp.slope(end),'kO','markerfacecolor','k')
xlabel('Current (\muA)')
ylabel('Abs fEPSP slope (V/s)')
title('I/O Curve')
hold on
grid on

function handles = plot_exp_popspike(handles)
axes(handles.io_curve_popspike)
plot(handles.exp.curr(end),abs(handles.exp.popspike_height(end)),'kO','markerfacecolor','k')
xlabel('Current (\muA)')
ylabel('Abs fEPSP slope (V/s)')
title('I/O Curve')
hold on
grid on
% --- Executes on button press in load_resp_params.
function load_resp_params_Callback(hObject, eventdata, handles)
% hObject    handle to load_resp_params (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fn,pn] = uigetfile('*.mat','Choose a example slope file','E:\CheetahData\');
fin = fullfile(pn,fn);
x = load(fin);
fn = fields(x);
y = x.(fn{1});
if logical(get(handles.slope_measure,'Value'))
    handles.exp.slope_learning_data = y.avg_slope_data;
end
if logical(get(handles.popspike_measure,'Value'))
    handles.exp.popspike_learning_data = y.avg_popspike_data;
end
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function set_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in clear_resp_data.
function clear_resp_data_Callback(hObject, eventdata, handles)
% hObject    handle to clear_resp_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clearTempRespData;

% --- Executes on button press in quantify_resp.
function quantify_resp_Callback(hObject, eventdata, handles)
% hObject    handle to quantify_resp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.inst_response)

if logical(get(handles.slope_measure,'Value'))
    compute_example_slope(hObject,eventdata,handles);
end
if logical(get(handles.popspike_measure,'Value'))
    compute_example_popspike(hObject,eventdata,handles);
end

function compute_example_popspike(hObject,eventsdata,handles)
y = handles.temp.uV;
tmsec = handles.temp.t;
axes(handles.inst_response)
title('PopSpike Measurement')
s = get(get(handles.popspike_measure_type,'SelectedObject'),'String');
np = str2double(s(1));
if np == 2
    [bounds,~] = ginput(2);
    [h,tt,yy,ypi] = get_popspike_height(tmsec,y,bounds,'auto',true);
elseif np==4
    [bounds,~] = ginput(4);
    [h,tt,yy,ypi] = get_popspike_height(tmsec,y,bounds,'auto',false);
end
axes(handles.inst_response)
hold on
plot(tt(2),ypi,'m*')
plot(tt(2),yy(2),'m*')
plot(tt([1 3]),yy([1 3]),'k-')
plot([tt(2) tt(2)],[yy(2) ypi],'m','linewidth',2)
xlim([-2 45])
xlabel('Time(ms)')
title('Instantaneous Resp Measurement')
set(gca,'YTickLabel','')
box off
hold off
% Save data
contents = cellstr(get(handles.trial_curr,'String'));
curr =  str2double(contents{get(handles.trial_curr,'Value')});
load(handles.tempPopspikeFn)
tempPopspikeData(end+1,:) = [curr h bounds(:)'];
save(handles.tempPopspikeFn,'tempPopspikeData')
handles = add_popspike_datapoint_to_io_plot(handles);
guidata(hObject,handles)

function compute_example_slope(hObject, eventdata, handles)
axes(handles.inst_response)
y = handles.temp.uV;
t = handles.temp.t;
% yso = mconv(y,getGausswin(0.5,1000*1/handles.Fs));
title('Slope Measurement')

% Ginput the start and end points to calculate slope
[tb,~] = ginput(1);
% Extract the trace between the bounds and fit a linear model
% and get the slope
sind = t>=tb & t<=(tb+handles.slope_win);
ts = t(sind);
ys = y(sind);
ts = ts(:);
X = [ts*1e-3,ones(size(ts))];
B = regress(ys,X);
yi = X*B;
plot(X(:,1)*1e3,yi,'m.')
hold off
xlabel('Time(ms)')
ylabel('Slope (V/s)')

box off
xlim([-2 45])

% Flip sign of slope if asked for
if logical(get(handles.flip_slope_sign,'Value'))
    slope = -B(1);
else
    slope = B(1);
end

% Save data
contents = cellstr(get(handles.trial_curr,'String'));
curr =  str2double(contents{get(handles.trial_curr,'Value')});
handles.temp.curr_slope_start_time = [curr slope tb];
load(handles.tempSlopeFn)

tempSlopeData(end+1,:) = [curr slope tb];
save(handles.tempSlopeFn,'tempSlopeData')
handles = add_slope_datapoint_to_io_plot(handles);
guidata(hObject,handles)
title('Instantaneous Resp Measurement')

function handles = compute_exp_slope(handles)
% hObject    handle to quantify_resp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.inst_response)
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'b')
hold all

sd = handles.exp.slope_learning_data; % curr slope timeStart
curr = handles.exp.curr(end);
% Get latency of slope computation onset based on the closest learned
% example
ex_curr = sd(:,1);
[~,ex_ind] = min(abs(ex_curr-curr));
lat = sd(ex_ind,3);
sind = find(t > lat & t < (lat + handles.slope_win));
ts = t(sind);
ys = y(sind);
ts = ts(:);
X = [ts*1e-3,ones(size(ts))];
B = regress(ys,X);
yi = X*B;
plot(X(:,1)*1e3,yi,'m.')
box off
hold off
set(gca,'YTicklabel','')
title('Resp Measurement')

% Flip sign of slope if asked for
if logical(get(handles.flip_slope_sign,'Value'))
    slope = -B(1);
else
    slope = B(1);
end

handles.exp.slope(handles.exp.cc) = slope;
% Save data
load(handles.expSlopeFn)


expSlopeData(end+1,:) = [curr slope];
save(handles.expSlopeFn,'expSlopeData')
avgExpSlopeData = compute_averaged_resp_measure_data(expSlopeData);
save(handles.avgExpSlopeFn,'avgExpSlopeData')

function handles = compute_exp_popspike(handles)
% hObject    handle to quantify_resp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.inst_response)
y = handles.temp.uV;
t = handles.temp.t;
tmsec = t;
plot(t,y,'b')
hold all


sd = handles.exp.popspike_learning_data; % curr slope timeStart
curr = handles.exp.curr(end);
% Get latency of slope computation onset based on the closest learned
% example
ex_curr = sd(:,1);
[~,ex_ind] = min(abs(ex_curr-curr));
bounds = sd(ex_ind,3:end);
np = length(bounds);

if np == 2
%     [bounds,~] = ginput(2);
    [h,tt,yy,ypi] = get_popspike_height(tmsec,y,bounds,'auto',true);
elseif np==4
%     [bounds,~] = ginput(4);
    [h,tt,yy,ypi] = get_popspike_height(tmsec,y,bounds,'auto',false);
end
handles.exp.popspike_height(handles.exp.cc) = h;

axes(handles.inst_response)
hold on
plot(tt(2),ypi,'m*')
plot(tt(2),yy(2),'m*')
plot(tt([1 3]),yy([1 3]),'k*-')
plot([tt(2) tt(2)],[yy(2) ypi],'b')
xlim([-2 50])
set(gca,'YTicklabel','')
xlabel('Time(ms)')
title('Resp Measurement')
box off
hold off

% Save data
load(handles.expPopspikeFn)
expPopspikeData(end+1,:) = [curr h];
save(handles.expPopspikeFn,'expPopspikeData')
avgExpPopspikeData = compute_averaged_resp_measure_data(expPopspikeData);
save(handles.avgExpPopspikeFn,'avgExpPopspikeData')

% --- Executes on button press in save_resp_params.
function save_resp_params_Callback(hObject, eventdata, handles)
% hObject    handle to save_resp_params (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if logical(get(handles.slope_measure,'Value'))
    load(handles.tempSlopeFn)
    example.slope_data = tempSlopeData;
    example.avg_slope_data = compute_averaged_resp_measure_data(tempSlopeData);
end
if logical(get(handles.popspike_measure,'Value'))
    load(handles.tempPopspikeFn)
    example.popspike_data = tempPopspikeData;
    example.avg_popspike_data = compute_averaged_resp_measure_data(tempPopspikeData);
end
dn = uigetdir('E:\CheetahData\','Choose a directory to save slope data');
str1 = [];
str2 = [];
if logical(get(handles.slope_measure,'Value'))
    str1 = 'Slope';
end
if logical(get(handles.popspike_measure,'Value'))
    str2 = 'Popspike';
end
rstr = ['_' str1 str2];
filename = ['example_data_' handles.objectToRetrieve rstr '_TTL_' num2str(str2double(get(handles.ttl,'String')))];
fin = fullfile(dn,filename);
save(fin,'example')

% --- Executes on button press in compute_cut_curr.
function compute_cut_curr_Callback(hObject, eventdata, handles)
% hObject    handle to compute_cut_curr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if exist(handles.expSlopeFn,'file')
    load(handles.expSlopeFn)
    if ~isempty(expSlopeData)
    plot(expSlopeData(:,1),expSlopeData(:,2),'kO','markerfacecolor','k')
    hold on
    end
end

if exist(handles.expPopspikeFn,'file') 
    load(handles.expPopspikeFn)
    if ~isempty(expPopspikeData)
    plot(expPopspikeData(:,1),abs(expPopspikeData(:,2)),'kO','markerfacecolor','k')
    hold on
    end
end

update_cut_curr(handles,1);

function data = compute_averaged_resp_measure_data(data)
d = data;
if isempty(d)
    return
end
curr = d(:,1);
ncol = size(d,2);
uc = unique(curr);
nu = length(uc);
data = nan(nu,ncol);
for i = 1:nu
    c = uc(i);
    data(i,1) = c;
    for nc = 2:ncol
        data(i,nc) = nanmean(d(curr==c,nc));
    end
end


function update_cut_curr(handles,lastStim)

cv = str2double(get(handles.cut_percent,'String'));

if logical(get(handles.slope_measure,'Value')) && exist('avgExpSlopeData.mat','file')
    load(handles.avgExpSlopeFn)
    sl = avgExpSlopeData(:,2);
%     mi = min(sl);
    ma = max(sl);
%     bi = mi+(ma-mi)*cv/100;
    bi = ma*cv/100;
    ci = round(interp1(sl,avgExpSlopeData(:,1),bi,'linear'));
    set(handles.cut_curr,'String',[num2str(ci) 'uA'])
    axes(handles.io_curve_slope)
    hold on
    grid on
    if lastStim
        plot([ci ci],ylim,'b--')
        % Plot averaged data
        plot(avgExpSlopeData(:,1),avgExpSlopeData(:,2),'k--')
        hold on
    end
end

if logical(get(handles.popspike_measure,'Value')) && exist('avgExpPopspikeData.mat','file')
    
    cv = str2double(get(handles.cut_percent,'String'));
    load(handles.avgExpPopspikeFn)
    sl = abs(avgExpPopspikeData(:,2));
%     mi = min(sl);
    ma = max(sl);
%     bi = mi+(ma-mi)*cv/100;
    bi = ma*cv/100;
    ci = round(interp1(sl,avgExpPopspikeData(:,1),bi,'linear'));
    set(handles.cut_curr,'String',[num2str(ci) 'uA'])
    axes(handles.io_curve_popspike)
    hold on
    if lastStim
        plot([ci ci],ylim,'b--')
        % Plot averaged data
        plot(avgExpPopspikeData(:,1),abs(avgExpPopspikeData(:,2)),'k--')
        hold on
    end
    grid on
end

% --- Executes on button press in clear_io_curve.
function clear_io_curve_Callback(hObject, eventdata, handles)
% hObject    handle to clear_io_curve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if logical(get(handles.slope_measure,'Value'))
    axes(handles.io_curve_slope)
    cla
end
if logical(get(handles.popspike_measure,'Value'))
    axes(handles.io_curve_popspike)
    cla
end
% --- Executes on button press in stimulate.
function stimulate_Callback(hObject, eventdata, handles)
% hObject    handle to stimulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
send_stim_pulse;


% --- Executes on button press in clear_last_datapoint.
function clear_last_datapoint_Callback(hObject, eventdata, handles)
% hObject    handle to clear_last_datapoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if logical(get(handles.slope_measure,'Value'))
    load(handles.tempSlopeFn)
    tempSlopeData(end,:) = [];
    save(handles.tempSlopeFn,'tempSlopeData')
end
if logical(get(handles.popspike_measure,'Value'))
    load(handles.tempPopspikeFn)
    tempPopspikeData(end,:) = [];
    save(handles.tempPopspikeFn,'tempPopspikeData')
end



% --- Executes on button press in update_example_io.
function update_example_io_Callback(hObject, eventdata, handles)
% hObject    handle to update_example_io (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load(handles.tempSlopeFn)
axes(handles.io_curve_slope)
plot(tempSlopeData(:,1),tempSlopeData(:,2),'ko','markerfacecolor','k')
hold on
grid on

% --- Executes on button press in clear_exp_data.
function clear_exp_data_Callback(hObject, eventdata, handles)
% hObject    handle to clear_exp_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Clear previous run data
clear_exp_data;

function clear_exp_data
if exist(handles.expSlopeFn,'file')
    load(handles.expSlopeFn)
    expSlopeData = [];
    save(handles.expSlopeFn,'expSlopeData')
end

if exist(handles.avgExpSlopeFn,'file')
    load(handles.avgExpSlopeFn)
    avgExpSlopeData = [];
    save(handles.avgExpSlopeFn,'avgExpSlopeData')
end


% --- Executes on button press in plot_temp_avg_slope.
function plot_temp_avg_slope_Callback(hObject, eventdata, handles)
% hObject    handle to plot_temp_avg_slope (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if logical(get(handles.slope_measure,'Value'))
    load(handles.tempSlopeFn)
    td = compute_averaged_resp_measure_data(tempSlopeData);
    axes(handles.io_curve_slope)
    hold on
    plot(td(:,1),td(:,2),'k')
    xlabel('Current (\muA)')
    ylabel('Abs fEPSP slope (V/s)')
    title('Slope I/O Curve')
    grid on
end
if logical(get(handles.popspike_measure,'Value'))
    load(handles.tempPopspikeFn)
    td = compute_averaged_resp_measure_data(tempPopspikeData);
    axes(handles.io_curve_popspike)
    grid on
    hold on
    plot(td(:,1),abs(td(:,2)),'k')
    xlabel('Current (\muA)')
    ylabel('Abs Popspike (V)')
    title('Popspike I/O Curve')
end
% --- Executes on button press in save_exp_data.
function save_exp_data_Callback(hObject, eventdata, handles)
% hObject    handle to save_exp_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Save data
load(handles.expSlopeFn)
exp.slope_data = expSlopeData;
exp.avg_slope_data = compute_averaged_resp_measure_data(expSlopeData);

load(handles.expPopspikeFn)
exp.popspike_data = expPopspikeData;
exp.avg_popspike_data = compute_averaged_resp_measure_data(expPopspikeData);

dn = uigetdir('E:\CheetahData\','Choose a directory to save exp slope data');
fin = fullfile(dn,'exp_slope_data');
save(fin,'exp')

function clearTempRespData
if exist(handles.tempSlopeFn,'file')
    load(handles.tempSlopeFn)
    tempSlopeData = [];
    save(handles.tempSlopeFn,'tempSlopeData')
end
if exist(handles.tempPopspikeFn,'file')
    load(handles.tempPopspikeFn)
    tempPopspikeData = [];
    save(handles.tempPopspikeFn,'tempPopspikeData')
end

% --- Executes on button press in update_exp_io_plot.
function update_exp_io_plot_Callback(hObject, eventdata, handles)
% hObject    handle to update_exp_io_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.io_curve_slope)
load(handles.expSlopeFn)
da = expSlopData;
hold on
plot(da(:,1),da(:,2),'k','markerfacecolor','k')
grid on

% --- Executes on button press in plot_exp_slope_avg.
function plot_exp_slope_avg_Callback(hObject, eventdata, handles)
% hObject    handle to plot_exp_slope_avg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load(handles.avgExpSlopeFn)
axes(handles.io_curve_slope)
hold on
grid on
plot(avgExpSlopeData(:,1),avgExpSlopeData(:,2),'k')


% --- Executes on button press in load_old_io_data.
function load_old_io_data_Callback(hObject, eventdata, handles)
% hObject    handle to load_old_io_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if logical(get(handles.slope_measure,'Value'))
    [fn,pn] = uigetfile('*.mat','Choose a example slope file','E:\CheetahData\');
    fin = fullfile(pn,fn);
    x = load(fin);
    fn = fields(x);
    y = x.(fn{1});
    handles.old.avg_slope_data = y.avg_slope_data;
    handles.old.slope_data = y.slope_data;
    guidata(hObject,handles)
end

if logical(get(handles.popspike_measure,'Value'))
    [fn,pn] = uigetfile('*.mat','Choose a example popspike file','E:\CheetahData\');
    fin = fullfile(pn,fn);
    x = load(fin);
    fn = fields(x);
    y = x.(fn{1});
    handles.old.avg_popspike_data = abs(y.avg_popspike_data);
    handles.old.popspike_data = abs(y.popspike_data);
    guidata(hObject,handles)
end





% --- Executes on button press in plot_old_io_data.
function plot_old_io_data_Callback(hObject, eventdata, handles)
% hObject    handle to plot_old_io_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if logical(get(handles.slope_measure,'Value'))
    axes(handles.io_curve_slope)
    col = [0 0 0];
    x = handles.old.avg_slope_data;
    hold on;
    plot(x(:,1),x(:,2),'O-','color',col)
    y = handles.old.slope_data;
    plot(y(:,1),y(:,2),'O','color',col)
    grid on
end

if logical(get(handles.popspike_measure,'Value'))
    axes(handles.io_curve_popspike)
    col = rand(1,3);
    x = handles.old.avg_popspike_data;
    y = handles.old.popspike_data;
    hold on;
    plot(x(:,1),x(:,2),'O-','color',col)
    plot(y(:,1),y(:,2),'O','color',col)
    grid on
end


% --- Executes on button press in simulate_resp.
function simulate_resp_Callback(hObject, eventdata, handles)
% hObject    handle to simulate_resp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('z:\users\mani\matlab\d')


% --- Executes on slider movement.
function zoom_y_Callback(hObject, eventdata, handles)
% hObject    handle to zoom_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider



% --- Executes during object creation, after setting all properties.
function zoom_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoom_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in fit_sigmoid_slope.
function fit_sigmoid_slope_Callback(hObject, eventdata, handles)
% hObject    handle to fit_sigmoid_slope (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[fn,pn] = uigetfile('*.mat','Choose a example slope file','E:\CheetahData\');
fin = fullfile(pn,fn);
d = load(fin);
fn = fieldnames(d);
fn = fn{:};
x = d.(fn).slope_data(:,1);
y = d.(fn).slope_data(:,2);
% Remove points from fitting
ec = get(handles.excluded_current,'String');
if ~isempty(ec)
    ec = str2double(ec);
    eind = x~=ec;
    x = x(eind);
    y = y(eind);
end
% sort data
[~,ind] = unique(x);

nf = 1/max(y);
y = y*nf;
b0 = [max(y) median(diff(y)) min(x)+((max(x)-min(x))/2)];

modelfun = @(b,x) b(1)./(1+exp(-b(2)*(x-b(3))));
b = nlinfit(x,y,modelfun,b0);
yhat = modelfun(b,x)/nf;
axes(handles.io_curve_slope)
hold all
plot(x(ind),yhat(ind),'rO-')
grid on

% Compute 40% cut current
cv = str2double(get(handles.cut_percent,'String'));
ma = max(yhat);
bi = ma*cv/100;
ci = round(interp1(yhat(ind),x(ind),bi,'linear'));
set(handles.cut_curr,'String',[num2str(ci) 'uA'])
plot([ci ci],ylim,'r')


% --- Executes on button press in fig_sigmoid_popspike.
function fig_sigmoid_popspike_Callback(hObject, eventdata, handles)
% hObject    handle to fig_sigmoid_popspike (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fn,pn] = uigetfile('*.mat','Choose a example slope file','E:\CheetahData\');
fin = fullfile(pn,fn);
d = load(fin);
fn = fieldnames(d);
fn = fn{:};
x = d.(fn).popspike_data(:,1);
% sort data
[~,ind] = unique(x);
y = abs(d.(fn).popspike_data(:,2));
nf = 1/max(y);
y = y*nf;
b0 = [max(y) nanmedian(diff(y)) x(1)+((x(end)-x(1))/2)];

modelfun = @(b,x) b(1)./(1+exp(-b(2)*(x-b(3))));
b = nlinfit(x,y,modelfun,b0);
yhat = modelfun(b,x)/nf;
axes(handles.io_curve_popspike)
hold all
plot(x(ind),yhat(ind),'rO-')
grid on
% Compute 40% cut current
cv = str2double(get(handles.cut_percent,'String'));
ma = max(yhat);
bi = ma*cv/100;
ci = round(interp1(yhat(ind),x(ind),bi,'linear'));
set(handles.cut_curr,'String',[num2str(ci) 'uA'])
plot([ci ci],ylim,'r')



function excluded_current_Callback(hObject, eventdata, handles)
% hObject    handle to excluded_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of excluded_current as text
%        str2double(get(hObject,'String')) returns contents of excluded_current as a double


% --- Executes during object creation, after setting all properties.
function excluded_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to excluded_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_traces.
function save_traces_Callback(hObject, eventdata, handles)
% hObject    handle to save_traces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of save_traces


% --- Executes on button press in flip_slope_sign.
function flip_slope_sign_Callback(hObject, eventdata, handles)
% hObject    handle to flip_slope_sign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of flip_slope_sign



function ttl_Callback(hObject, eventdata, handles)
% hObject    handle to ttl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ttl as text
%        str2double(get(hObject,'String')) returns contents of ttl as a double
ttl = str2double(get(hObject,'String'));
figname = sprintf('%s_TTL_%u',handles.objectToRetrieve,ttl);
set(handles.figure1,'Name',figname)

% --- Executes during object creation, after setting all properties.
function ttl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ttl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in slope_measure.
function slope_measure_Callback(hObject, eventdata, handles)
% hObject    handle to slope_measure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of slope_measure





function num_of_blocks_Callback(hObject, eventdata, handles)
% hObject    handle to num_of_blocks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of num_of_blocks as text
%        str2double(get(hObject,'String')) returns contents of num_of_blocks as a double


% --- Executes during object creation, after setting all properties.
function num_of_blocks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_of_blocks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_load_curr_levels.
function save_load_curr_levels_Callback(hObject, eventdata, handles)
% hObject    handle to save_load_curr_levels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
levels = str2num(get(handles.curr_levels,'String'));
assert(~isempty(levels),'Enter some current levels in the box')
nRep = str2double(get(handles.num_of_blocks,'String'));
assert(~isnan(nRep),'Enter number of blocks')
data = create_random_stim(levels,nRep);

% Save data
dn = uigetdir('E:\CheetahData\','Choose a directory to save current levels data');
if ischar(dn)
file = 'current_levels_io_uamps.mat';
fin = fullfile(dn,file);
save(fin,'data')
else
    warning('Current levels file not saved')
end
handles.exp.nBlocks = data.nBlocks;
handles.exp.current_levels = data.current_levels;
b1 = data.current_levels{1};
set(handles.set_current,'String',[num2str(b1(1)) 'uA'])
set(handles.block_num,'String','1')
guidata(hObject,handles)



function data = create_random_stim(levels,nRep)
%% Generate random current level for input-output curve
% levels = [25:25:100 150 250 350];
n = length(levels);
data.current_levels = cell(1,nRep);
for i = 1:nRep
    rn = randperm(n);
    data.current_levels{i} = levels(rn)';
    fprintf('set %d: %s\n',i,mat2str(data.current_levels{i}'))
end
% Save current levels to the directory of the latest data collection
% session
data.nBlocks = nRep;



function curr_levels_Callback(hObject, eventdata, handles)
% hObject    handle to curr_levels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of curr_levels as text
%        str2double(get(hObject,'String')) returns contents of curr_levels as a double


% --- Executes during object creation, after setting all properties.
function curr_levels_CreateFcn(hObject, eventdata, handles)
% hObject    handle to curr_levels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



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



function trace_file_name_prefix_Callback(hObject, eventdata, handles)
% hObject    handle to trace_file_name_prefix (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trace_file_name_prefix as text
%        str2double(get(hObject,'String')) returns contents of trace_file_name_prefix as a double


% --- Executes during object creation, after setting all properties.
function trace_file_name_prefix_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trace_file_name_prefix (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
