function varargout = ltp_online_mini(varargin)
% LTP_ONLINE_MINI MATLAB code for ltp_online_mini.fig
%      LTP_ONLINE_MINI, by itself, creates a new LTP_ONLINE_MINI or raises the existing
%      singleton*.
%
%      H = LTP_ONLINE_MINI returns the handle to a new LTP_ONLINE_MINI or the handle to
%      the existing singleton*.
%
%      LTP_ONLINE_MINI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LTP_ONLINE_MINI.M with the given input arguments.
%
%      LTP_ONLINE_MINI('Property','Value',...) creates a new LTP_ONLINE_MINI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ltp_online_mini_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ltp_online_mini_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to start (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ltp_online_mini

% Last Modified by GUIDE v2.5 20-Apr-2017 15:53:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ltp_online_mini_OpeningFcn, ...
    'gui_OutputFcn',  @ltp_online_mini_OutputFcn, ...
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


% --- Executes just before ltp_online_mini is made visible.
function ltp_online_mini_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ltp_online_mini (see VARARGIN)

% Choose default command line output for ltp_online_mini
handles.slope_win = 1; %  ms
handles.output = hObject;
handles.pre_event_time = 5; % ms
handles.post_event_time = 50; %ms
set(handles.stop,'UserData',0);
set(handles.postEventTime,'String',handles.post_event_time);
set(handles.preEventTime,'String',handles.pre_event_time);
% Update handles structure
% Default current levels to be tested
dc = {'1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16','17','18','19','20','21','22','23',...
    '24','25','26','27','28','29','30','31','32','33','34','35','36','37','38','39','40','41','42','43','44','45',...
    '46','47','48','49','50','51','52','53','54','55','56','57','58','59','60','61','62','63','64','65','66','67',...
    '68','69','70','71','72','73','74','75','76','77','78','79','80','81','82','83','84','85','86','87','88','89',...
    '90','91','92','93','94','95','96','97','98','99','100','105','110','115','120','125','130','135','140','145',...
    '150','155','160','165','170','175','180','185','190','195','200','205','210','215','220','225','250','275','300','350','400'};
set(handles.trial_curr,'String',dc);
handles.linehandles = [];
handles.temp.curr_slope_start_time = [];
handles.loaded_trial_slope_data = [];
handles.temp.event_detected = false;
handles.new.popspike_learning_data = [];
handles.new.slope_learning_data = [];
handles.exp = struct;
handles.time = 0;
handles.numEvents = 0;
linehandles = [];
save('linehandles','linehandles')

rfn = sprintf('%d',round(clock));
mkdir(rfn)

tempSlopeData = [];
tempPopspikeData = [];
ltpSlopeData = [];
ltpPopspikeData = [];
handles.old_slope_data = [];
handles.old_popspike_data = [];

handles.tempSlopeFn = fullfile(rfn,'tempSlopeData');
handles.tempPopspikeFn = fullfile(rfn,'tempPopspikeData');
handles.ltpSlopeFn = fullfile(rfn,'ltpSlopeData');
handles.ltpPopspikeFn = fullfile(rfn,'ltpPopspikeData');
handles.avgLtpSlopeFn = fullfile(rfn,'avgLtpSlopeData');
handles.avgLtpPopspikeFn = fullfile(rfn,'avgLtpPopspikeData');

save(handles.tempSlopeFn,'tempSlopeData')
save(handles.tempPopspikeFn,'tempPopspikeData')
save(handles.ltpSlopeFn,'ltpSlopeData')
save(handles.ltpPopspikeFn,'ltpPopspikeData')

set(handles.slope_latency_offset,'String','0')
set(handles.bin_size,'String',5); % 5 min bin size
set(handles.time_offset,'String','0')
set(handles.figure1,'Name','TTL_128')
handles.objectToRetrieve = '';
guidata(hObject, handles);

% UIWAIT makes ltp_online_mini wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ltp_online_mini_OutputFcn(hObject, eventdata, handles)
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
% open_data_and_event_stream(cheetahObjects);
suc = open_data_and_event_stream(cheetahObjects);
handles.cheetahObjects = cheetahObjects;
handles.cheetahTypes = cheetahTypes;
handles.sel_data = {};
handles.sel_time = {};
[~,dd] =  NlxSendCommand('-GetDataDirectory ');
dd = strtrim(strrep( dd{:},'"',''));
handles.cheetah_data_dir = dd;
guidata(hObject,handles)

NlxSetApplicationName('LTP online')

if suc
    mess = 'Opened data streams successfully';
else
    mess = 'Failed to open some data stream';
end
set(handles.message_update,'String',mess)


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

suc = all(succeeded);
if suc
    mess = 'PASSED close stream for all current objects';
else 
    mess = 'FAILED to close stream';
end
set(handles.message_update,'String',mess)

% --- Executes on button press in overlay.
function overlay_Callback(hObject, eventdata, handles)
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.overlay_plots = logical(get(hObject,'Value'));
guidata(hObject,handles)

% --- Executes on button press in clear_raw_traces.
function clear_raw_traces_Callback(hObject, eventdata, handles)
% hObject    handle to clear_raw_traces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.inst_slope)
cla
axes(handles.raw_trace)
cla
axes(handles.inst_popspike)
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



function handles = process_data(handles)

% Just use the start time of the block to get time zero
T = 1/handles.Fs;
d = handles.data_uV;
t1 = handles.data_t(1);
nsamples = length(d);
tvec = t1 + (0:nsamples)*T*1e6; % micro sec
t = (tvec - handles.temp.event_ts)/1000; % ms

segSel = t >= (-handles.pre) & t <= handles.post;
handles.temp.uV = d(segSel);
handles.temp.t = t(segSel);

% save traces if asked for
if logical(get(handles.save_traces,'Value'))
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
    ch = handles.objectToRetrieve;
    pfix = get(handles.trace_file_name_prefix,'String');
    ctime = round(clock);
    fnc = sprintf('%s%sh%02dm%02ds%02d_TTL_%u.mat',pfix,ch,ctime(4),ctime(5),ctime(6),str2double(get(handles.ttl,'String')));
    fn = fullfile(dataPath,fnc);
    data.t1_ms = handles.temp.t(1); % ms
    data.uV = handles.temp.uV; % 
    data.event_ts_us = handles.temp.event_ts;
    data.curr_uA = handles.exp.curr;
    data.chan = handles.objectToRetrieve;%#ok
    % Save data
    save(fn,'data')
end
plot_raw_trace(handles)

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
try
    % subtract the baseline trace just after the stimulus artifact
    sel = (t > 1.5) & (t < 2.5);
    by = mean(y(sel));
    y = y-by;
catch
    y = handles.temp.uV-mean(handles.temp.uV);
end
plot(t,y)
xlabel('Time (ms)','FontSize',6)
ylabel('uV','FontSize',6)
title('Raw data','FontSize',6)
set(gca,'TickLength',[0.005 0.005])
box off
set_xlim;

function postEventTime_Callback(hObject, eventdata, handles)
% hObject    handle to postEventTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of postEventTime as text
%        str2double(get(hObject,'String')) returns contents of postEventTime as a double
handles.post_event_time =  str2double(get(hObject,'String'));
guidata(hObject,handles);

function set_xlim
xlim([-2 45])

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
handles.stop_data = logical(get(hObject,'Value'));
set(handles.run_ltp_exp,'Value',false)
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

function send_stim_pulse
[succeeded, ~] = NlxSendCommand('-DigitalIOTtlPulse AcqSystem1_0 0 0 High');
if succeeded == 0
    disp 'FAILED to send TTL pulse to cheetah'
else
    disp 'PASSED TTL pulse'
end
% --- Executes on button press in run_ltp_exp.
function run_ltp_exp_Callback(hObject, eventdata, handles)
% hObject    handle to run_ltp_exp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clear_exp_data(handles);
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
pre = max(handles.pre/1000,1);
post = max(handles.post/1000,1);

buff_size = round((pre+post)/loop_period);
tic

% Get pulses
handles.n_pre_pulses = str2double(get(handles.nPrePulses,'String'));
% handles.n_ind_pulses = str2double(get(handles.nInductionPulses,'String'));
% handles.n_post_pulses = str2double(get(handles.nPostPulses,'String'));
% nTestStim = handles.n_pre_pulses + handles.n_post_pulses;

handles.n_ind_pulses = 0;
handles.n_post_pulses = 0;
cc = 0;
ddd = str2double(get(handles.trial_curr,'String'));
v = double(get(handles.trial_curr,'Value'));
handles.exp.curr = ddd(v);
handles.exp.slope = [];
handles.exp.popspike = [];
handles.numEvents = 0;
handles.time = nan;
% Before start, remove data in existing buffer of Cheetah by simply reading
% them off.
[~,  ~, ~, ~, ~, ~, ~ ] = NlxGetNewEventData('Events');
[~,~, ~, ~, ~, ~, ~, ~ ] = NlxGetNewCSCData(handles.objectToRetrieve);

while ~logical(get(handles.stop_watching,'Value'))
    nTestStim = str2double(get(handles.nPrePulses,'String'));
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
        % Keep counting detected events
        handles.numEvents(end+1) = length(find(f));
        set(handles.pulses_completed,'String',num2str(sum(handles.numEvents)))
    end
    np = sum( handles.numEvents);
    processNow = (np <= handles.n_pre_pulses) || ...
        (np > (handles.n_pre_pulses+handles.n_ind_pulses));
    if ~processNow
        disp('Not processing')
    end
    if any(f) && processNow
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
        
        currTime = clock;
        handles.time(cc) = (currTime(4)*60*60+currTime(5)*60+currTime(end));
        
        handles.exp.cc = cc;
        handles.temp.event_detected = false;
        handles = process_data(handles);
        
        getSlope = logical(get(handles.slope_measure,'Value'));
        getPopspike = logical(get(handles.popspike_measure,'Value'));
        if getSlope
            handles = compute_exp_slope(handles);
            handles = plot_exp_slope(handles);
        end
        if getPopspike
            handles = compute_exp_popspike(handles);
            handles = plot_exp_popspike(handles);
        end
        if getSlope && getPopspike
            plot_slope_vs_popspike(handles)
        end
        temp.data_uV = cell(1,1);
        temp.tsArray = cell(1,1);
        i = 0;
        %
        if cc==nTestStim
            % Turn off loop; Experiment is done.
            set(handles.stop_watching,'Value',1)
        end
        guidata(hObject,handles)
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
set(handles.run_ltp_exp,'Value',false)

function plot_slope_vs_popspike(handles)

axes(handles.slope_vs_popspike)
assert(length(handles.exp.slope)==length(handles.exp.popspike),'data points for slope and popspike are not paired')
hold on
plot(handles.exp.slope(end),handles.exp.popspike(end),'kO')
grid on
xlabel('Slope','FontSize',6)
ylabel('Popspike','FontSize',6)

function handles = plot_exp_slope(handles)
axes(handles.ltp_slope_plot)
y = abs(handles.exp.slope(end));
col = get_plot_color(handles);
toff = str2double(get(handles.time_offset,'String'));
t = toff+(handles.time(end)-handles.time(1))/60;
plot(t,y,'O','color',col,'markerfacecolor',col);
xlabel('Time (min)','FontSize',6)
% ylabel('Abs fEPSP slope (violet/s)')
title('Slope LTP','FontSize',6)
hold on
box off
function handles = plot_exp_popspike(handles)
axes(handles.ltp_popspike_plot)
y = abs(handles.exp.popspike(end));
col = get_plot_color(handles);
toff = str2double(get(handles.time_offset,'String'));
t = toff+(handles.time(end)-handles.time(1))/60;
plot(t,y,'O','color',col,'markerfacecolor',col);
xlabel('Time (min)','FontSize',6)
% ylabel('Abs fEPSP slope (violet/s)')
title('PopSpike LTP','FontSize',6)
hold on
box off
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
if isfield(y,'avg_slope_data')
    handles.exp.slope_learning_data = y.avg_slope_data;
else
    handles.exp.slope_learning_data = [];
end
if isfield(y,'avg_popspike_data')
    handles.exp.popspike_learning_data = y.avg_popspike_data;
else
    handles.exp.popspike_learning_data = [];
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

function handles = compute_exp_popspike(handles)
axes(handles.inst_popspike)
cla
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'b')
hold all
sd = handles.exp.popspike_learning_data;

if isempty(sd)
    disp('Popspike learning data not available')
    return
end
curr = handles.exp.curr;
ex_curr = sd(:,1);
[~,ex_ind] = min(abs(ex_curr-curr));
bounds = sd(ex_ind,3:end);
np = length(bounds);
if np == 4
    if str2double(get(get(handles.popspike_measure_type,'SelectedObject'),'String')) == 2
        nnp = 2;
    else
        nnp = 4;
    end
else
    nnp = 2;
end

if nnp == 2
    [h,tt,yy,ypi] = get_popspike_height(t,y,bounds([1 end]),'auto',true);
elseif nnp==4
    [h,tt,yy,ypi] = get_popspike_height(t,y,bounds,'auto',false);
end
axes(handles.inst_popspike)
hold on
plot(tt(2),ypi,'m*')
plot(tt(2),yy(2),'m*')
plot(tt([1 3]),yy([1 3]),'k-')
plot([tt(2) tt(2)],[yy(2) ypi],'m','linewidth',2)
set_xlim;
xlabel('Time(ms)','FontSize',6)
title('PopSpike','FontSize',6)
set(gca,'YTickLabel','')
box off

handles.exp.popspike(handles.exp.cc) = h;
% Save data
load(handles.ltpPopspikeFn)
ltpPopspikeData(end+1,:) = [handles.time(end) h];
save(handles.ltpPopspikeFn,'ltpPopspikeData')


function handles = compute_exp_slope(handles)
% hObject    handle to compute_slope (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.inst_slope)
cla
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'b')
hold all
set_xlim;
% if exist('newSlopeLearningData.mat','file') && logical(get(handles.slope_measure,'Value'));
%     load('newSlopeLearningData')
%     handles.exp.slope_learning_data = newSlopeLearningData;
% end
sd = handles.exp.slope_learning_data; % curr slope timeStart
if isempty(sd)
    disp('Slope learning data not available')
    return
end
curr = handles.exp.curr;
% Get latency of slope computation onset based on the closest learned
% example
ex_curr = sd(:,1);
[~,ex_ind] = min(abs(ex_curr-curr));
lat = sd(ex_ind,3);

% Add custom offset to the latency
offset = str2double(get(handles.slope_latency_offset,'String'));
lat = lat + offset;
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
set(gca,'YTickLabel','')
if logical(get(handles.flip_slope_sign,'Value'))
    slope = -B(1);
else
    slope = B(1);
end
handles.exp.slope(handles.exp.cc) = slope;
% Save data
title('Slope','FontSize',6)

load(handles.ltpSlopeFn)
ltpSlopeData(end+1,:) = [handles.time(end) slope];
save(handles.ltpSlopeFn,'ltpSlopeData')
% avgLtpSlopeData = compute_averaged_slope_data(ltpSlopeData);
% save('avgLtpSlopeData','avgLtpSlopeData')


function data = compute_averaged_slope_data(slopeData)
d = slopeData;
t = d(:,1);
x = d(:,2);
% b = str2double(get(handles.bin_size,'String'));
b= 5;
[data(:,1),data(:,2),data(:,3)] = bin_slopes(x,t,b);


function [tb,bs,se] = bin_slopes(x,t,b)
tw = diff(t(1:2));
ns = round(b/tw);
nBins = round(length(x)/ns);
bs = nan(1,nBins);
se = bs;
tb = bs;

for i = 1:nBins
    s1 = (i-1)*ns + 1;
    s2 = i*ns;
    xs = x(s1:s2);
    bs(i) = mean(xs);
    se(i) = std(xs)/sqrt(ns);
    tb(i) = mean(t([s1 s2]));
end



% --- Executes on button press in clear_resp_plot.
function clear_resp_plot_Callback(hObject, eventdata, handles)
% hObject    handle to clear_resp_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.ltp_slope_plot)
cla
axes(handles.ltp_popspike_plot)
cla


% --- Executes on button press in stimulate.
function stimulate_Callback(hObject, eventdata, handles)
% hObject    handle to stimulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
send_stim_pulse;



% --- Executes on button press in clear_exp_slope_data.
function clear_exp_slope_data_Callback(hObject, eventdata, handles)
% hObject    handle to clear_exp_slope_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% clear_raw_traces previous run data
clear_exp_data(handles);

function clear_exp_data(handles)
if exist(handles.ltpSlopeFn,'file')
    load(handles.ltpSlopeFn)
    ltpSlopeData = [];
    save(handles.ltpSlopeFn,'ltpSlopeData')
end

if exist(handles.avgLtpSlopeFn,'file')
    load(handles.avgLtpSlopeFn)
    avgLtpSlopeData = [];
    save(handles.avgLtpSlopeFn,'avgLtpSlopeData')
end

if exist(handles.ltpPopspikeFn,'file')
    load(handles.ltpPopspikeFn)
    ltpPopspikeData = [];
    save(handles.ltpPopspikeFn,'ltpPopspikeData')
end

if exist(handles.avgLtpPopspikeFn,'file')
    load(handles.avgLtpPopspikeFn)
    avgLtpPopspikeData = [];
    save(handles.avgLtpPopspikeFn,'avgLtpPopspikeData')
end
% --- Executes on button press in save_exp_data.
function save_exp_data_Callback(hObject, eventdata, handles)
% hObject    handle to save_exp_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Save data
load(handles.ltpSlopeFn)
ltp.slope_data = ltpSlopeData;
load(handles.ltpPopspikeFn)
ltp.popspike_data = ltpPopspikeData;

% ltp.avg_slope_data = compute_averaged_slope_data(ltpSlopeData);
ttl = [handles.objectToRetrieve '_' num2str(str2double(get(handles.ttl,'String')))];
fin = fullfile('E:\CheetahData\',['ltp_slope_data_' ttl '_' get_data_epoch(handles)]);
uisave('ltp',fin);


function clearTempSlopeData
if exist(handles.tempSlopeFn,'file')
    load(handles.tempSlopeFn)
    tempSlopeData = [];
    save(handles.tempSlopeFn,'tempSlopeData')
end


% --- Executes on button press in update_exp_io_plot.
function update_exp_io_plot_Callback(hObject, eventdata, handles)
% hObject    handle to update_exp_io_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.ltp_slope_plot)
load(handles.ltpSlopeFn)
da = ltpSlopeData;
hold on
% exp.avg_slope_data = compute_averaged_slope_data(ltpSlopeData);
toff = str2double(get(handles.time_offset,'String'));
t = toff + (da(:,1)-da(1,1))/60;
plot(t,abs(da(:,2)),'kO','markerfacecolor','k')


% --- Executes on button press in plot_slope_avg.
function plot_slope_avg_Callback(hObject, eventdata, handles)
% hObject    handle to plot_slope_avg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load(handles.avgLtpSlopeFn)
d = avgLtpSlopeData;
axes(handles.ltp_slope_plot)
hold on
pc = [0 0 0];
toff = str2double(get(handles.time_offset,'String'));
t = toff+d(:,1);
plot(t,d(:,2),'O','color',pc,'markerfacecolor',pc)
hold on
errorbar(d(:,1),d(:,2),d(:,3),'color',pc,'linestyle','none')


function nInductionPulses_Callback(hObject, eventdata, handles)
% hObject    handle to nInductionPulses (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nInductionPulses as text
%        str2double(get(hObject,'String')) returns contents of nInductionPulses as a double


% --- Executes during object creation, after setting all properties.
function nInductionPulses_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nInductionPulses (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function nPrePulses_Callback(hObject, eventdata, handles)
% hObject    handle to nPrePulses (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nPrePulses as text
%        str2double(get(hObject,'String')) returns contents of nPrePulses as a double


% --- Executes during object creation, after setting all properties.
function nPrePulses_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nPrePulses (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function nPostPulses_Callback(hObject, eventdata, handles)
% hObject    handle to nPostPulses (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nPostPulses as text
%        str2double(get(hObject,'String')) returns contents of nPostPulses as a double


% --- Executes during object creation, after setting all properties.
function nPostPulses_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nPostPulses (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bin_size_Callback(hObject, eventdata, handles)
% hObject    handle to bin_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% get(hObject,'String') returns contents of bin_size as text
handles.bin_val = str2double(get(hObject,'String')); % returns contents of bin_size as a double
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function bin_size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bin_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in average_slopes.
function average_slopes_Callback(hObject, eventdata, handles)
% hObject    handle to average_slopes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load(handles.ltpSlopeFn)
avgLtpSlopeData = compute_averaged_slope_data(ltpSlopeData);
save(handles.avgLtpSlopeFn,'avgLtpSlopeData')



% --- Executes on button press in load_old_data.
function load_old_data_Callback(hObject, eventdata, handles)
% hObject    handle to load_old_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fn,pn] = uigetfile('*.mat','Choose a example slope file','E:\CheetahData\');
fin = fullfile(pn,fn);
load(fin)
if isfield(ltp,'slope_data')
    handles.old_slope_data = ltp.slope_data;
end
if isfield(ltp,'popspike_data')
    handles.old_popspike_data = ltp.popspike_data;
end
guidata(hObject,handles)

% --- Executes on button press in plot_old_data.
function plot_old_data_Callback(hObject, eventdata, handles)
% hObject    handle to plot_old_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
d = handles.old_slope_data;
if ~isempty(d)
    axes(handles.ltp_slope_plot)
    col = get_plot_color(handles);
    hold on
    toff = str2double(get(handles.time_offset,'String'));
    t = toff+(d(:,1)-d(1,1))/60;
    plot(t,abs(d(:,2)),'O','color',col,'markerfacecolor',col)
end

d = handles.old_popspike_data;
if ~isempty(d)
    axes(handles.ltp_popspike_plot)
    col = get_plot_color(handles);
    hold on
    toff = str2double(get(handles.time_offset,'String'));
    t = toff+(d(:,1)-d(1,1))/60;
    plot(t,abs(d(:,2)),'O','color',col,'markerfacecolor',col)
end
function col = get_plot_color(handles)
ob = get(handles.color_selected,'SelectedObject');
so = get(ob,'Tag');
cmatrix = {'red',[1 0 0]; 
            'green',[0 0.8 0];
            'blue',[0 0 1];
            'magenta',[1 0 1];
            'cyan',[0 1 1];
            'black',[0 0 0];
            'gray',[0.5 0.5 0.5];
            'violet',[.367 .234 .598];
            'skyblue',[.172 .481 .711]};
ind = strcmp(cmatrix(:,1),so);
col = cmatrix{ind,2};
            
function ep = get_data_epoch(handles)
ob = get(handles.data_epoch,'SelectedObject');
ep = get(ob,'String');

% --- Executes when selected object is changed in color_selected.
function color_selected_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in color_selected
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function color_selected_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to color_selected (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function time_offset_Callback(hObject, eventdata, handles)
% hObject    handle to time_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time_offset as text
%        str2double(get(hObject,'String')) returns contents of time_offset as a double


% --- Executes during object creation, after setting all properties.
function time_offset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in grid_on.
function grid_on_Callback(hObject, eventdata, handles)
% hObject    handle to grid_on (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of grid_on

if logical(get(hObject,'Value'))
    grid on
end
set(hObject,'Value',0)



% --- Executes on button press in grid_off.
function grid_off_Callback(hObject, eventdata, handles)
% hObject    handle to grid_off (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of grid_off
if logical(get(hObject,'Value'))
    grid off
end
set(hObject,'Value',0)


% --- Executes on button press in draw_h_line.
function draw_h_line_Callback(hObject, eventdata, handles)
% hObject    handle to draw_h_line (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on
[~,y] = ginput(1);
col = get_plot_color(handles);
h = plot(xlim,[y y],'Color',col);
load linehandles.mat
linehandles(end+1) = h;
save('linehandles','linehandles')

% --- Executes on button press in draw_v_line.
function draw_v_line_Callback(hObject, eventdata, handles)
% hObject    handle to draw_v_line (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on
[x,~] = ginput(1);
col = get_plot_color(handles);
h = plot([x x],ylim,'Color',col);
load linehandles.mat
linehandles(end+1) = h;
save('linehandles','linehandles')
% --- Executes on button press in remove_lines.
function remove_lines_Callback(hObject, eventdata, handles)
% hObject    handle to remove_lines (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load linehandles.mat
delete(linehandles)
linehandles = [];
save('linehandles','linehandles')



function pulses_completed_Callback(hObject, eventdata, handles)
% hObject    handle to pulses_completed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pulses_completed as text
%        str2double(get(hObject,'String')) returns contents of pulses_completed as a double


% --- Executes during object creation, after setting all properties.
function pulses_completed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pulses_completed (see GCBO)
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


% --- Executes on button press in popspike_measure.
function popspike_measure_Callback(hObject, eventdata, handles)
% hObject    handle to popspike_measure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of popspike_measure


% --- Executes on button press in use_2_points.
function use_2_points_Callback(hObject, eventdata, handles)
% hObject    handle to use_2_points (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of use_2_points


% --- Executes on button press in use_4_points.
function use_4_points_Callback(hObject, eventdata, handles)
% hObject    handle to use_4_points (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of use_4_points


% --- Executes on button press in avg_raw_data.
function avg_raw_data_Callback(hObject, eventdata, handles)
% hObject    handle to avg_raw_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
c = get(handles.raw_trace,'Children');
rd = arrayfun(@(x) get(x,'YData'),c,'uni',false);
t = get(c(1),'XData');
if ~isempty(rd)
    le = cellfun(@length, rd);
    ml = min(le);
    sr = cellfun(@(x) x(1:ml),rd,'uni',false);
    sr = cellfun(@(x) x(:)',sr,'uni',false);
    sr = cat(1,sr{:});
    ar = mean(sr,1);
    
    handles.avg.t = t(1:ml);
    handles.avg.y = ar;
    guidata(hObject,handles)
end

% --- Executes on button press in plot_avg_trace.
function plot_avg_trace_Callback(hObject, eventdata, handles)
% hObject    handle to plot_avg_trace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.avg_raw_trace)
hold on
plot(handles.avg.t,handles.avg.y,'color',get_plot_color(handles),'linewidth',2)
set_xlim;
% --- Executes on button press in clear_avg_traces.
function clear_avg_traces_Callback(hObject, eventdata, handles)
% hObject    handle to clear_avg_traces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.avg_raw_trace)
cla


% --- Executes on button press in clear_corr_plot.
function clear_corr_plot_Callback(hObject, eventdata, handles)
% hObject    handle to clear_corr_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.slope_vs_popspike)
cla


% % --- Executes on button press in new_example_params.
% function new_example_params_Callback(hObject, eventdata, handles)
% % hObject    handle to new_example_params (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% getSlope = logical(get(handles.slope_measure,'Value'));
% getPopspike = logical(get(handles.popspike_measure,'Value'));
% set(hObject,'Value',0)
% if getSlope
%     recompute_example_slope_params(hObject, eventdata, handles)
% end
% if getPopspike
%     recompute_example_popspike_params(hObject, eventdata, handles)
% end

function recompute_example_popspike_params(hObject,eventsdata,handles)

axes(handles.inst_popspike)
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'k')
hold on

title('PopSpike Measurement','FontSize',6)
[bounds,~] = ginput(4);
handles.new.popspike_learning_data = [handles.exp.curr bounds(:)'];
guidata(hObject,handles)


function recompute_example_slope_params(hObject, eventdata, handles)
axes(handles.inst_slope)
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'k')
hold on
yso = mconv(y,getGausswin(0.5,1000*1/handles.Fs));
title('Slope Measurement','FontSize',6)

dy = diff(yso);
mInd = t>0.5 & t < min(100,t(end)); % ms
dy = max(yso(mInd))*dy/max(dy(mInd));
plot(t(2:end),dy,'r')
xlim([-2 45])
xlabel('Time(ms)','FontSize',6)
ylabel('Slope (V/s)','FontSize',6)
box off

% Ginput the start and end points to calculate slope
[tb,~] = ginput(2);
handles.new.slope_learning_data = [handles.exp.curr tb(:)'];
newSlopeLearningData = [handles.exp.curr tb(:)'];
save('newSlopeLearningData','newSlopeLearningData')
guidata(hObject,handles)


% --- Executes on button press in accept_new_params.
function accept_new_params_Callback(hObject, eventdata, handles)
% hObject    handle to accept_new_params (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of accept_new_params

if logical(get(hObject,'Value'))
    %     if ~isempty(handles.new.slope_learning_data) && logical(get(handles.slope_measure,'Value'));
    %         load('newSlopeLearningData')
    %         handles.exp.slope_learning_data = handles.new.slope_learning_data;
    %     end
    if exist('newSlopeLearningData.mat','file') && logical(get(handles.slope_measure,'Value'));
        load('newSlopeLearningData')
        handles.exp.slope_learning_data = newSlopeLearningData;
        set(hObject,'Value',false)
    end
    if ~isempty(handles.new.popspike_learning_data) && logical(get(handles.popspike_measure,'Value'));
        handles.exp.popspike_learning_data = handles.new.popspike_learning_data;
        set(hObject,'Value',false)
    end
end


% --- Executes on button press in save_traces.
function save_traces_Callback(hObject, eventdata, handles)
% hObject    handle to save_traces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of save_traces



function slope_latency_offset_Callback(hObject, eventdata, handles)
% hObject    handle to slope_latency_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of slope_latency_offset as text
%        str2double(get(hObject,'String')) returns contents of slope_latency_offset as a double


% --- Executes during object creation, after setting all properties.
function slope_latency_offset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slope_latency_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in new_example_params.
function new_example_params_Callback(hObject, eventdata, handles)
% hObject    handle to new_example_params (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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
