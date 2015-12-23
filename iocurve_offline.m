function varargout = iocurve_offline(varargin)
% IOCURVE_OFFLINE MATLAB code for iocurve_offline.fig
%      IOCURVE_OFFLINE, by itself, creates a new IOCURVE_OFFLINE or raises the existing
%      singleton*.
%
%      H = IOCURVE_OFFLINE returns the handle to a new IOCURVE_OFFLINE or the handle to
%      the existing singleton*.
%
%      IOCURVE_OFFLINE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IOCURVE_OFFLINE.M with the given input arguments.
%
%      IOCURVE_OFFLINE('Property','Value',...) creates a new IOCURVE_OFFLINE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before iocurve_offline_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to iocurve_offline_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help iocurve_offline

% Last Modified by GUIDE v2.5 20-Dec-2015 18:49:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @iocurve_offline_OpeningFcn, ...
                   'gui_OutputFcn',  @iocurve_offline_OutputFcn, ...
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


% --- Executes just before iocurve_offline is made visible.
function iocurve_offline_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to iocurve_offline (see VARARGIN)

handles.slope_win = 1; %  ms
% Choose default command line output for iocurve_offline
handles.output = hObject;



expSlopeDataManual = [];
expPopspikeDataManual = [];

save('expSlopeDataManual','expSlopeDataManual')
save('expPopspikeDataManual','expPopspikeDataManual')



% Update handles structure
guidata(hObject, handles);

% UIWAIT makes iocurve_offline wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = iocurve_offline_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in load_traces.
function load_traces_Callback(hObject, eventdata, handles)
% hObject    handle to load_traces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.dataDir = uigetdir('Y:\ephys\raw');
d = dir(fullfile(handles.dataDir,'*.mat'));
handles.filenames = {d.name};
set(handles.file_list,'String',{d.name}')
guidata(hObject, handles)




% --- Executes on selection change in file_list.
function file_list_Callback(hObject, eventdata, handles)
% hObject    handle to file_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns file_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from file_list
contents = cellstr(get(hObject,'String'));
fileIndex = get(hObject,'Value');
handles.current_file = contents{fileIndex};
handles = update_raw_trace(hObject,handles,fileIndex);
guidata(hObject,handles)

function handles = update_raw_trace(hObject,handles,fileIndex)

handles.current_file = handles.filenames{fileIndex};
axes(handles.raw_trace)
load(fullfile(handles.dataDir,handles.current_file))
t0 = data.t1_ms;
dt = 1000*(1/data.Fs);
handles.temp.uV = data.uV-mean(data.uV);
handles.temp.t = t0 + (0:length(data.uV)-1)*dt;
plot(handles.temp.t,(handles.temp.uV),'k')
box off
axis tight
set_xlim

axes(handles.inst_response)
plot(handles.temp.t,(handles.temp.uV),'b')
box off
axis tight
if isfield(handles,'axis_limit')
axis(handles.axis_limit)
end

[curr,bn] = compute_condition(handles);
set(handles.current_set,'String',curr)
set(handles.block_num,'String',bn)
guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function file_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to file_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in prev.
function prev_Callback(hObject, eventdata, handles)
% hObject    handle to prev (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fn = get(handles.file_list,'Value');
fn = fn - 1;
if fn < 1
    fn = 1;
end
set(handles.file_list,'Value',fn)

handles = update_raw_trace(hObject,handles,fn);
update_resp_measure(hObject,eventdata,handles)
guidata(hObject,handles)


% --- Executes on button press in next.
function next_Callback(hObject, eventdata, handles)
% hObject    handle to next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

nF = length(handles.filenames);
fn = get(handles.file_list,'Value');
fn = fn + 1;
if fn > nF
    fn = nF;
end
set(handles.file_list,'Value',fn)
handles = update_raw_trace(hObject,handles,fn);
update_resp_measure(hObject,eventdata,handles)
guidata(hObject,handles)



function update_resp_measure(hObject,eventdata,handles)
axes(handles.inst_response)
cla
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'b')
hold all
if isfield(handles,'axis_limit')
axis(handles.axis_limit)
end

if logical(get(handles.slope_measure,'Value'))
    compute_example_slope(hObject,eventdata,handles);
end
if logical(get(handles.popspike_measure,'Value'))
    compute_example_popspike(hObject,eventdata,handles);
end



% --- Executes on button press in save_data.
function save_data_Callback(hObject, eventdata, handles)
% hObject    handle to save_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in quantify_resp.
function quantify_resp_Callback(hObject, eventdata, handles)
% hObject    handle to quantify_resp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

update_resp_measure(hObject,eventdata,handles)

% axes(handles.inst_response)
% cla
% y = handles.temp.uV;
% t = handles.temp.t;
% plot(t,y,'b')
% hold all
% 
% if logical(get(handles.slope_measure,'Value'))
%     compute_example_slope(hObject,eventdata,handles);
% end
% if logical(get(handles.popspike_measure,'Value'))
%     compute_example_popspike(hObject,eventdata,handles);
% end

function set_xlim
xlim([-1 50])

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
set_xlim;
xlabel('Time(ms)')
title('Instantaneous Resp Measurement')
set(gca,'YTickLabel','')
box off
hold off
% Save data
curr = compute_condition(handles);
load('expPopspikeDataManual')
fileIndex = get(handles.file_list,'Value');
expPopspikeDataManual(fileIndex,:) = [curr h bounds(:)'];
save('expPopspikeDataManual','expPopspikeDataManual')

handles = add_popspike_datapoint_to_io_plot(handles);
guidata(hObject,handles)

function handles = add_popspike_datapoint_to_io_plot(handles)
axes(handles.io_curve_popspike)
% Get current level used
load expPopspikeDataManual
d = expPopspikeDataManual;
ps = abs(d(end,2));
plot(d(end,1),ps,'kO','markerfacecolor','k')
hold on
xlabel('Current (\muA)')
% ylabel('Abs Popspike Amp (V)')
title('PopSpike I/O Curve')
box off
grid on

function handles = add_slope_datapoint_to_io_plot(handles)
axes(handles.io_curve_slope)
% Get current level used
load expSlopeDataManual
d = expSlopeDataManual;
slope = (d(end,2));
plot(d(end,1),slope,'kO','markerfacecolor','k')
hold on
grid on
xlabel('Current (\muA)')
% ylabel('Abs fEPSP slope (V/s)')
title('Slope I/O Curve')
box off

function compute_example_slope(hObject, eventdata, handles)
axes(handles.inst_response)
y = handles.temp.uV;
t = handles.temp.t;
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
set_xlim
% Save data
curr = compute_condition(handles);
load('expSlopeDataManual')
sv = B(1);
if logical(get(handles.flip_slope_sign,'Value'))
    sv = -B(1);
end
fileIndex = get(handles.file_list,'Value');
expSlopeDataManual(fileIndex,:) = [curr sv tb];
save('expSlopeDataManual','expSlopeDataManual')

handles = add_slope_datapoint_to_io_plot(handles);
guidata(hObject,handles)
title('Instantaneous Resp Measurement')


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

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
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
% [fn,pn] = uigetfile('*.mat','Choose a current levels file','E:\CheetahData\');
[fn,pn] = uigetfile('*.mat','Choose a current levels file','Y:\ephys\raw\2015-12-18_14-07-44');
fin = fullfile(pn,fn);
load(fin)
handles.exp.nBlocks = data.nBlocks;
handles.exp.current_levels = data.current_levels;
b1 = data.current_levels{1};
set(handles.current_set,'String',[num2str(b1(1)) 'uA'])
set(handles.block_num,'String','1')
guidata(hObject,handles)


function [curr, blockNum] = compute_condition(handles)
curr = NaN;
blockNum = NaN;
if isfield(handles,'exp')
    v = get(handles.file_list,'Value');
    cv = [handles.exp.current_levels{:}];
    cv = cv(:);
    curr = cv(v);
    blockNum = ceil(v/handles.exp.nBlocks);
end

% --- Executes on button press in multi_color.
function multi_color_Callback(hObject, eventdata, handles)
% hObject    handle to multi_color (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of multi_color


% --- Executes on button press in overlay.
function overlay_Callback(hObject, eventdata, handles)
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of overlay


% --- Executes on button press in flip_slope_sign.
function flip_slope_sign_Callback(hObject, eventdata, handles)
% hObject    handle to flip_slope_sign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of flip_slope_sign


% --- Executes on button press in keep_axis_limits.
function keep_axis_limits_Callback(hObject, eventdata, handles)
% hObject    handle to keep_axis_limits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of keep_axis_limits
axes(gca)
handles.axis_limit = axis;
guidata(hObject,handles)


% --- Executes on button press in iocurve.
function iocurve_Callback(hObject, eventdata, handles)
% hObject    handle to iocurve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of iocurve
