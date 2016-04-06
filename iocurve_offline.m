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

% Last Modified by GUIDE v2.5 22-Dec-2015 21:29:37

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
set(handles.cut_percent,'String','40')


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
% dd = uigetdir('Y:\ephys\raw');
dd = uigetdir('E:\CheetahData\2015-12-18_14-54-14');
if dd~=0
    handles.dataDir = dd;    
    d = dir(fullfile(handles.dataDir,'*.mat'));
    handles.filenames = {d.name};
    set(handles.file_list,'String',{d.name}')
    guidata(hObject, handles)
end



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
handles.Fs = data.Fs;
handles.curr = data.curr_uA;
plot(handles.temp.t,(handles.temp.uV),'k')
box off
axis tight
set_xlim

axes(handles.inst_response)
plot(handles.temp.t,(handles.temp.uV),'b')
box off
axis tight
if isfield(handles,'axis_limit')
    if get(handles.keep_axis_limits,'Value')==1
axis(handles.axis_limit)
    end
end

curr = handles.curr;
set(handles.current_set,'String',curr)
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
if fn >=1
    set(handles.file_list,'Value',fn)
    handles = update_raw_trace(hObject,handles,fn);
    update_resp_measure(hObject,eventdata,handles)
    guidata(hObject,handles)
end

% --- Executes on button press in next.
function next_Callback(hObject, eventdata, handles)
% hObject    handle to next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

nF = length(handles.filenames);
fn = get(handles.file_list,'Value');
fn = fn + 1;
if fn <= nF
    set(handles.file_list,'Value',fn)
    handles = update_raw_trace(hObject,handles,fn);
    update_resp_measure(hObject,eventdata,handles)
    guidata(hObject,handles)
end



function update_resp_measure(hObject,eventdata,handles)
axes(handles.inst_response)
cla
y = handles.temp.uV;
t = handles.temp.t;
plot(t,y,'b')
hold all
if isfield(handles,'axis_limit')
if get(handles.keep_axis_limits,'Value')==1
axis(handles.axis_limit)
    end
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
if logical(get(handles.slope_measure,'Value'))
    load('expSlopeDataManual')
    exp.slope_data = expSlopeDataManual;
    exp.avg_slope_data = compute_averaged_resp_measure_data(expSlopeDataManual);
end
if logical(get(handles.popspike_measure,'Value'))
    load('expPopspikeDataManual')
    exp.popspike_data = expPopspikeDataManual;
    exp.avg_popspike_data = compute_averaged_resp_measure_data(expPopspikeDataManual);
end
dn = uigetdir('E:\CheetahData\','Choose a directory to save exp slope data');
fin = fullfile(dn,'exp_resp_data_manual');
save(fin,'exp')

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


function set_xlim
xlim([-1 50])

function compute_example_popspike(hObject,eventsdata,handles)
y = handles.temp.uV;
tmsec = handles.temp.t;
curr = handles.curr;
axes(handles.inst_response)
title('PopSpike Measurement')
s = get(get(handles.popspike_measure_type,'SelectedObject'),'String');
np = str2double(s(1));
if np == 2
    [bounds,~] = ginput(2);
    [h,tt,yy,ypi] = get_popspike_height(tmsec,y,bounds,'auto',true);
elseif np == 3
    [x,y] = ginput(3);
    bnd = [x'; y'];
    bounds = x';
    [h,tt,~,ypi] = get_popspike_height(tmsec,y,bnd,'auto',false);
    yy = y';
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

xlabel('Time(ms)')
title('Instantaneous Resp Measurement')
set(gca,'YTickLabel','')
box off
hold off
box off
axis tight
if isfield(handles,'axis_limit')
if get(handles.keep_axis_limits,'Value')==1
axis(handles.axis_limit)
    end
end
% Save data

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


box off
yso = mconv(y,getGausswin(0.5,1000*1/handles.Fs));
% plot(t,y,'b')
hold all
plot(t,yso,'k')
dy = diff(yso);
mInd = t>0.5 & t < min(100,t(end)); % ms
dy = max(yso(mInd))*dy/max(dy(mInd));
plot(t(2:end),dy,'r')


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

xlabel('Time(ms)')
ylabel('Slope (V/s)')



hold off
box off
axis tight
if isfield(handles,'axis_limit')
if get(handles.keep_axis_limits,'Value')==1
axis(handles.axis_limit)
    end
end

% Save data
curr = handles.curr;
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
[fn,pn] = uigetfile('*.mat','Choose a example slope file','E:\CheetahData\');
fin = fullfile(pn,fn);
d = load(fin);
fn = fieldnames(d);
fn = fn{:};
x = d.(fn).slope_data(:,1);
y = abs(d.(fn).slope_data(:,2));
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

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
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


% % --- Executes on button press in load_curr_levels_file.
% function load_curr_levels_file_Callback(hObject, eventdata, handles)
% % hObject    handle to load_curr_levels_file (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% % [fn,pn] = uigetfile('*.mat','Choose a current levels file','E:\CheetahData\');
% [fn,pn] = uigetfile('*.mat','Choose a current levels file','E:\CheetahData\2015-12-18_14-07-44');
% fin = fullfile(pn,fn);
% load(fin)
% handles.exp.nBlocks = data.nBlocks;
% cl = [data.current_levels{:}];
% handles.exp.current_levels = cl(:);
% b1 = handles.exp.current_levels(1);
% set(handles.current_set,'String',[num2str(b1) 'uA'])
% set(handles.block_num,'String','1')
% guidata(hObject,handles)



% --- Executes on button press in multi_color.
function multi_color_Callback(hObject, eventdata, handles)
% hObject    handle to multi_color (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of multi_color
handles.multicolor = logical(get(hObject,'Value'));
guidata(hObject,handles);

% --- Executes on button press in overlay.
function overlay_Callback(hObject, eventdata, handles)
% hObject    handle to overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of overlay
handles.overlay_plots = logical(get(hObject,'Value'));
guidata(hObject,handles)

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
if get(hObject,'Value')==1
axes(gca)
handles.axis_limit = axis;
guidata(hObject,handles)
end




% --- Executes on button press in clear_traces.
function clear_traces_Callback(hObject, eventdata, handles)
% hObject    handle to clear_traces (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.inst_response)
cla
axes(handles.raw_trace)
cla



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


% --- Executes on button press in clearRespData.
function clearRespData_Callback(hObject, eventdata, handles)
% hObject    handle to clearRespData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clearResponseData

function clearResponseData

if exist('expSlopeDataManual.mat','file')
    load('expSlopeDataManual.mat')
    expSlopeDataManual = [];
    save('expSlopeDataManual','expSlopeDataManual')
end

if exist('avgExpSlopeDataManual.mat','file')
    load('avgExpSlopeDataManual')
    avgExpSlopeDataManual = [];
    save('avgExpSlopeDataManual','avgExpSlopeDataManual')
end

if exist('expPopSpikeDataManual.mat','file')
    load('expPopSpikeDataManual.mat')
    expPopSpikeDataManual = [];
    save('expPopSpikeDataManual','expPopSpikeDataManual')
end

if exist('avgExpSlopeDataManual.mat','file')
    load('avgExpSlopeDataManual')
    avgExpSlopeDataManual = [];
    save('avgExpSlopeDataManual','avgExpSlopeDataManual')
end

