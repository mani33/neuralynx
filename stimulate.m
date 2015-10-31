function varargout = stimulate(varargin)
% STIMULATE MATLAB code for stimulate.fig
%      STIMULATE, by itself, creates a new STIMULATE or raises the existing
%      singleton*.
%
%      H = STIMULATE returns the handle to a new STIMULATE or the handle to
%      the existing singleton*.
%
%      STIMULATE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STIMULATE.M with the given input arguments.
%
%      STIMULATE('Property','Value',...) creates a new STIMULATE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before stimulate_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to stimulate_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help stimulate

% Last Modified by GUIDE v2.5 19-Aug-2015 13:44:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @stimulate_OpeningFcn, ...
                   'gui_OutputFcn',  @stimulate_OutputFcn, ...
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


% --- Executes just before stimulate is made visible.
function stimulate_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to stimulate (see VARARGIN)
add_path_to_neuralynx_lib
connect_to_cheetah_server
% Choose default command line output for stimulate
handles.output = hObject;
handles.closeConn = onCleanup(@() closeConnection);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes stimulate wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = stimulate_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


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

function closeConnection
disp('Came to closeConn')
%Disconnects from the server and shuts down NetCom
succeeded = NlxDisconnectFromServer();
if succeeded ~= 1
    disp 'FAILED disconnect from server'
else
    disp 'PASSED disconnect from server'
end
