function varargout = licenseplate(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @licenseplate_OpeningFcn, ...
                   'gui_OutputFcn',  @licenseplate_OutputFcn, ...
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


% --- Executes just before licenseplate is made visible.
% --- ����������� ����� ���, ��� �������� ���� ���������� �������.
function licenseplate_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to licenseplate (see VARARGIN)

% ��� ������� �� ����� �������� ���������� ��. OutputFcn. 
% HObject ���������� � �������� 
% EVENTDATA �������� - ������ ���� ���������� � ������� ������ MATLAB 
% ������������ ��������� � ������� � ���������������� ������ (��. GUIDATA) 
% Varargin ��������� ��������� ������ ��� licenseplate (��. VARARGIN)

% Choose default command line output for licenseplate
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes licenseplate wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = licenseplate_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in StartKnop.
function StartKnop_Callback(hObject, eventdata, handles)
% hObject    handle to StartKnop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%ophalen afbeelding
naam=get(handles.naam,'String');
A=imread(strcat('images/',naam));
org=A;
axes(handles.axes1);
[h,w,f]=size(A);
imshow(A);
A = rgb2gray(A);
level = graythresh(A);
A = im2bw(A,level*1.3);
axes(handles.axes2);
A=edge(A,'roberts');
imshow(A);

horHist=zeros(w);
for i=1:w
    tot=0;
    for j=1:h
        if (A(j,i)==1)
            tot=tot+1;
        end
    end
    horHist(i)=tot;
end
axes(handles.axes3);
gem=max(horHist)/2.3;
plot(horHist);
hstart=0;
heinde=0;
width=0;
hcounter=0;
arc=0;
hcoor=zeros(1,2);
for i=1:w
    if horHist(i)>gem(1)
        if(hstart==0)
            hstart=i;
        end
        hcounter=0;
    else
        if hstart>0
            if hcounter>(w*0.07)
                heinde=i-hcounter;
                width=heinde-hstart;
                if(width>(w*0.1))
                    arc=arc+1;
                    hcoor(arc,1)=hstart;
                    hcoor(arc,2)=width;
                end
                hstart=0;
                hcounter=0;
                heinde=0;
                width=0;
            end
            hcounter=hcounter+1;
        end
    end
end
[ww,f]=size(hcoor);
hstart=0;
hwidth=0;
for i=1:ww
    if(hcoor(i,2)>hwidth)
        hwidth=hcoor(i,2);
        hstart=hcoor(i,1);
    end
end

A=A(:,hstart:(hstart+hwidth),:);
axes(handles.axes2);
imshow(A);
verHist=zeros(h);
for j=1:h
    tot=0;
    for i=2:hwidth
        if (A(j,i-1)==1 && A(j,i)==0) || (A(j,i-1)==0 && A(j,i)==1) 
            tot=tot+1;
        end
    end
    verHist(j)=tot;
end
axes(handles.axes4);
verh=zeros(1);
coun=1;
for i=1:h
    if(verHist(i)>0)
        verh(coun)=verHist(i);
        coun=coun+1;
    end
end
gem=mean(verh)
plot(verHist);
vstart=0;
veinde=0;
height=0;
vcounter=0;
arc=0;
vcoor=zeros(1,2);
h*0.07
for(i=1:h)
    if verHist(i)>gem(1)
        if(vstart==0)
            vstart=i;
        end
        vcounter=0;
    else
        if vstart>0
            if vcounter>(h*0.03)
                veinde=i-vcounter;
                height=veinde-vstart;
                if(height>(h*0.05))
                    arc=arc+1;
                    vcoor(arc,1)=vstart;
                    vcoor(arc,2)=height;
                end
                vstart=0;
                vcounter=0;
                veinde=0;
                height=0;
            end
            vcounter=vcounter+1;
        end
    end
end
[l,f]=size(vcoor);
axes(handles.axes5);
A=org(vcoor(l,1):vcoor(l,1)+vcoor(l,2),hstart:(hstart+hwidth),:);
imshow(A);
axes(handles.axes6);
f=ocr(A);
set(handles.plaat,'String',f);

function naam_Callback(hObject, eventdata, handles)
% hObject    handle to naam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of naam as text
%        str2double(get(hObject,'String')) returns contents of naam as a double


% --- Executes during object creation, after setting all properties.
function naam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to naam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function plaat_Callback(hObject, eventdata, handles)
% hObject    handle to plaat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of plaat as text
%        str2double(get(hObject,'String')) returns contents of plaat as a double


% --- Executes during object creation, after setting all properties.
function plaat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plaat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
