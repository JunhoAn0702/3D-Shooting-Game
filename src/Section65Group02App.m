%Engr 202, Section 65 Group 2 section65group02app Application
function varargout = Section65Group02App(varargin)
CM=[-188 -188 -188; 241 241 241;0 0 0];
% Section65Group02App MATLAB code for Section65Group02App.fig
%      Section65Group02App, by itself, creates a new Section65Group02App or raises the existing
%      singleton*.
%
%      H = Section65Group02App returns the handle to a new Section65Group02App or the handle to
%      the existing singleton*.
%
%      Section65Group02App('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in Section65Group02App.M with the given input arguments.
%
%      Section65Group02App('Property','Value',...) creates a new Section65Group02App or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Section65Group02App_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Section65Group02App_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Section65Group02App

% Last Modified by GUIDE v2.5 15-Mar-2015 22:20:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Section65Group02App_OpeningFcn, ...
                   'gui_OutputFcn',  @Section65Group02App_OutputFcn, ...
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


% --- Executes just before Section65Group02App is made visible.
function Section65Group02App_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Section65Group02App (see VARARGIN)
Fire_Callback=0;

% Choose default command line output for Section65Group02App
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
%Gretting Message
mbox = msgbox('Hi welcome to Section 65 SuperFox game!'); uiwait(mbox);
% UIWAIT makes Section65Group02App wait for user response (see UIRESUME)
% uiwait(handles.figure1);
%% RESTART BUTTON
function Restart_Callback(hObject, eventdata, handles)

% hObject    handle to Restart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Declaring variables 
XTime=zeros(1,10);
    YTime=zeros(1,10);
    ZTime=zeros(1,10);
    TimeZeros=[0 1 2 3 4 5 6 7 8 9];
    Movement = 0;
    EMovement=0;
    Time=0;
    EnemyExist=1;
%Perform until the user press the stop
while strcmp(get(handles.start,'String'),'Stop') %read accelerometer output

    [handles.gx handles.gy handles.gz] = readAcc(handles.accelerometer, handles.calCo);

%Setting the Threshold values to minimize the noise
if handles.gx<=5 && handles.gx>=-5
    handles.gx=0;
end
if handles.gy<=5 && handles.gy>=-5
    handles.gy=0;
end
if handles.gz<=5 && handles.gz>=-5
    handles.gz=0;
end
%Display
    XTime(1)=XTime(2);
    XTime(2)=XTime(3);
    XTime(3)=XTime(4);
    XTime(4)=XTime(5);
    XTime(5)=XTime(6);
    XTime(6)=XTime(7);
    XTime(7)=XTime(8);
    XTime(8)=XTime(9);
    XTime(9)=XTime(10);
    XTime(10)=handles.gx;
    YTime(1)=YTime(2);
    YTime(2)=YTime(3);
    YTime(3)=YTime(4);
    YTime(4)=YTime(5);
    YTime(5)=YTime(6);
    YTime(6)=YTime(7);
    YTime(7)=YTime(8);
    YTime(8)=YTime(9);
    YTime(9)=YTime(10);
    YTime(10)=handles.gy;
    ZTime(1)=ZTime(2);
    ZTime(2)=ZTime(3);
    ZTime(3)=ZTime(4);
    ZTime(4)=ZTime(5);
    ZTime(5)=ZTime(6);
    ZTime(6)=ZTime(7);
    ZTime(7)=ZTime(8);
    ZTime(8)=ZTime(9);
    ZTime(9)=ZTime(10);
    ZTime(10)=handles.gz;
    CM=[-188 -188 -188; 241 241 241;0 0 0];
    
    
    cla;            %clear everything from the current axis
    
    %plot X and Y acceleration vectors and resultant acceleration vector
    
    line([0 handles.gx], [0 0], [0,0], 'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
    line([0 0], [0 handles.gy], [0 0],'Color', 'b', 'LineWidth', 2, 'Marker', 'o');
    line([0 0], [0 0], [0 handles.gz],'Color', 'g', 'LineWidth', 2, 'Marker', 'o');
    line([0 handles.gx], [0 handles.gy],[0 handles.gz],'Color', 'y', 'LineWidth', 2, 'Marker', 'o');

    %limit plot to +/- 1.25 g in all directions and make axis square
    handles.limits = 2.5;
    axis([-handles.limits handles.limits -handles.limits handles.limits]);
    axis square;
    
    %calculate the angle of the resultant acceleration vector and print
    handles.theta = atand(handles.gy/handles.gx);
    title(['Accelerometer tilt angle: ' num2str(handles.theta, '%.0f')]);
    
    %Setting the threshold
    Deg=YTime(10);
    if Deg <= -40
        Deg=-40;
    end
    if Deg >= 40
        Deg = 40;
    end
    if Deg <= -25
        Movement = Movement+8;
    end
    if Deg >= 25
        Movement = Movement -8;
    end
    if Movement >= 150
        Movement = 150;
    end
    if Movement <= -150
        Movement = -150;
    end
    
%Declare the part of EArwing
WingLT=[85 142 166;-225 -193 -193;0 0 0]+CM;
WingLB=[85 166 146;-225 -193 -208;0 0 0]+CM;
BlueL=[136 156 166;-160 -205 -193;0 0 0]+CM;
BodyL=[166 188 188; -193 -173 -188;0 0 0]+CM;
Engine=[166 188 210;-193 -188 -193;0 0 0]+CM;
BodyR=[-BodyL(1,:);BodyL(2,:);0 0 0];
BlueR=[-BlueL(1,:);BlueL(2,:);0 0 0];
WingRB=[-WingLB(1,:);WingLB(2,:);0 0 0];
WingRT=[-WingLT(1,:);WingLT(2,:);0 0 0];
Arwing=[WingLT WingLB BlueL BodyL Engine BodyR BlueR WingRB WingRT];
Arwing=[WingLT WingLB BlueL BodyL Engine BodyR BlueR WingRB WingRT];
Arwing = Arwing*.8;
EWingLT=[85 142 166;-225 -193 -193;0 0 0]+CM;
EWingLB=[85 166 146;-225 -193 -208;0 0 0]+CM;
EBlueL=[136 156 166;-160 -205 -193;0 0 0]+CM;
EBodyL=[166 188 188; -193 -173 -188;0 0 0]+CM;
EEngine=[166 188 210;-193 -188 -193;0 0 0]+CM;
EBodyR=[-EBodyL(1,:);EBodyL(2,:);0 0 0];
EBlueR=[-EBlueL(1,:);EBlueL(2,:);0 0 0];
EWingRB=[-EWingLB(1,:);EWingLB(2,:);0 0 0];
EWingRT=[-EWingLT(1,:);EWingLT(2,:);0 0 0];

EArwing=[EWingLT EWingLB EBlueL EBodyL EEngine EBodyR EBlueR EWingRB EWingRT];
EArwing = EArwing*.4;
if EnemyExist ==0
    EArwing=EArwing*0;
end
%Redeclare Parts as part of EArwing
EWingLT=[EArwing(:,1) EArwing(:,2) EArwing(:,3)];
EWingLB=[EArwing(:,4) EArwing(:,5) EArwing(:,6)];
EBlueL=[EArwing(:,7) EArwing(:,8) EArwing(:,9)];
EBodyL=[EArwing(:,10) EArwing(:,11) EArwing(:,12)];
EEngine=[EArwing(:,13) EArwing(:,14) EArwing(:,15)];
EBodyR=[EArwing(:,16) EArwing(:,17) EArwing(:,18)];
EBlueR=[EArwing(:,19) EArwing(:,20) EArwing(:,21)];
EWingRB=[EArwing(:,22) EArwing(:,23) EArwing(:,24)];
EWingRT=[EArwing(:,25) EArwing(:,26) EArwing(:,27)];

%Done that thing I said
%Move the EArwing
Rotate=[cosd(Deg) -sind(Deg) 0;sind(Deg) cosd(Deg) 0;0 0 1];

Center=zeros(3,27);
Center(2,:)=0;
Rectify=-Center;
Arwing = Center+Arwing;
Arwing = Rotate*Arwing;
Arwing = Rectify+Arwing;

Arwing(1,:)=Arwing(1,:)+Movement;
Arwing(2,:)=Arwing(2,:)+100;
WingLT=[Arwing(:,1) Arwing(:,2) Arwing(:,3)];
WingLB=[Arwing(:,4) Arwing(:,5) Arwing(:,6)];
BlueL=[Arwing(:,7) Arwing(:,8) Arwing(:,9)];
BodyL=[Arwing(:,10) Arwing(:,11) Arwing(:,12)];
Engine=[Arwing(:,13) Arwing(:,14) Arwing(:,15)];
BodyR=[Arwing(:,16) Arwing(:,17) Arwing(:,18)];
BlueR=[Arwing(:,19) Arwing(:,20) Arwing(:,21)];
WingRB=[Arwing(:,22) Arwing(:,23) Arwing(:,24)];
WingRT=[Arwing(:,25) Arwing(:,26) Arwing(:,27)];
%

Time=Time+10;
%Random movement of enemy
%Deg=YTime(10);
EDeg=40*sind(Time);
ERotate=[cosd(EDeg) -sind(EDeg) 0;sind(EDeg) cosd(EDeg) 0;0 0 1];
    %Setting the threshold value of enemy's movement
    if EDeg <= -40
        EDeg=-40;
    end
    if EDeg >= 40
        EDeg = 40;
    end
    if EDeg <= -25
        EMovement = EMovement+15;
    end
    if EDeg >= 25
        EMovement = EMovement -15;
    end
    if EMovement >= 80
        EMovement = 80;
    end
    if EMovement <= -80
        EMovement = -80;
    end
    EMovement;
EArwing=ERotate*EArwing;
EArwing(1,:)=EArwing(1,:)+EMovement;
EWingLT=[EArwing(:,1) EArwing(:,2) EArwing(:,3)];
EWingLB=[EArwing(:,4) EArwing(:,5) EArwing(:,6)];
EBlueL=[EArwing(:,7) EArwing(:,8) EArwing(:,9)];
EBodyL=[EArwing(:,10) EArwing(:,11) EArwing(:,12)];
EEngine=[EArwing(:,13) EArwing(:,14) EArwing(:,15)];
EBodyR=[EArwing(:,16) EArwing(:,17) EArwing(:,18)];
EBlueR=[EArwing(:,19) EArwing(:,20) EArwing(:,21)];
EWingRB=[EArwing(:,22) EArwing(:,23) EArwing(:,24)];
EWingRT=[EArwing(:,25) EArwing(:,26) EArwing(:,27)];
hold off

%Displaying the background
img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
fill(WingLT(1,:),WingLT(2,:),[.7 .7 .7])
hold on
fill(WingLB(1,:),WingLB(2,:),[.3 .3 .3])
fill(BlueL(1,:),BlueL(2,:),'b')
fill(BodyL(1,:),BodyL(2,:),[.7 .7 .7])
fill(Engine(1,:),Engine(2,:),'y')

%Space visualization
fill(BodyR(1,:),BodyR(2,:),[.3 .3 .3])
fill(WingRB(1,:),WingRB(2,:),[.3 .3 .3])
fill(WingRT(1,:),WingRT(2,:),[.7 .7 .7])
fill(BlueR(1,:),BlueR(2,:),'b')
axis([-187 187 -134 241])

%Enemy goes here
fill(EWingLT(1,:),EWingLT(2,:),[.7 .7 .7])
hold on
fill(EWingLB(1,:),EWingLB(2,:),[.3 .3 .3])
fill(EBlueL(1,:),EBlueL(2,:),'r')
fill(EBodyL(1,:),EBodyL(2,:),[.7 .7 .7])
fill(EEngine(1,:),EEngine(2,:),'y')
%Space visualization

fill(EBodyR(1,:),EBodyR(2,:),[.3 .3 .3])
fill(EWingRB(1,:),EWingRB(2,:),[.3 .3 .3])
fill(EWingRT(1,:),EWingRT(2,:),[.7 .7 .7])
fill(EBlueR(1,:),EBlueR(2,:),'r')
axis([-187 187 -134 241])

hold off
pause(0.00000001)
 

%% Whole thing about lasers hitting goes here.
if abs(XTime(10)) >= 50 && abs(XTime(9)) <=20 % If you flick the thing, it shoots a laser
    x=8;
Thing1=[1;0;0];
Sign=zeros(3,x);
Sign(:,1)=[1;0;0];
Boom=zeros(3,2*x);
for i =1:x
    
    Sign(:,i+1)=[cosd(i*360/x) -sind(i*360/x) 0;sind(i*360/x) cosd(i*360/x) 0; 0 0 1]*Thing1;
end
SignInner=.4*[cosd((360/x)/2) -sind((360/x)/2) 0;sind((360/x)/2) cosd((360/x)/2) 0; 0 0 1]*Sign;

for i=1:x
    Boom(:,(-1+(2*i)))=Sign(:,i);
    Boom(:,2*i)=SignInner(:,i);
end
Boom=Boom*10;
Boom(1,:)=Boom(1,:);
Boom(2,:)=Boom(2,:);
BoomL=Boom;
Boom(1,:)=Boom(1,:)+BlueR(1,2);
Boom(2,:)=Boom(2,:)+BlueR(2,2);
BoomL(1,:)=BoomL(1,:)+BlueL(1,2);
BoomL(2,:)=BoomL(2,:)+BlueL(2,2);
hold on

Boom(1,:)=Boom(1,:);
img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
fill(Boom(1,:),Boom(2,:),[.1 1 1])
fill(BoomL(1,:),BoomL(2,:),[.1 1 1])
axis([-187 187 -134 241])

%For loop that shrinks blasts
pause(0.00001)
for i = 1:30
    Shrink=(100-i)/100;
    Boom=Boom*Shrink;
    BoomL=BoomL*Shrink;

%make Arwing
img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
    hold on
    fill(WingLT(1,:),WingLT(2,:),[.7 .7 .7])
hold on
fill(WingLB(1,:),WingLB(2,:),[.3 .3 .3])
fill(BlueL(1,:),BlueL(2,:),'b')
fill(BodyL(1,:),BodyL(2,:),[.7 .7 .7])
fill(Engine(1,:),Engine(2,:),'y')

%Space visualization
fill(BodyR(1,:),BodyR(2,:),[.3 .3 .3])
fill(WingRB(1,:),WingRB(2,:),[.3 .3 .3])
fill(WingRT(1,:),WingRT(2,:),[.7 .7 .7])
fill(BlueR(1,:),BlueR(2,:),'b')

axis([-187 187 -134 241])

fill(Boom(1,:),Boom(2,:),[.1 1 1])
fill(BoomL(1,:),BoomL(2,:),[.1 1 1])
fill(EWingLT(1,:),EWingLT(2,:),[.7 .7 .7])
hold on
fill(EWingLB(1,:),EWingLB(2,:),[.3 .3 .3])
fill(EBlueL(1,:),EBlueL(2,:),'r')
fill(EBodyL(1,:),EBodyL(2,:),[.7 .7 .7])
fill(EEngine(1,:),EEngine(2,:),'y')

%Space visualization
fill(EBodyR(1,:),EBodyR(2,:),[.3 .3 .3])
fill(EWingRB(1,:),EWingRB(2,:),[.3 .3 .3])
fill(EWingRT(1,:),EWingRT(2,:),[.7 .7 .7])
fill(EBlueR(1,:),EBlueR(2,:),'r')
axis([-187 187 -134 241])
hold off
    pause(0.001)
end

%% Detemine whether the ememy gets hit with the laser.
    if EMovement >= -10 && EMovement <= 10 %If enemy is within range when shot, explosion
    Explosion=[1,0.389971164872730,0.900968867902419,0.312732592987212,0.623489801858734,0.173553495647023,0.222520933956314,2.77555756156289e-17,-0.222520933956314,-0.173553495647023,-0.623489801858734,-0.312732592987212,-0.900968867902419,-0.389971164872729,-1,-0.389971164872730,-0.900968867902419,-0.312732592987212,-0.623489801858734,-0.173553495647023,-0.222520933956314,1.66533453693773e-16,0.222520933956314,0.173553495647023,0.623489801858733,0.312732592987212,0.900968867902419,0.389971164872729;0,0.0890083735825258,0.433883739117558,0.249395920743493,0.781831482468030,0.360387547160968,0.974927912181824,0.400000000000000,0.974927912181824,0.360387547160968,0.781831482468030,0.249395920743493,0.433883739117558,0.0890083735825258,0,-0.0890083735825258,-0.433883739117558,-0.249395920743493,-0.781831482468030,-0.360387547160968,-0.974927912181824,-0.400000000000000,-0.974927912181824,-0.360387547160968,-0.781831482468030,-0.249395920743494,-0.433883739117558,-0.0890083735825258;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]*20;
fill(Explosion(1,:),Explosion(2,:),'r')
axis([-187 187 -134 241])

%Part that the enemy's ship explodes
for t= 1:10:1000
    Explosion=[1,0.389971164872730,0.900968867902419,0.312732592987212,0.623489801858734,0.173553495647023,0.222520933956314,2.77555756156289e-17,-0.222520933956314,-0.173553495647023,-0.623489801858734,-0.312732592987212,-0.900968867902419,-0.389971164872729,-1,-0.389971164872730,-0.900968867902419,-0.312732592987212,-0.623489801858734,-0.173553495647023,-0.222520933956314,1.66533453693773e-16,0.222520933956314,0.173553495647023,0.623489801858733,0.312732592987212,0.900968867902419,0.389971164872729;0,0.0890083735825258,0.433883739117558,0.249395920743493,0.781831482468030,0.360387547160968,0.974927912181824,0.400000000000000,0.974927912181824,0.360387547160968,0.781831482468030,0.249395920743493,0.433883739117558,0.0890083735825258,0,-0.0890083735825258,-0.433883739117558,-0.249395920743493,-0.781831482468030,-0.360387547160968,-0.974927912181824,-0.400000000000000,-0.974927912181824,-0.360387547160968,-0.781831482468030,-0.249395920743494,-0.433883739117558,-0.0890083735825258;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]*20;
    Explosion2=Explosion;
    Deg=t;
    Deg2=-t;
    Rotate=[cosd(Deg) -sind(Deg) 0;sind(Deg) cosd(Deg) 0;0 0 1];
    Rotate2=[cosd(Deg2) -sind(Deg2) 0;sind(Deg2) cosd(Deg2) 0;0 0 1];
    Explosion=Rotate*Explosion;
    Explosion=Explosion*(sind(t)+1.5);
    Explosion2=Rotate2*Explosion2;
    Explosion2=Explosion2*(cosd(t)+1.5);
    fill(Explosion2(1,:),Explosion2(2,:),'y')
    hold on
    img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
    fill(Explosion(1,:),Explosion(2,:),'r')
    fill(WingLT(1,:),WingLT(2,:),[.7 .7 .7])
hold on
fill(WingLB(1,:),WingLB(2,:),[.3 .3 .3])
fill(BlueL(1,:),BlueL(2,:),'b')
fill(BodyL(1,:),BodyL(2,:),[.7 .7 .7])
fill(Engine(1,:),Engine(2,:),'y')

%Space visualization
fill(BodyR(1,:),BodyR(2,:),[.3 .3 .3])
fill(WingRB(1,:),WingRB(2,:),[.3 .3 .3])
fill(WingRT(1,:),WingRT(2,:),[.7 .7 .7])
fill(BlueR(1,:),BlueR(2,:),'b')
axis([-187 187 -134 241])
    hold off
axis([-187 187 -134 241])
%Make the enemy spaceship gone.
EnemyExist = 0;
pause(0.001)
end
        EnemyExist =0;
        mbox = msgbox('Great! You ready to play again? Hit Restart Button'); uiwait(mbox);
    end
end
end %End of while Loop


% --- Outputs from this function are returned to the command line.
function varargout = Section65Group02App_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%% Declare smaller Arwing
CM=[-188 -188 -188; 241 241 241;0 0 0];
Arwing=zeros(0,27);
WingLT=[85 142 166;-225 -193 -193;0 0 0]+CM;
WingLB=[85 166 146;-225 -193 -208;0 0 0]+CM;
BlueL=[136 156 166;-160 -205 -193;0 0 0]+CM;
BodyL=[166 188 188; -193 -173 -188;0 0 0]+CM;
Engine=[166 188 210;-193 -188 -193;0 0 0]+CM;
BodyR=[-BodyL(1,:);BodyL(2,:);0 0 0];
BlueR=[-BlueL(1,:);BlueL(2,:);0 0 0];
WingRB=[-WingLB(1,:);WingLB(2,:);0 0 0];
WingRT=[-WingLT(1,:);WingLT(2,:);0 0 0];
Arwing=[WingLT WingLB BlueL BodyL Engine BodyR BlueR WingRB WingRT];
Arwing = Arwing*.5;
%% Redeclaring the parts as a reference to Arwing
WingLT=[Arwing(:,1) Arwing(:,2) Arwing(:,3)];
WingLB=[Arwing(:,4) Arwing(:,5) Arwing(:,6)];
BlueL=[Arwing(:,7) Arwing(:,8) Arwing(:,9)];
BodyL=[Arwing(:,10) Arwing(:,11) Arwing(:,12)];
Engine=[Arwing(:,13) Arwing(:,14) Arwing(:,15)];
BodyR=[Arwing(:,16) Arwing(:,17) Arwing(:,18)];
BlueR=[Arwing(:,19) Arwing(:,20) Arwing(:,21)];
WingRB=[Arwing(:,22) Arwing(:,23) Arwing(:,24)];
WingRT=[Arwing(:,25) Arwing(:,26) Arwing(:,27)];
Movement=0;
EMovement=0;
% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Pushed=get(hObject,'Value');
if Pushed
    set(hObject,'String','Stop');
else
    set(hObject,'String','Start');
end
    %Declaring variables 
XTime=zeros(1,10);
    YTime=zeros(1,10);
    ZTime=zeros(1,10);
    TimeZeros=[0 1 2 3 4 5 6 7 8 9];
    Movement = 0;
    EMovement=0;
    Time=0;
    EnemyExist=1;
%Perform until the user press the stop
while strcmp(get(handles.start,'String'),'Stop') %read accelerometer output

    [handles.gx handles.gy handles.gz] = readAcc(handles.accelerometer, handles.calCo);

%Setting the Threshold values to minimize the noise
if handles.gx<=5 && handles.gx>=-5
    handles.gx=0;
end
if handles.gy<=5 && handles.gy>=-5
    handles.gy=0;
end
if handles.gz<=5 && handles.gz>=-5
    handles.gz=0;
end
%Display
    XTime(1)=XTime(2);
    XTime(2)=XTime(3);
    XTime(3)=XTime(4);
    XTime(4)=XTime(5);
    XTime(5)=XTime(6);
    XTime(6)=XTime(7);
    XTime(7)=XTime(8);
    XTime(8)=XTime(9);
    XTime(9)=XTime(10);
    XTime(10)=handles.gx;
    YTime(1)=YTime(2);
    YTime(2)=YTime(3);
    YTime(3)=YTime(4);
    YTime(4)=YTime(5);
    YTime(5)=YTime(6);
    YTime(6)=YTime(7);
    YTime(7)=YTime(8);
    YTime(8)=YTime(9);
    YTime(9)=YTime(10);
    YTime(10)=handles.gy;
    ZTime(1)=ZTime(2);
    ZTime(2)=ZTime(3);
    ZTime(3)=ZTime(4);
    ZTime(4)=ZTime(5);
    ZTime(5)=ZTime(6);
    ZTime(6)=ZTime(7);
    ZTime(7)=ZTime(8);
    ZTime(8)=ZTime(9);
    ZTime(9)=ZTime(10);
    ZTime(10)=handles.gz;
    CM=[-188 -188 -188; 241 241 241;0 0 0];
    
    
    cla;            %clear everything from the current axis
    
    %plot X and Y acceleration vectors and resultant acceleration vector
    
    line([0 handles.gx], [0 0], [0,0], 'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
    line([0 0], [0 handles.gy], [0 0],'Color', 'b', 'LineWidth', 2, 'Marker', 'o');
    line([0 0], [0 0], [0 handles.gz],'Color', 'g', 'LineWidth', 2, 'Marker', 'o');
    line([0 handles.gx], [0 handles.gy],[0 handles.gz],'Color', 'y', 'LineWidth', 2, 'Marker', 'o');

    %limit plot to +/- 1.25 g in all directions and make axis square
    handles.limits = 2.5;
    axis([-handles.limits handles.limits -handles.limits handles.limits]);
    axis square;
    
    %calculate the angle of the resultant acceleration vector and print
    handles.theta = atand(handles.gy/handles.gx);
    title(['Accelerometer tilt angle: ' num2str(handles.theta, '%.0f')]);
    
    %Setting the threshold
    Deg=YTime(10);
    if Deg <= -40
        Deg=-40;
    end
    if Deg >= 40
        Deg = 40;
    end
    if Deg <= -25
        Movement = Movement+8;
    end
    if Deg >= 25
        Movement = Movement -8;
    end
    if Movement >= 150
        Movement = 150;
    end
    if Movement <= -150
        Movement = -150;
    end
    
%Declare the part of EArwing
WingLT=[85 142 166;-225 -193 -193;0 0 0]+CM;
WingLB=[85 166 146;-225 -193 -208;0 0 0]+CM;
BlueL=[136 156 166;-160 -205 -193;0 0 0]+CM;
BodyL=[166 188 188; -193 -173 -188;0 0 0]+CM;
Engine=[166 188 210;-193 -188 -193;0 0 0]+CM;
BodyR=[-BodyL(1,:);BodyL(2,:);0 0 0];
BlueR=[-BlueL(1,:);BlueL(2,:);0 0 0];
WingRB=[-WingLB(1,:);WingLB(2,:);0 0 0];
WingRT=[-WingLT(1,:);WingLT(2,:);0 0 0];
Arwing=[WingLT WingLB BlueL BodyL Engine BodyR BlueR WingRB WingRT];
Arwing=[WingLT WingLB BlueL BodyL Engine BodyR BlueR WingRB WingRT];
Arwing = Arwing*.8;
EWingLT=[85 142 166;-225 -193 -193;0 0 0]+CM;
EWingLB=[85 166 146;-225 -193 -208;0 0 0]+CM;
EBlueL=[136 156 166;-160 -205 -193;0 0 0]+CM;
EBodyL=[166 188 188; -193 -173 -188;0 0 0]+CM;
EEngine=[166 188 210;-193 -188 -193;0 0 0]+CM;
EBodyR=[-EBodyL(1,:);EBodyL(2,:);0 0 0];
EBlueR=[-EBlueL(1,:);EBlueL(2,:);0 0 0];
EWingRB=[-EWingLB(1,:);EWingLB(2,:);0 0 0];
EWingRT=[-EWingLT(1,:);EWingLT(2,:);0 0 0];

EArwing=[EWingLT EWingLB EBlueL EBodyL EEngine EBodyR EBlueR EWingRB EWingRT];
EArwing = EArwing*.4;
if EnemyExist ==0
    EArwing=EArwing*0;
end
%Redeclare Parts as part of EArwing
EWingLT=[EArwing(:,1) EArwing(:,2) EArwing(:,3)];
EWingLB=[EArwing(:,4) EArwing(:,5) EArwing(:,6)];
EBlueL=[EArwing(:,7) EArwing(:,8) EArwing(:,9)];
EBodyL=[EArwing(:,10) EArwing(:,11) EArwing(:,12)];
EEngine=[EArwing(:,13) EArwing(:,14) EArwing(:,15)];
EBodyR=[EArwing(:,16) EArwing(:,17) EArwing(:,18)];
EBlueR=[EArwing(:,19) EArwing(:,20) EArwing(:,21)];
EWingRB=[EArwing(:,22) EArwing(:,23) EArwing(:,24)];
EWingRT=[EArwing(:,25) EArwing(:,26) EArwing(:,27)];

%Done that thing I said
%Move the EArwing
Rotate=[cosd(Deg) -sind(Deg) 0;sind(Deg) cosd(Deg) 0;0 0 1];

Center=zeros(3,27);
Center(2,:)=0;
Rectify=-Center;
Arwing = Center+Arwing;
Arwing = Rotate*Arwing;
Arwing = Rectify+Arwing;

Arwing(1,:)=Arwing(1,:)+Movement;
Arwing(2,:)=Arwing(2,:)+100;
WingLT=[Arwing(:,1) Arwing(:,2) Arwing(:,3)];
WingLB=[Arwing(:,4) Arwing(:,5) Arwing(:,6)];
BlueL=[Arwing(:,7) Arwing(:,8) Arwing(:,9)];
BodyL=[Arwing(:,10) Arwing(:,11) Arwing(:,12)];
Engine=[Arwing(:,13) Arwing(:,14) Arwing(:,15)];
BodyR=[Arwing(:,16) Arwing(:,17) Arwing(:,18)];
BlueR=[Arwing(:,19) Arwing(:,20) Arwing(:,21)];
WingRB=[Arwing(:,22) Arwing(:,23) Arwing(:,24)];
WingRT=[Arwing(:,25) Arwing(:,26) Arwing(:,27)];
%

Time=Time+10;
%Random movement of enemy
%Deg=YTime(10);
EDeg=40*sind(Time);
ERotate=[cosd(EDeg) -sind(EDeg) 0;sind(EDeg) cosd(EDeg) 0;0 0 1];
    %Setting the threshold value of enemy's movement
    if EDeg <= -40
        EDeg=-40;
    end
    if EDeg >= 40
        EDeg = 40;
    end
    if EDeg <= -25
        EMovement = EMovement+15;
    end
    if EDeg >= 25
        EMovement = EMovement -15;
    end
    if EMovement >= 80
        EMovement = 80;
    end
    if EMovement <= -80
        EMovement = -80;
    end
    EMovement;
EArwing=ERotate*EArwing;
EArwing(1,:)=EArwing(1,:)+EMovement;
EWingLT=[EArwing(:,1) EArwing(:,2) EArwing(:,3)];
EWingLB=[EArwing(:,4) EArwing(:,5) EArwing(:,6)];
EBlueL=[EArwing(:,7) EArwing(:,8) EArwing(:,9)];
EBodyL=[EArwing(:,10) EArwing(:,11) EArwing(:,12)];
EEngine=[EArwing(:,13) EArwing(:,14) EArwing(:,15)];
EBodyR=[EArwing(:,16) EArwing(:,17) EArwing(:,18)];
EBlueR=[EArwing(:,19) EArwing(:,20) EArwing(:,21)];
EWingRB=[EArwing(:,22) EArwing(:,23) EArwing(:,24)];
EWingRT=[EArwing(:,25) EArwing(:,26) EArwing(:,27)];
hold off

%Displaying the background
img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
fill(WingLT(1,:),WingLT(2,:),[.7 .7 .7])
hold on
fill(WingLB(1,:),WingLB(2,:),[.3 .3 .3])
fill(BlueL(1,:),BlueL(2,:),'b')
fill(BodyL(1,:),BodyL(2,:),[.7 .7 .7])
fill(Engine(1,:),Engine(2,:),'y')

%Space visualization
fill(BodyR(1,:),BodyR(2,:),[.3 .3 .3])
fill(WingRB(1,:),WingRB(2,:),[.3 .3 .3])
fill(WingRT(1,:),WingRT(2,:),[.7 .7 .7])
fill(BlueR(1,:),BlueR(2,:),'b')
axis([-187 187 -134 241])

%Enemy goes here
fill(EWingLT(1,:),EWingLT(2,:),[.7 .7 .7])
hold on
fill(EWingLB(1,:),EWingLB(2,:),[.3 .3 .3])
fill(EBlueL(1,:),EBlueL(2,:),'r')
fill(EBodyL(1,:),EBodyL(2,:),[.7 .7 .7])
fill(EEngine(1,:),EEngine(2,:),'y')
%Space visualization

fill(EBodyR(1,:),EBodyR(2,:),[.3 .3 .3])
fill(EWingRB(1,:),EWingRB(2,:),[.3 .3 .3])
fill(EWingRT(1,:),EWingRT(2,:),[.7 .7 .7])
fill(EBlueR(1,:),EBlueR(2,:),'r')
axis([-187 187 -134 241])

hold off
pause(0.00000001)
 

%% Whole thing about lasers hitting goes here.
if abs(XTime(10)) >= 50 && abs(XTime(9)) <=20 % If you flick the thing, it shoots a laser
    x=8;
Thing1=[1;0;0];
Sign=zeros(3,x);
Sign(:,1)=[1;0;0];
Boom=zeros(3,2*x);
for i =1:x
    
    Sign(:,i+1)=[cosd(i*360/x) -sind(i*360/x) 0;sind(i*360/x) cosd(i*360/x) 0; 0 0 1]*Thing1;
end
SignInner=.4*[cosd((360/x)/2) -sind((360/x)/2) 0;sind((360/x)/2) cosd((360/x)/2) 0; 0 0 1]*Sign;

for i=1:x
    Boom(:,(-1+(2*i)))=Sign(:,i);
    Boom(:,2*i)=SignInner(:,i);
end
Boom=Boom*10;
Boom(1,:)=Boom(1,:);
Boom(2,:)=Boom(2,:);
BoomL=Boom;
Boom(1,:)=Boom(1,:)+BlueR(1,2);
Boom(2,:)=Boom(2,:)+BlueR(2,2);
BoomL(1,:)=BoomL(1,:)+BlueL(1,2);
BoomL(2,:)=BoomL(2,:)+BlueL(2,2);
hold on

Boom(1,:)=Boom(1,:);
img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
fill(Boom(1,:),Boom(2,:),[.1 1 1])
fill(BoomL(1,:),BoomL(2,:),[.1 1 1])
axis([-187 187 -134 241])

%For loop that shrinks blasts
pause(0.00001)
for i = 1:30
    Shrink=(100-i)/100;
    Boom=Boom*Shrink;
    BoomL=BoomL*Shrink;

%make Arwing
img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
    hold on
    fill(WingLT(1,:),WingLT(2,:),[.7 .7 .7])
hold on
fill(WingLB(1,:),WingLB(2,:),[.3 .3 .3])
fill(BlueL(1,:),BlueL(2,:),'b')
fill(BodyL(1,:),BodyL(2,:),[.7 .7 .7])
fill(Engine(1,:),Engine(2,:),'y')

%Space visualization
fill(BodyR(1,:),BodyR(2,:),[.3 .3 .3])
fill(WingRB(1,:),WingRB(2,:),[.3 .3 .3])
fill(WingRT(1,:),WingRT(2,:),[.7 .7 .7])
fill(BlueR(1,:),BlueR(2,:),'b')

axis([-187 187 -134 241])

fill(Boom(1,:),Boom(2,:),[.1 1 1])
fill(BoomL(1,:),BoomL(2,:),[.1 1 1])
fill(EWingLT(1,:),EWingLT(2,:),[.7 .7 .7])
hold on
fill(EWingLB(1,:),EWingLB(2,:),[.3 .3 .3])
fill(EBlueL(1,:),EBlueL(2,:),'r')
fill(EBodyL(1,:),EBodyL(2,:),[.7 .7 .7])
fill(EEngine(1,:),EEngine(2,:),'y')

%Space visualization
fill(EBodyR(1,:),EBodyR(2,:),[.3 .3 .3])
fill(EWingRB(1,:),EWingRB(2,:),[.3 .3 .3])
fill(EWingRT(1,:),EWingRT(2,:),[.7 .7 .7])
fill(EBlueR(1,:),EBlueR(2,:),'r')
axis([-187 187 -134 241])
hold off
    pause(0.001)
end

%% Detemine whether the ememy gets hit with the laser.
    if EMovement >= -10 && EMovement <= 10 %If enemy is within range when shot, explosion
    Explosion=[1,0.389971164872730,0.900968867902419,0.312732592987212,0.623489801858734,0.173553495647023,0.222520933956314,2.77555756156289e-17,-0.222520933956314,-0.173553495647023,-0.623489801858734,-0.312732592987212,-0.900968867902419,-0.389971164872729,-1,-0.389971164872730,-0.900968867902419,-0.312732592987212,-0.623489801858734,-0.173553495647023,-0.222520933956314,1.66533453693773e-16,0.222520933956314,0.173553495647023,0.623489801858733,0.312732592987212,0.900968867902419,0.389971164872729;0,0.0890083735825258,0.433883739117558,0.249395920743493,0.781831482468030,0.360387547160968,0.974927912181824,0.400000000000000,0.974927912181824,0.360387547160968,0.781831482468030,0.249395920743493,0.433883739117558,0.0890083735825258,0,-0.0890083735825258,-0.433883739117558,-0.249395920743493,-0.781831482468030,-0.360387547160968,-0.974927912181824,-0.400000000000000,-0.974927912181824,-0.360387547160968,-0.781831482468030,-0.249395920743494,-0.433883739117558,-0.0890083735825258;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]*20;
fill(Explosion(1,:),Explosion(2,:),'r')
axis([-187 187 -134 241])

%Part that the enemy's ship explodes
for t= 1:10:1000
    Explosion=[1,0.389971164872730,0.900968867902419,0.312732592987212,0.623489801858734,0.173553495647023,0.222520933956314,2.77555756156289e-17,-0.222520933956314,-0.173553495647023,-0.623489801858734,-0.312732592987212,-0.900968867902419,-0.389971164872729,-1,-0.389971164872730,-0.900968867902419,-0.312732592987212,-0.623489801858734,-0.173553495647023,-0.222520933956314,1.66533453693773e-16,0.222520933956314,0.173553495647023,0.623489801858733,0.312732592987212,0.900968867902419,0.389971164872729;0,0.0890083735825258,0.433883739117558,0.249395920743493,0.781831482468030,0.360387547160968,0.974927912181824,0.400000000000000,0.974927912181824,0.360387547160968,0.781831482468030,0.249395920743493,0.433883739117558,0.0890083735825258,0,-0.0890083735825258,-0.433883739117558,-0.249395920743493,-0.781831482468030,-0.360387547160968,-0.974927912181824,-0.400000000000000,-0.974927912181824,-0.360387547160968,-0.781831482468030,-0.249395920743494,-0.433883739117558,-0.0890083735825258;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]*20;
    Explosion2=Explosion;
    Deg=t;
    Deg2=-t;
    Rotate=[cosd(Deg) -sind(Deg) 0;sind(Deg) cosd(Deg) 0;0 0 1];
    Rotate2=[cosd(Deg2) -sind(Deg2) 0;sind(Deg2) cosd(Deg2) 0;0 0 1];
    Explosion=Rotate*Explosion;
    Explosion=Explosion*(sind(t)+1.5);
    Explosion2=Rotate2*Explosion2;
    Explosion2=Explosion2*(cosd(t)+1.5);
    fill(Explosion2(1,:),Explosion2(2,:),'y')
    hold on
    img=imread('Starfoxbackground.png');
    image([-186 186],[-133 240],img);
    hold on
    set(gca,'YDir','normal')
    fill(Explosion(1,:),Explosion(2,:),'r')
    fill(WingLT(1,:),WingLT(2,:),[.7 .7 .7])
hold on
fill(WingLB(1,:),WingLB(2,:),[.3 .3 .3])
fill(BlueL(1,:),BlueL(2,:),'b')
fill(BodyL(1,:),BodyL(2,:),[.7 .7 .7])
fill(Engine(1,:),Engine(2,:),'y')

%Space visualization
fill(BodyR(1,:),BodyR(2,:),[.3 .3 .3])
fill(WingRB(1,:),WingRB(2,:),[.3 .3 .3])
fill(WingRT(1,:),WingRT(2,:),[.7 .7 .7])
fill(BlueR(1,:),BlueR(2,:),'b')
axis([-187 187 -134 241])
    hold off
axis([-187 187 -134 241])
%Make the enemy spaceship gone.
EnemyExist = 0;
pause(0.001)
end
        EnemyExist =0;
        mbox = msgbox('Great! You ready to play again? Hit Restart Button'); uiwait(mbox);
    end
end
end %End of while Loop


% --- Executes on button press in calibrate.
function calibrate_Callback(hObject, eventdata, handles)
% hObject    handle to calibrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
mbox = msgbox('The Z direction should be pointing up.'); uiwait(mbox);
if (~exist('calCo','var'))
    handles.calCo=calibrate(handles.accelerometer.s);
end
handles.output=hObject;
guidata(hObject,handles);
mbox = msgbox('Now Calibrated, In order to shoot, snap your wrist. Now hit Start button and Enjoy it'); uiwait(mbox);

% --- Executes on button press in serials.
function serials_Callback(hObject, eventdata, handles)
% hObject    handle to serials (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.comPort='/dev/tty.usbmodem1411';
if (~exist('handles.serialFlag','var'))
    [handles.accelerometer.s,handles.serialFlag]=setupSerial(handles.comPort);
end
handles.output=hObject;
guidata(hObject,handles);

% --- Executes on button press in serialclose.
function serialclose_Callback(hObject, eventdata, handles)
% hObject    handle to serialclose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
mbox = msgbox('Thank you for playing! Bye~'); uiwait(mbox);
closeSerial;


% --- Executes on button press in Music.
function Music_Callback(hObject, eventdata, handles)
% hObject    handle to Music (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Play the music only when the user clicks Music on button
K='Theme.mp3';
    [y,Fs]=audioread(K);
    handles.audio=audioplayer(y,Fs);
handles.MusicText=...
    get(handles.Music,'String');
if strcmp(handles.MusicText,'Music On')
    set(handles.Music,'String','Music Off')
    play(handles.audio);
   guidata(hObject,handles);
end
if strcmp(handles.MusicText,'Music Off')
    set(handles.Music,'String','Music On');
    stop(handles.audio);
    guidata(hObject,handles);
end

    
