
%Example, for implementing, easily, a discrete time model, via S-function, in Matlab; to be used from Simulink.
% You may modify it, for solving part of the problems of Project1/0.

% for MTRN4010/T1.2020
% Questions:   Ask the lecturer, via Moodle or by email (j.guivant@unw.edu.au)


% main function.
function S_Tanks(block)
    setup(block);
end
%-------------------------------------------------------
% Here we define certain characteristics of the block. 
% We also install certain useful and necessary callback functions.

function setup(block)
 % We register the number of ports. (input and output ports)
  block.NumInputPorts  = 1;             % one input u1(k)
  block.NumOutputPorts = 1;           % one output y1(k)
  % in this case, our block has one input line and 1 ouput line.
  % We will set the dimensionality of these, later.
     
  % Set up the port properties to be inherited or dynamic (keep this default way).
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % more details....  
  % u(t), are real numbers
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real'; %in our case, must be REAL, not COMPLEX
    
  % y(t), are real numbers 
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';

  % Register the parameters (offered in a Dialog box, we see those soon, when we run the model).
  block.NumDialogPrms     = 2;  % how many parameters?   (one for initial conditions X0, and the 2nd one for the valves' coefficients)
  
  % {'Tunable','Nontunable','SimOnlyTunable'}; //options
  block.DialogPrmsTunable = {'Tunable','Tunable'};
  
  % How many continuous states (analog) >
  block.NumContStates = 0;   % none, because, in this case, we run a purely discrete time simulation

  % Sample time type?
  % Matlab says this:
  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  
  % I chose this one.
  block.SampleTimes = [1 0];
  
      % -----------------------------------------------------------------
  block.SetAccelRunOnTLC(false);
  
  block.SimStateCompliance = 'DefaultSimState';
    % -----------------------------------------------------------------
  % Methods to be called during update diagram and compilation (callback functions).
  % We register those callbacks here:
  % -----------------------------------------------------------------
   block.RegBlockMethod('CheckParameters', @CheckPrms);
  %block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
  %block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
  %block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
  block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  % -----------------------------------------------------------------
  % Registestration of  methods, called at run-time
  % -----------------------------------------------------------------
  block.RegBlockMethod('ProcessParameters', @ProcessPrms);
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  block.RegBlockMethod('Start', @Start);                                        % called when the simulation just starts.
  block.RegBlockMethod('Outputs', @Outputs);                                % called each time the block's outputs need to be refreshed, i.e.  Y(k) = H( X(k) );
  block.RegBlockMethod('Update', @Update);                                  % called when the states of the block needs to be updates  (e.g. X(k+1)= F (X(k),u(k)) )
  %block.RegBlockMethod('Derivatives', @Derivatives);                   % for  analog systems, (not our case, which is time discrete)
  %block.RegBlockMethod('Projection', @Projection);
  block.RegBlockMethod('SimStatusChange', @SimStatusChange);   %called for events such as pause/contunue
  block.RegBlockMethod('Terminate', @Terminate);                           %called for termination event 
  block.RegBlockMethod('GetSimState', @GetSimState);                    %called if Simulink needs to know the current state of the block
  block.RegBlockMethod('SetSimState', @SetSimState);                    %called if Simulink needs to set the current state of the block
 % block.RegBlockMethod('WriteRTW', @WriteRTW);

 
 
 %----------------------------------------------------------------------
  % We indicate dimensionality of inputs and outputs
    % output #1
    block.InputPort(1).Dimensions = 1;  %u(k)  ; in this case I says: u is a scalar.
    block.OutputPort(1).Dimensions = [3,1];  %y(k);   %I want to output the full state X, which is 3x1
   % I do not have other outputs.
       
    dt=0; %double.
  block.InputPort(1).DataTypeID = dt;
  block.OutputPort(1).DataTypeID  = dt;
    
  block.OutputPort(1).Complexity      = 'Real'; % real
  block.InputPort(1).Complexity      = 'Real'; % real
      
  fd=0;
  
  block.InputPort(1).SamplingMode = fd; 
  block.OutputPort(1).SamplingMode  = fd;

end
% -------------------------------------------------------------------

% I defined certain parameters; if those are modified by the user, We can
% check validity here ("sanity check").
function CheckPrms(block)

  X0 = block.DialogPrm(1).Data;         % initial contitions (X(0))  (3x1)
  v = block.DialogPrm(2).Data;            % valves' coefficients (4 valves, so this one needs to be (4x1) or (1x4))      
  
  if numel(X0)~=3,                   % in this case, parameter 1 (x0) must be a vector of 3 components
                                            % if that is not the case ==> ERROR
      me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter')); throw(me);
  end;
  if numel(v)~=4,                   % in this case, parameter 2 ( valves' apertures) must be a vector of 4 components
                                            % if that is not the case ==> ERROR
      me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter')); throw(me);
  end;
  
  % they should be double precision....etc.
  if ~strcmp(class(X0), 'double')
    me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter'));   throw(me);
  end
% We may chack other parameters, if we expect those 
  %e.g. block.DialogPrm(2).Data;  
    
  fprintf('New Params#1=[%.2f][%.2f][%.2f]\n',X0);
  fprintf('New Params#2=[%.2f][%.2f][%.2f][%.2f]\n',v);
  
end

  
function ProcessPrms(block)
    block.AutoUpdateRuntimePrms;
end %--------------------------------------------



 
function SetInpPortDataType(block, idx, dt)
  block.InputPort(idx).DataTypeID = dt;
  %block.OutputPort(1).DataTypeID  = dt;
end %--------------------------------------------
  
function SetOutPortDataType(block, idx, dt)

  block.OutputPort(idx).DataTypeID  = dt;
  %block.InputPort(1).DataTypeID     = dt;
end %--------------------------------------------
  
function SetInpPortComplexSig(block, idx, c)
  
  block.InputPort(idx).Complexity = c;
  block.OutputPort(1).Complexity  = c;
end %--------------------------------------------
  
function SetOutPortComplexSig(block, idx, c)

  block.OutputPort(idx).Complexity = c;
  block.InputPort(1).Complexity    = c;
end %--------------------------------------------
  
function DoPostPropSetup(block)
  block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 3;      % my state vector,X, has dimension = 3x1 
  block.Dwork(1).DatatypeID      = 0;      % class  double
  block.Dwork(1).Complexity      = 'Real'; % real (i.e. not a complex number)
  block.Dwork(1).UsedAsDiscState = true;
   
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;
end %--------------------------------------------
  
function InitializeConditions(block)
    %block.ContStates.Data = 1;
end %--------------------------------------------
    
function Start(block)
  %block.Dwork(1).Data = [3;2;1];                    % initial conditions, X(k=0)
  block.Dwork(1).Data = block.DialogPrm(1).Data(1:3);   %I expect X(t0) in these user's parameters.
  
end %--------------------------------------------

function Outputs(block)
    block.OutputPort(1).Data = block.Dwork(1).Data(1:3)  ; 
  end %--------------------------------------------
  
  
  % this is the one called to implementing X(k+1)= F ( X(k), u(k) ..)
function Update(block)
    uu =  block.InputPort(1).Data ; 
    uu=uu(1);               % u1_1(k)
    xc = block.Dwork(1).Data;                                    % current X, i.e. X(k)
    param1 = block.DialogPrm(2).Data;
    xc = MyDiscreteModel(xc,uu,param1);                               
    block.Dwork(1).Data=xc;                                      % we set X(k+1), here
    % done
    
end %--------------------------------------------


function SimStatusChange(block, s)

  if s == 0
    disp('Pause in simulation.');
  elseif s == 1
    disp('Resuming simulation.');
  end
end %--------------------------------------------
    
function Terminate(block)
    disp('bye.....');
end %--------------------------------------------
 
function outSimState = GetSimState(block)
    outSimState = block.Dwork(1).Data;
end %--------------------------------------------

function SetSimState(block, inSimState)
    block.Dwork(1).Data = inSimState;
end %--------------------------------------------


% time derivatives, which I will use to approximate the solution via Euler,  in thise case.

% Actually, using this discrete time approximation makes sense for NON
% LINEAR cases. This case is LINEAR, so that a close form does exist, for
% obtaining the discrete time model (i.e. the transition matrix, etc).
% However, we try this way, so you can use the same approach for non-linear
% cases, such as the car's kinematic model.
% The intention of this example is showing a way to do it; so you adapt it
% for some of the problems in Project0/1.



function  dXdt  = F( X, u, pa )

    b=[1,1,1];
    dXdt=[0;0;0];
       
    
    dx=X(1)-X(2) ; q12 = pa(1)*dx;              % flow between tank1 and tank 2,  q12 (q12>0) if from Tank1 to Tank2
    dx=X(1)-X(3) ; q13 = pa(2)*dx;              % flow between tank1 and tank 3,  q13 (q13>0) if from Tank1 to Tank3
    dx=X(2)-X(3) ; q23 = pa(3)*dx;              % flow between tank2 and tank 3,  q24 (q23>0) if from Tank2 to Tank3
    q3A = pa(4)*X(3);                                    % flow from tank3,released to  to atmosphere.

    dXdt(1) = b(1)*(u(1)-q12-q13);
    dXdt(2) = b(2)*(q12-q23);
    dXdt(3) = b(3)*(q13+q23-q3A);
end
       
% X(k+1) = F ( X(k),u(k))
function x=MyDiscreteModel(xc,uc,pa)
    dt=0.05;                                        %<-------------- I assume 50ms sample time
    dxdt  =  F( xc, uc,pa ); 
    x=xc+dt*dxdt;                       %implement Euler approximation, for a dt=0.01 seconds.
end %--------------------------------------------
% In this case, I implemented it based on a Euler approximation, for a fixed
% sample time; however, it could have been a real discrete system X[k] =
% G(X[k],u[k]), not necessarily generated by an Euler approximation of a
% continuous time system.

% -------------------------------------------------
% OK done. This example can be modified for solving some problems in
% Project 1  (wrongly named "project0" in the released document.).

% -------------------------------------------------
% BTW: read Simulink documentation/help for more details.
% -------------------------------------------------
% questions:   Ask the lecturer, via Moodle or by email (j.guivant@unw.edu.au)
% -------------------------------------------------


% -------------------------------------------------
