clear; clc; close all;

% Vehicle parameters

m_s = 395;          % sprung mass [kg]
m_u = 50;           % unsprung mass [kg]

k_s = 76000;        % suspension stiffness [N/m]
k_t = 310000;       % tyre stiffness [N/m]

b_s = 3800;         % suspension damping [N s/m]
b_t = 350;          % tyre damping [N s/m]


% Actuator parameters

delta1 = 8e7;        % current-to-force gain
delta2 = 2*pi*50;      % actuator force decay bandwidth
delta3 = 1.0e7;        % relative velocity coupling

%H inf weights

s = tf('s');
Wt = 400  * (s/(2*pi*5)   + 1)/(s/(2*pi*20) + 1);   % tyre deflection
Ws = 150 * (s/(2*pi*3) + 1)/(s/(2*pi*8)  + 1);   % suspension travel
Wu = 10.0 * (s/(2*pi*8) + 1)/(s/(2*pi*50) + 1);   % current demand

%  Active 5-state model


A = [ 0        1          0             -1               0;
     -k_s/m_s   -b_s/m_s      0              b_s/m_s           1/m_s;
      0        0          0              1               0;
      k_s/m_u    b_s/m_u    -k_t/m_u      -(b_s+b_t)/m_u        -1/m_u;
      0      -delta3      0              delta3         -delta2 ];

% Control input u = i(t)
B = [0;
     0;
     0;
     0;
     delta1];

%Road disturbance input w = Zr_dot
E = [0;
     0;
    -1;
     b_t/m_u;
     0];

% Controller measurements

Cy = [1 0 0 0 0;      % x1 = Zs - Zu
      0 1 0 0 0;      % x2 = Zs_dot
      0 0 0 1 0];     % x4 = Zu_dot

Dyw = zeros(3,1);
Dyu = zeros(3,1);

nmeas = 3;
ncon  = 1;

%  Passive 4-state model

Ap = [ 0        1          0            -1;
      -k_s/m_s   -b_s/m_s      0             b_s/m_s;
       0        0          0             1;
       k_s/m_u    b_s/m_u    -k_t/m_u     -(b_s+b_t)/m_u ];

Ep = [0;
      0;
     -1;
      b_t/m_u];

P_pass = ss(Ap, Ep, eye(4), zeros(4,1));



%  Unweighted performance outputs


Cz_unw = [0 0 1 0 0;      % x3 = Zu - Zr
          1 0 0 0 0;      % x1 = Zs - Zu
          0 0 0 0 0];     % u enters through Dzu

Dzw_unw = [0;
           0;
           0];

Dzu_unw = [0;
           0;
           1];


% Generalised plant before weighting

Pbase = ss(A, ...
           [E B], ...
           [Cz_unw; Cy], ...
           [[Dzw_unw Dzu_unw]; ...
            [Dyw     Dyu    ]]);


%apply weighting

Wt_ss = ss(Wt);
Ws_ss = ss(Ws);
Wu_ss = ss(Wu);

Iy = ss(eye(nmeas));

W = blkdiag(Wt_ss, Ws_ss, Wu_ss, Iy);

P = W * Pbase;

%Hinf syn

[K,~,gamma] = hinfsyn(P,nmeas,ncon);
fprintf('H∞ gamma = %.4f\n', gamma);
fprintf('Controller order = %d\n', order(K));

n = 5;

Cxu = [eye(n);
       zeros(1,n)];

Dxu = [zeros(n,2);
       0 1];

Pxu = ss(A, ...
         [E B], ...
         [Cxu; Cy], ...
         [Dxu; [Dyw Dyu]]);

T_wxu = lft(Pxu, K, ncon, nmeas);   


%Passive comparison transfer functions

Gp_x1 = ss(Ap, Ep, [1 0 0 0], 0);    % passive w -> x1
Gp_x3 = ss(Ap, Ep, [0 0 1 0], 0);    % passive w -> x3

Ga_x1 = T_wxu(1,1);                   % active w -> x1
Ga_x3 = T_wxu(3,1);                   % active w -> x3
Ga_i  = T_wxu(6,1);                   % active w -> i(t), controller current


% Bode comparison: passive vs active

% Use the same frequency vector for all Bode plots so the resonant
% frequencies can be compared directly.
wplot = logspace(0,3,800);            % 1 to 1000 rad/s

figure;
bodemag(Gp_x1, Ga_x1, wplot);
grid on;
title('Passive vs Active: $|w\rightarrow x_1|$','Interpreter','latex','FontSize', 20);
legend({'Passive','Active'},'Interpreter','latex','Location','best');
xlabel('Frequency [rad/s]','Interpreter','latex','FontSize', 20);
ylabel('Magnitude [dB]','Interpreter','latex','FontSize', 20);

figure;
bodemag(Gp_x3, Ga_x3, wplot);
grid on;
title('Passive vs Active: $|w\rightarrow x_3|$','Interpreter','latex','FontSize', 20);
legend({'Passive','Active'},'Interpreter','latex','Location','best');
xlabel('Frequency [rad/s]','Interpreter','latex','FontSize', 20);
ylabel('Magnitude [dB]','Interpreter','latex','FontSize', 20);

figure;
bodemag(Ga_i, wplot);
grid on;
title('Active: $|w\rightarrow i(t)|$','Interpreter','latex','FontSize', 20);
legend({'Active current'},'Interpreter','latex','Location','best');
xlabel('Frequency [rad/s]','Interpreter','latex','FontSize', 20);
ylabel('Magnitude [dB]','Interpreter','latex','FontSize', 20);




run_test = @(name,tvec,Zr,w) local_run_test( ...
    name,tvec,Zr,w,T_wxu,P_pass);

dt = 1e-3;


%% Test 1: continuous sine road undulation

A1 = 0.004;        % road amplitude [m]
f1 = 1.5;          % frequency [Hz]
Tend1 = 3.0;

t1 = (0:dt:Tend1)';

Zr1 = A1*sin(2*pi*f1*t1);
w1  = 2*pi*f1*A1*cos(2*pi*f1*t1);
w1  = w1 - mean(w1);

run_test('Continuous sine undulation', t1, Zr1, w1);


%% Test 2: single kerb sine event

A2 = 0.03;         % kerb amplitude [m]
f2 = 10.0;         % kerb frequency [Hz]
t0 = 1.0;
T_half = 1/f2;

Tend2 = 3.0;
t2 = (0:dt:Tend2)';

Zr2 = zeros(size(t2));

idx = (t2 >= t0) & (t2 <= t0 + T_half);
tau = t2(idx) - t0;

Zr2(idx) = A2*sin(pi*tau/T_half);     % half-sine kerb

w2 = gradient(Zr2, dt);
w2 = w2 - mean(w2);

run_test('Kerb half-sine event', t2, Zr2, w2);
%% Settling time calculation for half-sine kerb test

% Re-simulate kerb responses so they are available in the main script
xu_act_kerb = lsim(T_wxu, w2, t2);
xp_pas_kerb = lsim(P_pass, w2, t2);

x1_act_kerb = xu_act_kerb(:,1);   % active x1 = Zs - Zu
x3_act_kerb = xu_act_kerb(:,3);   % active x3 = Zu - Zr

x1_pas_kerb = xp_pas_kerb(:,1);   % passive x1 = Zs - Zu
x3_pas_kerb = xp_pas_kerb(:,3);   % passive x3 = Zu - Zr

% Kerb event end time
kerb_end = t0 + T_half;

% Settling bands about zero
band_x1 = 0.0005;   % [m]
band_x3 = 0.0005;   % [m]

% Settling times measured from end of kerb event
Ts_x1_passive = localSettlingTimeAbs(t2, x1_pas_kerb, kerb_end, band_x1);
Ts_x1_active  = localSettlingTimeAbs(t2, x1_act_kerb, kerb_end, band_x1);

Ts_x3_passive = localSettlingTimeAbs(t2, x3_pas_kerb, kerb_end, band_x3);
Ts_x3_active  = localSettlingTimeAbs(t2, x3_act_kerb, kerb_end, band_x3);

fprintf('\n[Half-sine kerb settling times]\n');
fprintf('x1 passive settling time = %.4f s\n', Ts_x1_passive);
fprintf('x1 active  settling time = %.4f s\n', Ts_x1_active);
fprintf('x3 passive settling time = %.4f s\n', Ts_x3_passive);
fprintf('x3 active  settling time = %.4f s\n', Ts_x3_active);


% Test 3: mixed undulation and kerb event

As = 0.004;      
fs = 1.5;       

Ak = 0.03;         
fk = 10.0;        
t0k = 2.0;

Tkerb = 1/(2*fk);  

Tend3 = 6.0;
t3 = (0:dt:Tend3)';

Zr3 = As*sin(2*pi*fs*t3);

idxk = (t3 >= t0k) & (t3 <= t0k + Tkerb);
tauk = t3(idxk) - t0k;

%Half-sine kerb bump
Zr3(idxk) = Zr3(idxk) + Ak*sin(pi*tauk/Tkerb);

w3 = gradient(Zr3, dt);
w3 = w3 - mean(w3);

run_test('Mixed undulation and half-sine kerb event', t3, Zr3, w3);

%Weighting bode

figure;
bodemag(Wt, Ws, Wu);
grid on;
title('Weights: $W_t$, $W_s$, $W_u$','Interpreter','latex','FontSize', 20);
legend({'$W_t$','$W_s$','$W_u$'}, ...
       'Interpreter','latex','Location','best');
xlabel('Frequency [rad/s]','Interpreter','latex','FontSize', 20);
ylabel('Magnitude [dB]','Interpreter','latex','FontSize', 20);


%  LOCAL FUNCTION: RUN TIME TEST

function local_run_test(name,tvec,Zr,w,T_wxu,P_pass)

    if numel(tvec) < 2
        error('tvec must contain at least 2 samples.');
    end

    dt = tvec(2) - tvec(1);

    if dt <= 0
        error('Invalid time step.');
    end

    %Active: w -> [x;u]

    xu_act = lsim(T_wxu, w, tvec);

    x_act = xu_act(:,1:5);
    u_act = xu_act(:,6);

    x1_act = x_act(:,1);      % Zs - Zu
    x2_act = x_act(:,2);      % Zs_dot
    x3_act = x_act(:,3);      % Zu - Zr
    x4_act = x_act(:,4);      % Zu_dot
    Fa_act = x_act(:,5);      % actuator force

    %Passive 

    xp_pas = lsim(P_pass, w, tvec);

    x1_pas = xp_pas(:,1);     % Zs - Zu
    x3_pas = xp_pas(:,3);     % Zu - Zr

    %RMS 

    rms_val = @(x) sqrt(mean(x.^2));

    %Road

    figure;
    plot(tvec, Zr, 'LineWidth',2.0);
    grid on;
    xlabel('Time [s]','Interpreter','latex','FontSize', 20);
    ylabel('$Z_r(t)$ [m]','Interpreter','latex','FontSize', 20);
    title([name, ': road profile $Z_r(t)$'],'Interpreter','latex','FontSize', 20);

    %Suspension travel x1

    figure;
    plot(tvec, x1_pas,'--', tvec, x1_act,'-','LineWidth',2.0);
    grid on;
    xlabel('Time [s]','Interpreter','latex','FontSize', 20);
    ylabel('$x_1=Z_s-Z_u$ [m]','Interpreter','latex','FontSize', 20);
    title([name, ': suspension travel $x_1$'],'Interpreter','latex','FontSize', 20);
    legend({'Passive','Active'},'Interpreter','latex','Location','best');

    %Tyre deflection x3

    figure;
    plot(tvec, x3_pas,'--', tvec, x3_act,'-','LineWidth',2.0);
    grid on;
    xlabel('Time [s]','Interpreter','latex','FontSize', 20);
    ylabel('$x_3=Z_u-Z_r$ [m]','Interpreter','latex','FontSize', 20);
    title([name, ': tyre deflection $x_3$'],'Interpreter','latex','FontSize', 20);
    legend({'Passive','Active'},'Interpreter','latex','Location','best');

    % Controller current demand

    figure;
    plot(tvec, u_act, 'LineWidth',2.0);
    grid on;
    xlabel('Time [s]','Interpreter','latex','FontSize', 20);
    ylabel('$i(t)$ [A]','Interpreter','latex','FontSize', 20);
    title([name, ': controller current demand'],'Interpreter','latex','FontSize', 20);

    % Actuator force

    figure;
    plot(tvec, Fa_act, 'LineWidth',2.0);
    grid on;
    xlabel('Time [s]','Interpreter','latex','FontSize', 20);
    ylabel('$F_a$ [N]','Interpreter','latex','FontSize', 20);
    title([name, ': actuator force'],'Interpreter','latex','FontSize', 20);

    %Values

    fprintf('\n[%s]\n', name);
    fprintf('RMS x1 passive = %.6g, active = %.6g\n', rms_val(x1_pas), rms_val(x1_act));
    fprintf('RMS x3 passive = %.6g, active = %.6g\n', rms_val(x3_pas), rms_val(x3_act));

    fprintf('Peak |x1| passive = %.6g, active = %.6g\n', max(abs(x1_pas)), max(abs(x1_act)));
    fprintf('Peak |x3| passive = %.6g, active = %.6g\n', max(abs(x3_pas)), max(abs(x3_act)));

    fprintf('Peak |Zs_dot| active = %.6g m/s\n', max(abs(x2_act)));
    fprintf('Peak |Zu_dot| active = %.6g m/s\n', max(abs(x4_act)));
    fprintf('Peak |u| active = %.6g A\n', max(abs(u_act)));
    fprintf('Peak |Fa| active = %.6g N\n', max(abs(Fa_act)));

end

function Ts = localSettlingTimeAbs(t, y, t_event_end, band)
    % Settling time from the end of the event.
    % The response must enter and remain inside +/- band.

    idx0 = find(t >= t_event_end, 1, 'first');

    if isempty(idx0)
        Ts = NaN;
        warning('Event end time is outside the simulation time vector.');
        return;
    end

    y_after = y(idx0:end);
    t_after = t(idx0:end);

    outside = abs(y_after) > band;

    if ~any(outside)
        Ts = 0;
        return;
    end

    lastOutsideIdx = find(outside, 1, 'last');

    if lastOutsideIdx == length(t_after)
        Ts = NaN;
        warning('Signal did not settle within the simulation time.');
    else
        Ts = t_after(lastOutsideIdx + 1) - t_event_end;
    end
end

