function control_pid_giro_dashboard()
% ============================================================
% control_pid_giro_dashboard.m — blimp_definitivo/control_giro
% Control PID de Giro (Yaw) | Blimp autónomo
% ============================================================
% PROTOCOLO ESP32:
%   MATLAB → ESP32 : "modo,sp_deg,K,K1,Kd,mapeo\n"
%                     modo: 0=STOP 1=PID 2=Prop 3=LazoAb
%   ESP32  → MATLAB: "millis,yaw,sp,e,u_k,pwm_izq,pwm_der,omega,mapeo\n"
%   Modo 3: MATLAB envía "3,pwm,t_pulso,t_descanso,0,0\n"
% ============================================================

%% ── TCP ────────────────────────────────────────────────────
ESP32_IP   = '172.20.10.6';
ESP32_PORT = 80;

fprintf('[TCP] Conectando a %s:%d ...\n', ESP32_IP, ESP32_PORT);
try
    t = tcpclient(ESP32_IP, ESP32_PORT);
    t.Timeout = 5;
    configureTerminator(t, 'LF');
    t.Timeout = 0.15;
    fprintf('[TCP] Conectado!\n');
catch ME
    error('[TCP] No se pudo conectar: %s', ME.message);
end

%% ── Colores ────────────────────────────────────────────────
C_BG    = [0.06 0.06 0.08];
C_PANEL = [0.10 0.10 0.14];
C_CYAN  = [0.0  0.85 1.0];
C_YELLOW= [1.0  0.85 0.0];
C_GREEN = [0.2  0.9  0.4];
C_ORANGE= [1.0  0.5  0.1];
C_PURPLE= [0.7  0.3  1.0];
C_RED   = [1.0  0.3  0.3];
C_WHITE = [0.92 0.92 0.95];

%% ── Datos ──────────────────────────────────────────────────
t_data   = []; yaw_data = []; sp_data  = [];
ek_data  = []; uk_data  = []; izq_data = [];
der_data = []; w_data   = [];
t0 = [];
K_save = 2.50; K1_save = 4.94; Kd_save = 2.44;

%% ── Figura ─────────────────────────────────────────────────
fig = figure('Name','Control PID Giro — Blimp','Color',C_BG,...
    'Position',[60 40 1300 740],'MenuBar','none',...
    'CloseRequestFcn',@onClose);

% Panel izquierdo
pCtrl = uipanel('Parent',fig,'BackgroundColor',C_PANEL,...
    'Position',[0.01 0.01 0.21 0.98],'BorderType','none');

uicontrol(pCtrl,'Style','text','String','CONTROL GIRO (YAW)','Units','normalized',...
    'Position',[0.05 0.93 0.90 0.05],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_CYAN,'FontSize',11,'FontWeight','bold');
lbStatus = uicontrol(pCtrl,'Style','text','String','⬤ CONECTADO','Units','normalized',...
    'Position',[0.05 0.87 0.90 0.05],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_GREEN,'FontSize',10,'FontWeight','bold');

% Setpoint giro
uicontrol(pCtrl,'Style','text','String','SETPOINT (deg):','Units','normalized',...
    'Position',[0.05 0.81 0.60 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edSP = uicontrol(pCtrl,'Style','edit','String','0','Units','normalized',...
    'Position',[0.65 0.81 0.30 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',10);

% K, K1, Kd
uicontrol(pCtrl,'Style','text','String','K:','Units','normalized',...
    'Position',[0.05 0.75 0.30 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edK = uicontrol(pCtrl,'Style','edit','String','2.50','Units','normalized',...
    'Position',[0.38 0.75 0.57 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_WHITE,'FontSize',9);

uicontrol(pCtrl,'Style','text','String','K1:','Units','normalized',...
    'Position',[0.05 0.69 0.30 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edK1 = uicontrol(pCtrl,'Style','edit','String','4.94','Units','normalized',...
    'Position',[0.38 0.69 0.57 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_WHITE,'FontSize',9);

uicontrol(pCtrl,'Style','text','String','Kd:','Units','normalized',...
    'Position',[0.05 0.63 0.30 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edKd = uicontrol(pCtrl,'Style','edit','String','2.44','Units','normalized',...
    'Position',[0.38 0.63 0.57 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_WHITE,'FontSize',9);

% Botones de modo
modo_val = 0;
btnPID  = uicontrol(pCtrl,'Style','pushbutton','String','▶ INICIAR PID','Units','normalized',...
    'Position',[0.05 0.25 0.90 0.08],'BackgroundColor',[0.1 0.45 0.1],...
    'ForegroundColor',C_WHITE,'FontSize',10,'FontWeight','bold','Callback',@(~,~)setModo(1));
btnStop = uicontrol(pCtrl,'Style','pushbutton','String','■ PARAR','Units','normalized',...
    'Position',[0.05 0.15 0.90 0.08],'BackgroundColor',[0.45 0.1 0.1],...
    'ForegroundColor',C_WHITE,'FontSize',10,'FontWeight','bold','Callback',@(~,~)setModo(0));

uicontrol(pCtrl,'Style','text','String','T=50ms | MPU6050','Units','normalized',...
    'Position',[0.05 0.02 0.90 0.06],'BackgroundColor',C_PANEL,...
    'ForegroundColor',[0.5 0.5 0.6],'FontSize',8);

% Axes
ax1 = axes('Parent',fig,'Position',[0.24 0.68 0.74 0.28],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax1,'on'); hold(ax1,'on');
hSP  = plot(ax1,NaN,NaN,'--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Setpoint');
hYaw = plot(ax1,NaN,NaN,'-','Color',C_CYAN,'LineWidth',2,'DisplayName','Yaw');
ylabel(ax1,'Yaw (°)','Color','w');
title(ax1,'Ángulo de Giro vs Setpoint','Color','w','FontWeight','bold');
legend(ax1,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

ax2 = axes('Parent',fig,'Position',[0.24 0.38 0.74 0.25],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax2,'on'); hold(ax2,'on');
hOmega = plot(ax2,NaN,NaN,'-','Color',C_PURPLE,'LineWidth',1.8,'DisplayName','ω (°/s)');
ylabel(ax2,'ω (°/s)','Color','w'); title(ax2,'Velocidad Angular','Color','w','FontWeight','bold');
legend(ax2,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

ax3 = axes('Parent',fig,'Position',[0.24 0.06 0.74 0.25],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax3,'on'); hold(ax3,'on');
hUk  = plot(ax3,NaN,NaN,'-','Color',C_ORANGE,'LineWidth',2,'DisplayName','u[k]');
hIzq = stairs(ax3,NaN,NaN,'-','Color',C_GREEN,'LineWidth',1.4,'DisplayName','PWM Izq');
hDer = stairs(ax3,NaN,NaN,'-','Color',[0.8 0.4 1.0],'LineWidth',1.4,'DisplayName','PWM Der');
xlabel(ax3,'Tiempo (s)','Color','w'); ylabel(ax3,'Amplitud','Color','w');
title(ax3,'Acción de Control','Color','w','FontWeight','bold');
legend(ax3,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
linkaxes([ax1 ax2 ax3],'x');

%% ── Callbacks ──────────────────────────────────────────────


    function setModo(m)
        modo_val = m;
        if m > 0
            try; flush(t,'input'); catch; end
            pause(0.15); % Pequeña pausa para sincronización e inicio limpio a t=0s
            t_data = []; yaw_data = []; sp_data = []; ek_data = [];
            uk_data = []; izq_data = []; der_data = []; w_data = []; t0 = [];
        end
        labels = {'⬤ DETENIDO','⬤ PID ACTIVO'};
        cols   = {C_RED, C_CYAN};
        set(lbStatus,'String',labels{m+1},'ForegroundColor',cols{m+1});
    end

    function onClose(~,~)
        delete(fig);
    end

%% ── Bucle principal ────────────────────────────────────────
try
  while isvalid(fig)
    sp_v  = str2double(edSP.String);   if isnan(sp_v),  sp_v  = 0;    end
    K_v   = str2double(edK.String);    if isnan(K_v),   K_v   = 2.50; end
    K1_v  = str2double(edK1.String);   if isnan(K1_v),  K1_v  = 4.94; end
    Kd_v  = str2double(edKd.String);   if isnan(Kd_v),  Kd_v  = 2.44; end
    K_save = K_v; K1_save = K1_v; Kd_save = Kd_v;

    % Mapeo is forced to 0 for Yaw as requested by user previously
    cmd = sprintf('%d,%.2f,%.4f,%.4f,%.4f,0', ...
                  modo_val, sp_v, K_v, K1_v, Kd_v);
    
    if ~exist('last_cmd','var'), last_cmd = ''; end
    if ~exist('cmd_counter','var'), cmd_counter = 0; end
    cmd_counter = cmd_counter + 1;
    
    if ~strcmp(cmd, last_cmd) || cmd_counter > 10
        try; writeline(t, cmd); last_cmd = cmd; cmd_counter = 0; catch; end
    end

    try
      line = strtrim(readline(t));
      vals = sscanf(line, '%f,', inf);
      if numel(vals) >= 7
        t_ms  = vals(1); yaw = vals(2); sp = vals(3);
        e_v   = vals(4); u_k = vals(5); izq = vals(6); der = vals(7);
        omega = vals(8);

        if isempty(t0); t0 = t_ms; end
        t_s = (t_ms - t0) / 1000.0;

        t_data   = [t_data,   t_s];
        yaw_data = [yaw_data, yaw];
        sp_data  = [sp_data,  sp];
        ek_data  = [ek_data,  e_v];
        uk_data  = [uk_data,  u_k];
        izq_data = [izq_data, izq];
        der_data = [der_data, der];
        w_data   = [w_data,   omega];

        set(hSP,   'XData',t_data,'YData',sp_data);
        set(hYaw,  'XData',t_data,'YData',yaw_data);
        set(hOmega,'XData',t_data,'YData',w_data);
        set(hUk,   'XData',t_data,'YData',uk_data);
        set(hIzq,  'XData',t_data,'YData',izq_data);
        set(hDer,  'XData',t_data,'YData',der_data);

        if numel(t_data) > 1
          xlim(ax1,[max(0,t_s-120) max(1,t_s)]);
          xlim(ax2,[max(0,t_s-120) max(1,t_s)]);
          xlim(ax3,[max(0,t_s-120) max(1,t_s)]);
        end
      end
    catch; end

    drawnow limitrate;
  end
catch ME
  fprintf('[ERROR] %s\n', ME.message);
end

%% ── Guardado robusto ───────────────────────────────────────
try; writeline(t,'0,0,0,0,0,0'); pause(0.05); clear t; catch; end

n = numel(t_data);
if n > 0
    ts_str  = datestr(now,'yyyymmdd_HHMMSS');
    carpeta = sprintf('resultados_ctrl_giro_%s', ts_str);
    if ~exist(carpeta,'dir'); mkdir(carpeta); end

    try
        fname_c = fullfile(carpeta, sprintf('giro_%s.csv', ts_str));
        T_out = table(t_data',yaw_data',sp_data',ek_data',uk_data',izq_data',der_data',w_data',...
            'VariableNames',{'tiempo_s','yaw_deg','sp_deg','error_deg','u_k','pwm_izq','pwm_der','omega_dps'});
        writetable(T_out,fname_c);
        fprintf('[SAVE] CSV: %s\n',fname_c);
    catch ME_c; fprintf('[SAVE] Error CSV: %s\n',ME_c.message); end

    try
        fname_m = fullfile(carpeta, sprintf('giro_%s.mat', ts_str));
        save(fname_m,'t_data','yaw_data','sp_data','ek_data','uk_data',...
             'izq_data','der_data','w_data','K_save','K1_save','Kd_save');
        fprintf('[SAVE] MAT: %s\n',fname_m);
    catch ME_m; fprintf('[SAVE] Error MAT: %s\n',ME_m.message); end

    try
        fig_r = figure('Name','Resultados Ctrl Giro','Color',C_BG,'Position',[80 60 1100 720]);
        ax_r1 = subplot(3,1,1);
        set(ax_r1,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r1,'on'); hold(ax_r1,'on');
        plot(ax_r1,t_data,sp_data,'--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Setpoint');
        plot(ax_r1,t_data,yaw_data,'-','Color',C_CYAN,'LineWidth',2,'DisplayName','Yaw');
        ylabel(ax_r1,'Yaw (°)','Color','w');
        title(ax_r1,sprintf('Ctrl Giro | K=%.3f K1=%.3f Kd=%.3f | %d muestras',K_save,K1_save,Kd_save,n),'Color','w','FontWeight','bold');
        legend(ax_r1,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
        ax_r2 = subplot(3,1,2);
        set(ax_r2,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r2,'on'); hold(ax_r2,'on');
        plot(ax_r2,t_data,w_data,'-','Color',C_PURPLE,'LineWidth',1.8,'DisplayName','ω (°/s)');
        ylabel(ax_r2,'ω (°/s)','Color','w'); legend(ax_r2,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
        ax_r3 = subplot(3,1,3);
        set(ax_r3,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r3,'on'); hold(ax_r3,'on');
        plot(ax_r3,t_data,uk_data,'-','Color',C_ORANGE,'LineWidth',2,'DisplayName','u[k]');
        stairs(ax_r3,t_data,izq_data,'-','Color',C_GREEN,'LineWidth',1.4,'DisplayName','PWM Izq');
        stairs(ax_r3,t_data,der_data,'-','Color',[0.8 0.4 1.0],'LineWidth',1.4,'DisplayName','PWM Der');
        xlabel(ax_r3,'Tiempo (s)','Color','w'); ylabel(ax_r3,'Amplitud','Color','w');
        legend(ax_r3,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
        linkaxes([ax_r1 ax_r2 ax_r3],'x'); set(fig_r,'Color',C_BG);
        fname_p = fullfile(carpeta,sprintf('giro_%s.png',ts_str));
        try; exportgraphics(fig_r,fname_p,'Resolution',180,'BackgroundColor',C_BG);
        catch; saveas(fig_r,fname_p); end
        fprintf('[SAVE] PNG: %s\n',fname_p);
    catch ME_f; fprintf('[SAVE] Error figura: %s\n',ME_f.message); end

    fprintf('[SAVE] Guardado en: %s  (%d muestras | %.1f s)\n',carpeta,n,t_data(end));
else
    disp('[SAVE] Sin datos para guardar.');
end
end