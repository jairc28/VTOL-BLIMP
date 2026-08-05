function ident_giro_dashboard()
% ============================================================
% ident_giro_dashboard.m — blimp_definitivo/ident_giro
% Identificación de Giro (Yaw) — Lazo Abierto + Proporcional K
% ============================================================
% PROTOCOLO ESP32:
%   MATLAB → ESP32 :
%     Modo 0 (STOP):  "0,0,0,0,0\n"
%     Modo 1 (LA):    "1,pwm,t_on_s,t_off_s,n_ciclos\n"
%     Modo 2 (LC K):  "2,sp_deg,K,mapeo,0\n"
%   ESP32  → MATLAB:
%     "millis,yaw,sp_o_pwm,e_o_fase,u_k,pwm_izq,pwm_der,omega,fase\n"
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
der_data = []; w_data   = []; fase_data = [];
t0 = [];
K_save = 1.0;

%% ── Figura ─────────────────────────────────────────────────
fig = figure('Name','Identificación Giro — Blimp','Color',C_BG,...
    'Position',[50 30 1350 760],'MenuBar','none',...
    'CloseRequestFcn',@onClose);

pCtrl = uipanel('Parent',fig,'BackgroundColor',C_PANEL,...
    'Position',[0.01 0.01 0.22 0.98],'BorderType','none');

uicontrol(pCtrl,'Style','text','String','IDENT. GIRO (YAW)','Units','normalized',...
    'Position',[0.05 0.93 0.90 0.05],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_CYAN,'FontSize',12,'FontWeight','bold');
lbStatus = uicontrol(pCtrl,'Style','text','String','⬤ CONECTADO','Units','normalized',...
    'Position',[0.05 0.87 0.90 0.05],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_GREEN,'FontSize',10,'FontWeight','bold');
lbFase = uicontrol(pCtrl,'Style','text','String','Fase: ---','Units','normalized',...
    'Position',[0.05 0.82 0.90 0.04],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_ORANGE,'FontSize',9,'FontWeight','bold');

% ── Modo 1 — Lazo Abierto ───────────────────────────────────
uicontrol(pCtrl,'Style','text','String','── LAZO ABIERTO ──','Units','normalized',...
    'Position',[0.05 0.77 0.90 0.03],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_YELLOW,'FontSize',9,'FontWeight','bold');

uicontrol(pCtrl,'Style','text','String','PWM motores:','Units','normalized',...
    'Position',[0.05 0.72 0.55 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edPWM = uicontrol(pCtrl,'Style','edit','String','80','Units','normalized',...
    'Position',[0.62 0.72 0.33 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',10);

uicontrol(pCtrl,'Style','text','String','T encendido (s):','Units','normalized',...
    'Position',[0.05 0.66 0.60 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edTon = uicontrol(pCtrl,'Style','edit','String','40','Units','normalized',...
    'Position',[0.67 0.66 0.28 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_WHITE,'FontSize',10);

uicontrol(pCtrl,'Style','text','String','T reposo (s):','Units','normalized',...
    'Position',[0.05 0.60 0.60 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edToff = uicontrol(pCtrl,'Style','edit','String','40','Units','normalized',...
    'Position',[0.67 0.60 0.28 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_WHITE,'FontSize',10);

uicontrol(pCtrl,'Style','text','String','N° ciclos:','Units','normalized',...
    'Position',[0.05 0.54 0.60 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edNciclos = uicontrol(pCtrl,'Style','edit','String','3','Units','normalized',...
    'Position',[0.67 0.54 0.28 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_WHITE,'FontSize',10);

btnLA = uicontrol(pCtrl,'Style','pushbutton','String','▶  LAZO ABIERTO','Units','normalized',...
    'Position',[0.05 0.46 0.90 0.06],'BackgroundColor',[0.4 0.2 0.0],...
    'ForegroundColor',C_WHITE,'FontSize',9,'FontWeight','bold','Callback',@doLazoAbierto);

% ── Modo 2 — Lazo Cerrado K ─────────────────────────────────
uicontrol(pCtrl,'Style','text','String','── LAZO CERRADO K ──','Units','normalized',...
    'Position',[0.05 0.40 0.90 0.03],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_CYAN,'FontSize',9,'FontWeight','bold');

uicontrol(pCtrl,'Style','text','String','Setpoint (°):','Units','normalized',...
    'Position',[0.05 0.35 0.55 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edSP = uicontrol(pCtrl,'Style','edit','String','90','Units','normalized',...
    'Position',[0.62 0.35 0.33 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',10);

uicontrol(pCtrl,'Style','text','String','Ganancia K:','Units','normalized',...
    'Position',[0.05 0.29 0.55 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edK = uicontrol(pCtrl,'Style','edit','String','-0.5','Units','normalized',...
    'Position',[0.62 0.29 0.33 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',10);

% Botones preset K (-0.5 y -0.7)
uicontrol(pCtrl,'Style','pushbutton','String','K = -0.5','Units','normalized',...
    'Position',[0.05 0.23 0.43 0.04],'BackgroundColor',[0.18 0.25 0.35],...
    'ForegroundColor',C_WHITE,'FontSize',8,'FontWeight','bold','Callback',@(~,~) set(edK,'String','-0.5'));
uicontrol(pCtrl,'Style','pushbutton','String','K = -0.7','Units','normalized',...
    'Position',[0.52 0.23 0.43 0.04],'BackgroundColor',[0.18 0.25 0.35],...
    'ForegroundColor',C_WHITE,'FontSize',8,'FontWeight','bold','Callback',@(~,~) set(edK,'String','-0.7'));

btnLC = uicontrol(pCtrl,'Style','pushbutton','String','▶  LAZO CERRADO','Units','normalized',...
    'Position',[0.05 0.14 0.90 0.06],'BackgroundColor',[0.1 0.35 0.55],...
    'ForegroundColor',C_WHITE,'FontSize',9,'FontWeight','bold','Callback',@doLazoCerrado);

% STOP
btnStop = uicontrol(pCtrl,'Style','pushbutton','String','■  STOP','Units','normalized',...
    'Position',[0.05 0.05 0.90 0.07],'BackgroundColor',[0.45 0.1 0.1],...
    'ForegroundColor',C_WHITE,'FontSize',10,'FontWeight','bold','Callback',@doStop);

% ── Axes ────────────────────────────────────────────────────
ax1 = axes('Parent',fig,'Position',[0.25 0.68 0.73 0.28],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax1,'on'); hold(ax1,'on');
hSP  = plot(ax1,NaN,NaN,'--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Setpoint/PWM_ref');
hYaw = plot(ax1,NaN,NaN,'-','Color',C_CYAN,'LineWidth',2.0,'DisplayName','Yaw');
ylabel(ax1,'Yaw (°)','Color','w');
title(ax1,'Ángulo de Giro','Color','w','FontWeight','bold');
legend(ax1,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

ax2 = axes('Parent',fig,'Position',[0.25 0.38 0.73 0.25],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax2,'on'); hold(ax2,'on');
hOmega = plot(ax2,NaN,NaN,'-','Color',C_PURPLE,'LineWidth',1.8,'DisplayName','ω (°/s)');
ylabel(ax2,'ω (°/s)','Color','w'); title(ax2,'Velocidad Angular','Color','w','FontWeight','bold');
legend(ax2,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

ax3 = axes('Parent',fig,'Position',[0.25 0.06 0.73 0.25],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax3,'on'); hold(ax3,'on');
hIzq = stairs(ax3,NaN,NaN,'-','Color',C_GREEN,'LineWidth',2,'DisplayName','PWM Izq (→+)');
hDer = stairs(ax3,NaN,NaN,'-','Color',[0.8 0.4 1.0],'LineWidth',2,'DisplayName','PWM Der (→-)');
xlabel(ax3,'Tiempo (s)','Color','w'); ylabel(ax3,'PWM','Color','w');
title(ax3,'PWM Motores','Color','w','FontWeight','bold');
legend(ax3,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
linkaxes([ax1 ax2 ax3],'x');

%% ── Estado modo ─────────────────────────────────────────────
modo_val = 0;

%% ── Callbacks ───────────────────────────────────────────────
    function toggleMapeo(~,~)
        mapeo_val = 1 - mapeo_val;
        if mapeo_val
            set(btnMapeo,'String','MAPEO: INTERPOLADO','BackgroundColor',[0.1 0.35 0.5]);
        else
            set(btnMapeo,'String','MAPEO: DIRECTO','BackgroundColor',[0.2 0.2 0.25]);
        end
    end
    function doLazoAbierto(~,~)
        modo_val = 1;
        try; flush(t,'input'); catch; end
        pause(0.15); % Pequeña pausa para sincronizar y comenzar limpia la toma desde 0s
        t_data = []; yaw_data = []; sp_data = []; ek_data = []; uk_data = [];
        izq_data = []; der_data = []; w_data = []; fase_data = []; t0 = [];
        set(lbStatus,'String','⬤ LAZO ABIERTO','ForegroundColor',C_ORANGE);
    end
    function doLazoCerrado(~,~)
        modo_val = 2;
        try; flush(t,'input'); catch; end
        pause(0.15); % Pequeña pausa para sincronizar y comenzar limpia la toma desde 0s
        t_data = []; yaw_data = []; sp_data = []; ek_data = []; uk_data = [];
        izq_data = []; der_data = []; w_data = []; fase_data = []; t0 = [];
        set(lbStatus,'String','⬤ LAZO CERRADO K','ForegroundColor',C_CYAN);
    end
    function doStop(~,~)
        modo_val = 0;
        set(lbStatus,'String','⬤ DETENIDO','ForegroundColor',C_RED);
        set(lbFase,'String','Fase: ---');
    end
    function onClose(~,~)
        delete(fig);
    end

%% ── Nombres de fase ─────────────────────────────────────────
faseNombres = {'IZQ ON','REPOSO','DER ON','REPOSO','---','---','---','---','---','DONE'};

%% ── Bucle principal ─────────────────────────────────────────
try
  while isvalid(fig)
    pwm_v = str2double(edPWM.String);    if isnan(pwm_v),    pwm_v    = 50;   end
    ton_v = str2double(edTon.String);    if isnan(ton_v),    ton_v    = 40;   end
    toff_v= str2double(edToff.String);   if isnan(toff_v),   toff_v   = 40;   end
    nc_v  = str2double(edNciclos.String);if isnan(nc_v),     nc_v     = 3;    end
    sp_v  = str2double(edSP.String);     if isnan(sp_v),     sp_v     = 90;   end
    K_v   = str2double(edK.String);      if isnan(K_v),      K_v      = -0.5; end
    K_save = K_v;

    % Construir comando según modo
    if modo_val == 1
        cmd = sprintf('1,%.1f,%.1f,%.1f,%.0f', pwm_v, ton_v, toff_v, nc_v);
    elseif modo_val == 2
        cmd = sprintf('2,%.2f,%.4f,1,0', sp_v, K_v);
    else
        cmd = '0,0,0,0,0';
    end
    
    if ~exist('last_cmd','var'), last_cmd = ''; end
    if ~exist('cmd_counter','var'), cmd_counter = 0; end
    cmd_counter = cmd_counter + 1;
    
    if ~strcmp(cmd, last_cmd) || cmd_counter > 10
        try; writeline(t, cmd); last_cmd = cmd; cmd_counter = 0; catch; end
    end

    try
      line = strtrim(readline(t));
      vals = sscanf(line, '%f,', inf);
      if numel(vals) >= 8
        t_ms  = vals(1);
        yaw   = vals(2);
        sp_v2 = vals(3);
        e_f   = vals(4);
        u_k   = vals(5);
        izq   = vals(6);
        der   = vals(7);
        omega = vals(8);
        fase  = vals(9);

        if isempty(t0); t0 = t_ms; end
        t_s = (t_ms - t0) / 1000.0;

        t_data   = [t_data,   t_s];
        yaw_data = [yaw_data, yaw];
        sp_data  = [sp_data,  sp_v2];
        ek_data  = [ek_data,  e_f];
        uk_data  = [uk_data,  u_k];
        izq_data = [izq_data, izq];
        der_data = [der_data, der];
        w_data   = [w_data,   omega];
        fase_data= [fase_data,fase];

        set(hSP,   'XData',t_data,'YData',sp_data);
        set(hYaw,  'XData',t_data,'YData',yaw_data);
        set(hOmega,'XData',t_data,'YData',w_data);
        set(hIzq,  'XData',t_data,'YData',izq_data);
        set(hDer,  'XData',t_data,'YData',-der_data);  % negativo para visualización

        if numel(t_data) > 1
          xlim(ax1,[max(0,t_s-180) max(1,t_s)]);
          xlim(ax2,[max(0,t_s-180) max(1,t_s)]);
          xlim(ax3,[max(0,t_s-180) max(1,t_s)]);
        end

        % Etiqueta de fase
        if modo_val == 1
          fi = min(max(floor(fase)+1,1),numel(faseNombres));
          if fase >= 99
            set(lbFase,'String','✔ SECUENCIA COMPLETA','ForegroundColor',C_GREEN);
          else
            set(lbFase,'String',sprintf('Fase: %s  Yaw=%.1f°',faseNombres{fi},yaw));
          end
        elseif modo_val == 2
          set(lbFase,'String',sprintf('e=%.1f°  u=%.1f',e_f,u_k));
        end
      end
    catch; end

    drawnow limitrate;
  end
catch ME
  fprintf('[ERROR] %s\n', ME.message);
end

%% ── Guardado robusto ────────────────────────────────────────
try; writeline(t,'0,0,0,0,0'); pause(0.05); clear t; catch; end

n = numel(t_data);
if n > 0
    ts_str  = datestr(now,'yyyymmdd_HHMMSS');
    carpeta = sprintf('resultados_ident_giro_%s', ts_str);
    if ~exist(carpeta,'dir'); mkdir(carpeta); end

    try
        fname_c = fullfile(carpeta,sprintf('ident_giro_%s.csv',ts_str));
        T_out = table(t_data',yaw_data',sp_data',ek_data',uk_data',izq_data',der_data',w_data',fase_data',...
            'VariableNames',{'tiempo_s','yaw_deg','sp_o_pwm','e_o_fase','u_k',...
                             'pwm_izq','pwm_der','omega_dps','fase'});
        writetable(T_out,fname_c);
        fprintf('[SAVE] CSV: %s\n',fname_c);
    catch ME_c; fprintf('[SAVE] Error CSV: %s\n',ME_c.message); end

    try
        fname_m = fullfile(carpeta,sprintf('ident_giro_%s.mat',ts_str));
        T_ms = 50;
        save(fname_m,'t_data','yaw_data','sp_data','ek_data','uk_data',...
             'izq_data','der_data','w_data','fase_data','K_save','T_ms');
        fprintf('[SAVE] MAT: %s\n',fname_m);
    catch ME_m; fprintf('[SAVE] Error MAT: %s\n',ME_m.message); end

    try
        fig_r = figure('Name','Resultados Ident Giro','Color',C_BG,'Position',[80 60 1150 740]);

        ax_r1 = subplot(3,1,1);
        set(ax_r1,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r1,'on'); hold(ax_r1,'on');
        plot(ax_r1,t_data,sp_data,'--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Ref/PWM_ref');
        plot(ax_r1,t_data,yaw_data,'-','Color',C_CYAN,'LineWidth',2,'DisplayName','Yaw (°)');
        ylabel(ax_r1,'Yaw (°)','Color','w');
        title(ax_r1,sprintf('Ident Giro | K=%.4f | %d muestras | %.1f s',K_save,n,t_data(end)),...
              'Color','w','FontWeight','bold');
        legend(ax_r1,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

        ax_r2 = subplot(3,1,2);
        set(ax_r2,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r2,'on'); hold(ax_r2,'on');
        plot(ax_r2,t_data,w_data,'-','Color',C_PURPLE,'LineWidth',1.8,'DisplayName','ω (°/s)');
        ylabel(ax_r2,'ω (°/s)','Color','w'); title(ax_r2,'Velocidad Angular','Color','w','FontWeight','bold');
        legend(ax_r2,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

        ax_r3 = subplot(3,1,3);
        set(ax_r3,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r3,'on'); hold(ax_r3,'on');
        stairs(ax_r3,t_data,izq_data,'-','Color',C_GREEN,'LineWidth',2,'DisplayName','PWM Izq');
        stairs(ax_r3,t_data,-der_data,'-','Color',[0.8 0.4 1.0],'LineWidth',2,'DisplayName','-PWM Der');
        xlabel(ax_r3,'Tiempo (s)','Color','w'); ylabel(ax_r3,'PWM','Color','w');
        title(ax_r3,'Señales de Actuación','Color','w','FontWeight','bold');
        legend(ax_r3,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
        linkaxes([ax_r1 ax_r2 ax_r3],'x'); set(fig_r,'Color',C_BG);

        fname_p = fullfile(carpeta,sprintf('ident_giro_%s.png',ts_str));
        try; exportgraphics(fig_r,fname_p,'Resolution',180,'BackgroundColor',C_BG);
        catch; saveas(fig_r,fname_p); end
        fprintf('[SAVE] PNG: %s\n',fname_p);
    catch ME_f; fprintf('[SAVE] Error figura: %s\n',ME_f.message); end

    fprintf('[SAVE] Guardado en: %s  (%d muestras | %.1f s)\n',carpeta,n,t_data(end));
else
    disp('[SAVE] Sin datos para guardar.');
end
end