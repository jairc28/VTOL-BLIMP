    function ident_altura_dashboard()
% ============================================================
% ident_altura_dashboard.m — blimp_definitivo/ident_altura
% Identificación de Altura — Control Proporcional K
% ============================================================
% PROTOCOLO ESP32:
%   MATLAB → ESP32 : "modo,setpoint_mm,K,mapeo\n"
%                     modo: 0=STOP  1=Proporcional
%   ESP32  → MATLAB: "millis,dist_mm,sp_mm,e_m,u_k,pwm,emb\n"
% ============================================================

%% ── TCP ────────────────────────────────────────────────────
ESP32_IP   = '172.20.10.6';
ESP32_PORT = 80;

fprintf('[TCP] Conectando a %s:%d ...\n', ESP32_IP, ESP32_PORT);
try
    t = tcpclient(ESP32_IP, ESP32_PORT);
    t.Timeout = 5;
    configureTerminator(t, 'LF');
    t.Timeout = 1.0;
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
C_RED   = [1.0  0.3  0.3];
C_WHITE = [0.92 0.92 0.95];

%% ── Datos ──────────────────────────────────────────────────
t_data   = []; alt_data = []; sp_data  = [];
ek_data  = []; uk_data  = []; pwm_data = [];
t0 = [];
K_save = 1.0;

%% ── Figura ─────────────────────────────────────────────────
fig = figure('Name','Identificación Altura — Blimp','Color',C_BG,...
    'Position',[80 60 1200 700],'MenuBar','none',...
    'CloseRequestFcn',@onClose);

pCtrl = uipanel('Parent',fig,'BackgroundColor',C_PANEL,...
    'Position',[0.01 0.01 0.22 0.98],'BorderType','none');

uicontrol(pCtrl,'Style','text','String','IDENT. ALTURA','Units','normalized',...
    'Position',[0.05 0.93 0.90 0.05],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_CYAN,'FontSize',12,'FontWeight','bold');
lbStatus = uicontrol(pCtrl,'Style','text','String','⬤ CONECTADO','Units','normalized',...
    'Position',[0.05 0.87 0.90 0.04],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_GREEN,'FontSize',10,'FontWeight','bold');

sp_val_target = 1000;

% Setpoint
uicontrol(pCtrl,'Style','text','String','SETPOINT (mm):','Units','normalized',...
    'Position',[0.05 0.81 0.90 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
lbSPval = uicontrol(pCtrl,'Style','text','String','1000 mm','Units','normalized',...
    'Position',[0.05 0.76 0.90 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_YELLOW,'FontSize',11,'FontWeight','bold');

btnSP1 = uicontrol(pCtrl,'Style','pushbutton','String','1 m (1000)','Units','normalized',...
    'Position',[0.05 0.70 0.42 0.05],'BackgroundColor',[0.2 0.3 0.4],'ForegroundColor',C_WHITE,...
    'Callback',@(~,~) setSP(1000));
btnSP2 = uicontrol(pCtrl,'Style','pushbutton','String','2 m (2000)','Units','normalized',...
    'Position',[0.52 0.70 0.43 0.05],'BackgroundColor',[0.2 0.3 0.4],'ForegroundColor',C_WHITE,...
    'Callback',@(~,~) setSP(2000));

% Ganancia K
uicontrol(pCtrl,'Style','text','String','Ganancia K:','Units','normalized',...
    'Position',[0.05 0.63 0.50 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',9);
edK = uicontrol(pCtrl,'Style','edit','String','100.0','Units','normalized',...
    'Position',[0.58 0.63 0.37 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',10);

% Offsets
uicontrol(pCtrl,'Style','text','String','Off Izq:','Units','normalized',...
    'Position',[0.05 0.57 0.30 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',8);
edOffIzq = uicontrol(pCtrl,'Style','edit','String','20','Units','normalized',...
    'Position',[0.36 0.57 0.15 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',9);

uicontrol(pCtrl,'Style','text','String','Off Der:','Units','normalized',...
    'Position',[0.55 0.57 0.30 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',C_WHITE,'FontSize',8);
edOffDer = uicontrol(pCtrl,'Style','edit','String','10','Units','normalized',...
    'Position',[0.80 0.57 0.15 0.04],'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',C_YELLOW,'FontSize',9);

% Mapeo
mapeo_val = 0;
btnMapeo = uicontrol(pCtrl,'Style','pushbutton','String','MAPEO: DIRECTO','Units','normalized',...
    'Position',[0.05 0.49 0.90 0.06],'BackgroundColor',[0.2 0.2 0.25],...
    'ForegroundColor',C_WHITE,'FontSize',9,'FontWeight','bold','Callback',@toggleMapeo);

% Info zona muerta
uicontrol(pCtrl,'Style','text','String','Deadband: ±15 mm','Units','normalized',...
    'Position',[0.05 0.43 0.90 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',[0.5 0.5 0.6],'FontSize',8);
uicontrol(pCtrl,'Style','text','String','EMA α = 0.4 | T = 500 ms','Units','normalized',...
    'Position',[0.05 0.39 0.90 0.04],'BackgroundColor',C_PANEL,'ForegroundColor',[0.5 0.5 0.6],'FontSize',8);

% Botones
modo_val = 0;
btnStart = uicontrol(pCtrl,'Style','pushbutton','String','▶  INICIAR','Units','normalized',...
    'Position',[0.05 0.29 0.90 0.08],'BackgroundColor',[0.1 0.45 0.1],...
    'ForegroundColor',C_WHITE,'FontSize',10,'FontWeight','bold','Callback',@doStart);
btnStop = uicontrol(pCtrl,'Style','pushbutton','String','■  STOP','Units','normalized',...
    'Position',[0.05 0.19 0.90 0.08],'BackgroundColor',[0.45 0.1 0.1],...
    'ForegroundColor',C_WHITE,'FontSize',10,'FontWeight','bold','Callback',@doStop);

% Medición en vivo
lbLive = uicontrol(pCtrl,'Style','text','String','--- mm','Units','normalized',...
    'Position',[0.05 0.12 0.90 0.06],'BackgroundColor',C_PANEL,...
    'ForegroundColor',C_CYAN,'FontSize',14,'FontWeight','bold');

uicontrol(pCtrl,'Style','text','String','ToF VL53L1X | Servo vectorial','Units','normalized',...
    'Position',[0.05 0.02 0.90 0.06],'BackgroundColor',C_PANEL,'ForegroundColor',[0.4 0.4 0.5],'FontSize',7);

% Axes
ax1 = axes('Parent',fig,'Position',[0.26 0.55 0.72 0.40],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax1,'on'); hold(ax1,'on');
hSP  = plot(ax1,NaN,NaN,'--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Setpoint');
hAlt = plot(ax1,NaN,NaN,'-','Color',C_CYAN,'LineWidth',2,'DisplayName','Altura');
ylabel(ax1,'Altura (mm)','Color','w');
title(ax1,'Respuesta Proporcional — Altura','Color','w','FontWeight','bold');
legend(ax1,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);

ax2 = axes('Parent',fig,'Position',[0.26 0.08 0.72 0.40],...
    'Color',[0.08 0.08 0.11],'XColor','w','YColor','w',...
    'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
grid(ax2,'on'); hold(ax2,'on');
hUk  = plot(ax2,NaN,NaN,'-','Color',C_ORANGE,'LineWidth',2,'DisplayName','u[k] = K·e');
hPWM = stairs(ax2,NaN,NaN,'-','Color',C_GREEN,'LineWidth',1.5,'DisplayName','PWM');
xlabel(ax2,'Tiempo (s)','Color','w'); ylabel(ax2,'Amplitud','Color','w');
title(ax2,'Esfuerzo de Control u[k] y PWM','Color','w','FontWeight','bold');
legend(ax2,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
linkaxes([ax1 ax2],'x');

%% ── Callbacks ──────────────────────────────────────────────
    function toggleMapeo(~,~)
        mapeo_val = 1 - mapeo_val;
        if mapeo_val
            set(btnMapeo,'String','MAPEO: INTERPOLADO','BackgroundColor',[0.1 0.35 0.5]);
        else
            set(btnMapeo,'String','MAPEO: DIRECTO','BackgroundColor',[0.2 0.2 0.25]);
        end
    end
    function setSP(val)
        sp_val_target = val;
        set(lbSPval,'String',sprintf('%.0f mm',val));
    end

    t_start_timer = uint64(0);

    function doStart(~,~)
        modo_val = 1;
        t_start_timer = tic; % Inicia timer para el tiempo muerto automático
        try; flush(t,'input'); catch; end
        pause(0.15); % Pequeña pausa para sincronizar y comenzar limpia la toma desde 0s
        t_data = []; alt_data = []; sp_data = []; ek_data = []; uk_data = []; pwm_data = []; t0 = [];
        set(lbStatus,'String','⬤ PROPORCIONAL ACTIVO','ForegroundColor',C_CYAN);
    end
    function doStop(~,~)
        modo_val = 0;
        set(lbStatus,'String','⬤ DETENIDO','ForegroundColor',C_RED);
        pause(0.2);
        delete(fig);
    end
    function onClose(~,~)
        delete(fig);
    end

%% ── Bucle principal ────────────────────────────────────────
try
  while isvalid(fig)
    if modo_val == 0
        sp_v = 0; % No enviar setpoint si está detenido
    elseif modo_val == 1 && toc(t_start_timer) < 3.0
        sp_v = 0; % Fuerzo a 0 durante los primeros 3s (Tiempo Muerto)
    else
        sp_v = sp_val_target;
    end
    
    K_v  = str2double(edK.String); if isnan(K_v), K_v = 1.0; end
    K_save = K_v;
    
    off_i = str2double(edOffIzq.String); if isnan(off_i), off_i = 10; end
    off_d = str2double(edOffDer.String); if isnan(off_d), off_d = 10; end

    cmd = sprintf('%d,%.1f,%.4f,%d,%d,%d', modo_val, sp_v, K_v, mapeo_val, round(off_i), round(off_d));
    if ~exist('last_cmd','var'), last_cmd = ''; end
    if ~exist('cmd_counter','var'), cmd_counter = 0; end
    cmd_counter = cmd_counter + 1;
    
    if ~strcmp(cmd, last_cmd) || cmd_counter > 10
        try; writeline(t, cmd); last_cmd = cmd; cmd_counter = 0; catch; end
    end
    try
      line = strtrim(readline(t));
      vals = sscanf(line, '%f,', inf);
      if numel(vals) >= 6
        t_ms   = vals(1);
        alt_mm = vals(2);
        sp_mm  = vals(3);
        e_m    = vals(4);
        u_k    = vals(5);
        pwm    = vals(6);

        if isempty(t0); t0 = t_ms; end
        t_s = (t_ms - t0) / 1000.0;

        t_data   = [t_data,   t_s];
        alt_data = [alt_data, alt_mm];
        sp_data  = [sp_data,  sp_mm];
        ek_data  = [ek_data,  e_m];
        uk_data  = [uk_data,  u_k];
        pwm_data = [pwm_data, pwm];

        try
            assignin('base', 't_data', t_data');
            assignin('base', 'alt_data', alt_data');
            assignin('base', 'sp_data', sp_data');
            assignin('base', 'uk_data', uk_data');
            assignin('base', 'pwm_data', pwm_data');
            assignin('base', 'ek_data', ek_data');
        catch; end

        set(hSP,  'XData',t_data,'YData',sp_data);
        set(hAlt, 'XData',t_data,'YData',alt_data);
        set(hUk,  'XData',t_data,'YData',uk_data);
        set(hPWM, 'XData',t_data,'YData',pwm_data);

        set(lbSPval,'String',sprintf('%.0f mm', sp_v));
        set(lbLive,'String',sprintf('%.0f mm', alt_mm));

        if numel(t_data) > 1
          xlim(ax1,[max(0,t_s-120) max(1,t_s)]);
          xlim(ax2,[max(0,t_s-120) max(1,t_s)]);
        end
      end
    catch; end

    drawnow limitrate;
  end
catch ME
  fprintf('[ERROR] %s\n', ME.message);
end

%% ── Guardado robusto ───────────────────────────────────────
try; writeline(t,sprintf('0,%.1f,%.4f,0,0,0',sp_val_target,K_save)); pause(0.05); clear t; catch; end

n = numel(t_data);
if n > 0
    ts_str  = datestr(now,'yyyymmdd_HHMMSS');
    carpeta = sprintf('resultados_ident_altura_%s', ts_str);
    if ~exist(carpeta,'dir'); mkdir(carpeta); end

    try
        fname_c = fullfile(carpeta,sprintf('ident_altura_%s.csv',ts_str));
        T_out = table(t_data',alt_data',sp_data',ek_data',uk_data',pwm_data',...
            'VariableNames',{'tiempo_s','altura_mm','setpoint_mm','error_m','u_k','pwm'});
        writetable(T_out,fname_c);
        fprintf('[SAVE] CSV: %s\n',fname_c);
    catch ME_c; fprintf('[SAVE] Error CSV: %s\n',ME_c.message); end

    try
        fname_m = fullfile(carpeta,sprintf('ident_altura_%s.mat',ts_str));
        T_S = 500;
        save(fname_m,'t_data','alt_data','sp_data','ek_data','uk_data','pwm_data','K_save','T_S');
        fprintf('[SAVE] MAT: %s\n',fname_m);
    catch ME_m; fprintf('[SAVE] Error MAT: %s\n',ME_m.message); end

    try
        fig_r = figure('Name','Resultados Ident Altura','Color',C_BG,'Position',[80 60 1100 650]);
        ax_r1 = subplot(2,1,1);
        set(ax_r1,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r1,'on'); hold(ax_r1,'on');
        plot(ax_r1,t_data,sp_data,'--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Setpoint');
        plot(ax_r1,t_data,alt_data,'-','Color',C_CYAN,'LineWidth',2,'DisplayName','Altura');
        ylabel(ax_r1,'Altura (mm)','Color','w');
        title(ax_r1,sprintf('Ident Altura Proporcional | K=%.4f | %d muestras',K_save,n),...
              'Color','w','FontWeight','bold');
        legend(ax_r1,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
        ax_r2 = subplot(2,1,2);
        set(ax_r2,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w','GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',10);
        grid(ax_r2,'on'); hold(ax_r2,'on');
        plot(ax_r2,t_data,uk_data,'-','Color',C_ORANGE,'LineWidth',2,'DisplayName','u[k]');
        stairs(ax_r2,t_data,pwm_data,'-','Color',C_GREEN,'LineWidth',1.5,'DisplayName','PWM');
        xlabel(ax_r2,'Tiempo (s)','Color','w'); ylabel(ax_r2,'Amplitud','Color','w');
        legend(ax_r2,'Location','best','TextColor','w','Color',[0.12 0.12 0.16]);
        linkaxes([ax_r1 ax_r2],'x'); set(fig_r,'Color',C_BG);
        fname_p = fullfile(carpeta,sprintf('ident_altura_%s.png',ts_str));
        try; exportgraphics(fig_r,fname_p,'Resolution',180,'BackgroundColor',C_BG);
        catch; saveas(fig_r,fname_p); end
        fprintf('[SAVE] PNG: %s\n',fname_p);
    catch ME_f; fprintf('[SAVE] Error figura: %s\n',ME_f.message); end

    fprintf('[SAVE] Guardado en: %s  (%d muestras | %.1f s)\n',carpeta,n,t_data(end));
else
    disp('[SAVE] Sin datos para guardar.');
end
end
