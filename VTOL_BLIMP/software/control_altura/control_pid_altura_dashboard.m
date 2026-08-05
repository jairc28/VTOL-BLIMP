% control_pid_altura_dashboard.m — blimp_definitivo/control_altura
% Dashboard MATLAB para control PID de altura — Blimp autónomo
%
% Protocolo MATLAB→ESP32 : "modo,setpoint_mm,K,K1,Kd\n"  (5 campos)
% Protocolo ESP32→MATLAB  : "millis,dist_mm,sp_mm,e_m,u_k,pwm,emb\n"
%
% PWM en el ESP32 = |u[k]| + 40  (proporcional, mínimo 40)

clear; clc;

%% ================================================================
%  Conexión TCP
%% ================================================================
ESP32_IP   = '172.20.10.6';
ESP32_PORT = 80;

t = tcpclient(ESP32_IP, ESP32_PORT);
t.Timeout = 5;
configureTerminator(t, 'LF');
t.Timeout = 1.0;
fprintf('Conectado a %s:%d\n', ESP32_IP, ESP32_PORT);

%% ================================================================
%  Controladores predefinidos
%  u[k] = u[k-1] + K*e[k] - K1*e[k-1] + Kd*e[k-2]   (T=0.5s)
%% ================================================================
K_DEF   = 246.94;
K1_DEF  = 484.13;
Kd_DEF  = 237.20;
Kd_ind  = Kd_DEF;
Kp_DEF  = K1_DEF - 2*Kd_ind;
Ki_DEF  = K_DEF  - Kp_DEF - Kd_ind;

K_PREV  = 281.13;
K1_PREV = 548.15;
Kd_PREV = 267.19;
Kd_ind2 = Kd_PREV;
Kp_PREV = K1_PREV - 2*Kd_ind2;
Ki_PREV = K_PREV  - Kp_PREV - Kd_ind2;

T_S = 0.5;

%% ================================================================
%  Paleta de colores
%% ================================================================
C_BG     = [0.08 0.08 0.10];
C_PANEL  = [0.11 0.11 0.14];
C_CARD   = [0.16 0.16 0.20];
C_WHITE  = [0.95 0.95 0.95];
C_GRAY   = [0.55 0.55 0.60];
C_GREEN  = [0.20 0.85 0.45];
C_RED    = [0.90 0.25 0.25];
C_CYAN   = [0.18 0.90 0.85];
C_YELLOW = [1.00 0.85 0.00];
C_ORANGE = [1.00 0.55 0.20];
C_PURPLE = [0.78 0.45 0.98];

C_BTN_ON  = [0.18 0.42 0.72];
C_BTN_OFF = [0.16 0.16 0.20];

%% ================================================================
%  Figura principal
%% ================================================================
fig = uifigure('Name','Control PID Altura  |  T=0.5s  |  BLIMP', ...
    'Position',[40 40 1280 720],'Color',C_BG,'Resize','on');

%% ================================================================
%  Panel izquierdo
%% ================================================================
pan = uipanel(fig,'Position',[0 0 280 720], ...
    'BackgroundColor',C_PANEL,'BorderType','none');

uilabel(pan,'Text','CONTROL PID ALTURA', ...
    'Position',[5 696 270 22],'FontSize',13,'FontWeight','bold', ...
    'FontColor',C_CYAN,'HorizontalAlignment','center');

uilabel(pan,'Text','------------------------------------', ...
    'Position',[3 686 274 12],'FontColor',C_CARD);

% --- Setpoint ---
uilabel(pan,'Text','SETPOINT (mm desde el piso):', ...
    'Position',[10 671 260 16],'FontSize',9,'FontWeight','bold','FontColor',C_WHITE);
sld_sp = uislider(pan,'Position',[10 645 258 20], ...
    'Limits',[100 3000],'Value',800, ...
    'FontColor',C_WHITE,'MajorTicks',[100 500 1000 1500 2000 3000]);
lbl_sp = uilabel(pan,'Text','SP = 800 mm  (0.80 m)', ...
    'Position',[10 626 260 18],'FontSize',11,'FontWeight','bold','FontColor',C_CYAN, ...
    'HorizontalAlignment','center');

sld_sp.ValueChangedFcn = @(src,~) set(lbl_sp,'Text', ...
    sprintf('SP = %d mm  (%.2f m)', round(src.Value), src.Value/1000));

uilabel(pan,'Text','------------------------------------', ...
    'Position',[3 614 274 12],'FontColor',C_CARD);

% --- Botones INICIAR / STOP ---
btn_ini = uibutton(pan,'push','Text','INICIAR PID', ...
    'Position',[10 588 125 26],'BackgroundColor',[0.15 0.45 0.15], ...
    'FontColor',C_WHITE,'FontSize',11,'FontWeight','bold');
btn_stop = uibutton(pan,'push','Text','STOP', ...
    'Position',[145 588 125 26],'BackgroundColor',[0.45 0.12 0.12], ...
    'FontColor',C_WHITE,'FontSize',11,'FontWeight','bold');

lbl_modo = uilabel(pan,'Text','Modo: STOP', ...
    'Position',[10 570 260 18],'FontSize',10,'FontWeight','bold','FontColor',C_RED, ...
    'HorizontalAlignment','center');

btn_ini.ButtonPushedFcn  = @(~,~) setappdata(fig,'cmd_ini',true);
btn_stop.ButtonPushedFcn = @(~,~) setappdata(fig,'cmd_stop',true);

uilabel(pan,'Text','------------------------------------', ...
    'Position',[3 558 274 12],'FontColor',C_CARD);

%% ================================================================
%  Selector de controlador
%% ================================================================
uilabel(pan,'Text','CONTROLADOR:', ...
    'Position',[10 543 260 15],'FontSize',8,'FontWeight','bold','FontColor',C_WHITE);

btn_ctrl_def = uibutton(pan,'push','Text','★ Definitivo', ...
    'Position',[10 520 128 22],'BackgroundColor',C_BTN_ON, ...
    'FontColor',C_WHITE,'FontSize',9,'FontWeight','bold');

btn_ctrl_prev = uibutton(pan,'push','Text','Previo', ...
    'Position',[142 520 128 22],'BackgroundColor',C_BTN_OFF, ...
    'FontColor',C_GRAY,'FontSize',9,'FontWeight','bold');

lbl_ctrl_act = uilabel(pan,'Text', ...
    sprintf('Activo: ★ Definitivo  |  Kp=%.2f  Ki=%.2f', Kp_DEF, Ki_DEF), ...
    'Position',[10 503 260 16],'FontSize',7,'FontColor',C_CYAN, ...
    'HorizontalAlignment','center');

uilabel(pan,'Text','------------------------------------', ...
    'Position',[3 491 274 12],'FontColor',C_CARD);

%% ================================================================
%  Coeficientes PID
%% ================================================================
uilabel(pan,'Text','COEFICIENTES:', ...
    'Position',[10 476 260 15],'FontSize',8,'FontWeight','bold','FontColor',C_WHITE);

uilabel(pan,'Text','K  = Kp+Ki+Kd', ...
    'Position',[10 459 130 15],'FontSize',8,'FontColor',C_GRAY);
fld_K = uieditfield(pan,'numeric','Value',K_DEF, ...
    'Position',[145 457 125 18],'FontSize',9, ...
    'BackgroundColor',[0.18 0.18 0.22],'FontColor',C_YELLOW);

uilabel(pan,'Text','K1 = Kp+2*Kd', ...
    'Position',[10 438 130 15],'FontSize',8,'FontColor',C_GRAY);
fld_K1 = uieditfield(pan,'numeric','Value',K1_DEF, ...
    'Position',[145 436 125 18],'FontSize',9, ...
    'BackgroundColor',[0.18 0.18 0.22],'FontColor',C_YELLOW);

uilabel(pan,'Text','Kd', ...
    'Position',[10 417 130 15],'FontSize',8,'FontColor',C_GRAY);
fld_Kd = uieditfield(pan,'numeric','Value',Kd_DEF, ...
    'Position',[145 415 125 18],'FontSize',9, ...
    'BackgroundColor',[0.18 0.18 0.22],'FontColor',C_YELLOW);

btn_reset = uibutton(pan,'push','Text','Restaurar preset activo', ...
    'Position',[10 395 260 18],'BackgroundColor',[0.18 0.18 0.28], ...
    'FontColor',C_GRAY,'FontSize',8);

lbl_kp = uilabel(pan,'Text', ...
    sprintf('Kp=%.4f  Ki=%.4f  Kd=%.4f', Kp_DEF, Ki_DEF, Kd_DEF), ...
    'Position',[10 377 260 16],'FontSize',7,'FontColor',C_GRAY, ...
    'HorizontalAlignment','center');

uilabel(pan,'Text','------------------------------------', ...
    'Position',[3 365 274 12],'FontColor',C_CARD);

%% ================================================================
%  Indicadores numéricos
%% ================================================================
uilabel(pan,'Text','INDICADORES:', ...
    'Position',[10 349 260 14],'FontSize',8,'FontWeight','bold','FontColor',C_WHITE);
lbl_alt = uilabel(pan,'Text','Altura:   --- mm', ...
    'Position',[10 328 260 22],'FontSize',14,'FontWeight','bold','FontColor',C_CYAN);
lbl_err = uilabel(pan,'Text','Error:    --- mm', ...
    'Position',[10 308 260 18],'FontSize',10,'FontColor',C_WHITE);
lbl_uk  = uilabel(pan,'Text','u[k]:     ---', ...
    'Position',[10 290 260 18],'FontSize',10,'FontColor',C_ORANGE);
lbl_pwm = uilabel(pan,'Text','PWM:      ---', ...
    'Position',[10 272 260 18],'FontSize',10,'FontColor',C_GREEN);
lbl_emb = uilabel(pan,'Text','Embrague: IDLE', ...
    'Position',[10 255 260 16],'FontSize',9,'FontColor',C_GRAY);
lbl_ti  = uilabel(pan,'Text','Tiempo:   0.0 s', ...
    'Position',[10 240 260 14],'FontSize',8,'FontColor',C_GRAY);
lbl_ni  = uilabel(pan,'Text','Muestras: 0', ...
    'Position',[10 226 260 14],'FontSize',8,'FontColor',C_GRAY);

uilabel(pan,'Text','------------------------------------', ...
    'Position',[3 214 274 12],'FontColor',C_CARD);

lbl_stat = uilabel(pan,'Text','---', ...
    'Position',[10 198 260 16],'FontSize',9,'FontWeight','bold','FontColor',C_GRAY);

uilabel(pan,'Text','Ventana: 300 s', ...
    'Position',[10 182 260 14],'FontSize',7,'FontColor',C_GRAY);
uipanel(pan,'Position',[10 170 260 8], ...
    'BackgroundColor',C_CARD,'BorderType','none');
prog_bar = uipanel(pan,'Position',[10 170 1 8], ...
    'BackgroundColor',C_CYAN,'BorderType','none');

uilabel(pan,'Text', sprintf([ ...
    'u[k] = u[k-1] + K·e[k] - K1·e[k-1] + Kd·e[k-2]\n', ...
    'T = %.1f s  |  e[k] en metros\n', ...
    'PWM = |u[k]| + 40  (min 40, max 255)\n', ...
    'Anti-windup: u_prev = sat(u_k)'], T_S), ...
    'Position',[5 5 270 160],'FontSize',7,'FontColor',[0.30 0.30 0.38], ...
    'VerticalAlignment','top','WordWrap','on');

%% ================================================================
%  Almacenamiento de configuración en appdata
%% ================================================================
cfg = struct();
cfg.presets(1) = struct('K', K_DEF,  'K1', K1_DEF,  'Kd', Kd_DEF, ...
                        'Kp', Kp_DEF,'Ki', Ki_DEF,  'nombre', 'Definitivo');
cfg.presets(2) = struct('K', K_PREV, 'K1', K1_PREV, 'Kd', Kd_PREV, ...
                        'Kp', Kp_PREV,'Ki', Ki_PREV,'nombre', 'Previo');
cfg.preset_idx = 1;
cfg.C_BTN_ON   = C_BTN_ON;
cfg.C_BTN_OFF  = C_BTN_OFF;
cfg.C_WHITE    = C_WHITE;
cfg.C_GRAY     = C_GRAY;
cfg.C_CYAN     = C_CYAN;
cfg.C_PURPLE   = C_PURPLE;
setappdata(fig, 'cfg', cfg);
setappdata(fig, 'cmd_ini',  false);
setappdata(fig, 'cmd_stop', false);

%% ================================================================
%  Callbacks del selector de controlador y de los campos
%% ================================================================
btn_ctrl_def.ButtonPushedFcn = @(~,~) aplicarPreset(fig, 1, ...
    fld_K, fld_K1, fld_Kd, lbl_kp, lbl_ctrl_act, btn_ctrl_def, btn_ctrl_prev);

btn_ctrl_prev.ButtonPushedFcn = @(~,~) aplicarPreset(fig, 2, ...
    fld_K, fld_K1, fld_Kd, lbl_kp, lbl_ctrl_act, btn_ctrl_def, btn_ctrl_prev);

btn_reset.ButtonPushedFcn = @(~,~) aplicarPreset(fig, ...
    getappdata(fig,'cfg').preset_idx, ...
    fld_K, fld_K1, fld_Kd, lbl_kp, lbl_ctrl_act, btn_ctrl_def, btn_ctrl_prev);

fld_K.ValueChangedFcn  = @(~,~) actualizarKpKiKd(fld_K, fld_K1, fld_Kd, lbl_kp);
fld_K1.ValueChangedFcn = @(~,~) actualizarKpKiKd(fld_K, fld_K1, fld_Kd, lbl_kp);
fld_Kd.ValueChangedFcn = @(~,~) actualizarKpKiKd(fld_K, fld_K1, fld_Kd, lbl_kp);

%% ================================================================
%  Gráficas
%% ================================================================
ax1 = uiaxes(fig,'Position',[292 375 960 335]);
ax1.Color = [0.08 0.08 0.11];
ax1.XColor = C_WHITE; ax1.YColor = C_WHITE;
ax1.GridColor = [0.25 0.25 0.30]; ax1.GridAlpha = 0.4;
ax1.Title.String  = 'Altura vs Setpoint';
ax1.Title.Color   = C_WHITE; ax1.Title.FontSize = 11;
ax1.XLabel.String = 'Tiempo (s)'; ax1.XLabel.Color = C_GRAY;
ax1.YLabel.String = 'Altura desde el piso (mm)'; ax1.YLabel.Color = C_GRAY;
ax1.XLim = [0 30]; ax1.YLim = [0 2800];
grid(ax1,'on'); hold(ax1,'on');
ln_sp  = plot(ax1, NaN, NaN, '--', 'Color', C_YELLOW, 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
ln_alt = plot(ax1, NaN, NaN, '-',  'Color', C_CYAN,   'LineWidth', 2.5, 'DisplayName', 'Altura');
legend(ax1,'Location','northeast','TextColor',C_WHITE,'Color',C_PANEL, ...
    'EdgeColor',C_CARD,'FontSize',10);

ax2 = uiaxes(fig,'Position',[292 25 960 340]);
ax2.Color = [0.08 0.08 0.11];
ax2.XColor = C_WHITE; ax2.YColor = C_WHITE;
ax2.GridColor = [0.25 0.25 0.30]; ax2.GridAlpha = 0.4;
ax2.Title.String  = 'Acción de Control  u[k]  y  PWM  (PWM = |u[k]|+40)';
ax2.Title.Color   = C_WHITE; ax2.Title.FontSize = 11;
ax2.XLabel.String = 'Tiempo (s)'; ax2.XLabel.Color = C_GRAY;
ax2.YLabel.String = 'Amplitud'; ax2.YLabel.Color = C_GRAY;
ax2.XLim = [0 30];
grid(ax2,'on'); hold(ax2,'on');
ln_uk   = plot(ax2, NaN, NaN, '-','Color',C_ORANGE,'LineWidth',2.5,'DisplayName','u[k]');
ln_pwm2 = stairs(ax2, NaN, NaN,'-','Color',C_GREEN,'LineWidth',2.0,'DisplayName','PWM = |u|+40');
legend(ax2,'Location','northeast','TextColor',C_WHITE,'Color',C_PANEL, ...
    'EdgeColor',C_CARD,'FontSize',10);
linkaxes([ax1 ax2],'x');

%% ================================================================
%  Variables de datos
%% ================================================================
t_data   = [];
alt_data = [];
sp_data  = [];
uk_data  = [];
pwm_data = [];
ek_data  = [];

t0_ms       = 0;
n           = 0;
modo_actual = 0;
K_save  = K_DEF;
K1_save = K1_DEF;
Kd_save = Kd_DEF;

%% ================================================================
%  Bucle principal
%% ================================================================
try
while isvalid(fig)
    drawnow;

    sp_v  = round(sld_sp.Value);
    K_v   = fld_K.Value;
    K1_v  = fld_K1.Value;
    Kd_v  = fld_Kd.Value;

    % --- Comandos de modo ---
    if getappdata(fig,'cmd_ini')
        modo_actual = 1;
        setappdata(fig,'cmd_ini',false);
        try; flush(t,'input'); catch; end
        pause(0.15); % Pequeña pausa para sincronizar y comenzar limpia la toma desde 0s
        t_data = []; alt_data = []; sp_data = []; uk_data = []; pwm_data = []; ek_data = [];
        t0_ms = 0; n = 0;
        set(lbl_modo,'Text','Modo: PID ACTIVO','FontColor',C_GREEN);
        fprintf('INICIAR | SP=%d K=%.4f K1=%.4f Kd=%.4f (Datos reiniciados a t=0s)\n', sp_v, K_v, K1_v, Kd_v);
    end

    if getappdata(fig,'cmd_stop')
        modo_actual = 0;
        setappdata(fig,'cmd_stop',false);
        try; flush(t,'input'); catch; end
        set(lbl_modo,'Text','Modo: STOP','FontColor',C_RED);
        set(lbl_stat,'Text','STOP','FontColor',C_RED);
    end

    % --- Enviar comando al ESP32 (5 campos) ---
    cmd = sprintf('%d,%d,%.6f,%.6f,%.6f', modo_actual, sp_v, K_v, K1_v, Kd_v);
    if ~exist('last_cmd','var'), last_cmd = ''; end
    if ~exist('cmd_counter','var'), cmd_counter = 0; end
    cmd_counter = cmd_counter + 1;
    
    if ~strcmp(cmd, last_cmd) || cmd_counter > 10
        try; writeline(t, cmd); last_cmd = cmd; cmd_counter = 0; catch; end
    end

    % --- Leer telemetría del ESP32 ---
    try
        raw = char(readline(t));
        raw = strtrim(raw);
        if isempty(raw); continue; end

        vals = str2double(strsplit(raw,','));
        if numel(vals) < 7 || any(isnan(vals(1:7))); continue; end

        ms_raw = vals(1);
        dist_v = vals(2);   % mm
        sp_r   = vals(3);   % mm
        ek_m   = vals(4);   % m
        uk_v   = vals(5);   % acción de control
        pwm_v  = vals(6);   % PWM = |u[k]| + 40
        emb_v  = vals(7);   % estado embrague

        if t0_ms == 0 && modo_actual == 1; t0_ms = ms_raw; end
        if t0_ms > 0
            t_s = (ms_raw - t0_ms) / 1000.0;
        else
            t_s = 0;
        end

        try
            K_save  = fld_K.Value;
            K1_save = fld_K1.Value;
            Kd_save = fld_Kd.Value;
        catch; end

        n = n + 1;

        t_data   = [t_data;   t_s];
        alt_data = [alt_data; dist_v];
        sp_data  = [sp_data;  sp_r];
        uk_data  = [uk_data;  uk_v];
        pwm_data = [pwm_data; pwm_v];
        ek_data  = [ek_data;  ek_m * 1000];

        % Respaldo continuo en el Workspace de MATLAB por si se cierra la figura
        try
            assignin('base', 't_data', t_data);
            assignin('base', 'alt_data', alt_data);
            assignin('base', 'sp_data', sp_data);
            assignin('base', 'uk_data', uk_data);
            assignin('base', 'pwm_data', pwm_data);
            assignin('base', 'ek_data', ek_data);
        catch; end

        MAX_PTS = 600;
        if numel(t_data) > MAX_PTS
            idx = numel(t_data)-MAX_PTS+1 : numel(t_data);
            t_data   = t_data(idx);   alt_data = alt_data(idx);
            sp_data  = sp_data(idx);  uk_data  = uk_data(idx);
            pwm_data = pwm_data(idx); ek_data  = ek_data(idx);
        end

        set(ln_alt, 'XData', t_data, 'YData', alt_data);
        set(ln_sp,  'XData', t_data, 'YData', sp_data);
        set(ln_uk,  'XData', t_data, 'YData', uk_data);
        set(ln_pwm2,'XData', t_data, 'YData', pwm_data);

        if t_s > ax1.XLim(2) - 3
            xl = [max(0, t_s-50) t_s+5];
            ax1.XLim = xl; ax2.XLim = xl;
        end

        prog_bar.Position(3) = max(1, round(min(1, t_s/300) * 260));

        emb_str = {'IDLE','MOVING','ON'};
        emb_col = {C_GRAY, C_YELLOW, C_GREEN};
        ei = min(max(emb_v+1,1),3);

        set(lbl_alt,'Text', sprintf('Altura:   %d mm  (%.2f m)', round(dist_v), dist_v/1000));
        set(lbl_err,'Text', sprintf('Error:    %+.0f mm', ek_m*1000));
        set(lbl_uk, 'Text', sprintf('u[k]:     %.3f', uk_v));
        set(lbl_pwm,'Text', sprintf('PWM:      %d  (|u|+40)', round(pwm_v)));
        set(lbl_emb,'Text', sprintf('Embrague: %s', emb_str{ei}), 'FontColor', emb_col{ei});
        set(lbl_ti, 'Text', sprintf('Tiempo:   %.1f s', t_s));
        set(lbl_ni, 'Text', sprintf('Muestras: %d', n));

        if abs(ek_m*1000) < 30
            set(lbl_stat,'Text','En zona objetivo (±30 mm)','FontColor',C_GREEN);
        elseif uk_v > 0
            set(lbl_stat,'Text','Subiendo','FontColor',C_CYAN);
        else
            set(lbl_stat,'Text','Bajando','FontColor',C_ORANGE);
        end

    catch
    end

end
catch ME
    fprintf('[ERROR] %s\n', ME.message);
end

%% ================================================================
%  Guardado robusto al cerrar
%% ================================================================
try
    writeline(t, sprintf('0,%d,%.4f,%.4f,%.4f', ...
        round(sld_sp.Value), K_save, K1_save, Kd_save));
    pause(0.05); clear t;
catch; end

n = numel(t_data);
if n > 0
    ts_str  = datestr(now,'yyyymmdd_HHMMSS');
    carpeta = sprintf('resultados_pid_%s', ts_str);
    if ~exist(carpeta,'dir'); mkdir(carpeta); end

    try
        fname_c = fullfile(carpeta, sprintf('pid_altura_%s.csv', ts_str));
        T_out = table(t_data, alt_data, sp_data, ek_data, uk_data, pwm_data, ...
            'VariableNames',{'tiempo_s','altura_mm','setpoint_mm','error_mm','u_k','pwm'});
        writetable(T_out, fname_c);
        fprintf('[SAVE] CSV guardado: %s\n', fname_c);
    catch ME_csv
        fprintf('[SAVE] Error CSV: %s\n', ME_csv.message);
    end

    try
        fname_m = fullfile(carpeta, sprintf('pid_altura_%s.mat', ts_str));
        save(fname_m, 't_data','alt_data','sp_data','ek_data','uk_data','pwm_data', ...
             'K_save','K1_save','Kd_save','T_S');
        fprintf('[SAVE] .mat guardado: %s\n', fname_m);
    catch ME_mat
        fprintf('[SAVE] Error .mat: %s\n', ME_mat.message);
    end

    try
        fig_r = figure('Name','Resultados PID Altura','Color',C_BG,'Position',[80 60 1100 650]);

        ax_r1 = subplot(2,1,1);
        set(ax_r1,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w', ...
            'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',11);
        grid(ax_r1,'on'); hold(ax_r1,'on');
        plot(ax_r1, t_data, sp_data,  '--','Color',C_YELLOW,'LineWidth',1.5,'DisplayName','Setpoint');
        plot(ax_r1, t_data, alt_data, '-', 'Color',C_CYAN,  'LineWidth',2.5,'DisplayName','Altura');
        xlabel(ax_r1,'Tiempo (s)','Color','w','FontSize',12);
        ylabel(ax_r1,'Altura desde el piso (mm)','Color','w','FontSize',12);
        title(ax_r1, sprintf('Control PID Altura | %d muestras | %.1f s | K=%.3f  K1=%.3f  Kd=%.3f', ...
            n, t_data(end), K_save, K1_save, Kd_save), ...
            'Color','w','FontSize',12,'FontWeight','bold');
        legend(ax_r1,'Location','best','TextColor','w','FontSize',10, ...
            'Color',[0.13 0.13 0.16],'EdgeColor',[0.35 0.35 0.35]);

        ax_r2 = subplot(2,1,2);
        set(ax_r2,'Color',[0.08 0.08 0.11],'XColor','w','YColor','w', ...
            'GridColor',[0.25 0.25 0.30],'GridAlpha',0.45,'FontSize',11);
        grid(ax_r2,'on'); hold(ax_r2,'on');
        plot(ax_r2, t_data, uk_data,   '-','Color',C_ORANGE,'LineWidth',2.5,'DisplayName','u[k]');
        stairs(ax_r2, t_data, pwm_data,'-','Color',C_GREEN, 'LineWidth',2.0,'DisplayName','PWM = |u|+40');
        xlabel(ax_r2,'Tiempo (s)','Color','w','FontSize',12);
        ylabel(ax_r2,'Amplitud','Color','w','FontSize',12);
        title(ax_r2,'Acción de Control  u[k]  y  PWM','Color','w','FontSize',12,'FontWeight','bold');
        legend(ax_r2,'Location','best','TextColor','w','FontSize',10, ...
            'Color',[0.13 0.13 0.16],'EdgeColor',[0.35 0.35 0.35]);
        linkaxes([ax_r1 ax_r2],'x');
        set(fig_r,'Color',C_BG);

        fname_png = fullfile(carpeta, sprintf('grafica_%s.png', ts_str));
        try
            exportgraphics(fig_r, fname_png, 'Resolution',180,'BackgroundColor',C_BG);
        catch
            saveas(fig_r, fname_png);
        end
        fprintf('[SAVE] PNG guardado: %s\n', fname_png);
    catch ME_fig
        fprintf('[SAVE] Error figura: %s\n', ME_fig.message);
    end

    fprintf('[SAVE] Todo guardado en: %s  (%d muestras, %.1f s)\n', carpeta, n, t_data(end));
else
    disp('[SAVE] Sin datos para guardar.');
end

%% ================================================================
%  Funciones auxiliares locales
%% ================================================================

function aplicarPreset(fig, idx, fld_K, fld_K1, fld_Kd, ...
                       lbl_kp, lbl_ctrl_act, btn_def, btn_prev)
    cfg = getappdata(fig, 'cfg');
    p   = cfg.presets(idx);
    cfg.preset_idx = idx;
    setappdata(fig, 'cfg', cfg);
    fld_K.Value  = p.K;
    fld_K1.Value = p.K1;
    fld_Kd.Value = p.Kd;
    set(lbl_kp,'Text', sprintf('Kp=%.4f  Ki=%.4f  Kd=%.4f', p.Kp, p.Ki, p.Kd));
    C = cfg;
    if idx == 1
        set(btn_def,  'BackgroundColor', C.C_BTN_ON,  'FontColor', C.C_WHITE);
        set(btn_prev, 'BackgroundColor', C.C_BTN_OFF, 'FontColor', C.C_GRAY);
        set(lbl_ctrl_act, 'FontColor', C.C_CYAN, 'Text', ...
            sprintf('Activo: ★ %s  |  Kp=%.2f  Ki=%.2f', p.nombre, p.Kp, p.Ki));
    else
        set(btn_prev, 'BackgroundColor', C.C_BTN_ON,  'FontColor', C.C_WHITE);
        set(btn_def,  'BackgroundColor', C.C_BTN_OFF, 'FontColor', C.C_GRAY);
        set(lbl_ctrl_act, 'FontColor', C.C_PURPLE, 'Text', ...
            sprintf('Activo: %s  |  Kp=%.2f  Ki=%.2f', p.nombre, p.Kp, p.Ki));
    end
end

function actualizarKpKiKd(fld_K, fld_K1, fld_Kd, lbl_kp)
    K_v  = fld_K.Value;
    K1_v = fld_K1.Value;
    Kd_v = fld_Kd.Value;
    Kd_i = Kd_v;
    Kp_i = K1_v - 2*Kd_v;
    Ki_i = K_v  - Kp_i - Kd_i;
    set(lbl_kp, 'Text', sprintf('Kp=%.4f  Ki=%.4f  Kd=%.4f', Kp_i, Ki_i, Kd_i));
end
