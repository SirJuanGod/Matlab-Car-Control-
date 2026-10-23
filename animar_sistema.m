%% animar_sistema.m — Animacion cinematica del sistema (vista lateral).
% Uso: >> animar_sistema                   (escenario por defecto: 'carro_lejos')
%      >> ani.escenario = 'nominal'; animar_sistema
%      >> ani.salida = 'ninguna'; animar_sistema   (solo pantalla)
% Escenarios: 'carro_lejos' | 'nominal' | 'ventilador_debil' | 'perturbacion'
if ~exist('ani','var') || ~isstruct(ani), ani = struct(); end
def = struct('escenario','carro_lejos','salida','ambos','fps',24,'velocidad',1,'instantaneas',[]);
fn_def = fieldnames(def);
for i_a = 1:numel(fn_def)
    if ~isfield(ani, fn_def{i_a}), ani.(fn_def{i_a}) = def.(fn_def{i_a}); end
end
clear def fn_def i_a

%% --- 1. Escenario y simulacion ---
switch ani.escenario
    case 'carro_lejos'
        ovr = struct('theta_ref_deg',35,'theta0_deg',0,'ref_rate_deg',10,'d_ini_cm',18,'t_sim',15);
        txt_esc = 'Carro lejos (18 cm): la compuerta arranca en 0 deg y el carro debe acercarse para levantarla';
    case {'nominal','desde_suelo'}
        ovr = struct('theta_ref_deg',35,'theta0_deg',0,'ref_rate_deg',10,'t_sim',12);
        txt_esc = 'Nominal: la compuerta arranca en 0 deg y sube a 35 deg con el carro a 10 cm';
    case 'ventilador_debil'
        ovr = struct('theta_ref_deg',35,'theta0_deg',0,'ref_rate_deg',10,'fan_fac',0.5,'fan_t0',8,'fan_dur',8,'t_sim',25);
        txt_esc = 'Arranca en 0 deg; desde t = 8 s el ventilador pierde fuerza hasta 50 %';
    case 'perturbacion'
        ovr = struct('theta_ref_deg',35,'theta0_deg',0,'ref_rate_deg',10,'pert_tau_mNm',-5,'pert_t_on',8,'pert_dur',3,'t_sim',16);
        txt_esc = 'Arranca en 0 deg; pulso de torque externo de -5 mN*m (t = 8 a 11 s)';
    otherwise
        error('Escenario desconocido: %s', ani.escenario);
end
run('init_parametros.m');
mdl_ani = 'modelo_miso_compuerta';
load_system(mdl_ani);
out_ani = sim(mdl_ani);

t_log = out_ani.theta_out.Time;
S.th    = rad2deg(out_ani.theta_out.Data);   S.thr  = rad2deg(out_ani.thref_out.Data);
S.pwm   = out_ani.pwm_out.Data;              S.vfan = out_ani.vfan_out.Data;
S.d     = out_ani.d_out.Data*100;            S.dest = out_ani.d_est_out.Data*100;
S.dref  = out_ani.dref_out.Data*100;         S.dnom = out_ani.dnom_out.Data*100;
S.a     = out_ani.a_meas_out.Data;
dt_f   = ani.velocidad / ani.fps;                 % avance del tiempo de simulacion por cuadro [s]
tF     = 0:dt_f:t_log(end);
nF     = numel(tF);
Sf = structfun(@(y) interp1(t_log, y, tF, 'linear', 'extrap'), S, 'UniformOutput', false);
vdn = gradient(Sf.dnom, tF);                      % velocidad del punto de operacion [cm/s]

%% --- 2. Geometria del montaje (cm) ---
L  = gate_L*100;  lc = l_c*100;                   % largo de la compuerta y brazo del centro de masa
H  = 24;                                          % altura del pivote sobre el suelo
x_post = -26;  y_arm = H + 2;                     % poste y brazo de la L invertida (el brazo libra el giro de la compuerta)
car_len = 16;  wheel_r = 3.25;  y_top = 10;       % chasis: de y = 3.5 a 10; ruedas de radio 3.25
fan_w = 4;  fan_t = 2.8;  y_fan = y_top + 0.6 + fan_w/2;   % ventilador 40 x 40 x 28 mm sobre el chasis
ang_c = linspace(0, 2*pi, 40);
xlim_a = [-32 64];  ylim_a = [-8 31];

%% --- 3. Figura ---
fig = figure('Name','Cinematica del sistema','Color','w','Position',[60 60 1280 720],'Resize','off');
axM = axes(fig,'Position',[0.04 0.45 0.92 0.51]); hold(axM,'on'); box(axM,'on');
daspect(axM,[1 1 1]); axM.XLim = xlim_a; axM.YLim = ylim_a; axM.XLimMode = 'manual'; axM.YLimMode = 'manual';
axM.XTick = -30:10:60; axM.YTick = []; xlabel(axM,'x [cm]  (0 = posicion de reposo de la compuerta)');
title(axM, ['Cinematica del sistema - ' txt_esc], 'Interpreter','none');
fill(axM,[xlim_a(1) xlim_a(2) xlim_a(2) xlim_a(1)],[ylim_a(1) ylim_a(1) 0 0],[0.90 0.88 0.82],'EdgeColor','none');
plot(axM,xlim_a,[0 0],'k-','LineWidth',2);
% soporte en L invertida
patch(axM,'XData',x_post+[-6 6 6 -6],'YData',[0 0 1 1],'FaceColor',[0.3 0.3 0.34],'EdgeColor','k');
plot(axM,[x_post x_post 0 0],[1 y_arm y_arm H],'-','Color',[0.35 0.36 0.42],'LineWidth',7);
text(axM,x_post-2.2,13,'soporte en L invertida','FontSize',8,'Color',[0.3 0.3 0.35],'Rotation',90,'HorizontalAlignment','center');
plot(axM,[0 0],[H H-L],':','Color',[0.55 0.55 0.55]);                  % posicion de reposo colgante (theta = 0)
text(axM,0.8,H-L-1.2,'reposo (\theta = 0)','FontSize',8,'Color',[0.4 0.4 0.4]);
% compuerta (cuelga del pivote)
hRef  = plot(axM,[0 0],[H H-L],'--','Color',[0.35 0.35 0.35],'LineWidth',1.5);
hGate = plot(axM,[0 0],[H H-L],'-','LineWidth',10,'Color',[0.2 0.6 0.3]);
hMass = plot(axM,0,H-lc,'o','MarkerSize',8,'MarkerFaceColor','k','MarkerEdgeColor','w');
plot(axM,0,H,'o','MarkerSize',13,'MarkerFaceColor',[0.95 0.85 0.25],'MarkerEdgeColor','k','LineWidth',1.2);
text(axM,-2.5,H+4.2,'pivote + potenciometro','FontSize',8,'Color',[0.3 0.3 0.3],'HorizontalAlignment','right');
hArc  = plot(axM,NaN,NaN,'k-','LineWidth',1);
hArcT = text(axM,0,0,'','FontSize',10,'FontWeight','bold','BackgroundColor',[1 1 1 0.7]);
% carro
hBody = patch(axM,'XData',[0 1 1 0],'YData',[3.5 3.5 y_top y_top],'FaceColor',[0.25 0.45 0.80],'EdgeColor','k','LineWidth',1.2);
hMnt  = patch(axM,'XData',[0 1 1 0],'YData',[y_top y_top y_top+0.6 y_top+0.6],'FaceColor',[0.3 0.3 0.3],'EdgeColor','k');
hImu  = patch(axM,'XData',[0 1 1 0],'YData',[y_top y_top y_top+1.6 y_top+1.6],'FaceColor',[0.95 0.85 0.25],'EdgeColor','k');
hImuT = text(axM,0,y_top+0.8,'IMU','FontSize',7,'HorizontalAlignment','center');
hFanB = patch(axM,'XData',[0 1 1 0],'YData',[y_top+0.6 y_top+0.6 y_top+0.6+fan_w y_top+0.6+fan_w],'FaceColor',[0.55 0.55 0.58],'EdgeColor','k','LineWidth',1.2);
hBlad = plot(axM,[0 0],[0 0],'-','Color','k','LineWidth',3);
hW = gobjects(2,1); hSp = gobjects(2,2);
for k = 1:2
    hW(k) = patch(axM,'XData',zeros(1,40),'YData',zeros(1,40),'FaceColor',[0.15 0.15 0.15],'EdgeColor','k');
    hSp(k,1) = plot(axM,[0 0],[0 0],'-','Color',[0.85 0.85 0.85],'LineWidth',1.5);
    hSp(k,2) = plot(axM,[0 0],[0 0],'-','Color',[0.85 0.85 0.85],'LineWidth',1.5);
end
hAcc = quiver(axM,0,y_top+4.6,0,0,0,'Color',[0.8 0.4 0],'LineWidth',2,'MaxHeadSize',2);    % aceleracion de la IMU
% aire (particulas): sale de la cara frontal del ventilador
nS = 5; nP = 7;
yS = y_fan + (-2:1:2)*0.7;
s0 = repmat(linspace(0,1,nP+1),nS,1); s0 = s0(:,1:nP) + 0.07*(1:nS)';
hAir = plot(axM,NaN,NaN,'.','Color',[0.2 0.55 0.95],'MarkerSize',12);
ph_air = 0;
% distancias
plot(axM,xlim_a,[-3 -3],'-','Color',[0.85 0.85 0.85]);
plot(axM,[0 0],[-5 0],':','Color',[0.5 0.5 0.5]);
text(axM,0,-6.3,'d = 0','HorizontalAlignment','center','FontSize',8,'Color',[0.4 0.4 0.4]);
hDim  = plot(axM,[0 0],[-3 -3],'k-','LineWidth',2);
hDimT = text(axM,0,-1.8,'','HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
hDref = plot(axM,[0 0],[-5 -1.5],'-','Color',[0.1 0.6 0.1],'LineWidth',2);
hDest = plot(axM,0,-4.2,'v','MarkerSize',7,'MarkerFaceColor',[0.9 0.5 0.1],'MarkerEdgeColor','k');
hDnom = plot(axM,0,-4.2,'d','MarkerSize',7,'MarkerFaceColor',[0.6 0.2 0.8],'MarkerEdgeColor','k');
text(axM,62,-5.2,{'| verde: d_{ref}','v naranja: d estimada (IMU)','\diamondsuit violeta: d_{nom} (supervisor)'},'FontSize',7,'HorizontalAlignment','right','VerticalAlignment','top');
hInfo = text(axM,13,30.5,'','FontName','Consolas','FontSize',9,'VerticalAlignment','top','BackgroundColor',[1 1 1 0.85],'Margin',4,'Interpreter','none');
% inset: ventilador (vista frontal)
axF = axes(fig,'Position',[0.83 0.75 0.12 0.19]); hold(axF,'on'); axis(axF,'equal','off'); axF.XLim = [-1.4 1.4]; axF.YLim = [-1.4 1.4];
rectangle(axF,'Position',[-1.3 -1.3 2.6 2.6],'Curvature',0.15,'FaceColor',[0.8 0.8 0.82]);
rectangle(axF,'Position',[-1.1 -1.1 2.2 2.2],'Curvature',[1 1],'FaceColor',[0.95 0.95 0.97]);
nB = 7; hB = gobjects(nB,1);
for k = 1:nB, hB(k) = patch(axF,'XData',[0 0 0],'YData',[0 0 0],'FaceColor',[0.3 0.3 0.35],'EdgeColor','none'); end
rectangle(axF,'Position',[-0.22 -0.22 0.44 0.44],'Curvature',[1 1],'FaceColor',[0.2 0.2 0.2]);
title(axF,'Ventilador (frontal)','FontSize',9);
hFT = text(axF,0,-1.45,'','HorizontalAlignment','center','FontSize',9,'VerticalAlignment','top');
phi = 0;

% graficas inferiores
ax1 = axes(fig,'Position',[0.05 0.08 0.26 0.25]); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
plot(ax1,tF,Sf.thr,'--','Color',[0.4 0.4 0.4],'LineWidth',1.2); plot(ax1,tF,Sf.th,'-','Color',[0.1 0.5 0.2],'LineWidth',1.8);
ylabel(ax1,'\theta [deg]'); xlabel(ax1,'t [s]'); title(ax1,'Angulo de la compuerta (--- ref)'); xlim(ax1,[0 tF(end)]);
ax2 = axes(fig,'Position',[0.38 0.08 0.26 0.25]); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
plot(ax2,tF,Sf.pwm,'-','Color',[0.1 0.4 0.8],'LineWidth',1.8); yline(ax2,sup_v_hi,'--r',sprintf('%g %% (supervisor)',sup_v_hi),'LabelHorizontalAlignment','left');
yline(ax2,100,':k'); ylabel(ax2,'PWM [%]'); xlabel(ax2,'t [s]'); title(ax2,'PWM del ventilador'); xlim(ax2,[0 tF(end)]); ylim(ax2,[0 105]);
ax3 = axes(fig,'Position',[0.71 0.08 0.26 0.25]); hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
hL = gobjects(4,1);
hL(1) = plot(ax3,tF,Sf.d,'-','Color',[0 0 0],'LineWidth',1.8); hL(2) = plot(ax3,tF,Sf.dest,'-','Color',[0.9 0.5 0.1],'LineWidth',1);
hL(3) = plot(ax3,tF,Sf.dref,'-','Color',[0.1 0.6 0.1],'LineWidth',1); hL(4) = plot(ax3,tF,Sf.dnom,'-','Color',[0.6 0.2 0.8],'LineWidth',1.5);
ylabel(ax3,'d [cm]'); xlabel(ax3,'t [s]'); title(ax3,'Distancia carro-compuerta'); xlim(ax3,[0 tF(end)]); ylim(ax3,[4 26]);
legend(ax3,hL,{'d real','d IMU','d_{ref}','d_{nom}'},'Location','best','FontSize',7,'AutoUpdate','off');
cur = gobjects(3,1); mk = gobjects(3,1);
axs3 = [ax1 ax2 ax3]; yl = {ylim(ax1), ylim(ax2), ylim(ax3)};
for k = 1:3
    cur(k) = plot(axs3(k),[0 0],yl{k},'-','Color',[0.8 0.1 0.1],'LineWidth',1);
end
mk(1) = plot(ax1,0,Sf.th(1),'o','MarkerFaceColor',[0.1 0.5 0.2],'MarkerEdgeColor','k');
mk(2) = plot(ax2,0,Sf.pwm(1),'o','MarkerFaceColor',[0.1 0.4 0.8],'MarkerEdgeColor','k');
mk(3) = plot(ax3,0,Sf.d(1),'o','MarkerFaceColor','k','MarkerEdgeColor','w');

%% --- 4. Salidas ---
out_dir = fullfile(fileparts(mfilename('fullpath')), 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
base_nom = fullfile(out_dir, sprintf('animacion_%s', ani.escenario));
use_mp4 = any(strcmp(ani.salida,{'mp4','ambos'}));
use_gif = any(strcmp(ani.salida,{'gif','ambos'}));
if use_mp4, vw = VideoWriter([base_nom '.mp4'],'MPEG-4'); vw.FrameRate = ani.fps; vw.Quality = 90; open(vw); end
gif_cada = max(1, round(ani.fps/12)); gif_ok = false;

%% --- 5. Bucle de animacion ---
for k = 1:nF
    if ~isvalid(fig), warning('animar_sistema:cerrada','Ventana cerrada: se detiene la animacion.'); break; end
    t = tF(k);  th = Sf.th(k);  thr = Sf.thr(k);  pwm = Sf.pwm(k);  vf = Sf.vfan(k);
    d = Sf.d(k);  dest = Sf.dest(k);  dref = Sf.dref(k);  dnom = Sf.dnom(k);
    % compuerta colgando del pivote (0, H); el aire la empuja hacia el soporte (x < 0)
    sx = -sind(th);  sy = -cosd(th);
    set(hGate,'XData',[0 L*sx],'YData',[H H+L*sy]); set(hMass,'XData',lc*sx,'YData',H+lc*sy);
    set(hRef,'XData',[0 -L*sind(thr)],'YData',[H H-L*cosd(thr)]);
    aa = linspace(-pi/2, -pi/2-deg2rad(th), 30); set(hArc,'XData',7*cos(aa),'YData',H+7*sin(aa));
    am = -pi/2 - deg2rad(max(th,10))/2; set(hArcT,'Position',[9*cos(am) H+9*sin(am)],'String',sprintf('%.1f%s',th,char(176)));
    if th < 2,        col_g = [0.45 0.45 0.55]; est_g = 'en reposo (colgando)';
    elseif th > 150,  col_g = [0.75 0.25 0.25]; est_g = 'SOBREPASADA (>150 deg)';
    elseif abs(th-thr) < 2, col_g = [0.2 0.6 0.3]; est_g = 'sosteniendo la consigna';
    else,             col_g = [0.95 0.65 0.1]; est_g = 'fuera de consigna'; end
    set(hGate,'Color',col_g);
    % carro: cara frontal (salida del ventilador) en x = d
    xf = d;
    set(hBody,'XData',[xf xf+car_len xf+car_len xf]);
    set(hMnt,'XData',[xf xf+6 xf+6 xf]);
    set(hFanB,'XData',[xf xf+fan_t xf+fan_t xf]);
    set(hImu,'XData',[xf+9 xf+13 xf+13 xf+9]); set(hImuT,'Position',[xf+11 y_top+0.8]);
    wheel_ang = (d_ini_cm - d)/wheel_r;
    for kw = 1:2
        xw = xf + (4.5 + 7*(kw-1));
        set(hW(kw),'XData',xw + wheel_r*cos(ang_c),'YData',wheel_r + wheel_r*sin(ang_c));
        for ks = 1:2
            a = wheel_ang + (ks-1)*pi/2;
            set(hSp(kw,ks),'XData',xw + wheel_r*[-cos(a) cos(a)],'YData',wheel_r + wheel_r*[-sin(a) sin(a)]);
        end
    end
    % ventilador: aspas y flujo
    phi = phi + 38*(vf/100);
    hb = 1.9*abs(cosd(phi));
    set(hBlad,'XData',[xf+fan_t/2 xf+fan_t/2],'YData',[y_fan-hb y_fan+hb]);
    for kb = 1:nB
        a0 = phi + (kb-1)*360/nB;
        set(hB(kb),'XData',[0 cosd(a0)*1.0 cosd(a0+22)*0.95],'YData',[0 sind(a0)*1.0 sind(a0+22)*0.95]);
    end
    set(hFT,'String',sprintf('PWM %.0f %%   v_{fan} %.0f %%',pwm,vf));
    % flujo de aire
    ph_air = mod(ph_air + 0.03 + 0.05*(vf/100), 1);
    xo = xf - 0.3; ax_ = []; ay_ = [];
    if vf > 3
        for ky = 1:nS
            dy = H - yS(ky);                                       % profundidad bajo el pivote
            if th < 89 && dy > 0 && dy/cosd(th) <= L - 0.5, x_end = -dy*tand(th); else, x_end = -14; end
            sp = mod(s0(ky,:) + ph_air, 1);
            ax_ = [ax_ xo - sp*(xo - x_end) NaN]; %#ok<AGROW>
            ay_ = [ay_ yS(ky)*ones(1,nP) NaN];    %#ok<AGROW>
        end
    end
    set(hAir,'XData',ax_,'YData',ay_,'MarkerSize',6 + 8*(vf/100));
    % aceleracion de la IMU (visualizada)
    set(hAcc,'XData',xf+11,'UData',-max(min(Sf.a(k),4),-4)*3);
    % distancias
    set(hDim,'XData',[0 xf]); set(hDimT,'Position',[xf/2 -1.8],'String',sprintf('d = %.1f cm',d));
    set(hDref,'XData',[dref dref]); set(hDest,'XData',dest); set(hDnom,'XData',dnom);
    % estado del carro / supervisor
    if vdn(k) < -0.3, est_c = 'ACERCANDO (supervisor)'; elseif vdn(k) > 0.3, est_c = 'REGRESANDO'; else, est_c = 'manteniendo'; end
    set(hInfo,'String',sprintf(['t = %5.2f s\n' ...
        'theta = %5.1f deg (ref %4.1f)   %s\n' ...
        'PWM = %5.1f %%   v_fan = %4.1f %%\n' ...
        'd = %5.2f cm (d_ref %5.2f | IMU %5.2f | d_nom %5.2f)\n' ...
        'Carro: %s'], t, th, thr, est_g, pwm, vf, d, dref, dest, dnom, est_c));
    set(cur,'XData',[t t]); set(mk(1),'XData',t,'YData',th); set(mk(2),'XData',t,'YData',pwm); set(mk(3),'XData',t,'YData',d);
    drawnow;
    if use_mp4 || use_gif || ~isempty(ani.instantaneas)
        fr = getframe(fig);
        if ~exist('fsz','var'), fsz = size(fr.cdata); end
        if ~isequal(size(fr.cdata), fsz)
            cv = uint8(255*ones(fsz)); hh = min(fsz(1),size(fr.cdata,1)); ww = min(fsz(2),size(fr.cdata,2));
            cv(1:hh,1:ww,:) = fr.cdata(1:hh,1:ww,:); fr.cdata = cv;
        end
        if use_mp4, writeVideo(vw, fr); end
        if use_gif && mod(k-1,gif_cada) == 0
            [A,map] = rgb2ind(fr.cdata,128);
            if ~gif_ok, imwrite(A,map,[base_nom '.gif'],'gif','LoopCount',inf,'DelayTime',gif_cada/ani.fps); gif_ok = true;
            else,       imwrite(A,map,[base_nom '.gif'],'gif','WriteMode','append','DelayTime',gif_cada/ani.fps); end
        end
        for ti = ani.instantaneas(:)'
            if abs(t - ti) < dt_f/2, imwrite(fr.cdata, sprintf('%s_t%04.1fs.png', base_nom, ti)); end
        end
    end
end
if use_mp4, close(vw); fprintf('Video: %s\n', fullfile(pwd,[base_nom '.mp4'])); end
if use_gif, fprintf('GIF:   %s\n', fullfile(pwd,[base_nom '.gif'])); end
fprintf('Escenario "%s": theta final %.2f deg | d final %.2f cm | PWM max %.1f %%\n', ani.escenario, S.th(end), S.d(end), max(S.pwm));
