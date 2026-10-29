%% exportar_lqr_c.m — Genera lqr_tablas.h para el firmware del carro desde init_parametros.m.
% Unica fuente de las ganancias: tras cambiar pesos o parametros, volver a correr este script
% y recompilar el firmware (code/C/esp32_carro_control).

run('init_parametros.m');
salida = fullfile(fileparts(mfilename('fullpath')), '..', 'C', 'esp32_carro_control', 'main', 'lqr_tablas.h');

f = fopen(salida, 'w');
c = onCleanup(@() fclose(f));
fprintf(f, '// Generado por code/MATLAB/exportar_lqr_c.m desde init_parametros.m: no editar a mano.\n');
fprintf(f, '#ifndef LQR_TABLAS_H\n#define LQR_TABLAS_H\n\n');

constantes = {
    'LQR_D0_M',        d0,                   'distancia de referencia aerodinamica [m]'
    'LQR_K_M',         k_m,                  'ganancia aerodinamica en d0 [N*m/%]'
    'LQR_MGL',         m_g*g*l_c,            'm_g*g*l_c [N*m]'
    'LQR_D_MIN_M',     d_min,                'limite de distancia [m]'
    'LQR_D_MAX_M',     d_max,                'limite de distancia [m]'
    'LQR_D_INI_M',     d_ini,                'distancia inicial y de regreso del supervisor [m]'
    'LQR_PWM_MIN',     pwm_min,              'saturacion del PWM [%]'
    'LQR_PWM_MAX',     pwm_max,              'saturacion del PWM [%]'
    'LQR_XI_BAND_RAD', deg2rad(xi_band_deg), 'la integral solo acumula con |e| menor que esto [rad]'
    'LQR_XI_LIM',      xi_lim,               'limite de la integral del error [rad*s]'
    'LQR_TAU_W',       tau_w,                'filtro de la derivada que estima omega [s]'
    'LQR_TAU_FAN',     fan_tau,              'modelo del ventilador para estimar v_fan [s]'
    'LQR_TAU_REF',     tau_ref,              'prefiltro de la referencia [s]'
    'LQR_REF_RATE',    deg2rad(ref_rate_deg),'limite de rampa de la referencia [rad/s]'
    'LQR_GOV_RAD',     deg2rad(gov_deg),     'gobernador: theta_ref <= theta_meas + esto [rad]'
    'LQR_SUP_V_HI',    sup_v_hi,             'supervisor: acerca el carro si p_raw supera esto [%]'
    'LQR_SUP_V_LO',    sup_v_lo,             'supervisor: regresa hacia d_ini si p_raw baja de esto [%]'
    'LQR_SUP_ADV',     sup_adv/100,          'supervisor: velocidad maxima de d_nom [m/s]'
    'LQR_SUP_RET',     sup_ret/100,          'supervisor: velocidad de regreso de d_nom [m/s]'
};
for k = 1:size(constantes, 1)
    fprintf(f, '#define %-16s %#.9gf  // %s\n', constantes{k, :});
end

fprintf(f, '\n#define LQR_N_TH %d  // rejilla de scheduling en theta_ref\n', nGs);
fprintf(f, '#define LQR_N_D  %d  // rejilla de scheduling en d_nom\n\n', nDs);
fprintf(f, 'static const float LQR_THG_DEG[LQR_N_TH] = {%s};\n', strjoin(compose('%#.9gf', thg_simo), ', '));
fprintf(f, 'static const float LQR_DG_CM[LQR_N_D] = {%s};\n\n', strjoin(compose('%#.9gf', dg_simo), ', '));
fprintf(f, '// Fila (j*LQR_N_TH + i) <-> (theta_i, d_j); K(2x5) por columnas: K[r][c] = fila[c*2 + r]\n');
fprintf(f, 'static const float LQR_KG[LQR_N_TH * LQR_N_D][10] = {\n');
for r = 1:size(Kg_simo, 1)
    fprintf(f, '    {%s},\n', strjoin(compose('%#.9gf', Kg_simo(r, :)), ', '));
end
fprintf(f, '};\n\n#endif // LQR_TABLAS_H\n');
fprintf('Escrito %s (%d ganancias)\n', salida, numel(Kg_simo));
