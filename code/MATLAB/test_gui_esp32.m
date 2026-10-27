function tests = test_gui_esp32
% Pruebas de gui_esp32 sin hardware: inyecta tramas $TEL como las de uart_matlab.c.
% Uso: >> runtests('test_gui_esp32')
tests = functiontests(localfunctions);
end

function setup(tc)
tc.TestData.fig = gui_esp32();
tc.TestData.procesar = tc.TestData.fig.UserData.procesar;
end

function teardown(tc)
if isvalid(tc.TestData.fig), close(tc.TestData.fig); end
end

function testTramaValidaActualizaIndicadores(tc)
tc.TestData.procesar("$TEL,34.50,10.20,0.010,0.050,40.00,-3.00,0.20,0");
tc.verifySubstring(etiqueta(tc, 'PWM enviado'), '40.0 %');
tc.verifySubstring(etiqueta(tc, 'theta ='), '34.50');
tc.verifySubstring(etiqueta(tc, 'Carro:'), 'enlazado');
end

function testIgnoraTextoYTramasIncompletas(tc)
tc.TestData.procesar("I (312) MAIN_BASE: Iniciando");
tc.TestData.procesar("$TEL,34.50,10.20");
tc.verifySubstring(etiqueta(tc, 'PWM enviado'), '--');
end

function testCerosIndicanCarroSinEnlace(tc)
% Formato que envia la base cuando no tiene telemetria del carro
tc.TestData.procesar("$TEL,12.00,0.00,0.000,0.000,0.00,0.00,0.00,0");
tc.verifySubstring(etiqueta(tc, 'Carro:'), 'sin enlace');
end

function testIndicaSaturacionYMinMax(tc)
tc.TestData.procesar("$TEL,30.00,10.00,0.000,0.000,25.00,0.00,0.00,0");
tc.TestData.procesar("$TEL,30.00,10.00,0.000,0.000,90.00,0.00,0.00,0");
tc.verifySubstring(etiqueta(tc, 'PWM enviado'), 'SATURADO');
tc.verifySubstring(etiqueta(tc, 'min '), 'min 25.0 % | max 90.0 %');
end

function testComandoArrancaSeguroYCumpleProtocolo(tc)
% Formato que parsea uart_matlab.c: $CMD,<th>,<d>,<pwm>,<mode>,<en_motors>,<en_fan>
tc.verifyEqual(tc.TestData.fig.UserData.lineaCmd(), "$CMD,35.00,10.00,0.00,0,0,0");
end

function testErrorEstacionarioEsperaYPromedia(tc)
tc.TestData.procesar("$TEL,30.00,10.00,0.000,0.000,30.00,0.00,0.40,0");
tc.verifySubstring(etiqueta(tc, 'Error estacionario'), 'esperando');
pause(2.2); % la consigna (35 deg) lleva quieta mas de T_ASIENTO
tc.TestData.procesar("$TEL,34.40,10.20,0.000,0.000,30.00,0.00,0.20,0");
tc.TestData.procesar("$TEL,34.60,10.20,0.000,0.000,30.00,0.00,0.20,0");
% Promedio de la ultima ventana de 1 s: theta 34.5 -> 0.5 deg; e_d 0.20 cm
tc.verifySubstring(etiqueta(tc, 'Error estacionario'), 'theta 0.500 deg | d 0.20 cm');
end

function testCerrarGuardaDatos(tc)
tc.TestData.procesar("$TEL,34.50,10.20,0.010,0.050,40.00,-3.00,0.20,0");
tc.TestData.procesar("$TEL,34.60,10.10,0.010,0.050,41.00,-2.00,0.10,0");
close(tc.TestData.fig);
datos = evalin('base', 'datos_esp32');
tc.verifyEqual(datos.u_fan, [40; 41]);
tc.verifyEqual(datos.theta_deg, [34.5; 34.6]);
end

function txt = etiqueta(tc, prefijo)
lbl = findall(tc.TestData.fig, 'Type', 'uilabel');
txt = string({lbl.Text});
txt = txt(startsWith(txt, prefijo));
tc.assertNotEmpty(txt, "No hay etiqueta que empiece por " + prefijo);
txt = txt(1);
end
