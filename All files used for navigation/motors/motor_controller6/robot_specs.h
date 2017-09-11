#ifndef ROBOT_SPECS_H
#define ROBOT_SPECS_H

#define encoder_pulse   16  // conteos por revolución de los encoders en cada canal
#define gear_ratio      131.25  // relación de transformación propia del motorreductor c/encoder
#define wheel_diameter  0.14   //diámetro en m
#define wheel_width     0.037   // ancho o espesor de las ruedas en m
#define track_width     0.4   // distancia de separación de las ruedas (...creo) en m
#define MAX_RPM         10    // velocidad máxima de los motorreductores sin carga al parecer. Valores mayores retrasaban respuesta de teleop (debería ser de 80RPM)
#define pi              3.1415926  //pi
#define two_pi          6.2831853  //2pi
#endif
