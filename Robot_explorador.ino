/* Autor : Santiago Ventura Gomez . 2017
Curso Experto Universitario en Robótica , Programación e Impresión 3D
Asignatura : App Inventor
Profesor : Jorge Campo
Actividad 5 Libre : Robot Explorador 
*/

/*
Objetivo : 

Trabajo:

En este ejercicio se aplicarán los conocimientos obtenidos durante el curso para crear un proyecto libre de robótica 
que incluya la creación de una aplicación Android y la programación mediante Bitbloq o Arduino. 
Pueden utilizarse piezas impresas en 3D, fabricadas con cartulina, cartón o cualquier tipo de material reciclado.

La temática utilizada es de elección libre pero será imprescindible que haya comunicación Bluetooth entre el dispositivo móvil y la placa.

Acerca de mi solución a la actividad:

He diseñado un robot que dispone de las siguientes caracteristicas fundamentales :

  * La estructura básica fundamental es el robot Printbot evolution , a la que le he añadido unas estructuras laterales en las ruedas
    que sirven de soporte a los sensores IR , que conjuntamente con unos diagramas de sectores blancos y negros localizados en dichas ruedas
    nos configuran unos encoders básicos para saber en todo momento los grados que ha girado cada una de las ruedas .
  * La planta motriz es de tipo triangular con dos ruedas con motor independiente y un punto de apoyo trasero .Los motores son servos de movimiento continuo
  * El robot dispone de dos sensores IR para configurar los encoders de las ruedas , un sensor de ultrasonidos para medir distancias a los obstáculos del entorno,
    un micro servomotor 0-180º que se encarga de controlar la orientacion del sensor US , un pulsador que controla la activación - desactivación del robot ,un potenciometro
    que he utilizado en pruebas para regular la velocidad de avance , comunicación mediante bluetooth ,dos leds decorativos que se iluminan alternativamente para indicar que 
    el robot está en funcionamiento normal y un zumbador que nos señala con un pitido los cambios de estado .
  * En todo momento el robot será capaz de conocer la posición de su centro de masas y su orientación o ángulo girado con respecto a una posición y orientación inicial , con los
    errores propios del sistema , inercias y sensores utilizados por reflexion .
  * Los movimientos siempre serán de avance y en linea recta desde la última localización conocida , y las instrucciones sobre avances y giros le vendrán ordenadas desde 
    la aplicación móvil realizada en app inventor y denominada " Robot_Explorador.aia " .
  * Cuando la aplicación móvil se lo demande , el robot informará de su posición y orientación
  * En cualquier posición en la que se encuentre parado , se podrá ordenar desde la aplicación del movil que el robot realice una exploración de los obstáculos de su entorno
  * La información sobre dichos muestreos de los obstaculos encontrados será transferida al móvil que lo utilizará para mapear la zona por la que se ha desplazado el robot
*/

/* Para definir los lados Izquierda y Derecha del Robot se hace visto desde la parte trasera y mirando en la dirección
    de avance del mismo */
    
 /* Asignación de pines :
 
   * Sensor IRizqda       :  Pin 2
   * Sensor IRdrcha       :  Pin 3
   * Servo RuedaDrcha     :  Pin 4
   * Servo RuedaIzqda     :  Pin 5
   * Zumbador             :  Pin 6
   * Servo ControlUS      :  Pin 7
   * Led izquierda        :  Pin 8
   * Led derecha          :  Pin 9
   * PulsadorInicio       :  Pin 10
   * Sensor ultrasonidos  :  TRIG = PIN 11 , ECHO = PIN 12
   * Potenciómetro        :  Pin A0

*/
/*  Funciones de movimiento :

   * Avance Robot :
       Rueda Derecha : Va_drcha = 90 + Aceleracion + Ajuste R derecha
       Rueda Izquierda : Va_izqda = 90 - Aceleracion - Ajuste R izquierda
   
   * Parar :
       Rueda Derecha : 90º
       Rueda Izquierda : 90º
       
   * Giro a derechas del robot sin variación del centro de gravedad:
       Rueda Derecha : Va_drcha = 90 - Aceleracion - Ajuste R derecha
       Rueda Izquierda : Va_izqda = 90 - Aceleracion - Ajuste R izquierda
    
   * Giro a izquierdas del robot sin variación del centro de gravedad:
       Rueda Derecha : Va_drcha = 90 + Aceleracion + Ajuste R derecha
       Rueda Izquierda : Va_izqda = 90 + Aceleracion + Ajuste R izquierda
  */
  
  /* Estados de la máquina :
    *  S0:  OFF
    *  S1:  Funcionamiento normal . Dentro del estado S1 , existen 3 modos diferentes de movimiento :
            M0:  AVANCE
            M1:  GIRO A DERECHAS
            M2:  GIRO A IZQUIERDAS
 */
 
 /* Eventos de la máquina :
  
   *  E0:  Pulsador ON/OFF 
 */
 

/********************************* DEFINICION DE LIBRERIAS Y VARIABLES GLOBALES **********************************************/

#include <Servo.h>                            // Libreria Arduino que controla funciones especiales de los servomotores
#include <BitbloqUS.h>                        // Libreria Bitbloq que sirve `para controlar un sensor de ultrasonidos
#include <SoftwareSerial.h>                   // Libreria Arduino de comunicaciones serie
#include <BitbloqSoftwareSerial.h>            // Libreria Bitbloq de comunicaciones serie
#include <math.h>                             // Libreria Arduino con calculos avanzados de matematicas

Servo RuedaIzqda;                             // Declaracion de variable tipo Servo para el motor que mueve la rueda Izquierda
Servo RuedaDrcha;                             // Declaracion de variable tipo Servo para el motor que mueve la rueda Derecha
Servo ControlUS;                              // Declaracion de variable tipo Servo para el microservo que controla la orientacion del sensor US

bqSoftwareSerial bluetooth(0,1,19200);        // Declaracion de puerto de comunicaciones por bluetooth a 19200 baudios de velocidad
US ultrasonidos(11,12);                       // Declaracion de variable tipo US para el sensor de ultrasonidos y pines TRIG y ECHO donde
                                              // está conectado
int IRizqda = 2;                              // Declaracion de pin asignado al sensor IR de la izquierda
int IRdrcha = 3;                              // Declaracion de pin asignado al sensor IR de la derecha
int zumbador = 6;                             // Declaracion de pin asignado al zumbador
int ledIzqda = 8;                             // Declaracion de pin asignado al led zona izquierda
int ledDrcha = 9;                             // Declaracion de pin asignado al led zona derecha
int Pulsador = 10;                            // Declaracion del pin asignado al boton que iniciará el programa
int Acelerador = A0;                          // Declaracion del pin signado al potenciometro que regula la velocidad de las ruedas

String mensaje = "";                          // Variable encargada de almacenar la cadena de caracteres recibida por el canal de bluetooth
String mensajeRobot = "";                     // Variable encargada de almacenar la cadena de caracteres que se enviaran desde el Robot al movil

int Aceleracion = 13;                         // Variable mide el incremento de velocidad de giro de cada rueda respecto a 90º ( Vgiro = 0 )
int ajusteRdrcha = 0;                         // Correccion velocidad de ambas ruedas( Depende de cada robot . Los servos no tienen igual potencia ni exactamente igual radio )
int ajusteRizqda = 0;                         // Son dos parámetros que nos ayudan a equilibrar la potencia de ambas ruedas
                                        
float Dr = 67.5;                              // Diametro de las ruedas del robot en mm
float R = 20;                                 // Resolucion del encoder . Numero de segmentos  de arco blancos + negros en los que está dividido el encoder
float Pi = 3.14159;                           // Definición del numero Pi
float Larc = Pi*Dr/R;                         // Longitud de arco o distancia recorrida por un punto exterior de la rueda por cada segmento del encoder detectado medida en mm
float factorInercia = 1.00;                   /* Valor contrastado de desviacion de la traslacion en las pruebas realizadas con mi robot : DistaciaRealRecorrida = DistanciaTeorica x factorInercia
                                                 depende mucho de la velocidad de movimiento del robot */
float distancia = 0;                          // Variable que guarda el valor de distancia en cm recibido desde el sensor US
int angulo = 45;                              // Variable que guarda el valor del angulo de orientacion que tiene en cada momento el servomotor que mueve el sensor de ultrasonidos
float distancia_recorrida;                    // Variable que guarda el valor de la distancia recorrida por el Centro de Gravedad del robot
float orientacion = 0;                        // Variable que guarda el valor del angulo de orientacion del vector robot , parte anterior -----> parte frontal

int estado = 0;                               // Variable que guarda el estado de mi maquina : estado 0 ( OFF ) , estado 1 ( Funcionamiento Normal )
int modo = 0;                                 // Variable que guarda el modo de movimiento del robot : modo 0 ( avance ) , modo 1 ( giro Derecha ) , modo 2 ( giro Izquierda )

volatile int contadorIzq = 0;                 // Variable que guarda el numero de segmentos blanco/negro que ha detectado el sensor IR de la rueda Izquierda
volatile int contadorDcha = 0;                // Variable que guarda el numero de segmentos blanco/negro que ha detectado el sensor IR de la rueda Derecha

boolean interrupcionRI;                       /* Variable que guarda el tipo de interrupcion activa en la rueda izquierda :
                                                 true   ->  RISING , sensible al paso de negro a blanco
                                                 false  ->  FALLING , sensible al paso de blanco a negro  */
boolean interrupcionRD;                       /* Variable que guarda el tipo de interrupcion activa en la rueda derecha   :
                                                 true   ->  RISING , sensible al paso de negro a blanco
                                                 false  ->  FALLING , sensible al paso de blanco a negro  */

float T_inic = 0;                             // Valor inicial de tiempo al comenzar un intervalo de medida temporal                                            
float T_final = 0;                            // Valor actual de tiempo una vez hemos iniciado la medida de un intervalo temporal
float T_transcurrido = 0;                     // Valor de la diferencia de tiempo entre dos instantes ( T_final , T_inic )
boolean intermitente = true;                  // Variable que controla la alternancia de encendido de los leds del robot



/***********************    DEFINICION DE FUNCIONES LIGADAS Al MOVIMIENTO DE LAS RUEDAS DEL ROBOT    **********************************/


void avanzar(Servo Rueda_D,Servo Rueda_I,int ArD,int ArI,int Aceler){                 // Funcion que hace avanzar al robot
        Rueda_D.write(90 + ArD + Aceler);
        Rueda_I.write(90 - ArI - Aceler);        
}

void giroDerecha(Servo Rueda_D,Servo Rueda_I,int ArD,int ArI,int Aceler){             // Funcion que hace girar a la derecha al robot sin mover su CG
        Rueda_D.write(90 - ArD - Aceler);
        Rueda_I.write(90 - ArI - Aceler);        
}

void giroIzquierda(Servo Rueda_D,Servo Rueda_I,int ArD,int ArI,int Aceler){           // Funcion que hace girar a la izquierda al robot sin mover su CG
        Rueda_D.write(90 + ArD + Aceler);
        Rueda_I.write(90 + ArI + Aceler);        
}

void parar(Servo Rueda_D,Servo Rueda_I){                                              // Funcion que hace pararse al robot
        Rueda_D.write(90);
        Rueda_I.write(90);        
}

/**********************    DEFINICION DE LAS FUNCIONES LIGADAS A LAS INTERRUPCIONES DE LOS SENSORES IR DE LAS RUEDAS   **********************/

void encoderIzquierda(){                                    // Cada vez que se provoca una interrupcion por parte del sensor IR Izquierdo ...
        contadorIzq ++;                                     // Se aumenta el contador de segmentos izquierdo en 1 unidad ...
        interrupcionRI = !interrupcionRI;                   // Cambiamos el valor de la interrupcion asignada a la rueda izquierda
        if (interrupcionRI == true){                        // Rueda Izquierda ....
        detachInterrupt(0);                                 // Anula la interrupcion activa 
        attachInterrupt(0,encoderIzquierda,RISING);         // Establece interrupcion en el IR izquierdo por flanco de subida , RISING ( Negro -> Blanco )       
        }
        else {
        detachInterrupt(0);                                 // Anula la interrupcion activa 
        attachInterrupt(0,encoderIzquierda,FALLING);        // Establece interrupcion en el IR izquierdo por flanco de bajada , FALLING ( Blanco -> Negro )       
        }     
}

void encoderDerecha(){                                      // Cada vez que se provoca una interrupcion por parte del sensor IR Derecho ...                                               
        contadorDcha ++;                                    // Se aumenta el contador de segmentos derecho en 1 unidad ...
        interrupcionRD = !interrupcionRD;                   // Cambiamos el valor de la interrupcion asignada a la rueda derecha
        if (interrupcionRD == true){                              // Rueda Derecha ....
        detachInterrupt(1);                                 // Anula la interrupcion activa 
        attachInterrupt(1,encoderDerecha,RISING);           // Establece interrupcion en el IR derecho por flanco de subida , RISING ( Negro -> Blanco )       
        }
        else {
        detachInterrupt(1);                                 // Anula la interrupcion activa 
        attachInterrupt(1,encoderDerecha,FALLING);          // Establece interrupcion en el IR derecho por flanco de bajada , FALLING ( Blanco -> Negro )       
        }
}


void setup() {

  
  RuedaDrcha.attach(4);                         // Declaracion del pin al que está conectado el servomotor de la rueda derecha
  RuedaIzqda.attach(5);                         // Declaracion del pin al que está conectado el servomotor de la rueda izquierda
  ControlUS.attach(7);                          // Declaracion del pin al que está conectado el microservo del sensor US
    
  pinMode(IRizqda,INPUT);                       // Declaramos el pin del sensor IR izquierda como entrada
  pinMode(IRdrcha,INPUT);                       // Declaramos el pin del sensor IR derecha como entrada
  pinMode(zumbador,OUTPUT);                     // Declaramos el pin del zumbador como salida
  pinMode(ledIzqda,OUTPUT);                     // Declaramos el pin del led izquierdo como salida
  pinMode(ledDrcha,OUTPUT);                     // declaramos el pin del led derecho como salida
  pinMode(Pulsador,INPUT);                      // Declaramos el pin del pulsador como entrada
  pinMode(Acelerador,INPUT);                    // Declaramos el pin del potenciometro como entrada
    

  tone(zumbador,261,250);                       // Señal acustica que indica Robot activado
  digitalWrite(ledIzqda,LOW);                   // Apagamos led de la izquierda
  digitalWrite(ledDrcha,LOW);                   // Apagamos led de la derecha
  ControlUS.write(90);                          // Orientamos el sensor US al frente
  parar(RuedaDrcha,RuedaIzqda);                 // Inicialmente las dos ruedas paradas
  
}



void loop() {

/****************************************                  INICIO PROGRAMA PRINCIPAL              ********************************/

if (bluetooth.available() >0){                  // En caso de que se reciba algo por el puerto bluetooth ...
      mensaje = bluetooth.readString();         // Leer cadena de caracteres y asignar a la variable mensaje
}

/**************************************** CONTROL DE INTERRUPCIONES Y MODO DE MOVIMIENTO DEL ESTADO 1 ****************************/

if (estado == 1){                               // Instrucciones activas solamente en el estado 1 de funcionamiento normal


/*************************************         LOGICA DE CAMBIO ENTRE MODOS DE MOVIMIENTO  ******************************************/

if (modo == 0 && mensaje == "derecha"){                   // Si estamos en modo AVANCE y recibimos mensaje "derecha" ....
      modo = 1;                                           // Cambiamos a modo GIRO DERECHA
      interrupcionRD = !interrupcionRD;                   // Cambiamos el valor logico de la interrupcion asignada a la rueda derecha
}
else if (modo == 0 && mensaje == "izquierda"){            // Si estamos en modo AVANCE y recibimos mensaje "izquierda" ...
      modo = 2;                                           // Cambiamos a modo GIRO IZQUIERDA
      interrupcionRI = !interrupcionRI;                   // Cambiamos el valor de la interrupcion asignada a la rueda izquierda           
}
else if (modo == 1 && mensaje == "adelante"){             // Si estamos en modo GIRO DERECHA y recibimos mensaje "adelante" ...
      modo = 0;                                           // Cambiamos a modo AVANCE
      interrupcionRD = !interrupcionRD;                   // Cambiamos el valor logico de la interrupcion asignada a la rueda derecha
}
else if (modo == 1 && mensaje == "izquierda"){            // Si estamos en modo GIRO DERECHA y recibimos mensaje "izquierda" ...
      modo = 2;                                           // Cambiamos a modo GIRO IZQUIERDA
      interrupcionRD = !interrupcionRD;                   // Cambiamos el valor logico de la interrupcion asignada a la rueda derecha
      interrupcionRI = !interrupcionRI;                   // Cambiamos el valor de la interrupcion asignada a la rueda izquierda
}
else if (modo == 2 && mensaje == "adelante"){             // Si estamos en modo GIRO IZQUIERDA y recibimos mensaje "adelante" ...
      modo = 0;                                           // Cambiamos a modo AVANCE
      interrupcionRI = !interrupcionRI;                   // Cambiamos el valor de la interrupcion asignada a la rueda izquierda
}
else if (modo == 2 && mensaje == "derecha"){              // Si estamos en modo GIRO IZQUIERDA y recibimos mensaje "derecha" ...
      modo = 1;                                           // Cambiamos a modo GIRO DERECHA
      interrupcionRD = !interrupcionRD;                   // Cambiamos el valor logico de la interrupcion asignada a la rueda derecha
      interrupcionRI = !interrupcionRI;                   // Cambiamos el valor de la interrupcion asignada a la rueda izquierda      
}

/*************************************   FIN DE LA LOGICA DE CAMBIO ENTRE MODOS DE MOVIMIENTO    *************************************/

/*****************************************   DESCRIPCION DE LA LOGICA DE LAS INTERRUPCIONES **************************************/

if (interrupcionRI == true){                              // Rueda Izquierda ....
      detachInterrupt(0);                                 // Anula la interrupcion activa 
      attachInterrupt(0,encoderIzquierda,RISING);         // Establece interrupcion en el IR izquierdo por flanco de subida , RISING ( Negro -> Blanco )       
}
else {
      detachInterrupt(0);                                 // Anula la interrupcion activa 
      attachInterrupt(0,encoderIzquierda,FALLING);        // Establece interrupcion en el IR izquierdo por flanco de bajada , FALLING ( Blanco -> Negro )       
}

if (interrupcionRD == true){                              // Rueda Derecha ....
      detachInterrupt(1);                                 // Anula la interrupcion activa 
      attachInterrupt(1,encoderDerecha,RISING);           // Establece interrupcion en el IR derecho por flanco de subida , RISING ( Negro -> Blanco )       
}
else {
      detachInterrupt(1);                                 // Anula la interrupcion activa 
      attachInterrupt(1,encoderDerecha,FALLING);          // Establece interrupcion en el IR derecho por flanco de bajada , FALLING ( Blanco -> Negro )       
}

/*****************************************      FIN DE LA LOGICA DE LAS INTERRUPCIONES     ******************************************/


}

/****************************  FIN DEL CONTROL DE INTERRUPCIONES Y MODOS DE MOVIMIENTO DEL ESTADO 1  *********************************/


/******************************************   DESCRIPCION DE LA LOGICA DE LA MAQUINA DE ESTADOS  ************************************************/


if (estado == 0 && digitalRead(Pulsador) == 1){       // Si el robot está parado y pulsamos boton ...
      while (digitalRead(Pulsador) == 1){             // Rutina para evitar efecto rebote
          delay(10);
      }
      estado = 1;                                     // Cambiamos a estado 1, funcionamiento normal
      interrupcionRI = true;
      interrupcionRD = true;                          // Cuando comenzamos , ambas interrupciones en RISING , sensibles al paso de negro a blanco
      tone(zumbador,349,250);                         // Señal acustica que indica cambio de estado
      T_inic = millis();                              // Tomamos una primera referencia del valor del tiempo interno del micro 
}

else if (estado == 1 && digitalRead(Pulsador) == 1){
      while (digitalRead(Pulsador) == 1){             // Rutina para evitar efecto rebote
          delay(10);
      }
      estado = 0;                                     // Cambiamos a estado 0 , Todo desconectado
      detachInterrupt(0);                             // Desconectamos la interrupcion definida en el pin 2
      detachInterrupt(1);                             // Desconectamos la interrupcion definida en el pin 3
      tone(zumbador,440,250);                         // Señal acustica que indica cambio de estado
}



/*************************************    DESCRIPCION DE LA ACCIONES DE LA MAQUINA DE ESTADOS   ************************************************/

switch (estado){
  case 0:  
      ControlUS.write(90);                          // Orientamos el sensor US al frente
      parar(RuedaDrcha,RuedaIzqda);                 // Inicialmente las dos ruedas paradas
      digitalWrite(ledIzqda,LOW);
      digitalWrite(ledDrcha,LOW);                   // Leds apagados
      delay(50);
      break;
  
  case 1:    

      if (modo == 0 && mensaje == "adelante"){                                       // Si estamos en modo AVANCE y recibimos mensaje "adelante" ....
          distancia_recorrida = R * Larc * factorInercia;                            // Distancia TEORICA recorrida por el centro de gravedad del Robot en R segmentos corregida con el factor de inercia
          mensajeRobot = String(distancia_recorrida);                                // Generamos mensaje que enviará el Robot al movil
          bluetooth.println(mensajeRobot);                                           // Enviamos el mensaje al movil
          mensaje = "";
          mensajeRobot = "";
          RuedaDrcha.attach(4);                                                      // Declaracion del pin al que está conectado el servomotor de la rueda derecha
          RuedaIzqda.attach(5);                                                      // Declaracion del pin al que está conectado el servomotor de la rueda izquierda
          ControlUS.detach();
          contadorIzq = 0;                                                           // Se pone el contador de segmentos izquierdo a cero
          contadorDcha = 0;                                                          // Se pone el contador de segmentos derecho a cero
          avanzar(RuedaDrcha,RuedaIzqda,ajusteRdrcha,ajusteRizqda,Aceleracion);      // Ponemos en marcha el robot hacia delante
          while (contadorIzq <= 19 || contadorDcha <= 19){}                          // Esperamos a que se sucedan 20 interrupciones en ambas ruedas ( 1 vuelta completa )....
          RuedaDrcha.detach();
          RuedaIzqda.detach();                                                       // Desconectamos los motores ....
          contadorIzq = 0;                                                           // Se pone el contador de segmentos izquierdo a cero
          contadorDcha = 0;                                                          // Se pone el contador de segmentos derecho a cero

      }
      else if (modo == 1 && mensaje == "derecha"){                                   // Si estamos en modo GIRO DERECHA y recibimos mensaje "derecha" ....
          orientacion = orientacion + 90;                                            // Añadimos 90º a la orientacion actual del robot
          mensajeRobot = String(orientacion);                                        // Generamos mensaje que enviará el robot al movil
          bluetooth.println(mensajeRobot);                                           // Enviamos mensaje al movil
          mensaje = "";
          mensajeRobot = "";
          RuedaDrcha.attach(4);                                                      // Declaracion del pin al que está conectado el servomotor de la rueda derecha
          RuedaIzqda.attach(5);                                                      // Declaracion del pin al que está conectado el servomotor de la rueda izquierda
          ControlUS.detach();
          contadorIzq = 0;                                                           // Se pone el contador de segmentos izquierdo a cero
          contadorDcha = 0;                                                          // Se pone el contador de segmentos derecho a cero
          giroDerecha(RuedaDrcha,RuedaIzqda,ajusteRdrcha,ajusteRizqda,Aceleracion);  // Ponemos en marcha el robot girando a la derecha
          while (contadorIzq <= 5 || contadorDcha <= 5){}                            // Esperamos a que se sucedan 5 interrupciones en ambas ruedas ( 1 cuarto de vuelta )....
          RuedaDrcha.detach();
          RuedaIzqda.detach();                                                       // Desconectamos los motores ....
          contadorIzq = 0;                                                           // Se pone el contador de segmentos izquierdo a cero
          contadorDcha = 0;                                                          // Se pone el contador de segmentos derecho a cero         
      }
      else if (modo == 2 && mensaje == "izquierda"){                                 // Si estamos en modo GIRO IZQUIERDA y recibimos mensaje "izquierda" ....
          orientacion = orientacion - 90;                                            // Restamos 90º a la orientacion actual del robot
          mensajeRobot = String(orientacion);                                        // Generamos mensaje que enviará el robot al movil
          bluetooth.println(mensajeRobot);                                           // Enviamos mensaje al movil
          mensaje = "";  
          mensajeRobot = "";
          RuedaDrcha.attach(4);                                                      // Declaracion del pin al que está conectado el servomotor de la rueda derecha
          RuedaIzqda.attach(5);                                                      // Declaracion del pin al que está conectado el servomotor de la rueda izquierda
          ControlUS.detach();
          contadorIzq = 0;                                                           // Se pone el contador de segmentos izquierdo a cero
          contadorDcha = 0;                                                          // Se pone el contador de segmentos derecho a cero
          giroIzquierda(RuedaDrcha,RuedaIzqda,ajusteRdrcha,ajusteRizqda,Aceleracion);// Ponemos en marcha el robot girandoa la izquierda
          while (contadorIzq <= 5 || contadorDcha <= 5){}                            // Esperamos a que se sucedan 5 interrupciones en ambas ruedas ( 1 cuarto de vuelta )....
          RuedaDrcha.detach();
          RuedaIzqda.detach();                                                       // Desconectamos los motores ....
          contadorIzq = 0;                                                           // Se pone el contador de segmentos izquierdo a cero
          contadorDcha = 0;                                                          // Se pone el contador de segmentos derecho a cero         
      }
      else if (mensaje == "escanear"){                                               // Independiente del modo en el que estemos , si recibimos mensaje "escanear" ....                                              
        RuedaDrcha.detach();
        RuedaIzqda.detach();                                                         // Desconectamos motores de las ruedas
        ControlUS.attach(7);                                                         // Conectamos servo del sensor US
        if (angulo <= 135){                                                          // Empezamos en 45º , y mientras angulo sea menor o igual a 135º ....
            ControlUS.write(angulo);                                                 // Orientamos servo en la direccion angulo
            distancia = ultrasonidos.read();                                         // Asignamos a la variable distancia la lectura del sensor US
            mensajeRobot = String(distancia) + String(",") + String(angulo);         // Montamos la cadena de texto con el mensaje a enviar al movil ( distancia,angulo )
            bluetooth.println(mensajeRobot);                                         // Enviamos mensaje al movil
            angulo = angulo + 1;                                                     // Aumentamos en una unidad el angulo
            mensaje = "";                                                            // mensaje en blanco
         }
        else {                                                                       // En caso de que el angulo supere los 135º ....
        ControlUS.write(90);                                                         // Orientamos el sensor de US al frente
        delay(100);
        angulo = 45;                                                                 // Retornamos la variable angulo a 45º
        mensaje = "";
        mensajeRobot = "";                                                           //  mensajes en blanco
        RuedaDrcha.attach(4);
        RuedaIzqda.attach(5);                                                        // Conectamos motores de las ruedas
        parar(RuedaDrcha,RuedaIzqda);                                                // Pero las dos ruedas las dejamos paradas
        ControlUS.detach();                                                          // Desconectamos el servo que mueve al sensor US
      }
      }
     
      /************************** Rutina que hace parpadear los led alternativamente *****************************************/
      
      T_final = millis();                                                 // Actualizamos el valor de tiempo actual
      T_transcurrido = T_final - T_inic;                                  // Calculamos el tiempo transcurrido desde el ultimo control
      if (T_transcurrido < 500 && intermitente == true){                  // Si no han pasado 500 mS e intermitente es true ...
          digitalWrite(ledIzqda,HIGH);                                     
          digitalWrite(ledDrcha,LOW);                                     // Led Izq encendido y Led Dcha apagado
      }
      else if (T_transcurrido < 500 && intermitente == false){            // En cambio si no han pasado 500 mS e intermitente es false ...
          digitalWrite(ledIzqda,LOW);                                     
          digitalWrite(ledDrcha,HIGH);                                     // Led Izq apagado y Led Dcha encendido     
      }
      else if (T_transcurrido >= 500){                                     // Y si no quiere decir que ya hemos sobrepasado T_transcurrido y entonces ...
          intermitente = !intermitente;                                    // Cambiamos el valor de intermitente
          T_inic = T_final;                                                // Marcamos como nuevo tiempo de partida el actual final
      }
      
      break;
     
  }
 
  /****************************************                       FIN DEL PROGRAMA                   ********************************/

}
