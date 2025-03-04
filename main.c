/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#define mainDELAY_LOOP_COUNT (0xFFFFF)
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
//////incluimos las librerias neesarias
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "het.h"
#include "gio.h"
#include "os_semphr.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
////declaracion de tareas
void vTarea1(void *pvParameters);
void vPosition(void *pvParameters);
void vPlanif(void *pvParameters);
/////declaracion de funciones
void gioNotification(gioPORT_t *port, uint32 bit);
void limitControl(float MIN, float MAX, float *x);
float calcangle(float Angle, float x0, float y0, float z0);
int limitpwm(uint32_t MAX, uint32_t x);
///// declaracion de el manejador de cada cola
xQueueHandle xQueueGio;
xQueueHandle xQueuePos1;
xQueueHandle xQueueRef;
xQueueHandle xQueueFlag;
///////declaracion de estructuras usadas
typedef struct{
    uint32 giointer;
    uint32 giocomp;
} gioint;

typedef struct{
    int32_t countpos;
    int32_t countpos2;
    int32_t countpos3;
} counts;

typedef struct{
    float theta1;
    float theta2;
    float theta3;
}thetas;

typedef struct{
    float ref1;
    float ref2;
    float ref3;
} refs;

typedef struct{
    int i;
} flags;

const float pi = 3.141592 ; //Valor de pi
int flag = 0;


/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    gioInit();
    hetInit();
    //declaracion de estructuras para inicializar las colas
    counts count;
    thetas tets;
    flags flag;

    ////valores de estructuras que se envian a las colas
    count.countpos = 0.0;
    count.countpos2 = 0.0;
    count.countpos3 = 0.0;

    tets.theta1 = 0.0;
    tets.theta2 = 0.0;
    tets.theta3 = 0.0;

    flag.i = 0;

    /////creacion de colas
    xQueueGio =  xQueueCreate(15, sizeof(gioint));
    xQueuePos1 = xQueueCreate(1, sizeof(counts));
    xQueueRef = xQueueCreate(1, sizeof(thetas));
    xQueueFlag = xQueueCreate(1, sizeof(flags));

    //////inicializamos las colas
    xQueueSend(xQueuePos1, &count, 0);
    xQueueSend(xQueueRef, &tets, 0);
    xQueueSend(xQueueFlag, &flag, 0);


    ////se crean las tareas
    if(xTaskCreate(vPosition, "Position", configMINIMAL_STACK_SIZE, NULL, 3, NULL)!=pdTRUE) {while(1);}
    if(xTaskCreate(vTarea1, "Tarea1", 4*configMINIMAL_STACK_SIZE, NULL, 4, NULL)!=pdTRUE) {while(1);}//control
    if(xTaskCreate(vPlanif, "Planif", configMINIMAL_STACK_SIZE, NULL, 2, NULL)!=pdTRUE) {while(1);}


    vTaskStartScheduler();//Se inicializa el scheduler

    while(1);

/* USER CODE END */

    return 0;
}

int limitpwm(uint32_t MAX, uint32_t x)
{
    uint32_t aux = x;

    if(aux > MAX)
    {
        aux = MAX;
    }

    return aux;
}

/* USER CODE BEGIN (4) */
/*
 * Funcion para limitar el control
 */
void limitControl(float MIN, float MAX, float *x)
{
    float *aux = x;
    if(*aux < MIN)
    {
        *aux = MIN;
    }
    if(*aux > MAX)
    {
        *aux = MAX;
    }
}

/*
 * En esta funcion se ejecuta el MCI,recibe las posiciones
 * en los 3 ejes y obtiene el angulo en el eje Y y Z, por lo tanto
 * en el planificador se realizar el proceso de traslacion y
 * rotacion por la posicion de cada motor.
 */
float calcangle(float Angle, float x0, float y0, float z0)
{
    float pi = 3.14159;
    ///variables de medidas del robot
    double PlatformTri = 7;
    double BassTri = 3;
    double ArmLength = 6.5;
    double RodLength = 27;

    ////comienzan ecuaciones de MCI
    double y1 = -0.5 * tan(30) * BassTri;
    y0 -= 0.5 * tan(30) * PlatformTri;

    double aV = (x0 * x0 + y0 * y0 + z0 * z0 + ArmLength * ArmLength - RodLength * RodLength - y1 * y1) / (2.0 * z0);
    double bV = (y1 - y0) / z0;


    double dV = -(aV + bV * y1) * (aV + bV * y1) + ArmLength * (bV * bV * ArmLength + ArmLength);
    if (dV < 0)
    {
      Angle=0;
    }
    else{
        double yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1);
        double zj = aV + bV * yj;
        Angle = atan2(-zj, (y1 - yj)) * 180.0 / pi;
    }


    return Angle; //la variable de retorno es el angulo

}
/*
 * En la tarea de planificador realizamos las rutas
 * a seguir para formar las figuras deseadas.
 */
void vPlanif(void * pvParameters)
{
    thetas tets;//estrucuta de referencias
    flags flag;//estructura de la cola que bloquea la tarea
    TickType_t puls = 700/portTICK_PERIOD_MS;
    float  x0 = 0, y0 = 0, z0 = 19.0, Angle;//variables para inicializar la posicion
    float theta1 = 0, theta2 = 0.0, theta3 = 0.0;
    int t = 0, i = 1;
    float resoluc = 3.6, resoluc2 = 3.6;//recoluciones de motores


    for(;;)
    {
        //al recibir el elemento de la cola la tarea pasa de bloqueada a ejecucion
        xQueueReceive(xQueueFlag, &flag, portMAX_DELAY);


        //switch para ir cambiando de referencia cada vez que se ejecute el planificador
            switch(i)
            {
                case 1:
                    z0 = 19.0;
                    x0 = 1;
                    y0 = 1;
                    break;

                case 2:
                    z0 = 19.0;
                    x0 = 1;
                    y0 = -1;
                    break;

                case 3:
                    z0 = 19.0;
                    x0 = -1;
                    y0 = -1;
                    break;

                case 4:
                    z0 = 19.0;
                    x0 = -1;
                    y0 = 1;
                    break;
                case 5:
                    z0 = 20.0;
                    x0 = 1;
                    y0 = 1;
                    break;

                case 6:
                    z0 = 20.0;
                    x0 = 1;
                    y0 = -1;
                    break;

                case 7:
                    z0 = 20.0;
                    x0 = -1;
                    y0 = -1;
                    break;

                case 8:
                    z0 = 20.0;
                    x0 = -1;
                    y0 = 1;
                    break;
                case 9:
                    z0 = 21.0;
                    x0 = 1;
                    y0 = 1;
                    break;

                case 10:
                    z0 = 21.0;
                    x0 = 1;
                    y0 = -1;
                    break;

                case 11:
                    z0 = 21.0;
                    x0 = -1;
                    y0 = -1;
                    break;

                case 12:
                    z0 = 21.0;
                    x0 = -1;
                    y0 = 1;
                    i = 0;
                    break;

            }
            i++;
             //variable para ir cambiando de referencia enviada


            /*
             * calculo de los angulo de cada motor segun la posicion deseada
             * el angulo 2 y 3 tienen las operaciones de translacion y rotacion
             * debido a las posciones de los motores.
             */
          tets.theta1 = calcangle(Angle,x0, y0, z0);
          tets.theta2 = calcangle(Angle, x0 * cos(2.0944) + y0 * sin(2.0944),
              y0 * cos(2.0944) - x0 * sin(2.0944),
              z0);
          tets.theta3 = calcangle(Angle, x0 * cos(2.0944) - y0 * sin(2.0944),
              y0 * cos(2.0944) + x0 * sin(2.0944),
              z0);

          //convertimos los angulos a conteos de encoder
          tets.theta1 = tets.theta1 * resoluc;
          tets.theta2 = tets.theta2 * resoluc;
          tets.theta3 = tets.theta3 * resoluc;


          xQueueOverwrite(xQueueRef, &tets);//enviamos la estructura de 3 angulos al control

          vTaskDelay(puls);
    }


}
/*
 * En esta tarea se realiza el control de los tres motores,
 * recibe la posicion real de cada motor de la cola sin borrarla
 * de  dicha cola, igualmente recibe la referencia generada por el
 * planificador y realiza el control
 */
void vTarea1(void *pvParameters)
{
    //MI: KI=1.5 ERROR 15
    volatile float kp2 = .4, kp= .4, kp3 = .4, kd = 0.005, ki3 = 2.5, T = .02;//ganancias para motores
    volatile float ki1 = 2.5, ki2 = 2.5;//2.6
    //variables usadas para las salidas de control de cada motor
    volatile float Up1, Ud1, Ui1, Uc1, e1[5] = {0};
    volatile float Up2, Ud2, Ui2 = 0, Uc2, e2[5] = {0};
    volatile float Up3, Ud3, Ui3, Uc3, e3[5] = {0};
    //inicializamos los arreglos usados en cero
    float step[3] = {0.0,0.0,0.0};
    float pos_d[3] = {0.0, 0.0, 0.0};
    float pos_r[3] = {0.0, 0.0, 0.0};
    float pos_ff[3] = {0.0, 0.0, 0.0};
    float comp_grav[3]={0.0, 0.0, 0.0};
    //declaracion de pwm's y su periodo
    hetSIGNAL_t pwm0het0, pwm1het12, pwm2het18;
    pwm2het18.period =1000.0;
    pwm1het12.period = 1000.0;
    pwm0het0.period = 1000.0;
    uint32_t i=0, tcount = 0;
    //variable para el tiempo de espera
    volatile TickType_t xTicksToWait = 0;
    //declaracion de estructuras utilizadas
    counts position;
    thetas tets;
    flags flag;
    //variable para modificar la velocidad de cada motor, suele no utilizarse por falla de motor 2
    int calc_step=0;
    float pi = 3.14159;
    for(;;)
    {

        //recepcion de estructura de posicion real y referecia
        xQueuePeek(xQueuePos1, &position, xTicksToWait); //TOMA POSICION DE ENCODERS
        xQueueReceive(xQueueRef, &tets, xTicksToWait);

        //asignacion de referencias al arreglo usado
        pos_ff[0] = tets.theta1;
        pos_ff[1] = tets.theta2;
        pos_ff[2] = tets.theta3;


       /*if(calc_step==0){ //CALCULA EL STEP SOLO CUANDO NO ESTA CONTROLANDO
            for(i = 0; i<3; i++)
                {
                    step[i]=(pos_ff[i]-pos_r[i])/10; //PARA QUE TODOS LLEGUEN AL MISMO TIEMPO**
                }
                calc_step=1; //BLOQUEAMOS CALCSTEP
        }

        if(abs(pos_d[0]) < abs(pos_ff[0]) && abs(pos_d[1]) < abs(pos_ff[1]) && abs(pos_d[2]) < abs(pos_ff[2]))//SUMAR STEP SI LA DIFERENCIA ENTRE LAS POSICIONES ES MAYOR A 1
        {
            for(i = 0; i<3;i++){
                pos_d[i] += step[i];
            }
        }
        else{//STEP EN 0 POR SI LAS DUDAS
                //step[i]=0;
        }*/

        //asignacion de posiciones reales al arreglo utilzado
        pos_r[0] = (float)(position.countpos);//*(resoluc);
        pos_r[1] = (float)(position.countpos2);//*(resoluc);
        pos_r[2] = (float)(position.countpos3);//*(resoluc);

        ////////////calculo de errores de cada motor
        e1[4] = e1[3]; e1[3] = e1[2]; e1[2] = e1[1]; e1[1] = e1[0];
        e1[0] = (float)(pos_ff[0] - pos_r[0]);
        e2[4] = e2[3];  e2[3] = e2[2];  e2[2] = e2[1];  e2[1] = e2[0];
        e2[0] = (float)(pos_ff[1] - pos_r[1]);
        e3[4] = e3[3];  e3[3] = e3[2];  e3[2] = e3[1];  e3[1] = e3[0];
        e3[0] =(float)(pos_ff[2] - pos_r[2]);

        //comp_grav[0]=cos((pos_r[0])*pi*0.25714/180);
        //comp_grav[1]=cos((pos_r[1])*pi*0.25714/180);
        //comp_grav[2]=cos((pos_r[2])*pi*0.25714/180);

        //////calculo de salidas de control de cada motor////////
        Up1 = kp *(e1[0]);
        Up2 = kp2 * (e2[0]);
        Up3 = kp3 *(e3[0]);
        // *****
        Ui1 = Ui1 + ki1*T*e1[0];
        Ui2 = Ui2 + ki2*T*e2[0];
        Ui3 = Ui3 + ki3*T*e3[0];

        limitControl(-60, 60, &Ui1);
        limitControl(-60, 60, &Ui2);
        limitControl(-60, 60, &Ui3);
        // *****
        Ud1 = (kd/T) * (e1[0] - e1[4]);
        Ud2 = (kd/T) * (e2[0] - e2[4]);
        Ud3 = (kd/T) * (e3[0] - e3[4]);
        // *****
        Uc1 = Up1 + Ud1 + Ui1;
        Uc2 = Up2 + Ud2 + Ui2;
        Uc3 = Up3 + Ud3 + Ui3;
        // *****
        ////////limitamos la salida de control de cada motor


        limitControl( -150.0, 150.0, &Uc1 );
        limitControl( -150.0, 150.0, &Uc2 );
        limitControl( -150.0, 150.0, &Uc3 );



        ///// para cada motor, cambiamos el sentido de giro segun el signo de su salida de control/////////
        if(Uc1 > 00.0)
        {
            gioSetBit(hetPORT1, (uint32)2, (uint32)1);//2
            gioSetBit(hetPORT1, (uint32)4, (uint32)0);//4
            pwm0het0.duty =(uint32_t)(Uc1) + 35 + (int32_t)(cos((pos_r[0])*pi*0.25714/180) * 30);
            //pwm0het0.duty = (uint32_t)(limitpwm(80, pwm0het0.duty));

        }
        else
        {
            gioSetBit(hetPORT1, (uint32)2, (uint32)0);//2
            gioSetBit(hetPORT1, (uint32)4, (uint32)1);//4
            pwm0het0.duty= (uint32_t)(-Uc1) + 5 +  (uint32_t)(cos((pos_r[0])*pi*0.25714/180) * 8);
            //pwm0het0.duty = (uint32_t)(limitpwm(80, pwm0het0.duty));

        }


        if(Uc2>00.0)
        {
            gioSetBit(hetPORT1, (uint32)6, (uint32)1);
            gioSetBit(hetPORT1, (uint32)10, (uint32)0);
            pwm1het12.duty=  (uint32_t)(Uc2) + 35 + (int32_t)(cos((pos_r[1])*pi*0.25714/180) * 30);
            //pwm1het12.duty = (uint32_t)(limitpwm(80, pwm1het12.duty));

        }
        else
        {
            gioSetBit(hetPORT1, (uint32)6, (uint32)0);
            gioSetBit(hetPORT1, (uint32)10, (uint32)1);
            pwm1het12.duty= (uint32_t)(-Uc2) + 15 + (int32_t)(cos((pos_r[1])*pi*0.25714/180) * 8);
            //pwm1het12.duty = (uint32_t)(limitpwm(80, pwm1het12.duty));

        }

        if(Uc3>00.0)
        {
            gioSetBit(hetPORT1, (uint32)14, (uint32)1);
            gioSetBit(hetPORT1, (uint32)16, (uint32)0);
            pwm2het18.duty= (uint32_t)(Uc3) + 35 + (int32_t)(cos((pos_r[2])*pi*0.25714/180) * 30);
            //pwm2het18.duty = (uint32_t)(limitpwm(80, pwm2het18.duty));

        }
        else
        {
            gioSetBit(hetPORT1, (uint32)14, (uint32)0);
            gioSetBit(hetPORT1, (uint32)16, (uint32)1);
            pwm2het18.duty= (uint32_t)(-Uc3) + 5 + (int32_t)(cos((pos_r[2])*pi*0.25714/180) * 8);
            //pwm2het18.duty = (uint32_t)(limitpwm(80, pwm2het18.duty));
        }

        ///inicializamos el pwm de cada motor


        //condicional para ejecutarse cuando el error de los motores llegue al deseado
        if(abs(e1[0]) < 15 &&  abs(e2[0]) < 15 &&  abs(e3[0]) < 15)
        {
            pwm0het0.duty = 10;
            pwm2het18.duty = 10;
            pwm1het12.duty = 10;
            Ui1 = 0.0;
            Ui2 = 0.0;
            Ui3 = 0.0;
            calc_step = 0;
            flag.i = 1;//variable que se envia a la cola que desbloquea al planificador, no importa su valor
            xQueueSend(xQueueFlag, &flag, 0);//se envia un valor a la cola para que el planificador lo reciba y se active

        }
        pwmSetSignal(hetRAM1, pwm0,pwm0het0);
        pwmSetSignal(hetRAM1, pwm1,pwm1het12);
        pwmSetSignal(hetRAM1, pwm2,pwm2het18);

        vTaskDelay(20/portTICK_PERIOD_MS);//delay del control

    }
}
/*
 * En esta tarea realizamos el conteo de posicion de
 * cada motor, al recibir cada elemento de la cola
 * generada por gioNotifcation, aumenta o reduce la
 * posicion real de cada motor segun los valores
 * de los puertos GIO que reciba, se crea una estructura
 * la cual cuenta con 3 variables, una para cada posicion
 * de cada motor. La estructura se envia a una cola
 * sobreescribiendo la posicion de cada motor, dicha cola
 * se envia a la tarea de control
 */
void vPosition(void *pvParameters)
{
    volatile int32_t position1;
    volatile int32_t position2;
    volatile int32_t position3;
    counts position;
    gioint datagio;
    position.countpos = 0;
    position.countpos2 = 0;
    position.countpos3 = 0;

    for(;;)
    {
        if(xQueueReceive(xQueueGio, &datagio, portMAX_DELAY) != pdPASS)
        {

        }
        else{
            switch(datagio.giointer)
            {
            case 0:
                if(datagio.giocomp == 0)
                {
                    position.countpos --;

                }else
                {
                    position.countpos ++;

                }
                break;
            case 1:
                if(datagio.giocomp == 0)
                {
                    position.countpos ++;

                }else
                {
                    position.countpos --;
                }
                break;
            case 3:
                if(datagio.giocomp == 0)
                {
                    position.countpos2 --;
                }else
                {
                    position.countpos2 ++;
                }
                break;
            case 4:
                if(datagio.giocomp == 0)
                {
                    position.countpos2 ++;
                }else
                {
                    position.countpos2 --;
                }
                break;
            case 5:
                if(datagio.giocomp == 0)
                {
                    position.countpos3 --;
                }else
                {
                    position.countpos3 ++;
                }
                break;
            case 6:
                if(datagio.giocomp == 0)
                {
                    position.countpos3 ++;
                }else
                {
                    position.countpos3 --;
                }
                break;
            }
        }


        xQueueOverwrite(xQueuePos1, &position);//sobreescritura de la estructura de posiciones en la cola

    }
}
/*Funcion en la que se almacena el bit del puerto GIO activo
 *  y el valor del bit de su puerto GIO complemento, todo esto
 *  en una estructura, la cual se envia a una cola de 16 elementos
 *  de dicha estructura. La configuracion del microcontrolador
 *  solo detecta flancos de subida, por lo tanto cuando un GIO se
 *  activa, almacena ese GIO y el valor de su GIO complementos,
 *  con esto puede detectar el sentido de giro y la cantidad de giro
 *  de cada motor.
 *
 */
void gioNotification(gioPORT_t *port, uint32 bit)
{
    BaseType_t xHigherPriorityTaskWoken;
    gioint datagiointerr;

    datagiointerr.giointer = bit;

    switch(bit){
    case (uint32)0:

        datagiointerr.giocomp = gioGetBit(gioPORTA, (uint32)1);

        break;
    case (uint32)1:

        datagiointerr.giocomp = gioGetBit(gioPORTA, (uint32)0);

        break;
    case (uint32)3:

        datagiointerr.giocomp = gioGetBit(gioPORTA, (uint32)4);

        break;
    case (uint32)4:

        datagiointerr.giocomp = gioGetBit(gioPORTA, (uint32)3);

        break;
    case (uint32)5:

        datagiointerr.giocomp = gioGetBit(gioPORTA, (uint32)6);

        break;
    case (uint32)6:

        datagiointerr.giocomp = gioGetBit(gioPORTA, (uint32)5);

        break;
    }
    xQueueSendFromISR(xQueueGio, &datagiointerr, 0);//envio constante de dicha estructura
}