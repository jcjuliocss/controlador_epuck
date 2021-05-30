#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 256


int main(int argc, char** argv)
{

    int i = 0;
    double sensores_prox[8];
    double ac_esquerdo=1, ac_direito=1;

    wb_robot_init();
    
    const double *pos;

    WbNodeRef robot_node = wb_supervisor_node_get_from_def("e_puck");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    
    WbDeviceTag motor_esquerdo = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_direito = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(motor_esquerdo, INFINITY);
    wb_motor_set_position(motor_direito, INFINITY);

    wb_motor_set_velocity(motor_esquerdo, 0);
    wb_motor_set_velocity(motor_direito, 0);
    
    WbDeviceTag acelerometro;

    acelerometro = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(acelerometro, 10);

    WbDeviceTag sensor_prox[8];

    sensor_prox[0] = wb_robot_get_device("ps0");
    sensor_prox[1] = wb_robot_get_device("ps1");
    sensor_prox[2] = wb_robot_get_device("ps2");
    sensor_prox[3] = wb_robot_get_device("ps3");
    sensor_prox[4] = wb_robot_get_device("ps4");
    sensor_prox[5] = wb_robot_get_device("ps5");
    sensor_prox[6] = wb_robot_get_device("ps6");
    sensor_prox[7] = wb_robot_get_device("ps7");
    for(int i = 0;i < 8;i++){
      wb_distance_sensor_enable(sensor_prox[i], TIME_STEP);
    }

    while (wb_robot_step(TIME_STEP) != -1){
        for(i=0;i<8;i++){
          sensores_prox[i] = wb_distance_sensor_get_value(sensor_prox[i]);
        }
        
        pos = wb_supervisor_field_get_sf_vec3f(trans_field);
        
        if (pos[0] > -0.65 && pos[0] < -0.3 && pos[2] > -0.65 && pos[2] < -0.3){
          ac_direito = 1;
          ac_esquerdo = 1;

          if (sensores_prox[0] > 500 || sensores_prox[1] > 500 || sensores_prox[6] > 500 || sensores_prox[7] > 500){
            printf("caixa leve detectada.");
            for(i = 0; i < 3; i++){
                wb_robot_step(400);
            }
           }
          }else if (sensores_prox[0] > 500){
              ac_direito = 1;
              ac_esquerdo = -0.1;
          }else if (sensores_prox[1] > 400){
              ac_direito = 1;
              ac_esquerdo = -0.2;
          }else if (sensores_prox[2] > 400){
              ac_direito = 1;
              ac_esquerdo = -0.3;
          }else if (sensores_prox[7] > 400){
              ac_direito = -0.1;
              ac_esquerdo = 1;
          }else if (sensores_prox[6] > 400){
              ac_direito = -0.2;
              ac_esquerdo = 1;
          }else if (sensores_prox[5] > 400){
              ac_direito = -0.3;
              ac_esquerdo = 1;
          }else{
              ac_direito = 1;
              ac_esquerdo = 1;
          }
        
        wb_motor_set_velocity(motor_esquerdo, 6.28 * ac_esquerdo);
        wb_motor_set_velocity(motor_direito, 6.28 * ac_direito);
    };

    wb_robot_cleanup();
 
    return 0;
}
