/*
 * File:
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <ode/ode.h>
#include <plugins/physics.h>

#define NORMAL_FORCE 50
static pthread_mutex_t mutex; // needed to run with multi-threaded version of ODE
//static dGeomID touch_sensor_geom[4];
static dBodyID touch_sensor_body[4];
void webots_physics_init() 
{
  pthread_mutex_init(&mutex, NULL);
  touch_sensor_body[0] = dWebotsGetBodyFromDEF("LF_touch_sensor");
  touch_sensor_body[1] = dWebotsGetBodyFromDEF("RF_touch_sensor");
  touch_sensor_body[2] = dWebotsGetBodyFromDEF("LH_touch_sensor");
  touch_sensor_body[3] = dWebotsGetBodyFromDEF("RH_touch_sensor");
  // if(touch_sensor_geom[0]&&touch_sensor_geom[1]&&touch_sensor_geom[2]&&touch_sensor_geom[3])
  // {
  // touch_sensor_body[0] = dGeomGetBody(touch_sensor_geom[0]);
  // touch_sensor_body[1] = dGeomGetBody(touch_sensor_geom[1]);
  // touch_sensor_body[2] = dGeomGetBody(touch_sensor_geom[2]);
  // touch_sensor_body[3] = dGeomGetBody(touch_sensor_geom[3]);
  // }
}
float timeP = 0;
float around = 0;
void webots_physics_step() 
{
dWebotsConsolePrintf("%d,\n",1);
  float T = 4;
  float n_force = -NORMAL_FORCE;
  int size;
  const float *timeSend = dWebotsReceive(&size);

  if(timeSend!=0)
  {
    dWebotsConsolePrintf("%f, %f\n",timeSend[0],timeSend[1]);
    timeP=timeP+0.01;
    dWebotsConsolePrintf("%f, %d\n",timeP,around);
  }
   if(timeP>=0&&timeP<=T/2)
      {
        BodyAddRelForce(touch_sensor_body[0], 0, 0, n_force);
        BodyAddRelForce(touch_sensor_body[3], 0, 0, n_force);
      // }
      // if(timeP>T/2&&timeP<T)
      // {
        BodyAddRelForce(touch_sensor_body[1], 0, 0, n_force);
        BodyAddRelForce(touch_sensor_body[2], 0, 0, n_force);
      }
  // if(around>=0&&around<2)
  // {
    
  // }
   // if(around==2)
   // {
   // if(timeP>=0&&timeP<=T/2){
    // dBodyAddRelForce(touch_sensor_body[0], 0, n_force, 0);
     // dBodyAddRelForce(touch_sensor_body[1], 0, n_force, 0);
     // dBodyAddRelForce(touch_sensor_body[2], 0, n_force, 0);
     // //dBodyAddRelForce(touch_sensor_body[3], 0, n_force, 0);}
   // }
   // if(around==3)
   // {
     // if((timeP>=0&&timeP<=T)){
     // dBodyAddRelForce(touch_sensor_body[1], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[3], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[2], 0, 0, n_force);}
   // }
   if(timeP>=T)
    {
      timeP=0;
      around=around+1;
    }
   // if(timeP>=0&&timeP<=T/4)
   // {
     // dBodyAddRelForce(touch_sensor_body[0], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[3], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[2], 0, 0, n_force);
   // }
      // if(timeP>=T/4&&timeP<=T/2)
   // {
     // dBodyAddRelForce(touch_sensor_body[0], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[1], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[2], 0, 0, n_force);
   // }
      // if(timeP>=T/2&&timeP<=3*T/4)
   // {
     // dBodyAddRelForce(touch_sensor_body[1], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[3], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[2], 0, 0, n_force);
   // }
     // if(timeP>=3*T/4&&timeP<=T)
   // {
     // dBodyAddRelForce(touch_sensor_body[1], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[0], 0, 0, n_force);
     // dBodyAddRelForce(touch_sensor_body[3], 0, 0, n_force);
   // }
  
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  /*
   * This function needs to be implemented if you want to overide Webots collision detection.
   * It must return 1 if the collision was handled and 0 otherwise.
   * Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
   *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   *   dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
   *   dWorldID world = dBodyGetWorld(body1);
   *   ...
   *   pthread_mutex_lock(&mutex);
   *   dJointCreateContact(world, contact_joint_group, &contact[i])
   *   dJointAttach(contact_joint, body1, body2);
   *   pthread_mutex_unlock(&mutex);
   *   ...
   */
  return 0;
}

void webots_physics_cleanup() {
  /*
   * Here you need to free any memory you allocated in above, close files, etc.
   * You do not need to free any ODE object, they will be freed by Webots.
   */
  pthread_mutex_destroy(&mutex);
}