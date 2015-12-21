// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*

    This is an example illustrating the use of the linear model predictive
    control tool from the dlib C++ Library.  To explain what it does, suppose
    you have some process you want to control and the process dynamics are
    described by the linear equation:
        x_{i+1} = A*x_i + B*u_i + C
    That is, the next state the system goes into is a linear function of its
    current state (x_i) and the current control (u_i) plus some constant bias or
    disturbance.  
                
    A model predictive controller can find the control (u) you should apply to
    drive the state (x) to some reference value, which is what we show in this
    example.  In particular, we will simulate a simple vehicle moving around in
    a planet's gravity.  We will use MPC to get the vehicle to fly to and then
    hover at a certain point in the air.
    
*/

#include<iostream>
#include <dlib/gui_widgets.h>
#include <dlib/control.h>
#include <dlib/image_transforms.h>
#include <math.h>

#include <mgl2/mgl.h>
#include <mgl2/qt.h>
#include <mgl2/glut.h>

using namespace std;
using namespace dlib;

//  ----------------------------------------------------------------------------

int sample(mglGraph*);

void run_mpc();

//  ----------------------------------------------------------------------------
const int STATES = 4;
const int CONTROLS = 2;
const int HORIZON = 30;
const int STEPS = 630;
const int WIN_SIZE = 400;

double dt = 0.1;

int main()
{

  run_mpc();

  return 0;
}

//  ----------------------------------------------------------------------------

void run_mpc() {

  matrix<double,STATES,STATES> A;
  A = 1, 0, dt,  0,  // next_pos = pos + velocity
      0, 1,  0, dt,  // next_pos = pos + velocity
      0, 0,  1,  0,  // next_velocity = velocity
      0, 0,  0,  1;  // next_velocity = velocity

  matrix<double,STATES,CONTROLS> B;
  B =  0,  0,
       0,  0,
      dt,  0,
       0, dt;

  matrix<double,STATES,1> C;
  C = 0,
      0,
      0,
      0;

  matrix<double,STATES,1> Q;
  Q = 1, 1, 0, 0;

  matrix<double,CONTROLS,1> R, lower, upper;
  R = 1, 1;
  lower = -5, -5;
  upper =  5,  5;

  // Finally, create the MPC controller.
  mpc<STATES,CONTROLS,HORIZON> controller(A,B,C,Q,R,lower,upper);

  // generate random target
  dlib::rand rnd;
  matrix<double,STATES,STEPS> target;

  for (int i = 0; i<STEPS; i++) {
    target(0,i) = 50*cos(0.1*i*dt) + 200;
    target(1,i) = 50*sin(0.1*i*dt) + 200;
    if (i==0) {
      target(2,i) = 0;
      target(3,i) = 5;
    } else {
      target(2,i) = ( target(0,i) - target(0,i-1) ) /dt;
      target(3,i) = ( target(1,i) - target(1,i-1) ) /dt;
    }
  }

  cout << target << endl;

  // world
  matrix<rgb_pixel> world(WIN_SIZE,WIN_SIZE);
  image_window win;
  matrix<double,STATES,1> current_state;

  // initial state
  current_state = 250,200,0,0;

  int i = 0;
  while(!win.is_closed())
  {
    // draw scene
    assign_all_pixels(world, rgb_pixel(255,255,255));
    const dpoint pos = point(current_state(0),current_state(1));
    const dpoint goal = point(target(0,i),target(1,i+1));
    draw_solid_circle(world, goal, 9, rgb_pixel(100,255,100));
    draw_solid_circle(world, pos, 7, 0);

    // update target
    controller.set_target(colm(target,i));

    // find the best control action given our current state.
    matrix<double,CONTROLS,1> action = controller(current_state);
    cout << "best control: " << trans(action);

    // take a step in the simulation
    current_state = A*current_state + B*action + C;

    // draw the control action.
    draw_line(world, pos, pos-5*action, rgb_pixel(255,0,0));
    win.set_image(world);

    // sleep and update trajectory step
    dlib::sleep(dt*1000);
    i++;
    if (i > STEPS)
      i = 0;
  }

}
