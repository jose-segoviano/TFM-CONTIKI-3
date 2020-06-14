#include "net/rpl/rpl.h"
#include <stdio.h>
//#define THETA1  -0.5875 // Variable for Sky and Z1 motes
//#define THETA2  -5.5794 // Variable for Sky and Z1 motes
#define THETA1  -0.3928 // Variable for wismote motes
#define THETA2  -15.921 // Variable for wismote motes

/*---------------------------------------------------------------------------*/
int32_t convert_distancia(int32_t rssi){
  return THETA1*rssi + THETA2;
}
/*---------------------------------------------------------------------------*/
void set_node_position_calculated(rpl_node_position_t *node_position) {
  if (node_position->x[1] != 0 && node_position->x[2] != 0 && node_position->x[3] != 0) {
    int32_t d_aux, x, y;
    long a, b, c;
    int32_t r1x, r2x, r3x, r1y, r2y, r3y;
    r1x = node_position->x[1];
    r2x = node_position->x[2];
    r3x = node_position->x[3];

    r1y = node_position->y[1];
    r2y = node_position->y[2];
    r3y = node_position->y[3];

    d_aux = convert_distancia(node_position->rssi[1]);
    a = ((r1x*r1x) + (r1y*r1y) - (d_aux*d_aux));
    d_aux = convert_distancia(node_position->rssi[2]);
    b = ((r2x*r2x) + (r2y*r2y) - (d_aux*d_aux));
    d_aux = convert_distancia(node_position->rssi[3]);
    c = ((r3x*r3x) + (r3y*r3y) - (d_aux*d_aux));

    long Y32, Y13, Y21, X32, X13, X21;
    X32 = (r3x - r2x);
    X13 = (r1x - r3x);
    X21 = (r2x - r1x);
    Y32 = (r3y - r2y);
    Y13 = (r1y - r3y);
    Y21 = (r2y - r1y);

    x = (a*Y32 + b*Y13 + c*Y21)/(2*(r1x*Y32 + r2x*Y13 + r3x*Y21));
    y = (a*X32 + b*X13 + c*X21)/(2*(r1y*X32 + r2y*X13 + r3y*X21));
    node_position->x[0] = x;
    node_position->y[0] = y;
    node_position->last_update[0] = clock_seconds();
  }
}
