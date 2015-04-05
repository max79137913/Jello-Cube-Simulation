/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

// personal added funciton
double ComputeLength(Vector dest);
double ADotB(Vector A,Vector B);
Vector Normalize(Vector dest);
bool CollisionDetection(struct point pos,double boxsize,int *result);
bool IsOtherForce(struct world * jello);
Vector BackForce(struct world * jello,int i,int j, int k);
Vector InclinedForce(struct world * jello,int i,int j, int k);
Vector ExternalForce(struct world * jello,int i,int j, int k);
Vector ForceInSpring(struct world * jello,double RLength,struct point pos1, struct point pos2, struct point V1,struct point V2);
Vector MouseForce(struct world * jello);
#endif

