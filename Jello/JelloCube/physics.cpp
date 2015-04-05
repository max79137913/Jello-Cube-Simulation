/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <cmath>        // std::abs

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{

    Vector FFinal[8][8][8]={0};
   // F=ForceInSpring(jello,jello->p[0][0][0],jello->p[0][0][1],jello->v[0][0][0],jello->v[0][0][1]);

    int i,j,k,ip,jp,kp;
    Vector oneDirectF;
    double SR=JelloFixedR,HR=sqrt(2)*JelloFixedR,HHR=sqrt(3)*JelloFixedR,BR=2*JelloFixedR;


     #define PROCESS_NEIGHBOUR(di,dj,dk,R) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) ) \
    {\
     oneDirectF=ForceInSpring(jello,R,jello->p[i][j][k],jello->p[ip][jp][kp],jello->v[i][j][k],jello->v[ip][jp][kp]);\
     pSUM(FFinal[i][j][k],oneDirectF,FFinal[i][j][k]);\
    }\

   for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
            /////////////////////////Structural
            PROCESS_NEIGHBOUR(1,0,0,SR);
            PROCESS_NEIGHBOUR(0,1,0,SR);
            PROCESS_NEIGHBOUR(0,0,1,SR);
            PROCESS_NEIGHBOUR(-1,0,0,SR);
            PROCESS_NEIGHBOUR(0,-1,0,SR);
            PROCESS_NEIGHBOUR(0,0,-1,SR);
            //////////////////////////Shear
            PROCESS_NEIGHBOUR(1,1,0,HR);
            PROCESS_NEIGHBOUR(-1,1,0,HR);
            PROCESS_NEIGHBOUR(-1,-1,0,HR);
            PROCESS_NEIGHBOUR(1,-1,0,HR);
            PROCESS_NEIGHBOUR(0,1,1,HR);
            PROCESS_NEIGHBOUR(0,-1,1,HR);
            PROCESS_NEIGHBOUR(0,-1,-1,HR);
            PROCESS_NEIGHBOUR(0,1,-1,HR);
            PROCESS_NEIGHBOUR(1,0,1,HR);
            PROCESS_NEIGHBOUR(-1,0,1,HR);
            PROCESS_NEIGHBOUR(-1,0,-1,HR);
            PROCESS_NEIGHBOUR(1,0,-1,HR);

            PROCESS_NEIGHBOUR(1,1,1,HHR);
            PROCESS_NEIGHBOUR(-1,1,1,HHR);
            PROCESS_NEIGHBOUR(-1,-1,1,HHR);
            PROCESS_NEIGHBOUR(1,-1,1,HHR);
            PROCESS_NEIGHBOUR(1,1,-1,HHR);
            PROCESS_NEIGHBOUR(-1,1,-1,HHR);
            PROCESS_NEIGHBOUR(-1,-1,-1,HHR);
            PROCESS_NEIGHBOUR(1,-1,-1,HHR);
            ///////////////////////////Bend
            PROCESS_NEIGHBOUR(2,0,0,BR);
            PROCESS_NEIGHBOUR(0,2,0,BR);
            PROCESS_NEIGHBOUR(0,0,2,BR);
            PROCESS_NEIGHBOUR(-2,0,0,BR);
            PROCESS_NEIGHBOUR(0,-2,0,BR);
            PROCESS_NEIGHBOUR(0,0,-2,BR);
         ///////////////////////// count collision and response
         Vector BF;
         BF=BackForce(jello,i,j,k);
         pSUM(FFinal[i][j][k],BF,FFinal[i][j][k]);

         ///////////////////////// count inclined collision and response
         if(jello->incPlanePresent)
         {
         Vector INF;
         INF=InclinedForce(jello,i,j,k);
         pSUM(FFinal[i][j][k],INF,FFinal[i][j][k]);
         }
         ///////////////////////// count external force
         if(IsOtherForce(jello))
         {
         Vector EF;
         EF=ExternalForce(jello,i,j,k);
         pSUM(FFinal[i][j][k],EF,FFinal[i][j][k]);
         }
         ///////////////////////// count MouseForce
         if(isMouseForce)
         {
         Vector MF;
         MF=MouseForce(jello);
         pSUM(FFinal[i][j][k],MF,FFinal[i][j][k]);
         }

            a[i][j][k].x=FFinal[i][j][k].x/jello->mass;
            a[i][j][k].y=FFinal[i][j][k].y/jello->mass;
            a[i][j][k].z=FFinal[i][j][k].z/jello->mass;
      }
  if(isMouseForce){isMouseForce=false;}

  /* for you to implement ... */
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }

}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8],
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;
}
double ComputeLength(Vector dest)
{
    double length;
    length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);
    return length;
}
double ADotB(Vector A,Vector B)
{
   double result= A.x*B.x+A.y*B.y+A.z*B.z;
   return result;
}
Vector Normalize(Vector src)
{
    double length;
    Vector dest;
    length = sqrt((src).x * (src).x + (src).y * (src).y + (src).z * (src).z);


    (dest).x =src.x/length;
    (dest).y =src.y/length;
    (dest).z =src.z/length;

    return dest;
}
Vector ForceInSpring(struct world * jello,double RestLength,struct point pos1, struct point pos2, struct point V1,struct point V2)
{
     //double distanceR=1.0/7.0;
    double distanceL,RLength;
    Vector L,NorL;
    Vector FHook;
    //jello.p[0][0][0],jello.p[0][0][1];
    RLength=RestLength;

    pDIFFERENCE(pos1,pos2,L);
    distanceL=ComputeLength(L);
    NorL=Normalize(L);

    FHook.x = -1* jello->kElastic*(distanceL-RLength)*NorL.x;
    FHook.y = -1* jello->kElastic*(distanceL-RLength)*NorL.y;
    FHook.z = -1* jello->kElastic*(distanceL-RLength)*NorL.z;

    ///////////// FHook
    Vector V1_V2;
    Vector FDampling;

    pDIFFERENCE(V1,V2,V1_V2);
    ADotB(V1_V2,L);

    FDampling.x=-1*jello->dElastic*(ADotB(V1_V2,L)/distanceL)* NorL.x;
    FDampling.y=-1*jello->dElastic*(ADotB(V1_V2,L)/distanceL)* NorL.y;
    FDampling.z=-1*jello->dElastic*(ADotB(V1_V2,L)/distanceL)* NorL.z;

    ///////////// FDampling
    Vector FinalForce;

    pSUM(FHook,FDampling,FinalForce);

    return FinalForce;

}
Vector BackForce(struct world * jello,int i,int j, int k)
{
 int condition=0;

 if(CollisionDetection(jello->p[i][j][k],2.0,&condition))
{
    Vector FHook,L;
    Vector NorL;
  switch(condition){
/*
  case 1:   L.x=2-jello->p[i][j][k].x;  L.y=0;  L.z=0;break;
  case -1:  L.x=(-2)-jello->p[i][j][k].x;   L.y=0;  L.z=0;break;
  case 2:   L.x=0;  L.y=2-jello->p[i][j][k].y;  L.z=0;break;
  case -2:  L.x=0;  L.y=(-2)-jello->p[i][j][k].y;  L.z=0;break;
  case 3:   L.x=0;  L.y=0;  L.z=2-jello->p[i][j][k].z;break;
  case -3:  L.x=0;  L.y=0;  L.z=(-2)-jello->p[i][j][k].z;break;
*/
  case 1:   L.x=jello->p[i][j][k].x-2;  L.y=0;  L.z=0;break;
  case -1:  L.x=jello->p[i][j][k].x-(-2);   L.y=0;  L.z=0;break;
  case 2:   L.x=0;  L.y=jello->p[i][j][k].y-2;  L.z=0;break;
  case -2:  L.x=0;  L.y=jello->p[i][j][k].y-(-2);  L.z=0;break;
  case 3:   L.x=0;  L.y=0;  L.z=jello->p[i][j][k].z-2;break;
  case -3:  L.x=0;  L.y=0;  L.z=jello->p[i][j][k].z-(-2);break;

  }

///// FHook
  NorL=Normalize(L);
  FHook.x=-1*jello->kCollision*(ComputeLength(L)-0)*NorL.x;
  FHook.y=-1*jello->kCollision*(ComputeLength(L)-0)*NorL.y;
  FHook.z=-1*jello->kCollision*(ComputeLength(L)-0)*NorL.z;

///// FDampling
   Vector FDampling;
   Vector Va;
   Va.x=jello->v[i][j][k].x;
   Va.y=jello->v[i][j][k].y;
   Va.z=jello->v[i][j][k].z;

   FDampling.x=-1*jello->dCollision*(ADotB(Va,L))/ComputeLength(L)*NorL.x;
   FDampling.y=-1*jello->dCollision*(ADotB(Va,L))/ComputeLength(L)*NorL.y;
   FDampling.z=-1*jello->dCollision*(ADotB(Va,L))/ComputeLength(L)*NorL.z;

////////
    Vector FinalCollisionForce;

    pSUM(FHook,FDampling,FinalCollisionForce);

    return FinalCollisionForce;
}
 else{
     Vector zero;
        zero.x=0;
        zero.y=0;
        zero.z=0;
     return zero;
     }
}
Vector InclinedForce(struct world * jello,int i,int j, int k)
{
    double state;
    state=jello->a*jello->p[i][j][k].x+ jello->b*jello->p[i][j][k].y+ jello->c*jello->p[i][j][k].z+ jello->d;
   if(state*VerticesState[i][j][k]<=0)
   {
    Vector FHook,L;
    Vector PlaneNormal;
    Vector NorL;
    double distance;
    double ABSstate;
    ABSstate=std::abs(state);
    distance=ABSstate/(sqrt(jello->a * jello->a + jello->b * jello->b + jello->c * jello->c));
    PlaneNormal.x=-jello->a;
    PlaneNormal.y=-jello->b;
    PlaneNormal.z=-jello->c;
    NorL=Normalize(PlaneNormal);
    L.x=NorL.x*distance;
    L.y=NorL.y*distance;
    L.z=NorL.z*distance;

    ///// FHook
    FHook.x=-1*jello->kCollision*(distance-0)*NorL.x;
    FHook.y=-1*jello->kCollision*(distance-0)*NorL.y;
    FHook.z=-1*jello->kCollision*(distance-0)*NorL.z;
    ///// FDampling
    Vector FDampling;
    Vector Va;
    Va.x=jello->v[i][j][k].x;
    Va.y=jello->v[i][j][k].y;
    Va.z=jello->v[i][j][k].z;

    if(distance==0){distance=1;}
    FDampling.x=-1*jello->dCollision*(ADotB(Va,L))/distance*NorL.x;
    FDampling.y=-1*jello->dCollision*(ADotB(Va,L))/distance*NorL.y;
    FDampling.z=-1*jello->dCollision*(ADotB(Va,L))/distance*NorL.z;
    ////////
    Vector FinalInclinedForce;

    pSUM(FHook,FDampling,FinalInclinedForce);

    return FinalInclinedForce;

   }
   else
   {
     Vector zero;
        zero.x=0;
        zero.y=0;
        zero.z=0;
     return zero;

   }
}
bool CollisionDetection(struct point pos,double boxsize,int *result)
{
    int i,j,k;
/*
     if(pos.x>boxsize||pos.x<-1*boxsize||pos.y>boxsize||pos.y<-1*boxsize
           ||pos.z>boxsize||pos.z<-1*boxsize)
           {
                printf("%lf\n",pos.x);
                printf("%lf\n",pos.y);
                printf("%lf\n",pos.z);
               // return true;
           }
*/
    if(pos.x>boxsize){*result=1;return true;}else if(pos.x<-1*boxsize){*result=-1;return true;}
    else if(pos.y>boxsize){*result=2;return true;}else if(pos.y<-1*boxsize){*result=-2;return true;}
    else if(pos.z>boxsize){*result=3;return true;}else if(pos.z<-1*boxsize){*result=-3;return true;}


    else{return false;}

}
bool IsOtherForce(struct world * jello)
{
  if(jello->resolution>0&&(EXforce%2)){return true;}
  else{return false;}


}
Vector ExternalForce(struct world * jello,int i,int j, int k)
{
    point pos;

    pos.x=((jello->p[i][j][k].x+2)/4.0)*(jello->resolution-1); //change to force field coordinate
    pos.y=((jello->p[i][j][k].y+2)/4.0)*(jello->resolution-1);
    pos.z=((jello->p[i][j][k].z+2)/4.0)*(jello->resolution-1);

    if(pos.x>(jello->resolution-1)){pos.x=(jello->resolution-1);}else if(pos.x<0){pos.x=0;}// boundary check
    if(pos.y>(jello->resolution-1)){pos.y=(jello->resolution-1);}else if(pos.y<0){pos.y=0;}
    if(pos.z>(jello->resolution-1)){pos.z=(jello->resolution-1);}else if(pos.z<0){pos.z=0;}

    int xf,xc,yf,yc,zf,zc;  // f=floor c=ceil
    double s,t,g; //distance to each axis

    xf=floor(pos.x); xc=ceil(pos.x);
    yf=floor(pos.y); yc=ceil(pos.y);
    zf=floor(pos.z); zc=ceil(pos.z);

    s=pos.x-xf; //x
    t=pos.y-yf; //y
    g=pos.z-zf; //z

    Vector ExternalF;

    ExternalF.x= (1-s)*(1-t)*(1-g)*jello->forceField[xf * jello->resolution * jello->resolution + yf * jello->resolution + zf].x
                +s*(1-t)*(1-g)*jello->forceField[xc * jello->resolution * jello->resolution + yf * jello->resolution + zf].x
                +(1-s)*t*(1-g)*jello->forceField[xf * jello->resolution * jello->resolution + yc * jello->resolution + zf].x
                +s*t*(1-g)*jello->forceField[xc * jello->resolution * jello->resolution + yc * jello->resolution + zf].x
                +s*(1-t)*g*jello->forceField[xc * jello->resolution * jello->resolution + yf * jello->resolution + zc].x
                +(1-s)*t*g*jello->forceField[xf * jello->resolution * jello->resolution + yc * jello->resolution + zc].x
                +(1-s)*(1-t)*g*jello->forceField[xf * jello->resolution * jello->resolution + yf * jello->resolution + zc].x
                +s*t*g*jello->forceField[xc * jello->resolution * jello->resolution + yc * jello->resolution + zc].x;
    ExternalF.y= (1-s)*(1-t)*(1-g)*jello->forceField[xf * jello->resolution * jello->resolution + yf * jello->resolution + zf].y
                +s*(1-t)*(1-g)*jello->forceField[xc * jello->resolution * jello->resolution + yf * jello->resolution + zf].y
                +(1-s)*t*(1-g)*jello->forceField[xf * jello->resolution * jello->resolution + yc * jello->resolution + zf].y
                +s*t*(1-g)*jello->forceField[xc * jello->resolution * jello->resolution + yc * jello->resolution + zf].y
                +s*(1-t)*g*jello->forceField[xc * jello->resolution * jello->resolution + yf * jello->resolution + zc].y
                +(1-s)*t*g*jello->forceField[xf * jello->resolution * jello->resolution + yc * jello->resolution + zc].y
                +(1-s)*(1-t)*g*jello->forceField[xf * jello->resolution * jello->resolution + yf * jello->resolution + zc].y
                +s*t*g*jello->forceField[xc * jello->resolution * jello->resolution + yc * jello->resolution + zc].y;

    ExternalF.z= (1-s)*(1-t)*(1-g)*jello->forceField[xf * jello->resolution * jello->resolution + yf * jello->resolution + zf].z
                +s*(1-t)*(1-g)*jello->forceField[xc * jello->resolution * jello->resolution + yf * jello->resolution + zf].z
                +(1-s)*t*(1-g)*jello->forceField[xf * jello->resolution * jello->resolution + yc * jello->resolution + zf].z
                +s*t*(1-g)*jello->forceField[xc * jello->resolution * jello->resolution + yc * jello->resolution + zf].z
                +s*(1-t)*g*jello->forceField[xc * jello->resolution * jello->resolution + yf * jello->resolution + zc].z
                +(1-s)*t*g*jello->forceField[xf * jello->resolution * jello->resolution + yc * jello->resolution + zc].z
                +(1-s)*(1-t)*g*jello->forceField[xf * jello->resolution * jello->resolution + yf * jello->resolution + zc].z
                +s*t*g*jello->forceField[xc * jello->resolution * jello->resolution + yc * jello->resolution + zc].z;

    return ExternalF;

}
Vector MouseForce(struct world * jello)
{
  //double distanceR=1.0/7.0;
    double distanceL;
    double magnitude=0.8;
    Vector L,NorL;
    Vector MouseF;

    point pos1,pos2;
    pos1.x=0; pos2.x=0;
    pos1.y=UpPos[0];pos2.y=DownPos[0];
    pos1.z=-UpPos[1];pos2.z=-DownPos[1];

    pDIFFERENCE(pos1,pos2,L);

    distanceL=ComputeLength(L);
    if(distanceL!=0)NorL=Normalize(L);
    else{NorL.x=0;NorL.y=0;NorL.z=0;}

    MouseF.x =  magnitude*(distanceL)*NorL.x;
    MouseF.y =  magnitude*(distanceL)*NorL.y;
    MouseF.z =  magnitude*(distanceL)*NorL.z;

    return MouseF;


}

