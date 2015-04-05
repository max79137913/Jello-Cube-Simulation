<Please submit this file with your solution.>

CSCI 520, Assignment 1  <Po-Ting Kuo>

<Coding Environment>

CodeBlock 2012 Version 10.05

<Description of what you have accomplished>

My world coordinate is

    z   
    |  
    |              
    |------>y
   / 
  /
 x
1. Mass spring system.(structural,shear,bend spring)---I use Hook's law for creating the spring force and dampling force to slow down the motion. Also, I implement these force on the network of spring(structural,shear,bend spring). All are followed from the slides from helper slides.

2. Incorporate a bounding box, including collision detection and response--- I set up a CollisionDetection function to detect the collision. Once it detected collision it will detect which face that it is collided then give a force to push it back. 

3. Implement an arbitrary non-homogeneous time-independent force field, with appropriate force-field interpolation(ex. external.w)--- set the IsOtherForce function to detect whether has force applied. 

If it is, I transform each points in the cube to force field coordinate and see how much force each points get by interpolating the eight force nodes around the cube.(Similar like 3D texture mapping) Apply that force in each points in cube, also, you can press ¡¨f¡¨ to deactivate or reactivate it.  

4. Able to produce JPEG frames and do the animation.

Related function:
bool CollisionDetection(struct point pos,double boxsize,int *result);
bool IsOtherForce(struct world * jello);
Vector BackForce(struct world * jello,int i,int j, int k);
Vector ExternalForce(struct world * jello,int i,int j, int k);
Vector ForceInSpring(struct world * jello,double RLength,struct point pos1, struct point pos2, struct point V1,struct point V2);


<Also, explain any extra credit that you have implemented.>

1. If the world file has external force, you can push 'f' to deactivate it or active it again.(ex. external.w)
2. Implement collision detection with inclined plane---

If there is an inclined plane, once the cube has touched it, cube will be pushed away. And the inclined plane is "invisible". (According to helper slide, initally all points on the same side of plane)

And the world file is incline.w(has an inclined plane in X+Y=0) or inclinez.w(z=-1.5)

First check every point whether they are on the same side of inclined plane. If the points in cube cross the plain then give it a force to push it back. And that force is produced by calculating the L vector and same method as incorporating a bounding box. 

3. Drag the mouse to give the force to all the vertices in cube.
For example, press the left button and drag the mouse from left to right and release will cause the cube get a force from left to right.(up or down too, and any Direction is ok.) I also printf the mouse location from start to end. To do this, by calculating the length from start point to end point. And give a coefficient to modify the force, then apply that force to every points in cube.


4. Set up texture mapping to the cube. 
While pressing "t" and the cube is not at triangle rendering mode. It will apply the checkerboard texture to the cube. 
I accomplish this by creating the checkerboard image first, then use glGenTextures(1, &texName); glBindTexture(GL_TEXTURE_2D, texName),etc¡Kfunction to create texture, then apply into the cube when press the¡¨t¡¨. 


f: activate or deactivate the external force
t: display checkerboard texture or hide it

Related function:
bool IsOtherForce(struct world * jello);
Vector InclinedForce(struct world * jello,int i,int j, int k);
Vector MouseForce(struct world * jello);
void showTexture(struct world * jello);

<Animation>

In 300 pictures, first I give the cube velocity to collide the invisable inclined plain(X+Y=0). Then, when the cube be pushed back,
I use the mouse and drag from up to down to give the cube a force which causes the cube move down.
Next, I press "t" and close the triangle rendering to show my checkerboard texture. 


