#include <iostream>   // For string handling
#include <stdlib.h>	  // Just for some standard functions
#include <stdio.h>    // Just for some ASCII messages
#include <glut.h>     // An interface and windows management library
#include <vector>
#include "visuals.h"  // Header file for our OpenGL functions
#include "Matrices.h"
#include "Vectors.h"
#include "Quaternion.h"
#include "State.h"
#include "BoxArea.h"
#include "ColorModel.h"
#include "RandomUtil.h"
#include "Vector3D.h"
#include "Edge3D.h"
#include "Triangle3D.h"
#include "Contact.h"
#include "RigidBody.h"
#include "Sphere.h"
#include "Cube.h"
#include "Physics.h"
#include <fstream>
#include <sstream>
#include <string>

#define KEY_ESC 27
#define PI 3.1415927

using namespace std;

/*******************************************************************************
  State Variables
 ******************************************************************************/

float translateY = 0.0f;
float vel = 40.0;
float aspect;

/*******************************************************************************
  Implementation of Auxiliary Functions
 ******************************************************************************/

BoxArea *boxArea;
ColorModel *colorModel;
RandomUtil *randomUtil;
int pressedButton,prevX,prevY,mouse_state;
bool ctrl_pressed;

//Sphere
vector<Sphere*> spheres;
vector<Vector3D*> m_vertices;
vector<Triangle3D> m_triangles;
vector<bool> showTriangles;

//Cube
vector<Cube*> cubes;
vector<Vector3D*> c_vertices;
vector<Triangle3D> c_triangles;

int currentTriangle=0;
int refine_times=1;
float dtime=0,t=0,dt = 0.002;
bool visible_lines;

float width=0,height=0;

int mode=0;
int sphere_cube_mode=1;

Physics *physicsModel;

float H_input,S_input,V_input;
bool default_HSV;

float distance(Vector3D *v1,Vector3D *v2);

void loadModel(char* c,vector<Vector3D*> &v,vector<Triangle3D> &t);

float frameCount = 0;
float currentTime=0;
float previousTime=0;
float fps;

void writeText(char *str, void *font)
{
    //-------------Task3a---------------
	//glPushMatrix()
    char *c;
    for (c = str; *c != '\0'; c++){
        glutStrokeCharacter(font, *c);
	}
	//glPopMatrix();
}

void drawString(char *str){
	glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
	writeText(str,GLUT_STROKE_ROMAN);
	glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
}

//------------------------------------------------------------------------------

/*******************************************************************************
  Implementation of Visual Functions
 ******************************************************************************/

void setup()
{ 
    // Parameter handling

    // Make models with smooth edges
    glShadeModel(GL_SMOOTH);
    
    //-------------Task2b---------------
    //*
    // Points represented as circles
    glEnable(GL_POINT_SMOOTH);
    // Set point size
	glPointSize(3.0);

	glEnable(GL_LINE_SMOOTH);
    // Set Line width
    glLineWidth(2.0);
    //*/

    // Blending
    glEnable(GL_BLEND);
    //           incoming             stored
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    //-------------Task7a---------------
    //*
    // Depth test
    glEnable(GL_DEPTH_TEST);
    // Renders a fragment if its z value is less or equal of the stored value
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0f);
    //*/
    
    // Polygon rendering mode
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    
    // Set up light source
    GLfloat lightPosition[]  = {40.0, 40.0, 150.0, 1.0};
    GLfloat lightAmbient[]   = { 0.2,  0.2,   0.2, 1.0};
    GLfloat lightDiffuse[]   = { 0.8,  0.8,   0.8, 1.0};
     
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 90.0);
    //glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 0.1);

    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    
    // polygon rendering mode and material properties
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // Black background
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	//init stuff
	boxArea = new BoxArea();
	colorModel = new ColorModel();
	randomUtil = new RandomUtil();

	loadModel("models/icosahedron.obj",m_vertices,m_triangles);
	//loadModel("models/B_sphere4.obj");
	loadModel("models/cube.obj",c_vertices,c_triangles);

	//refine triangles of icosahedron to icosphere
	float radius=1.0;
	
	int N;
	for(int k=0;k<2;k++){
		N = m_triangles.size();
		for(int i=0;i<N;i++){
			if(showTriangles[i]==false)continue;
			Triangle3D tri = m_triangles[i];

			Vector3D n1 = tri.linear1(0.5);
			Vector3D n2 = tri.linear2(0.5);
			Vector3D n3 = tri.linear3(0.5);

			//Vector3D norm = tri->normal();
			//n1 = (*n1) + (*((*norm)*(radius-n1->length())));
			//n2 = (*n2) + (*((*norm)*(radius-n2->length())));
			//n3 = (*n3) + (*((*norm)*(radius-n3->length())));
			n1 = n1.normalize();
			n2 = n2.normalize();
			n3 = n3.normalize();
			tri.v1 = tri.v1.normalize();
			tri.v2 = tri.v2.normalize();
			tri.v3 = tri.v3.normalize();

			//Vector3D *center = tri->centroid();
			Triangle3D t1 = Triangle3D(tri.v1,n1,n3);
			Triangle3D t2 = Triangle3D(tri.v2,n2,n1);
			Triangle3D t3 = Triangle3D(tri.v3,n3,n2);
			Triangle3D t4 = Triangle3D(n1,n2,n3);


			//m_triangles.push_back(t1);
			m_triangles[i]=t1;
			m_triangles.push_back(t2);m_triangles.push_back(t3);m_triangles.push_back(t4);
			showTriangles.push_back(true);showTriangles.push_back(true);showTriangles.push_back(true);showTriangles.push_back(true);
			//showTriangles[i]=false;
			
		}
	}
	cout << m_triangles.size() << endl;


	//centralize cube
	Vector3 cm = Vector3(0,0,0);
	for(int i=0;i<c_vertices.size();i++){
		Vector3D v = *c_vertices[i];
		cm = cm+v.toVec3();
	}
	cm /= ((float)c_vertices.size());

	for(int i=0;i<c_triangles.size();i++){
		Triangle3D tri = c_triangles[i];
		Vector3D v1 = tri.getV1();
		Vector3D v2 = tri.getV2();
		Vector3D v3 = tri.getV3();

		//v1 = v1-0.5;
		//v2 = v2-0.5;
		//v3 = v3-0.5;
		v1 = v1-cm;
		v2 = v2-cm;
		v3 = v3-cm;

		tri.setV1(v1);tri.setV2(v2);tri.setV3(v3);
		c_triangles[i] = tri;
	}

	for(int i=0;i<c_vertices.size();i++){
		Vector3D &v =*c_vertices[i];
		v = v-cm;
	}

	//spheres.push_back(new Sphere(3,0,0,0,0));
	physicsModel = new Physics();

	
	//cout << "Default HSV?";
	string answer;
	cout << "Default HSV?(Y/N)";
	getline(cin,answer);
	if(answer == "Y")default_HSV=true;
	else default_HSV=false;

	if(default_HSV==false){
		string hsv_string;
		cout << "Set H S V Color :";
		getline(cin,hsv_string);
		stringstream ssin(hsv_string);
		string hsv_arr[3];
		int i=0;
		while (ssin.good() && i < 3){
			ssin >> hsv_arr[i];++i;
		}
		H_input = stof(hsv_arr[0]);
		S_input = stof(hsv_arr[1]);
		V_input = stof(hsv_arr[2]);
	}
}


void loadModel(char *s,vector<Vector3D*> &m_vertices,vector<Triangle3D> &m_triangles){
	//read file
	std::ifstream infile(s);
	std::string line;

	//for splitting string into array
	string arr[4];
	int i = 0;
	

	//parse every line
	while (std::getline(infile, line)){
		stringstream ssin(line);
		
		if(line.size()==0)continue;
		if(line[0]=='v'){
			
			 while (ssin.good() && i < 4){
				ssin >> arr[i];++i;
			 }
			 m_vertices.push_back(new Vector3D(stof(arr[1]),stof(arr[2]),stof(arr[3])));
		}
		else if(line[0]=='f'){
			while (ssin.good() && i < 4){
				ssin >> arr[i];++i;
			}
			m_triangles.push_back(Triangle3D(m_vertices[stoi(arr[1])-1],m_vertices[stoi(arr[2])-1],m_vertices[stoi(arr[3])-1]));
			showTriangles.push_back(true);
			
		}
		i=0;
		//cout << line<<endl;

	}
}

void refine(){


	if(currentTriangle==20*powf(4,refine_times)){
		currentTriangle=0;
		refine_times++;
	}

	int i = currentTriangle;
	Triangle3D tri = m_triangles[i];

	Vector3D n1 = tri.linear1(0.5);
	Vector3D n2 = tri.linear2(0.5);
	Vector3D n3 = tri.linear3(0.5);

	//Vector3D norm = tri->normal();
	//n1 = (*n1) + (*((*norm)*(radius-n1->length())));
	//n2 = (*n2) + (*((*norm)*(radius-n2->length())));
	//n3 = (*n3) + (*((*norm)*(radius-n3->length())));
	n1 = n1.normalize();
	n2 = n2.normalize();
	n3 = n3.normalize();
	tri.v1 = tri.v1.normalize();
	tri.v2 = tri.v2.normalize();
	tri.v3 = tri.v3.normalize();

	//Vector3D *center = tri->centroid();
	Triangle3D t1 = Triangle3D(tri.v1,n1,n3);
	Triangle3D t2 = Triangle3D(tri.v2,n2,n1);
	Triangle3D t3 = Triangle3D(tri.v3,n3,n2);
	Triangle3D t4 = Triangle3D(n1,n2,n3);


	//m_triangles.push_back(t1);
	m_triangles[i]=t1;
	m_triangles.push_back(t2);m_triangles.push_back(t3);m_triangles.push_back(t4);
	//showTriangles.push_back(true);showTriangles.push_back(true);showTriangles.push_back(true);showTriangles.push_back(true);
	//showTriangles[i]=false;
	currentTriangle++;
	
}

//------------------------------------------------------------------------------

void render()
{    

	
    // Clean up the colour of the window and the depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
	
	//draw 2d String in 3d world
	/*glColor3f(1.0,0.4,0.2);
	glPushMatrix();
	
	glTranslatef(0,0,-10);
	glRotatef(56,0,1,0);
	glScalef(0.01,0.01,0.01);
	
    drawString("Hello World!");
	glPopMatrix();*/

	glPushMatrix();
	GLfloat lightDirection[]  = {0.0, -1.0,  0.0, 0.0};
	GLfloat lightPosition[]  = { 0.0, 20.0,   0.0, 1.0};
    GLfloat lightAmbient[]   = { 0.2,  0.2,   0.2, 1.0};
    GLfloat lightDiffuse[]   = { 0.8,  0.8,   0.8, 1.0};
     
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, lightDirection);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glPopMatrix();

	//draw Box Area
	boxArea->transform();
	colorModel->displayHSV(0.5,0.3,0.7);
	boxArea->render();

	

	physicsModel->checkObjectCollision();
	if(!spheres.empty()){
		//physicsModel->checkWallCollision();
		//physicsModel->checkSphereCollision();
		//physicsModel->checkSphereWallCollision();
		//physicsModel->calculateMinDist();

		for(int i=0;i<spheres.size();i++){
			
			//spheres[i]->move();
			spheres[i]->update(t);
			spheres[i]->transform();
			//spheres[i]->update();
			physicsModel->calculateMinDist(i,0);
			spheres[i]->render();
		}
	}

	if(!cubes.empty()){
		//physicsModel->checkCubeCollision();
		for(int i=0;i<cubes.size();i++){
			//cubes[i]->reset();
			cubes[i]->update(t);
			//cubes[i]->move();
			cubes[i]->transform();
			physicsModel->calculateMinDist(i,1);
			cubes[i]->render();
		}
	}
	
	if(mode==1){

		vector<Triangle3D> tris;
		if(sphere_cube_mode==0)tris=m_triangles;
		else tris = c_triangles;

		glPushMatrix();
		glScalef(4,4,4);
	
		for(int i=0;i<tris.size();i++){
			//if(showTriangles[i]==false) continue;

			Triangle3D tri = tris[i];
			Vector3D v1 = tri.getV1();
			Vector3D v2 = tri.getV2();
			Vector3D v3 = tri.getV3();

			glColor3f(0,1,0);
			glBegin(GL_TRIANGLES);
			{


				colorModel->displayHSV(v1.phi(),v1.slength(),0.5*(1+v1.y));
				v1.r=colorModel->getRed();v1.g=colorModel->getGreen();v1.b=colorModel->getBlue();
				glColor3f(v1.r,v1.g,v1.b);
				glVertex3f(v1.x,v1.y,v1.z);
			
				colorModel->displayHSV(v2.phi(),v2.slength(),0.5*(1+v2.y));
				v2.r=colorModel->getRed();v2.g=colorModel->getGreen();v2.b=colorModel->getBlue();
				glColor3f(v2.r,v2.g,v2.b);
				glVertex3f(v2.x,v2.y,v2.z);
			
				colorModel->displayHSV(v3.phi(),v3.slength(),0.5*(1+v3.y));
				v3.r=colorModel->getRed();v3.g=colorModel->getGreen();v3.b=colorModel->getBlue();
				glColor3f(v3.r,v3.g,v3.b);
				glVertex3f(v3.x,v3.y,v3.z);
			}
			glEnd();

		}
		glPopMatrix();
	}


	boxArea->drawWalls();
	calculateFPS();
   /*
    glTranslatef(-10.0, -20.0, -150.0);
    glRotatef(-30, 0, 1, 0);
    
    glColor3f(0.5, 0.5, 0.8);
    
    glTranslatef(-100, 35, 50);
    glScalef(0.15, 0.15, 0.15);
    writeText("Graphics Lab 2", GLUT_STROKE_ROMAN);
    //writeText("Graphics Lab 2", GLUT_STROKE_MONO_ROMAN);
    //*/
    
    // All drawing commands applied to the
    // hidden buffer, so now, bring forward
    // the hidden buffer and hide the visible one
    glutSwapBuffers();
}

//------------------------------------------------------------------------------

void resize(int w, int h)
{
  // w and h are window sizes returned by glut
  // define the visible area of the window ( in pixels )

  width = w;height=h;
  if (h==0) h=1;

  // Aspect ratio
  aspect = (float)w/(float)h;

  glViewport(0, 0, w, h);
  //glViewport(0, h/2, w/2, h/2);

  // Setup viewing volume
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  /*gluPerspective(45.0, aspect, 1.0, -200.0);

  glViewport(0, 0, w/2, h/2);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();*/

  gluPerspective(45.0, aspect, 1.0, -200.0);
  //         L,      R,       B,      T,       N,       F
  //glOrtho(-100.0f, 100.0f, -100.0f, 100.0f, -200.0f, 200.0f);
  
  //gluPerspective(135.0, aspect, 1.0, -200.0);
  
  
}

//------------------------------------------------------------------------------

void idle()
{
    // Make your changes here
    //-------------Task6a---------------
    /*
    float g = 10;
    float dt = 0.5;

    if (translateY < 0) vel = -vel;
    else vel = vel - g*dt;

    translateY = translateY + vel * dt;
    //*/
	//dtime+= 0.01;
	/*if(!cubes.empty()){
		for(int i=0;i<cubes.size();i++){
			cubes[i]->update(t);
		}
	}*/

	t = t+dt;

    // After your changes rerender
    glutPostRedisplay();
}

//general functions
void generateSphere(){

	float r,x,y,z;
	bool doFit=false;
	bool isFree;
	while(doFit==false){
		r = randomUtil->randFloat(1,3);
		printf("%f\n",r);

		int loopI=0;
	
		doFit=true;
		isFree=true;
		do{
			x = randomUtil->randFloat(r,20-r);
			y = randomUtil->randFloat(r,20-r);
			z = randomUtil->randFloat(r,20-r);
			isFree=true;
			loopI++;
			if(loopI>1){
				//cant fit sphere with specific r
				//return;
				doFit=false;
				break;
			}
			for(int i=0;i<spheres.size();i++){
				//if(r1+r2>distance())
				Sphere *s = spheres[i];
				if((r+s->getRadius()>distance(new Vector3D(x,y,z),new Vector3D(s->getX(),s->getY(),s->getZ())))){
					isFree=false;
					break;
				}
			}
		
		}while(isFree==false);
	}	
	

	float velX,velY,velZ;
	bool isVelInSphere;
	do{
		isVelInSphere=true;
		//0,20 default
		velX = randomUtil->randFloat(-80,80);
		velY = randomUtil->randFloat(-80,80);
		velZ = randomUtil->randFloat(-80,80);
		if(sqrt((velX-x)*(velX-x)+(velY-y)*(velY-y)+(velZ-z)*(velZ-z))>r)isVelInSphere=false;
	}while(isVelInSphere==true);
	
	Sphere *sphere = new Sphere(Vector3(r,r,r),Vector3(velX,velY,velZ),Vector3(x,y,z),&visible_lines,&mode,&fps);
	//colorModel->displayHSV(log10f(randomUtil->randFloat(1,100))/log10f(100),randomUtil->randFloat(0,1),0.7);
	//sphere->setHSVColor(colorModel->getHue(),colorModel->getSaturation(),colorModel->getBrightness());
	//sphere->setRGBColor(colorModel->getRed(),colorModel->getGreen(),colorModel->getBlue());
	sphere->setColorModel(colorModel);
	vector<Vector3D*> new_vertices(m_vertices);
	vector<Triangle3D> new_triangles(m_triangles);
	sphere->setVertices(new_vertices);
	sphere->setTriangles(new_triangles);
	sphere->refine(0);
	if(default_HSV==true)sphere->setHSVColor();
	else sphere->setHSVColor(H_input,S_input,V_input);
	sphere->build(0);
	
	spheres.push_back(sphere);
	physicsModel->addSphere(sphere);
	cout << "Spheres: " << spheres.size() << endl;
}

void generateCube(){
	float size,r;
	float x,y,z;

	bool doFit=false;
	bool isFree;
	while(doFit==false){
		size = randomUtil->randFloat(1,3);
		r = sqrt(3.)*size/2;
		//printf("%f\n",size);

		int loopI=0;
	
		doFit=true;
		isFree=true;
		do{
			x = randomUtil->randFloat(r,20-r);
			y = randomUtil->randFloat(r,20-r);
			z = randomUtil->randFloat(r,20-r);
			isFree=true;
			loopI++;
			if(loopI>1){
				//cant fit sphere with specific r
				//return;
				doFit=false;
				break;
			}
			for(int i=0;i<cubes.size();i++){
				//if(r1+r2>distance())
				Cube *c = cubes[i];
				if((r+c->getRadius()>distance(new Vector3D(x,y,z),new Vector3D(c->getX(),c->getY(),c->getZ())))){
					isFree=false;
					break;
				}
			}
		
		}while(isFree==false);
	}	

	float velX,velY,velZ;
	bool isVelInCube;
	do{
		isVelInCube=true;
		//0,20 default
		//2nd set: 30,80
		velX = randomUtil->randFloat(-80,80);
		velY = randomUtil->randFloat(-80,80);
		velZ = randomUtil->randFloat(-80,80);
		if(sqrt((velX-x)*(velX-x)+(velY-y)*(velY-y)+(velZ-z)*(velZ-z))>r)isVelInCube=false;
	}while(isVelInCube==true);
	//cout << "Hello man!" << endl;

	Cube *cube = new Cube(Vector3(size,size,size),Vector3(velX,velY,velZ),Vector3(x,y,z),&visible_lines,&mode,&fps);
	cube->setColorModel(colorModel);
	cubes.push_back(cube);
	vector<Vector3D*> new_vertices(c_vertices);
	vector<Triangle3D> new_triangles(c_triangles);
	cube->setVertices(new_vertices);
	cube->setTriangles(new_triangles);
	//cube->refine(1);
	if(default_HSV==true)cube->setHSVColor();
	else cube->setHSVColor(H_input,S_input,V_input);
	cube->build(0);

	//cout << "x: " << x << "y: " << y << "z: " << z << endl;
	physicsModel->addCube(cube);
	cout << "Cubes: " << cubes.size() << endl;
}

float distance(Vector3D *v1,Vector3D *v2){
	return sqrt((v1->x-v2->x)*(v1->x-v2->x)+(v1->y-v2->y)*(v1->y-v2->y)+(v1->z-v2->z)*(v1->z-v2->z));
}

void recolorObjects(){

	if(!spheres.empty()){
		for(int i=0;i<spheres.size();i++){
			if(default_HSV==true)spheres[i]->setHSVColor();
			else spheres[i]->setHSVColor(H_input,S_input,V_input);
		}
	}

	if(!cubes.empty()){
		for(int i=0;i<cubes.size();i++){
			if(default_HSV==true)cubes[i]->setHSVColor();
			else cubes[i]->setHSVColor(H_input,S_input,V_input);
		}
	}
}

void calculateFPS()
{
    //  Increase frame count
    frameCount++;
 
    //  Get the number of milliseconds since glutInit called
    //  (or first call to glutGet(GLUT ELAPSED TIME)).
    currentTime = glutGet(GLUT_ELAPSED_TIME);
 
    //  Calculate time passed
    int timeInterval = currentTime - previousTime;
 
    if(timeInterval > 1000)
    {
        //  calculate the number of frames per second
        fps = frameCount / (timeInterval / 1000.0f);
		
        //  Set time
        previousTime = currentTime;
 
        //  Reset frame count
        frameCount = 0;
    }
}

//------------------------------------------------------------------------------
//mouse functions

void mousePressed(int button,int state,int x,int y){
	
	if ( button == 3 ){
		printf("Wheel Up\n");
	}
	else if( button == 4 ){
		printf("Wheel Down\n");
	}

	pressedButton = button;
	mouse_state=state;
	prevX = x;
	prevY = y;

	int modifiers = glutGetModifiers();

	if(modifiers & GLUT_ACTIVE_CTRL)ctrl_pressed=true;

	if(state==1)ctrl_pressed=false;

}

void mouseDragged(int x,int y){
	
	

    
	if ((pressedButton==0 && ctrl_pressed==true) || pressedButton==2){
		//if(prevY-y<0)boxArea->applyXRot(boxArea->getRotX()+1);
		//else if(prevY-y>0)boxArea->applyXRot(boxArea->getRotX()-1);
		
		//if(prevX-x>0)boxArea->applyYRot(boxArea->getRotY()-1);
		//else if(prevX-x<0)boxArea->applyYRot(boxArea->getRotY()+1);
		boxArea->applyXRot(boxArea->getRotX()-(prevY-y));
		boxArea->applyYRot(boxArea->getRotY()-(prevX-x));

		prevX = x;
		prevY = y;
	}
    
	
}



//------------------------------------------------------------------------------

void keyboardDown(unsigned char key, int x, int y)
{
    
    switch(key)
    {

		case 'q':
			generateSphere();
			break;
		case 'e':
			generateCube();
			break;
		case 'r':
			refine();
			break;
        case 'w':
            visible_lines=!visible_lines;
			if(visible_lines==true){
				glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
			}
			else glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
            break;
        case 'A':
        case 'a':
            cout << "a pressed" << endl;
            break;
        case 'S':
        case 's':
            cout << "s pressed" << endl;
            break;
        case 'D':
        case 'd':
            cout << "d pressed" << endl;
            break;
        case KEY_ESC:
            exit(0);
            break;
		case '0':
			//default
			mode = 0;
			recolorObjects();
			break;
		case '1':
			//show hsv coloring
			mode = 1;
			sphere_cube_mode++;
			sphere_cube_mode%=2;
			break;
		case '2':
			//show distance based coloring
			mode = 2;
			break;
		case '3':
			//show impact based coloring
			mode = 3;
			break;
		case '4':
			mode = 4;
			spheres.clear();
			cubes.clear();
			physicsModel->clearAll();
			break;
		case '5':
			mode = 5;
			break;
		case '6':
			mode = 6;
        default:
            break;
    }
    glutPostRedisplay();
    //*/
}

//------------------------------------------------------------------------------

void keyboardUp(unsigned char key, int x, int y)
{
    //-------------Task1b---------------
    /*
    switch(key)
    {
        case 'W':
        case 'w':
            cout << "w relesed" << endl;
            break;
        case 'A':
        case 'a':
            cout << "a relesed" << endl;
            break;
        case 'S':
        case 's':
            cout << "s relesed" << endl;
            break;
        case 'D':
        case 'd':
            cout << "d relesed" << endl;
            break;
        case KEY_ESC:
            exit(0);
            break;
        default:
            break;
    }
    glutPostRedisplay();
    //*/
}

//------------------------------------------------------------------------------
//keyboard functions
void keyboardSpecialDown(int key, int x, int y)
{
    //-------------Task1c---------------
    /*
    switch(key)
    {
        case GLUT_KEY_UP:
            cout << "up key pressed" << endl;
            break;
        case GLUT_KEY_DOWN:
            cout << "down key pressed" << endl;
            break;
        case GLUT_KEY_RIGHT:
            cout << "right key pressed" << endl;
            break;
        case GLUT_KEY_LEFT:
            cout << "left key pressed" << endl;
            break;
        case GLUT_KEY_F1:
            cout << "F1 key pressed" << endl;
            break;
        case GLUT_KEY_PAGE_UP:
            cout << "page up key pressed" << endl;
            break;
        case GLUT_KEY_PAGE_DOWN:
            cout << "page down key pressed" << endl;
            break;
        case GLUT_KEY_HOME:
            cout << "home key pressed" << endl;
            break;
        case GLUT_KEY_END:
            cout << "end key pressed" << endl;
            break;
        case GLUT_KEY_INSERT:
            cout << "insert key pressed" << endl;
            break;
        default:
            break;
    }
    //*/ 

    //-------------Task1e---------------
    // Get all modifiers on an bitwise flag
    /*
    int modifier = glutGetModifiers();
    // Check modifier with each flag using bitwise AND
    if (modifier & GLUT_ACTIVE_SHIFT)
        cout << "shift key was pressed" << endl;
    if (modifier & GLUT_ACTIVE_CTRL)
        cout << "ctrl key was pressed" << endl;
    if (modifier & GLUT_ACTIVE_ALT)
        cout << "alt key was pressed" << endl;
    //*/
    glutPostRedisplay();
    //*/
}

//------------------------------------------------------------------------------

void keyboardSpecialUp(int key, int x, int y)
{
    //-------------Task1d---------------
    /*
    switch(key)
    {
        case GLUT_KEY_UP:
            cout << "up key released" << endl;
            break;
        case GLUT_KEY_DOWN:
            cout << "down key released" << endl;
            break;
        case GLUT_KEY_RIGHT:
            cout << "right key released" << endl;
            break;
        case GLUT_KEY_LEFT:
            cout << "left key released" << endl;
            break;
        case GLUT_KEY_F1:
            cout << "F1 key released" << endl;
            break;
        case GLUT_KEY_PAGE_UP:
            cout << "page up key released" << endl;
            break;
        case GLUT_KEY_PAGE_DOWN:
            cout << "page down key released" << endl;
            break;
        case GLUT_KEY_HOME:
            cout << "home key released" << endl;
            break;
        case GLUT_KEY_END:
            cout << "end key released" << endl;
            break;
        case GLUT_KEY_INSERT:
            cout << "insert key released" << endl;
            break;
        default:
            break;
    }
    glutPostRedisplay();
    //*/
}
