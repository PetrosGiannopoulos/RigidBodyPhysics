#ifndef BoxArea_H
#define BoxArea_H
#endif

#define PI 3.1415927

class BoxArea{

private: 
	double rotX,rotY,rotZ;
public: 
		
	BoxArea(){
		rotX = 0;rotY=0;rotZ=0;
	}

	void applyXRot(double rotX){
		this->rotX = rotX;
	}

	void applyYRot(double rotY){
		this->rotY = rotY;
	}

	void applyZRot(double rotZ){
		this->rotZ = rotZ;
	}

	double getRotX(){return rotX;}
	double getRotY(){return rotY;}
	double getRotZ(){return rotZ;}

	void transform(){
		glTranslatef(0,0,-40);

		//rotate x-axis
		glRotatef(rotX,1,0,0);
		//rotate y-axis
		glRotatef(rotY,0,1,0);
		//rotate z-axis
		glRotatef(rotZ,0,0,1);

		/*gluLookAt(-40*sin(rotY*(PI/180.))*cos(rotX*(PI/180.)),-40*sin(rotX*(PI/180.)),-40*cos(rotY*(PI/180.))*cos(rotX*(PI/180.)),
			0,0,0,
			0,1,0);*/

	}

	void render(){
		glColor4f(1,1,1,1);
		glutWireCube(20.0);
		
	}

	void drawWalls(){
		glColor4f(1,1,1,0.1);
		glBegin(GL_TRIANGLES);
		{
			
			//Up Wall 
			glVertex3f(-10,10,10);
			glVertex3f(10,10,10);
			glVertex3f(10,10,-10);

			glVertex3f(-10,10,10);	
			glVertex3f(-10,10,-10);
			glVertex3f(10,10,-10);

			//Down Wall 
			glVertex3f(-10,-10,10);	
			glVertex3f(10,-10,10);
			glVertex3f(10,-10,-10);

			glVertex3f(-10,-10,10);	
			glVertex3f(-10,-10,-10);
			glVertex3f(10,-10,-10);

			//Right Wall
			glVertex3f(10,-10,10);
			glVertex3f(10,10,10);
			glVertex3f(10,-10,-10);

			glVertex3f(10,-10,-10);	
			glVertex3f(10,10,-10);
			glVertex3f(10,10,10);
			
			//Left Wall
			glVertex3f(-10,-10,10);
			glVertex3f(-10,10,10);
			glVertex3f(-10,-10,-10);

			glVertex3f(-10,-10,-10);	
			glVertex3f(-10,10,-10);
			glVertex3f(-10,10,10);

			//Front Wall
			glVertex3f(-10,-10,10);
			glVertex3f(-10,10,10);
			glVertex3f(10,-10,10);

			glVertex3f(10,-10,10);	
			glVertex3f(10,10,10);
			glVertex3f(-10,10,10);

			//Back Wall
			glVertex3f(-10,-10,-10);
			glVertex3f(-10,10,-10);
			glVertex3f(10,-10,-10);

			glVertex3f(10,-10,-10);	
			glVertex3f(10,10,-10);
			glVertex3f(-10,10,-10);

		}
		glEnd();
	}

};

