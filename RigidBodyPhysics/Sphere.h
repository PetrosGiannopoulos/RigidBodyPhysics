#ifndef SPHERE_H
#define SPHERE_H
#endif

using namespace std;

class Sphere: public RigidBody{


public:
	/*float r;

	Vector3 vel;
	Vector3 nvel;
	Vector3 pos;
	Vector3 minDistPoint;
	Vector3 impactPoint;

	float dt;
	float density;
	bool *show_mesh;

	float mass;

	float max_vel;
	float max_dist;
	float min_dist;

	float impactTimeCounter;
	float impactTimeLength;
	float impactSatValue;
	
	float impactDist1;
	float impactDist2;
	float impactDist3;

	bool impactHappen;
	bool impactFirstTime;

	Vector3 rot;
	Vector3 P;
	Vector3 L;
	Vector3 f;
	Vector3 tau;
	Quaternion q;

	Matrix3 I_body;
	Matrix3 I_inv_body;

	Matrix3 I;
	Matrix3 I_inv;
	State state;

	int *mode;*/

public:

	vector<Vector3D*> vertices;
	/*vector<Triangle3D> triangles;
	vector<Triangle3D> offset;
	ColorModel *colorModel;*/

	vector<float> impactDist1_;
	vector<float> impactDist2_;
	vector<float> impactDist3_;

	
	Sphere(Vector3 r,Vector3 vel,Vector3 pos,bool *show_mesh,int *mode,float *fps):RigidBody(r,vel,pos,show_mesh,mode,0){
		this->r = r;
		
		this->pos = Vector3(pos.x-10,pos.y-10,pos.z-10);
		this->vel = vel;//(vel)-(pos);
		this->dt = 0.002;
		//this->dt = 0.01;
		this->density = 8.15;
		this->mass = density*getVolume();
		this->show_mesh=show_mesh;
		this->max_vel=100;
		this->max_dist=34.641;
		this->minDistPoint = Vector3(0,0,0);
		this->impactTimeCounter=0;
		this->impactTimeLength = 3000; //ms
		this->impactSatValue=0.5;
		this->impactHappen=false;
		this->impactDist1=0.5;
		this->impactDist2=0.5;
		this->impactDist3=0.5;
		this->impactFirstTime=true;
		this->mode=mode;
		this->fps=fps;

		this->rot = this->tau = this->f = this->L = Vector3(0,0,0);
		this->P = this->mass*this->vel;
		this->q = Quaternion(rot,1);

		this->I_body = this->I = Matrix3(
			this->mass*(this->r.y*this->r.y+this->r.z*this->r.z), 0, 0,
			0, this->mass*(this->r.x*this->r.x+this->r.z*this->r.z), 0,
			0, 0, this->mass*(this->r.x*this->r.x+this->r.y*this->r.y));

		this->L = this->I_body * this->rot;

		this->I_inv_body = this->I_inv =  this->I_body.invert();

		this->state=State(this->pos,this->q,this->P,this->L);
	}

	void move(){
		this->pos = this->pos+this->vel*this->dt;
	}

	void transform(){

		glPushMatrix();
		/*glTranslatef(this->pos.x-10,this->pos.y-10,10-this->pos.z);

		const float* mm = this->q.rotationMatrix().toGLMatrix4();
		glMultMatrixf(mm);
		delete [] mm;

		glScalef(this->r.x,this->r.y,this->r.z);*/

	}

	void render(){
		//glColor3f(this->red,this->green,this->blue);
		//glutSolidSphere(r,50,50);
		if(*this->mode==2)setDistanceHSVColor();

		if(*this->mode==3)setImpactHSVColor();
		
		for(int i=0;i<this->offset.size();i++){
			
			Triangle3D tri = this->offset[i];
			Vector3D v1 = tri.getV1();
			Vector3D v2 = tri.getV2();
			Vector3D v3 = tri.getV3();

			glBegin(GL_TRIANGLES);
			{

				glColor3f(v1.r,v1.g,v1.b);
				glVertex3f(v1.x,v1.y,-v1.z);
			
				glColor3f(v2.r,v2.g,v2.b);
				glVertex3f(v2.x,v2.y,-v2.z);
			
				glColor3f(v3.r,v3.g,v3.b);
				glVertex3f(v3.x,v3.y,-v3.z);
			}
			glEnd();


		}

		glPopMatrix();
	}


	void setVelocity(Vector3 vel){
		this->vel = vel;
	}

	void setVertices(vector<Vector3D*> vertices){
		this->vertices=vertices;
	}

	void setTriangles(vector<Triangle3D> triangles){
		this->triangles=triangles;
	}

	void refine(int times){
	
		int N;
		for(int k=0;k<times;k++){
			N = this->triangles.size();
			for(int i=0;i<N;i++){
				//if(showTriangles[i]==false)continue;
				Triangle3D tri = this->triangles[i];

				Vector3D n1 = tri.linear1(0.5);
				Vector3D n2 = tri.linear2(0.5);
				Vector3D n3 = tri.linear3(0.5);

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

				
				//this->triangles.push_back(t1);
				this->triangles[i]=t1;
				this->triangles.push_back(t2);this->triangles.push_back(t3);this->triangles.push_back(t4);
				//showTriangles.push_back(true);showTriangles.push_back(true);showTriangles.push_back(true);showTriangles.push_back(true);
				//showTriangles[i]=false;

			}
		}
		cout << "Triangles: " << this->triangles.size() << endl;

	}

	void setColorModel(ColorModel *colorModel){
		this->colorModel=colorModel;
	}

	void setDistanceHSVColor(){
		float n_vel = this->vel.length()/this->max_vel;
		//float n_dist = this->min_dist/this->max_dist;
		float invDist = 1./this->max_dist;
		float dist1,dist2,dist3;
		
		for(int i=0;i<this->triangles.size();i++){
			//if(showTriangles[i]==false) continue;
			Triangle3D tri = this->triangles[i];
			Vector3D fv1 = tri.getV1();
			Vector3D fv2 = tri.getV2();
			Vector3D fv3 = tri.getV3();


			Triangle3D ofTri = this->offset[i];
			Vector3D of1 = ofTri.getV1();
			Vector3D of2 = ofTri.getV2();
			Vector3D of3 = ofTri.getV3();

			Vector3 mv1 = of1.toVec3();
			Vector3 mv2 = of2.toVec3();
			Vector3 mv3 = of3.toVec3();

			Vector3D v1 = Vector3D(mv1.x,mv1.y,mv1.z);
			Vector3D v2 = Vector3D(mv2.x,mv2.y,mv2.z);
			Vector3D v3 = Vector3D(mv3.x,mv3.y,mv3.z);

			of1.transferColor(v1);
			of2.transferColor(v2);
			of3.transferColor(v3);

			dist1 = (v1.distance(this->minDistPoint))*invDist*v1.slength(pos);
			
			
			this->colorModel->displayHSV(0.8-n_vel,dist1,1.0);
			v1.r=this->colorModel->getRed();v1.g=this->colorModel->getGreen();v1.b=this->colorModel->getBlue();
			
			dist2 = (v2.distance(this->minDistPoint))*invDist*v2.slength(pos);
			
			
			this->colorModel->displayHSV(0.8-n_vel,dist2,1.0);
			v2.r=this->colorModel->getRed();v2.g=this->colorModel->getGreen();v2.b=this->colorModel->getBlue();
			
			dist3 = (v3.distance(this->minDistPoint))*invDist*v3.slength(pos);
			
			this->colorModel->displayHSV(0.8-n_vel,dist3,1.0);
			v3.r=this->colorModel->getRed();v3.g=this->colorModel->getGreen();v3.b=this->colorModel->getBlue();

			v1.transferColor(fv1);
			v2.transferColor(fv2);
			v3.transferColor(fv3);

			//this->triangles[i]->setV1(v1);this->triangles[i]->setV2(v2);this->triangles[i]->setV3(v3);
			this->triangles[i].setV1(fv1);this->triangles[i].setV2(fv2);this->triangles[i].setV3(fv3);
						
			//cout << "dist1: " << dist1 << ",dist2: " << dist2 << ",dist3: " << dist3 << endl;
			//cout << "slength1: " << v1.slength() << ",slength2: " << v2.slength() << ",slength3: " << v3.slength() << endl;
		}
	}

	void setImpactHSVColor(){

		float n_vel = this->vel.length()/this->max_vel;

		if(this->impactTimeCounter>0){
			this->impactTimeCounter -= (1./this->impactTimeLength)*dt*1000*(*this->fps/10.);
			this->impactSatValue = this->impactTimeCounter;
		}
		else {
			this->impactSatValue=0.5;
			//impactDist1 = 0.5;
			//impactDist2 = 0.5;
			//impactDist3 = 0.5;
			//impactHappen=false;
		}

		if(this->impactSatValue<0)this->impactSatValue=0.5;

		float invR=(1.)/(2*this->r.length()/2);
		float dist1,dist2,dist3;
		float h1,h2,h3;
		for(int i=0;i<this->triangles.size();i++){
			//if(showTriangles[i]==false) continue;
			Triangle3D tri = this->triangles[i];
			Vector3D fv1 = tri.getV1();
			Vector3D fv2 = tri.getV2();
			Vector3D fv3 = tri.getV3();

			Triangle3D ofTri = this->offset[i];
			Vector3D of1 = ofTri.getV1();
			Vector3D of2 = ofTri.getV2();
			Vector3D of3 = ofTri.getV3();

			Vector3 mv1 = of1.toVec3();
			Vector3 mv2 = of2.toVec3();
			Vector3 mv3 = of3.toVec3();

			Vector3D v1 = Vector3D(mv1.x,mv1.y,mv1.z);
			Vector3D v2 = Vector3D(mv2.x,mv2.y,mv2.z);
			Vector3D v3 = Vector3D(mv3.x,mv3.y,mv3.z);

			of1.transferColor(v1);
			of2.transferColor(v2);
			of3.transferColor(v3);
		
			
			if(this->impactHappen==true)this->impactDist1_.push_back(v1.distance(this->impactPoint)*invR);
			
			if(this->impactFirstTime==false)dist1 = this->impactDist1_[i];
			else dist1 = 0.5;
			if(this->impactSatValue==0.5)dist1=0.5;

			this->colorModel->displayHSV(0.8-n_vel,this->impactSatValue*v1.slength(pos),dist1);

			v1.r=this->colorModel->getRed();v1.g=this->colorModel->getGreen();v1.b=this->colorModel->getBlue();

			if(this->impactHappen==true)this->impactDist2_.push_back(v2.distance(this->impactPoint)*invR);

			if(this->impactFirstTime==false)dist2 = this->impactDist2_[i];
			else dist2 = 0.5;
			if(this->impactSatValue==0.5)dist2=0.5;

			this->colorModel->displayHSV(0.8-n_vel,this->impactSatValue*v2.slength(pos),dist2);

			v2.r=this->colorModel->getRed();v2.g=this->colorModel->getGreen();v2.b=this->colorModel->getBlue();

			
			if(this->impactHappen==true)this->impactDist3_.push_back(v3.distance(this->impactPoint)*invR);

			if(this->impactFirstTime==false)dist3 = this->impactDist3_[i];
			else dist3 = 0.5;
			if(this->impactSatValue==0.5)dist3=0.5;

			this->colorModel->displayHSV(0.8-n_vel,this->impactSatValue*v3.slength(pos),dist3);

			v3.r=this->colorModel->getRed();v3.g=this->colorModel->getGreen();v3.b=this->colorModel->getBlue();


			v1.transferColor(fv1);
			v2.transferColor(fv2);
			v3.transferColor(fv3);

			this->triangles[i].setV1(fv1);this->triangles[i].setV2(fv2);this->triangles[i].setV3(fv3);
		}

		if(this->impactHappen==true){
			this->impactHappen=false;
		}

	}

	void setHSVColor(float h,float s,float v){
		for(int i=0;i<this->triangles.size();i++){
			Triangle3D tri = this->triangles[i];
			Vector3D v1 = tri.getV1();
			Vector3D v2 = tri.getV2();
			Vector3D v3 = tri.getV3();
		

			this->colorModel->displayHSV(h,s,v);
			v1.r=this->colorModel->getRed();v1.g=this->colorModel->getGreen();v1.b=this->colorModel->getBlue();
			
			this->colorModel->displayHSV(h,s,v);
			v2.r=this->colorModel->getRed();v2.g=this->colorModel->getGreen();v2.b=this->colorModel->getBlue();
				
			this->colorModel->displayHSV(h,s,v);
			v3.r=this->colorModel->getRed();v3.g=this->colorModel->getGreen();v3.b=this->colorModel->getBlue();
			
			tri.v1.r=v1.r;tri.v1.g=v1.g;tri.v1.b=v1.b;
			tri.v2.r=v2.r;tri.v2.g=v2.g;tri.v2.b=v2.b;
			tri.v3.r=v3.r;tri.v3.g=v3.g;tri.v3.b=v3.b;
			this->triangles[i].setV1(tri.v1);this->triangles[i].setV2(tri.v2);this->triangles[i].setV3(tri.v3);
		}
	}

	void setHSVColor(){
		/*this->h=h;
		this->s=s;
		this->v=v;*/
		for(int i=0;i<this->triangles.size();i++){
			//if(showTriangles[i]==false) continue;
			Triangle3D tri = this->triangles[i];
			Vector3D v1 = tri.getV1();
			Vector3D v2 = tri.getV2();
			Vector3D v3 = tri.getV3();
		

			this->colorModel->displayHSV(v1.phi(),v1.slength()*0.7,0.5*(1+v1.y));
			//this->colorModel->displayHSV(v1->phi(),v1->slength(),v1->theta()*0.5*(1+v1->y));
			v1.r=this->colorModel->getRed();v1.g=this->colorModel->getGreen();v1.b=this->colorModel->getBlue();
			
			this->colorModel->displayHSV(v2.phi(),v2.slength()*0.7,0.5*(1+v2.y));
			//this->colorModel->displayHSV(v2->phi(),v1->slength(),v2->theta()*0.5*(1+v2->y));
			v2.r=this->colorModel->getRed();v2.g=this->colorModel->getGreen();v2.b=this->colorModel->getBlue();
				
			this->colorModel->displayHSV(v3.phi(),v3.slength()*0.7,0.5*(1+v3.y));
			//this->colorModel->displayHSV(v3->phi(),v3->slength(),v3->theta()*0.5*(1+v3->y));
			v3.r=this->colorModel->getRed();v3.g=this->colorModel->getGreen();v3.b=this->colorModel->getBlue();
			
			tri.v1.r=v1.r;tri.v1.g=v1.g;tri.v1.b=v1.b;
			tri.v2.r=v2.r;tri.v2.g=v2.g;tri.v2.b=v2.b;
			tri.v3.r=v3.r;tri.v3.g=v3.g;tri.v3.b=v3.b;
			this->triangles[i].setV1(tri.v1);this->triangles[i].setV2(tri.v2);this->triangles[i].setV3(tri.v3);
		}

	}

	void build(int value=1){
		for(int i=0;i<this->triangles.size();i++){
			Triangle3D tri = this->triangles[i];

			Matrix3 R = this->q.rotationMatrix();
			Vector3 v1 = this->pos+R*tri.getV1().toVec3()*this->r;
			Vector3 v2 = this->pos+R*tri.getV2().toVec3()*this->r;
			Vector3 v3 = this->pos+R*tri.getV3().toVec3()*this->r;

			Vector3D fv1 = Vector3D(v1.x,v1.y,v1.z,tri.getV1().r,tri.getV1().g,tri.getV1().b);
			Vector3D fv2 = Vector3D(v2.x,v2.y,v2.z,tri.getV2().r,tri.getV2().g,tri.getV2().b);
			Vector3D fv3 = Vector3D(v3.x,v3.y,v3.z,tri.getV3().r,tri.getV3().g,tri.getV3().b);

			Triangle3D offTri = Triangle3D(fv1,fv2,fv3);
			if(value==0)this->offset.push_back(offTri);
			else this->offset[i]=offTri;
		}
	

	}

	void update(float t){
		f = Vector3(0,0,0);
		
		//collision
		handleWallCollision(dt);
		//forces
		
		Vector3 gravity = this->mass*9.81*Vector3(0,-1,0)*10;
		if(*this->mode==5)f = gravity;
		if(*this->mode==6)this->vel = Vector3(0,0,0);

		//integration
		//float* new_state = integrate(t, STATES, getState(), dt);
		//setState(new_state);
		State new_s = integrate(t,getState(),dt);
		setState(new_s);

		build();

	}

	void handleWallCollision(float t){
		Vector3 prevel;
		//Up Wall Collision
		if((this->pos.y+this->vel.y*dt)>=(20-this->getRadius())){
			Vector3 n = Vector3(0,1,0);

			this->pos.y = 20-this->getRadius();

			prevel = this->vel;
			this->vel = this->vel-2*this->vel.dot(n)*n;

			Vector3 minPoint = Vector3();
			for(int i=0;i<this->triangles.size();i++){
				Triangle3D tri = this->offset[i];
				Vector3 v1 = tri.getV1().toVec3();
				Vector3 v2 = tri.getV2().toVec3();
				Vector3 v3 = tri.getV3().toVec3();

				Vector3 minTemp = v1;

				if(v1.y>v2.y && v1.y>v3.y)minTemp=v1;
				if(v2.y>v1.y && v2.y>v3.y)minTemp=v2;
				if(v3.y>v1.y && v3.y>v2.y)minTemp=v3;

				if(minTemp.y>minPoint.y)minPoint=minTemp;
			}
			Vector3 rP = minPoint-pos;

			//tau = rot/dt;
			//a = vel/dt;
			//f = m*a
			//tau = fxr
			Vector3 force = -2*this->mass*(-prevel);
			Vector3 taf = rP.cross(force);
			this->rot = this->rot-this->I_inv*taf*t;
			//this->f = force;
			//this->tau = taf;

			//this->q = Quaternion(this->rot,1);
			this->L = this->I*this->rot;
			this->P = this->mass*this->vel;
		}

		//Down Wall Collision
		if((this->pos.y+this->vel.y*this->dt)<=(0+this->getRadius())){
			Vector3 n = Vector3(0,-1,0);	
				
			this->pos.y = this->getRadius();

			prevel = this->vel;
			this->vel = this->vel-2*this->vel.dot(n)*n;	

			Vector3 minPoint = Vector3();
			for(int i=0;i<this->triangles.size();i++){
				Triangle3D tri = this->offset[i];
				Vector3 v1 = tri.getV1().toVec3();
				Vector3 v2 = tri.getV2().toVec3();
				Vector3 v3 = tri.getV3().toVec3();

				Vector3 minTemp = v1;

				if(v1.y<v2.y && v1.y<v3.y)minTemp=v1;
				if(v2.y<v1.y && v2.y<v3.y)minTemp=v2;
				if(v3.y<v1.y && v3.y<v2.y)minTemp=v3;

				if(minTemp.y<minPoint.y)minPoint=minTemp;
			}
			Vector3 rP = minPoint-pos;
			
			Vector3 force = -2*this->mass*(-prevel);
			Vector3 taf = rP.cross(force);
			this->rot = this->rot-this->I_inv*taf*t;
			//this->f = force;
			//this->tau = taf;
			//this->q = Quaternion(this->rot,1);
			this->L = this->I*this->rot;
			this->P = this->mass*this->vel;
		}

		//Right Wall Collision
		if((this->pos.x+this->vel.x*this->dt)>=(20-this->getRadius())){
			Vector3 n = Vector3(1,0,0);	
				
			this->pos.x = 20-this->getRadius();

			prevel = this->vel;
			this->vel = this->vel-2*this->vel.dot(n)*n;

			Vector3 minPoint = Vector3();
			for(int i=0;i<this->triangles.size();i++){
				Triangle3D tri = this->offset[i];
				Vector3 v1 = tri.getV1().toVec3();
				Vector3 v2 = tri.getV2().toVec3();
				Vector3 v3 = tri.getV3().toVec3();

				Vector3 minTemp = v1;

				if(v1.x>v2.x && v1.x>v3.x)minTemp=v1;
				if(v2.x>v1.x && v2.x>v3.x)minTemp=v2;
				if(v3.x>v1.x && v3.x>v2.x)minTemp=v3;

				if(minTemp.x>minPoint.x)minPoint=minTemp;
			}
			Vector3 rP = minPoint-pos;
			
			Vector3 force = -2*this->mass*(-prevel);
			Vector3 taf = rP.cross(force);
			this->rot = this->rot-this->I_inv*taf*t;
			//this->f = force;
			//this->tau = taf;
			//this->q = Quaternion(this->rot,1);
			this->L = this->I*this->rot;
			this->P = this->mass*this->vel;
		}

		//Left Wall Collision
		if((this->pos.x+this->vel.x*this->dt)<=(0+this->getRadius())){
			Vector3 n = Vector3(-1,0,0);	
				
			this->pos.x = this->getRadius();

			prevel = this->vel;
			this->vel = this->vel-2*this->vel.dot(n)*n;	

			Vector3 minPoint = Vector3();
			for(int i=0;i<this->triangles.size();i++){
				Triangle3D tri = this->offset[i];
				Vector3 v1 = tri.getV1().toVec3();
				Vector3 v2 = tri.getV2().toVec3();
				Vector3 v3 = tri.getV3().toVec3();

				Vector3 minTemp = v1;

				if(v1.x<v2.x && v1.x<v3.x)minTemp=v1;
				if(v2.x<v1.x && v2.x<v3.x)minTemp=v2;
				if(v3.x<v1.x && v3.x<v2.x)minTemp=v3;

				if(minTemp.x<minPoint.x)minPoint=minTemp;
			}
			Vector3 rP = minPoint-pos;
			
			Vector3 force = -2*this->mass*(-prevel);
			Vector3 taf = rP.cross(force);
			this->rot = this->rot-this->I_inv*taf*t;
			//this->f = force;
			//this->tau = taf;
			//this->q = Quaternion(this->rot,1);
			this->L = this->I*this->rot;
			this->P = this->mass*this->vel;
		}

		//Front Wall Collision
		if((this->pos.z+this->vel.z*this->dt)<=(this->getRadius())){
			Vector3 n = Vector3(0,0,-1);	
				
			this->pos.z = this->getRadius();

			prevel = this->vel;
			this->vel = this->vel-2*this->vel.dot(n)*n;

			Vector3 minPoint = Vector3();
			for(int i=0;i<this->triangles.size();i++){
				Triangle3D tri = this->offset[i];
				Vector3 v1 = tri.getV1().toVec3();
				Vector3 v2 = tri.getV2().toVec3();
				Vector3 v3 = tri.getV3().toVec3();

				Vector3 minTemp = v1;

				if(v1.z<v2.z && v1.z<v3.z)minTemp=v1;
				if(v2.z<v1.z && v2.z<v3.z)minTemp=v2;
				if(v3.z<v1.z && v3.z<v2.z)minTemp=v3;

				if(minTemp.z<minPoint.z)minPoint=minTemp;
			}
			Vector3 rP = minPoint-pos;
			
			Vector3 force = -2*this->mass*(-prevel);
			Vector3 taf = rP.cross(force);
			this->rot = this->rot-this->I_inv*taf*t;
			//this->f = force;
			//this->tau = taf;
			//this->q = Quaternion(this->rot,1);
			this->L = this->I*this->rot;
			this->P = this->mass*this->vel;
		}

		//Back Wall Collision
		if((this->pos.z+this->vel.z*dt)>=(20-this->getRadius())){
			Vector3 n = Vector3(0,0,1);	
				
			this->pos.z = 20-this->getRadius();

			prevel = this->vel;
			this->vel = this->vel-2*this->vel.dot(n)*n;

			Vector3 minPoint = Vector3();
			for(int i=0;i<this->triangles.size();i++){
				Triangle3D tri = this->offset[i];
				Vector3 v1 = tri.getV1().toVec3();
				Vector3 v2 = tri.getV2().toVec3();
				Vector3 v3 = tri.getV3().toVec3();

				Vector3 minTemp = v1;

				if(v1.z>v2.z && v1.z>v3.z)minTemp=v1;
				if(v2.z>v1.z && v2.z>v3.z)minTemp=v2;
				if(v3.z>v1.z && v3.z>v2.z)minTemp=v3;

				if(minTemp.z>minPoint.z)minPoint=minTemp;
			}
			Vector3 rP = minPoint-pos;

			Vector3 force = -2*this->mass*(-prevel);
			Vector3 taf = rP.cross(force);
			this->rot = this->rot-this->I_inv*taf*t;
			//this->f = force;
			//this->tau = taf;
			//this->q = Quaternion(this->rot,1);
			this->L = this->I*this->rot;
			this->P = this->mass*this->vel;
		}
	}

	void update(){
		
		for(int i=0;i<triangles.size();i++){
			Triangle3D tri = triangles[i];

			Vector3 v1 = tri.getV1().toVec3();
			Vector3 v2 = tri.getV2().toVec3();
			Vector3 v3 = tri.getV3().toVec3();

			v1 = v1+this->vel*this->dt;v1.z -= 2*this->vel.z*this->dt;
			v2 = v2+this->vel*this->dt;v2.z -= 2*this->vel.z*this->dt;
			v3 = v3+this->vel*this->dt;v3.z -= 2*this->vel.z*this->dt;
			

			tri.setV1(Vector3D(v1.x,v1.y,v1.z));
			tri.setV2(Vector3D(v2.x,v2.y,v2.z));
			tri.setV3(Vector3D(v3.x,v3.y,v3.z));

		}
	}

	void update(Vector3D nvel){
		for(int i=0;i<triangles.size();i++){
			Triangle3D tri = triangles[i];

			Vector3D v1 = tri.getV1();
			Vector3D v2 = tri.getV2();
			Vector3D v3 = tri.getV3();

			v1 = v1+nvel*this->dt;v1.z -= 2*nvel.z*this->dt;
			v2 = v2+nvel*this->dt;v2.z -= 2*nvel.z*this->dt;
			v3 = v3+nvel*this->dt;v3.z -= 2*nvel.z*this->dt;
			

			tri.setV1(v1);
			tri.setV2(v2);
			tri.setV3(v3);

		}
	}

	void setMinDist(float min_dist){
		this->min_dist=min_dist;
	}

	void setMinDistPoint(Vector3 minDistPoint){
		this->minDistPoint = minDistPoint;
	}

	void setImpactPoint(Vector3 impactPoint){
		this->impactPoint = impactPoint;
		this->impactHappen=true;
		this->impactDist1_.clear();
		this->impactDist2_.clear();
		this->impactDist3_.clear();
		this->impactFirstTime=false;
	}

	void setImpactTimeCounter(float impactTimeCounter){
		this->impactTimeCounter=impactTimeCounter;
	}

	void setPos(Vector3 pos){
		this->pos = pos;
		build();
	}

	float getRadius(){
		return this->r.length()/sqrt(3.);
	}

	Vector3 getVelocity(){
		return this->vel;
	}

	float getX(){
		return this->pos.x;
	}

	float getY(){
		return this->pos.y;
	}

	float getZ(){
		return this->pos.z;
	}

	Vector3 getPos(){
		return this->pos;
	}

	float getDT(){
		return this->dt;
	}

	float getVolume(){
		return ((4/3.)*PI*this->r.x*this->r.y*this->r.z);
	}

	float getMass(){
		//return density*getVolume();
		return this->mass;
	}

	State getState(){
		State s = State();
		s.pos = this->pos;
		s.q = this->q;
		s.P = this->P;
		s.L = this->L;

		return s;
	}

	void setState(State s){
		this->pos = s.pos;
		this->q = s.q;
		this->P = s.P;
		this->L = s.L;

		//momentum
		this->vel = this->P / this->mass;

		//update inertia matrix
		this->q.normalize();
		Matrix3 R = this->q.rotationMatrix();
		this->I_inv = R * this->I_inv_body * R.transpose();
		this->I = R* this->I_body * R.transpose();

		//angular momentum
		this->rot = this->I_inv * this->L;
	}

	State dxdt(float t,State ns){
		//State ns = State();

		//x_dot = u
		//ns.vel=this->vel;
		ns.pos=this->vel;

		//q_dot = 1 / 2 * w * q;
		Quaternion w_hat = Quaternion(this->rot, 1);
		Quaternion q_dot = (w_hat * this->q) / 2;

		ns.q = q_dot;

		//P_dot = f
		//ns.f = this->f;
		ns.P = this->f;

		//L_dot = tau
		//ns.tau = this->tau;
		ns.L = this->tau;

		return ns;
	}

	State integrate(float t0,State s,float step){
		int i;
		float t1;
		float t2;
		float t3;

		State f0 = dxdt(t0,s);
		t1 = t0 + step*0.5;
		State u1 = s;
		u1.raise(f0,step*0.5);
		State f1 = dxdt(t1,u1);
		t2 = t0 + step*0.5;
		State u2 = s;
		u2.raise(f1,step*0.5);
		State f2 = dxdt(t2,u2);
		t3 = t0 + step;
		State u3 = s;
		u3.raise(f2,step);
		State f3 = dxdt(t3,u3);


		State u = s;

		u.combine(f0,f1,f2,f3,step/6.0f);

		return u;

	}
};