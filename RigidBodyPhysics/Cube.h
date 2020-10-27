#ifndef CUBE_H
#define CUBE_H
#endif

using namespace std;

class Cube: public RigidBody{
	
public:

	/*vector<Triangle3D> triangles;
	ColorModel *colorModel;

	vector<Triangle3D> offset;

	Vector3 pos;
	Vector3 vel;
	
	Vector3 rot;
	Vector3 P;
	Vector3 L;
	Vector3 f;
	Vector3 tau;

	float mass;
	float dt;
	float density;
	float size;
	float SR3;

	State state;

	Matrix3 I_body;
	Matrix3 I_inv_body;

	Matrix3 I;
	Matrix3 I_inv;
	Quaternion q;

	Vector3 minDistPoint;
	float max_vel;
	float max_dist;
	float min_dist;

	int *mode;
	bool *show_mesh;*/

	Cube(Vector3 size,Vector3 vel,Vector3 pos,bool *show_mesh,int *mode,float *fps):RigidBody(size,vel,pos,show_mesh,mode,1){
		this->size=size;
		this->vel=vel;
		//this->vel=vel-pos;
		//this->vel = Vector3(0,0,0);
		this->pos=Vector3(pos.x-10,pos.y-10,pos.z-10);
		SR3 = sqrt(3.);
		this->r = Vector3(getRadius(),getRadius(),getRadius());
		this->density = 8.15;
		this->dt = 0.002;
		this->rot = Vector3(0,0,0);
		this->mass = density*getVolume();
		this->max_vel=100;
		this->max_dist=34.641;
		this->minDistPoint = Vector3(0,0,0);
		this->mode=mode;
		this->fps=fps;
		this->show_mesh=show_mesh;

		this->rot = this->tau = this->f = this->L = Vector3(0,0,0);
		this->P = this->mass*this->vel;
		this->q = Quaternion(rot,1);

		this->I_body = this->I = Matrix3(
			1./6*this->mass*(this->size.y*this->size.y+this->size.z*this->size.z), 0, 0,
			0, 1./6*this->mass*(this->size.x*this->size.x+this->size.z*this->size.z), 0,
			0, 0, 1./6*this->mass*(this->size.x*this->size.x+this->size.y*this->size.y));

		L = I_body * rot;

		I_inv_body = I_inv =  I_body.invert();

		this->state=State(pos,q,P,L);
	}


	void move(){
		//pos = pos+vel*dt;
		
		for(int i=0;i<triangles.size();i++){
			Triangle3D tri = offset[i];
			Vector3 v1 = pos+tri.getV1().toVec3();
			v1.x -= 10;v1.y -= 10;v1.z = 10-v1.z+2*tri.getV1().toVec3().z;

			Vector3 v2 = pos+tri.getV2().toVec3();
			v2.x -= 10;v2.y -= 10;v2.z = 10-v2.z+2*tri.getV2().toVec3().z;

			Vector3 v3 = pos+tri.getV3().toVec3();
			v3.x -= 10;v3.y -= 10;v3.z = 10-v3.z+2*tri.getV3().toVec3().z;

			glColor4f(1,1,1,0.3);
			glBegin(GL_TRIANGLES);
			{
				glVertex3f(v1.x,v1.y,v1.z);
				glVertex3f(v2.x,v2.y,v2.z);
				glVertex3f(v3.x,v3.y,v3.z);
			}
			glEnd();

			Vector3 n = tri.normal().toVec3();
			Vector3 f3 = v3+10*n;

			glColor3f(1,0,0);
			glBegin(GL_LINES);
			{
				glVertex3f(v3.x,v3.y,v3.z);
				glVertex3f(f3.x,f3.y,f3.z);
			}
			glEnd();
		}
		
		


	}

	void transform(){
		glPushMatrix();
		//glTranslatef(pos.x-10,pos.y-10,10-pos.z);

		/*const float* mm = q.rotationMatrix().toGLMatrix4();
		glMultMatrixf(mm);
		delete [] mm;*/
		//rotate x-axis
		/*glRotatef(rot.x,1,0,0);
		//rotate y-axis
		glRotatef(rot.y,0,1,0);
		//rotate z-axis
		glRotatef(rot.z,0,0,1);*/
		
		//glScalef(size.x,size.y,size.z);
		
		
	}

	void render(){

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

		for(int i=0;i<offset_v.size();i++){
			Vector3D v = offset_v[i];

			glColor3f(0,0,1);
			glBegin(GL_POINTS);
			{
				glVertex3f(v.x,v.y,-v.z);
			}
			glEnd();
		}

		if(*show_mesh==true){
			glColor4f(1,1,1,0.1);
			glPushMatrix();
			glTranslatef(pos.x,pos.y,-pos.z);
			glutSolidSphere(getRadius(),20,20);
			glPopMatrix();
		}

		glPopMatrix();
	}

	void build(int value=1){
		for(int i=0;i<triangles.size();i++){
			Triangle3D tri = triangles[i];
			
			Matrix3 R = q.rotationMatrix();
			Vector3 v1 = pos+R*tri.getV1().toVec3()*this->size;
			Vector3 v2 = pos+R*tri.getV2().toVec3()*this->size;
			Vector3 v3 = pos+R*tri.getV3().toVec3()*this->size;

			Vector3D fv1 = Vector3D(v1.x,v1.y,v1.z,tri.getV1().r,tri.getV1().g,tri.getV1().b);
			Vector3D fv2 = Vector3D(v2.x,v2.y,v2.z,tri.getV2().r,tri.getV2().g,tri.getV2().b);
			Vector3D fv3 = Vector3D(v3.x,v3.y,v3.z,tri.getV3().r,tri.getV3().g,tri.getV3().b);

			Triangle3D offTri = Triangle3D(fv1,fv2,fv3);
			if(value==0)offset.push_back(offTri);
			else offset[i]=offTri;
		}

		for(int i=0;i<vertices.size();i++){
			Vector3D v = *vertices[i];
			Matrix3 R = q.rotationMatrix();
			Vector3 v_ = pos+R*v.toVec3()*this->size;
			Vector3D offV = Vector3D(v_);
			if(value==0)offset_v.push_back(offV);
			else offset_v[i] = offV;
		}
	}

	void update(float t){
		f = Vector3(0,0,0);
		//tau = Vector3(0,0,0);

		//collision
		handleWallCollisionS(dt);
		//forces
		Vector3 gravity = this->mass*9.81*Vector3(0,-1,0);
		if(*this->mode==5)f = gravity;
		if(*this->mode==6)this->vel = Vector3(0,0,0);
		
		//integration
		//float* new_state = integrate(t, STATES, getState(), dt);
		//setState(new_state);
		State new_s = integrate(t,getState(),dt);
		setState(new_s);

		build();

	}

	void reset(){
		this->tau = this->f = this->L = Vector3(0,0,0);
		
	}

	void applyImpulse(Vector3 n,Vector3 rP){

		Vector3 prevel = this->vel;
		this->vel = this->vel-(1+eps)*this->vel.dot(n)*n;

		this->rot = vel.cross(rP)/(rP.length()*rP.length());
			
		this->L = this->I*this->rot;
		this->P = this->mass*this->vel;

	}

	void handleWallCollision(float t){
		Vector3 minX = Vector3( 40,0,0),minY = Vector3(0, 40,0),minZ = Vector3(0,0, 40);
		Vector3 maxX = Vector3(-40,0,0),maxY = Vector3(0,-40,0),maxZ = Vector3(0,0,-40);
		build();
		for(int i=0;i<offset_v.size();i++){
			Vector3 v = offset_v[i].toVec3();

			if(v.x<minX.x)minX = v;
			if(v.x>maxX.x)maxX = v;
			if(v.y<minY.y)minY = v;
			if(v.y>maxY.y)maxY = v;
			if(v.z<minZ.z)minZ = v;
			if(v.z>maxZ.z)maxZ = v;
		}

		if(minX.x<(this->r.length()/2.)){
			//check left
			Vector3 n = Vector3(-1,0,0);
			Vector3 v = minX;
			pos.x += this->r.length()/2.-v.x;
			build();
			Vector3 rP = v-pos;
			//if(sideCollide(v,'l'))rP = pos+size/2*n;
			applyImpulse(n,rP);

		}
		if(maxX.x>=(20+this->r.length()/2.)){
			//check right
			Vector3 n = Vector3(1,0,0);
			Vector3 v = maxX;
			pos.x -= v.x-20-this->r.length()/2.;
			build();
			Vector3 rP = v-pos;
			//if(sideCollide(v,'r'))rP = pos+size/2*n;
			applyImpulse(n,rP);
		}
		if(minY.y<(this->r.length()/2)){
			//check down
			Vector3 n = Vector3(0,-1,0);
			Vector3 v = minY;
			pos.y += this->r.length()/2.-v.y;
			build();
			Vector3 rP = v-pos;
			//if(sideCollide(v,'d'))rP = pos+size/2*n;
			applyImpulse(n,rP);
		}
		if(maxY.y>(20+this->r.length()/2.)){
			//check up
			Vector3 n = Vector3(0,1,0);
			Vector3 v = maxY;
			pos.y -= v.y-20-this->r.length()/2.;
			build();
			Vector3 rP = v-pos;
			//if(sideCollide(v,'u'))rP = pos+size/2*n;
			applyImpulse(n,rP);
		}
		if(minZ.z<(this->r.length()/2.)){
			//check front
			Vector3 n = Vector3(0,0,-1);
			Vector3 v = minZ;
			pos.z += this->r.length()/2.-v.z;
			build();
			Vector3 rP = v-pos;
			//if(sideCollide(v,'f'))rP = pos+size/2*n;
			applyImpulse(n,rP);
		}
		if(maxZ.z>(20+this->r.length()/2.)){
			//check back
			Vector3 n = Vector3(0,0,1);
			Vector3 v = maxZ;
			pos.z -= v.z-20-this->r.length()/2.;
			build();
			Vector3 rP = v-pos;
			//if(sideCollide(v,'b'))rP = pos+size/2*n;
			applyImpulse(n,rP);
		}

	}

	void handleWallCollisionS(float t){

		/*Matrix3 I_wall = Matrix3(
			density*400*67, 0, 0,
			0, density*400*67, 0,
			0, 0, density*400*67);
		Matrix3 I_wall_inv = I_wall.invert();*/
		Vector3 prevel;
		
		//Up Wall Collision
		if((pos.y/*+vel.y*dt*/)>=(20-this->getRadius())){
			Vector3 n = Vector3(0,1,0);

			pos.y = 20-this->getRadius();
			build();

			Vector3 minPoint = Vector3(0,-40,0);
			for(int i=0;i<triangles.size();i++){
				Triangle3D tri = offset[i];
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
			if(sideCollide(minPoint,'u'))rP = size/2*n;
			
			applyImpulse(n,rP);
			
		}

		//Down Wall Collision
		if((pos.y/*+vel.y*dt*/)<=(0+this->getRadius())){
			
			Vector3 n = Vector3(0,-1,0);	
				
			pos.y = this->getRadius();
			build();

			Vector3 minPoint = Vector3(0,40,0);
			for(int i=0;i<triangles.size();i++){
				Triangle3D tri = offset[i];
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
			if(sideCollide(minPoint,'d'))rP = size/2*n;
			applyImpulse(n,rP);
			
		}

		//Right Wall Collision
		if((pos.x/*+vel.x*dt*/)>=(20-this->getRadius())){
			Vector3 n = Vector3(1,0,0);	
				
			pos.x = 20-this->getRadius();
			build();

			Vector3 minPoint = Vector3(-40,0,0);
			for(int i=0;i<triangles.size();i++){
				Triangle3D tri = offset[i];
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
			if(sideCollide(minPoint,'r'))rP = size/2*n;
			applyImpulse(n,rP);
			
		}

		//Left Wall Collision
		if((pos.x/*+vel.x*dt*/)<=(0+this->getRadius())){
			Vector3 n = Vector3(-1,0,0);	
				
			pos.x = this->getRadius();
			build();
			
			Vector3 minPoint = Vector3(40,0,0);
			for(int i=0;i<triangles.size();i++){
				Triangle3D tri = offset[i];
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
			if(sideCollide(minPoint,'l'))rP = size/2*n;

			applyImpulse(n,rP);
		}

		//Front Wall Collision
		if((pos.z/*+vel.z*dt*/)<=(this->getRadius())){
			Vector3 n = Vector3(0,0,-1);	
				
			pos.z = this->getRadius();
			build();

			Vector3 minPoint = Vector3(0,0,40);
			for(int i=0;i<triangles.size();i++){
				Triangle3D tri = offset[i];
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
			if(sideCollide(minPoint,'f'))rP = size/2*n;

			applyImpulse(n,rP);
		}

		//Back Wall Collision
		if((pos.z/*+vel.z*dt*/)>=(20-this->getRadius())){
			Vector3 n = Vector3(0,0,1);	
				
			pos.z = 20-this->getRadius();
			build();

			Vector3 minPoint = Vector3(0,0,-40);
			for(int i=0;i<triangles.size();i++){
				Triangle3D tri = offset[i];
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
			if(sideCollide(minPoint,'b'))rP = size/2*n;
			applyImpulse(n,rP);
			
		}
		
	}

	bool sideCollide(Vector3 v, char c){
	

		float eps = 0.005;
		int cntr=0;
		switch(c){
			case 'u':
				cntr=0;
				for(int i=0;i<triangles.size();i++){
					Triangle3D tri = offset[i];
					Vector3 v1 = tri.getV1().toVec3();
					Vector3 v2 = tri.getV2().toVec3();
					Vector3 v3 = tri.getV3().toVec3();

					if(abs(v1.y-v.y)<eps)cntr++;
					if(abs(v2.y-v.y)<eps)cntr++;
					if(abs(v3.y-v.y)<eps)cntr++;
				}
				if(cntr>=4)return true;
				break;
			case 'd':
				cntr=0;
				for(int i=0;i<triangles.size();i++){
					Triangle3D tri = offset[i];
					Vector3 v1 = tri.getV1().toVec3();
					Vector3 v2 = tri.getV2().toVec3();
					Vector3 v3 = tri.getV3().toVec3();

					if(abs(v1.y-v.y)<eps)cntr++;
					if(abs(v2.y-v.y)<eps)cntr++;
					if(abs(v3.y-v.y)<eps)cntr++;
				}
				if(cntr>=4)return true;
				break;
			case 'r':
				cntr=0;
				for(int i=0;i<triangles.size();i++){
					Triangle3D tri = offset[i];
					Vector3 v1 = tri.getV1().toVec3();
					Vector3 v2 = tri.getV2().toVec3();
					Vector3 v3 = tri.getV3().toVec3();

					if(abs(v1.x-v.x)<eps)cntr++;
					if(abs(v2.x-v.x)<eps)cntr++;
					if(abs(v3.x-v.x)<eps)cntr++;
				}
				if(cntr>=4)return true;
				break;
			case 'l':
				cntr=0;
				for(int i=0;i<triangles.size();i++){
					Triangle3D tri = offset[i];
					Vector3 v1 = tri.getV1().toVec3();
					Vector3 v2 = tri.getV2().toVec3();
					Vector3 v3 = tri.getV3().toVec3();

					if(abs(v1.x-v.x)<eps)cntr++;
					if(abs(v2.x-v.x)<eps)cntr++;
					if(abs(v3.x-v.x)<eps)cntr++;
				}
				if(cntr>=4)return true;
				break;
			case 'f':
				cntr=0;
				for(int i=0;i<triangles.size();i++){
					Triangle3D tri = offset[i];
					Vector3 v1 = tri.getV1().toVec3();
					Vector3 v2 = tri.getV2().toVec3();
					Vector3 v3 = tri.getV3().toVec3();

					if(abs(v1.z-v.z)<eps)cntr++;
					if(abs(v2.z-v.z)<eps)cntr++;
					if(abs(v3.z-v.z)<eps)cntr++;
				}
				if(cntr>=4)return true;
				break;
			case 'b':
				cntr=0;
				for(int i=0;i<triangles.size();i++){
					Triangle3D tri = offset[i];
					Vector3 v1 = tri.getV1().toVec3();
					Vector3 v2 = tri.getV2().toVec3();
					Vector3 v3 = tri.getV3().toVec3();

					if(abs(v1.z-v.z)<eps)cntr++;
					if(abs(v2.z-v.z)<eps)cntr++;
					if(abs(v3.z-v.z)<eps)cntr++;
				}
				if(cntr>=4)return true;
				break;
			default:
				break;
		};

		return false;
	}

	void setVelocity(Vector3 vel){
		this->vel=vel;
	}

	void setTriangles(vector<Triangle3D> triangles){
		this->triangles=triangles;
	}

	void setVertices(vector<Vector3D*> vertices){
		this->vertices=vertices;
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
			this->impactTimeCounter -= (1./this->impactTimeLength)*dt*(*this->fps/10)*(1000);
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

		float invR=(1.)/(2*this->getRadius());
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
		

			this->colorModel->displayHSV(v1.phi(),v1.slength(),0.5*(1+v1.y));
			//this->colorModel->displayHSV(v1->phi(),v1->slength(),v1->theta()*0.5*(1+v1->y));
			v1.r=this->colorModel->getRed();v1.g=this->colorModel->getGreen();v1.b=this->colorModel->getBlue();
			
			this->colorModel->displayHSV(v2.phi(),v2.slength(),0.5*(1+v2.y));
			//this->colorModel->displayHSV(v2->phi(),v1->slength(),v2->theta()*0.5*(1+v2->y));
			v2.r=this->colorModel->getRed();v2.g=this->colorModel->getGreen();v2.b=this->colorModel->getBlue();
				
			this->colorModel->displayHSV(v3.phi(),v3.slength(),0.5*(1+v3.y));
			//this->colorModel->displayHSV(v3->phi(),v3->slength(),v3->theta()*0.5*(1+v3->y));
			v3.r=this->colorModel->getRed();v3.g=this->colorModel->getGreen();v3.b=this->colorModel->getBlue();
			
			tri.v1.r=v1.r;tri.v1.g=v1.g;tri.v1.b=v1.b;
			tri.v2.r=v2.r;tri.v2.g=v2.g;tri.v2.b=v2.b;
			tri.v3.r=v3.r;tri.v3.g=v3.g;tri.v3.b=v3.b;
			this->triangles[i].setV1(tri.v1);this->triangles[i].setV2(tri.v2);this->triangles[i].setV3(tri.v3);
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
		this->pos=pos;
		build();
	}


	Vector3 getVelocity(){
		return this->vel;
	}

	Vector3 getPos(){
		return this->pos;
	}

	
	float getDT(){
		return this->dt;
	}

	float getRadius(){
		return (this->size.length())/2.;
	}

	Vector3 getSize(){
		return this->size;
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

	float getVolume(){
		return size.x*size.y*size.z;
	}

	float getMass(){
		return this->mass;
	}

	vector<Triangle3D> getTriangles(){
		return this->triangles;
	}

	vector<Triangle3D> getOffset(){
		return this->offset;
	}

	State getState(){
		State s = State();
		s.pos = pos;
		s.q = q;
		s.P = P;
		s.L = L;

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
		Quaternion w_hat = Quaternion(rot, 1);
		Quaternion q_dot = (w_hat * q) / 2;

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

	/*float* integrate(float t0, int n, float u0 [], float step){

		float *f0;
		float *f1;
		float *f2;
		float *f3;
		int i;
		float t1;
		float t2;
		float t3;
		float *u;
		float *u1;
		float *u2;
		float *u3;
		//
		//  Get four sample values of the derivative.
		//
		f0 = dxdt(t0, n, u0);

		t1 = t0 + step / 2.0f;
		u1 = new float[n];
		for (i = 0; i < n; i++){
			u1[i] = u0[i] + step * f0[i] / 2.0f;
		}
		f1 = dxdt(t1, n, u1);

		t2 = t0 + step / 2.0f;
		u2 = new float[n];
		for (i = 0; i < n; i++){
			u2[i] = u0[i] + step * f1[i] / 2.0f;
		}
		f2 = dxdt(t2, n, u2);

		t3 = t0 + step;
		u3 = new float[n];
		for (i = 0; i < n; i++){
			u3[i] = u0[i] + step * f2[i];
		}
		f3 = dxdt(t3, n, u3);
		//
		//  Combine them to estimate the solution.
		//
		u = new float[n];
		for (i = 0; i < n; i++){
			u[i] = u0[i] + step * (f0[i] + 2.0f * f1[i] + 2.0f * f2[i] + f3[i]) / 6.0f;
		}
		//
		//  Free memory.
		//
		delete [] f0;
		delete [] f1;
		delete [] f2;
		delete [] f3;
		delete [] u1;
		delete [] u2;
	    delete [] u3;

		return u;
	}*/
};