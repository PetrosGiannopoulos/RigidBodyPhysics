#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#endif

class RigidBody{

public:

	Vector3 r;
	float mass;
	Vector3 size;
	float SR3;

	float dt;
	Vector3 vel;
	Vector3 nvel;
	Vector3 pos;
	Vector3 minDistPoint;
	Vector3 impactPoint;

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

	float density;
	int *mode;
	float *fps;

	vector<float> impactDist1_;
	vector<float> impactDist2_;
	vector<float> impactDist3_;

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

	float eps;

	bool *show_mesh;
	bool isStatic;

	vector<Triangle3D> triangles;
	vector<Vector3D*> vertices;
	ColorModel *colorModel;

	vector<Triangle3D> offset;
	vector<Vector3D> offset_v;

	int type;

	RigidBody(){}

	RigidBody(Vector3 rsize,Vector3 vel,Vector3 pos,bool *show_mesh,int *mode,int type){
		this->type=type;
		SR3 = sqrt(3.);
		if(type==0)this->r = rsize;
		else{
			this->size = rsize;
			this->r = Vector3(getRadius(),getRadius(),getRadius());
		}
		
		this->pos = Vector3(pos.x-10,pos.y-10,pos.z-10);
		this->vel = vel;//(vel)-(pos);
		this->dt = 0.002;
		//this->dt = 0.01;
		this->density = 8.15;
		this->mass = density*getVolume(type);
		this->show_mesh=show_mesh;
		this->max_vel=100;
		this->max_dist=34.641;
		this->minDistPoint = Vector3(0,0,0);
		this->impactTimeCounter=0;
		this->impactTimeLength = 1000; //ms
		this->impactSatValue=0.5;
		this->impactHappen=false;
		this->impactDist1=0.5;
		this->impactDist2=0.5;
		this->impactDist3=0.5;
		this->impactFirstTime=true;
		this->mode=mode;
		this->eps=0.4;
		this->isStatic=false;

		this->rot = this->tau = this->f = this->L = Vector3(0,0,0);
		this->P = this->mass*this->vel;
		this->q = Quaternion(rot,1);

		if(type==0){
			this->I_body = this->I = Matrix3(
			this->mass*(this->r.y*this->r.y+this->r.z*this->r.z), 0, 0,
			0, this->mass*(this->r.x*this->r.x+this->r.z*this->r.z), 0,
			0, 0, this->mass*(this->r.x*this->r.x+this->r.y*this->r.y));
		}
		else{
			this->I_body = this->I = Matrix3(
			1./6*this->mass*(this->size.y*this->size.y+this->size.z*this->size.z), 0, 0,
			0, 1./6*this->mass*(this->size.x*this->size.x+this->size.z*this->size.z), 0,
			0, 0, 1./6*this->mass*(this->size.x*this->size.x+this->size.y*this->size.y));
		}

		this->L = this->I_body * this->rot;

		this->I_inv_body = this->I_inv =  this->I_body.invert();

		this->state=State(this->pos,this->q,this->P,this->L);
	}

	
	virtual void setPos(Vector3 pos){this->pos=pos;}

	virtual void setVelocity(Vector3 vel){this->vel=vel;}

	virtual float getDT(){return this->dt;}

	virtual Vector3 getPos(){return this->pos;}

	virtual Vector3 getVelocity(){return this->vel;}

	virtual float getMass(){return this->mass;}

	virtual float getRadius(){return this->r.length()/2;}

	virtual float getVolume(int type){
		float vol = 0;
		if(type == 0)vol = ((4/3.)*PI*r.x*r.y*r.z);
		else vol = size.x*size.y*size.z;
		return vol;
	}

	virtual void setImpactPoint(Vector3 impactPoint){}

	virtual void setImpactTimeCounter(float impactTimeCounter){}
	virtual void update(float t){};
};