#ifndef VECTOR3D_H
#define VECTOR3D_H
#endif

#define PI 3.1415927

class Vector3D{

public:
	float x,y,z;
	float r,g,b;

	Vector3D(){}
	~Vector3D(){}

	float sup_aX,sup_aY,sup_aZ;
	float sup_bX,sup_bY,sup_bZ;

	Vector3D(float x,float y,float z){
		this->x=x;this->y=y;this->z=z;
	}

	Vector3D(float x,float y,float z,float r,float g,float b){
		this->x=x;this->y=y;this->z=z;
		this->r=r;this->g=g;this->b=b;
	}

	Vector3D(Vector3 v){
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
	}

	bool equals(Vector3D v){
		float eps = 0.005;
		if(abs(v.x-x)<eps && abs(v.y-y)<eps && abs(v.z-z)<eps)return true;
		return false;
	}

	void transferColor(Vector3D &v){
		v.r = this->r;
		v.g = this->g;
		v.b = this->b;
	}

	float getX(){
		return this->x;
	}

	float getY(){
		return this->y;
	}

	float getZ(){
		return this->z;
	}

	float length(){
		return sqrt(x*x+y*y+z*z);
	}
	
	float slength(){
		//return sqrt(x*x+z*z)*0.7;
		return sqrt(x*x+z*z);
	}

	float slength(Vector3D v){
		//return sqrt(x*x+z*z)*0.7;
		return sqrt((x-v.x)*(x-v.x)+(z-v.z)*(z-v.z));
	}

	float slength(Vector3 v){
		//return sqrt(x*x+z*z)*0.7;
		return sqrt((x-v.x)*(x-v.x)+(z-v.z)*(z-v.z));
	}


	float theta(){
		//return acos(y/length());
		float th =  atan(y/length());

		th += PI/2;
		th /= PI;

		return th;
	}

	float phi(){
		float ph,var;
		/*if(x > 0) ph= asin( y / slength());
		else ph=PI - asin( y / slength());
		
		if (x > 0 && z <0 ) ph= PI - ph;*/

		//ph = asin(y/length());
		var = (1./PI)*0.5;
		/*if(z>=0 && x>0)ph = atan2(z,x);
		else if(z<=0 && x>=0)ph = atan2(-z,x);
		else if(z>=0 && x<=0) ph = atan2(z,-x);
		else ph = atan2(-z,-x);*/

		
		ph = atan2(z,x);
		if (x < 0 && z ==0 ) ph = atan2(z+0.01f,x);
		
		return ((ph*var)+0.5);
	}

	Vector3D normalize(){
		return (*this)/length();
	}

	Vector3D cross(Vector3D v){
		Vector3D res = Vector3D();
		res.x = y*v.z-z*v.y;
		res.y = z*v.x-x*v.z;
		res.z = x*v.y-y*v.x;
		return res;
	}

	float dot(Vector3D *v){
		float res = x*v->x+y*v->y+z*v->z;
		return res;
	}

	float dot(Vector3D v){
		float res = x*v.x+y*v.y+z*v.z;
		return res;
	}

	float distance(Vector3D *v){
		return sqrt((x-v->x)*(x-v->x)+(y-v->y)*(y-v->y)+(z-v->z)*(z-v->z));
	}

	float distance(Vector3D v){
		return sqrt((x-v.x)*(x-v.x)+(y-v.y)*(y-v.y)+(z-v.z)*(z-v.z));
	}

	float distance(Vector3 v){
		return sqrt((x-v.x)*(x-v.x)+(y-v.y)*(y-v.y)+(z-v.z)*(z-v.z));
	}

	float betweenAngle(Vector3D v){
		return acos((v.dot(this))/(v.length()*this->length()));
	}

	float betweenAngleD(Vector3D v){
		return (acos((v.dot(this))/(v.length()*this->length()))*(180./PI));
	}

	Vector3 toVec3(){
		return Vector3(this->x,this->y,this->z);
	}

	Vector3D* operator+(const Vector3D *v){
		Vector3D *vec = new Vector3D();

		vec->x = x+v->x;
		vec->y = y+v->y;
		vec->z = z+v->z;
		vec->r=r;
		vec->g=g;
		vec->b=b;

		return vec;
	}

	Vector3D operator+(const Vector3D v){
		Vector3D vec = Vector3D();

		vec.x = x+v.x;
		vec.y = y+v.y;
		vec.z = z+v.z;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	Vector3D* operator-(const Vector3D *v){
		Vector3D *vec = new Vector3D();

		vec->x = x-v->x;
		vec->y = y-v->y;
		vec->z = z-v->z;
		vec->r=r;
		vec->g=g;
		vec->b=b;

		return vec;
	}

	Vector3D operator-(const Vector3D v){
		Vector3D vec = Vector3D();

		vec.x = x-v.x;
		vec.y = y-v.y;
		vec.z = z-v.z;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	Vector3D* operator*(const Vector3D *v){
		Vector3D *vec = new Vector3D();

		vec->x = x*v->x;
		vec->y = y*v->y;
		vec->z = z*v->z;
		vec->r=r;
		vec->g=g;
		vec->b=b;

		return vec;
	}

	Vector3D operator*(const Vector3D v){
		Vector3D vec = Vector3D();

		vec.x = x*v.x;
		vec.y = y*v.y;
		vec.z = z*v.z;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	Vector3D operator*(const float v){
		Vector3D vec = Vector3D();

		vec.x = x*v;
		vec.y = y*v;
		vec.z = z*v;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	friend Vector3D operator*(const float v,Vector3D vH){
		Vector3D vec = Vector3D();

		vec.x = vH.x*v;
		vec.y = vH.y*v;
		vec.z = vH.z*v;
		vec.r=vH.r;
		vec.g=vH.g;
		vec.b=vH.b;

		return vec;
	}

	Vector3D operator/(const float v){
		Vector3D vec = Vector3D();

		vec.x = x/v;
		vec.y = y/v;
		vec.z = z/v;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	Vector3D operator-(const float v){
		Vector3D vec = Vector3D();

		vec.x = x-v;
		vec.y = y-v;
		vec.z = z-v;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	Vector3D operator+(const float v){
		Vector3D vec = Vector3D();

		vec.x = x+v;
		vec.y = y+v;
		vec.z = z+v;
		vec.r=r;
		vec.g=g;
		vec.b=b;

		return vec;
	}

	friend Vector3D operator+(const float v,Vector3D vH){
		Vector3D vec = Vector3D();

		vec.x = vH.x+v;
		vec.y = vH.y+v;
		vec.z = vH.z+v;
		vec.r=vH.r;
		vec.g=vH.g;
		vec.b=vH.b;

		return vec;
	}
};