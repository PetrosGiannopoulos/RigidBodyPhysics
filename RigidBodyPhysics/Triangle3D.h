#ifndef TRIANGLE3D_H
#define TRIANGLE3D_H
#endif

class Triangle3D{

public:
	Vector3D *v1_ptr;
	Vector3D *v2_ptr;
	Vector3D *v3_ptr;

	Vector3D v1;
	Vector3D v2;
	Vector3D v3;

	float a,b,c,d;

	Triangle3D(){}

	Triangle3D(Vector3D *v1_ptr,Vector3D *v2_ptr,Vector3D *v3_ptr){

		this->v1 = Vector3D(v1_ptr->x,v1_ptr->y,v1_ptr->z,v1_ptr->r,v1_ptr->g,v1_ptr->b);
		this->v2 = Vector3D(v2_ptr->x,v2_ptr->y,v2_ptr->z,v2_ptr->r,v2_ptr->g,v2_ptr->b);
		this->v3 = Vector3D(v3_ptr->x,v3_ptr->y,v3_ptr->z,v3_ptr->r,v3_ptr->g,v3_ptr->b);

		computePlane();
	}
	Triangle3D(Vector3D v1,Vector3D v2,Vector3D v3){

		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;

		computePlane();
	}

	Triangle3D(Vector3 v1,Vector3 v2,Vector3 v3){

		this->v1 = Vector3D(v1.x,v1.y,v1.z);
		this->v2 = Vector3D(v2.x,v2.y,v2.z);
		this->v3 = Vector3D(v3.x,v3.y,v3.z);

		this->v1.sup_aX = v1.sup_aX;this->v1.sup_aY = v1.sup_aY;this->v1.sup_aZ = v1.sup_aZ;
		this->v1.sup_bX = v1.sup_bX;this->v1.sup_bY = v1.sup_bY;this->v1.sup_bZ = v1.sup_bZ;

		this->v2.sup_aX = v2.sup_aX;this->v2.sup_aY = v2.sup_aY;this->v2.sup_aZ = v2.sup_aZ;
		this->v2.sup_bX = v2.sup_bX;this->v2.sup_bY = v2.sup_bY;this->v2.sup_bZ = v2.sup_bZ;

		this->v3.sup_aX = v3.sup_aX;this->v3.sup_aY = v3.sup_aY;this->v3.sup_aZ = v3.sup_aZ;
		this->v3.sup_bX = v3.sup_bX;this->v3.sup_bY = v3.sup_bY;this->v3.sup_bZ = v3.sup_bZ;

		computePlane();
		
	}

	void setV1(Vector3D v1){
		this->v1 = v1;
	}

	void setV2(Vector3D v2){
		this->v2 = v2;
	}

	void setV3(Vector3D v3){
		this->v3 = v3;
	}

	Vector3D getV1(){
		return this->v1;
	}

	Vector3D getV2(){
		return this->v2;
	}

	Vector3D getV3(){
		return this->v3;
	}

	float edgeDistance(){
		return sqrt((this->v1.x-this->v2.x)*(this->v1.x-this->v2.x)+(this->v1.y-this->v2.y)*(this->v1.y-this->v2.y)+(this->v1.z-this->v2.z)*(this->v1.z-this->v2.z));
	}

	bool intersects(Triangle3D tri,Vector3D &intersectPoint,Vector3D &i_norm,float &dist){

		bool intersectFlag = false;

		Vector3D vv1 = tri.getV1();
		Vector3D vv2 = tri.getV2();
		Vector3D vv3 = tri.getV3();

		float dV1 = a*vv1.x+b*vv1.y+c*vv1.z+d;
		float dV2 = a*vv2.x+b*vv2.y+c*vv2.z+d;
		float dV3 = a*vv3.x+b*vv3.y+c*vv3.z+d;

		float dVV1 = tri.a*v1.x+tri.b*v1.y+tri.c*v1.z+tri.d;
		float dVV2 = tri.a*v2.x+tri.b*v2.y+tri.c*v2.z+tri.d;
		float dVV3 = tri.a*v3.x+tri.b*v3.y+tri.c*v3.z+tri.d;

		Vector3D tri_n = tri.normal();
		Vector3D cent = tri.centroid();
		
		Vector3D res1;
		Vector3D res2;

		float a12 = (v2.y-v1.y)/(v2.x-v1.x);
		float b12 = v1.y-a12*v1.x;

		float a13 = (v3.y-v1.y)/(v3.x-v1.x);
		float b13 = v1.y-a13*v1.x;

		float a23 = (v3.y-v2.y)/(v3.x-v2.x);
		float b23 = v2.y-a23*v2.x;

		//ax+by+cz-d=0
		if((dV1>=0 && dV2<0 && dV3<0) || (dV1<=0 && dV2>0 && dV3>0)){
			if(dV2<0)res1 = vv1+dV2*((vv1-vv2).normalize());
			else res1 = vv1+dV2*((vv2-vv1).normalize());
			if(dV3<0)res2 = vv1+dV3*((vv1-vv3).normalize());
			else res2 = vv1+dV3*((vv3-vv1).normalize());
			if((res1.y>=a12*res1.x+b12) && (res1.y>=a13*res1.x+b13) && (res1.y>=a23*res1.x+b23)){intersectFlag = true;intersectPoint=res1;}
			if((res2.y>=a12*res2.x+b12) && (res2.y>=a13*res2.x+b13) && (res2.y>=a23*res2.x+b23)){intersectFlag = true;intersectPoint=res2;}
			i_norm = tri_n;
			dist = dV1;
		}

		if((dV2>=0 && dV1<0 && dV3<0) || (dV2<=0 && dV1>0 && dV3>0)){
			if(dV2<0)res1 = vv1+dV2*((vv1-vv2).normalize());
			else res1 = vv1+dV2*((vv2-vv1).normalize());
			if(dV3<0)res2 = vv1+dV3*((vv1-vv3).normalize());
			else res2 = vv1+dV3*((vv3-vv1).normalize());
			if((res1.y>=a12*res1.x+b12) && (res1.y>=a13*res1.x+b13) && (res1.y>=a23*res1.x+b23)){intersectFlag = true;intersectPoint=res1;}
			if((res2.y>=a12*res2.x+b12) && (res2.y>=a13*res2.x+b13) && (res2.y>=a23*res2.x+b23)){intersectFlag = true;intersectPoint=res2;}
			i_norm = tri_n;
			dist = dV2;
		}

		if((dV3>=0 && dV1<0 && dV2<0) || (dV3<=0 && dV1>0 && dV2>0)){
			if(dV2<0)res1 = vv1+dV2*((vv1-vv2).normalize());
			else res1 = vv1+dV2*((vv2-vv1).normalize());
			if(dV3<0)res2 = vv1+dV3*((vv1-vv3).normalize());
			else res2 = vv1+dV3*((vv3-vv1).normalize());
			if((res1.y>=a12*res1.x+b12) && (res1.y>=a13*res1.x+b13) && (res1.y>=a23*res1.x+b23)){intersectFlag = true;intersectPoint=res1;}
			if((res2.y>=a12*res2.x+b12) && (res2.y>=a13*res2.x+b13) && (res2.y>=a23*res2.x+b23)){intersectFlag = true;intersectPoint=res2;}
			i_norm = tri_n;
			dist = dV3;
		}

		

		return intersectFlag;
	}

	void computePlane(){
		this->a = v1.y * (v2.z - v3.z) + v2.y * (v3.z - v1.z) + v3.y * (v1.z - v2.z);
		this->b = v1.z * (v2.x - v3.x) + v2.z * (v3.x - v1.x) + v3.z * (v1.x - v2.x);
		this->c = v1.x * (v2.y - v3.y) + v2.x * (v3.y - v1.y) + v3.x * (v1.y - v2.y);
		this->d = -( v1.x * ( v2.y * v3.z - v3.y * v2.z ) + v2.x * (v3.y * v1.z - v1.y * v3.z) + v3.x * (v1.y * v2.z - v2.y * v1.z) );
	}

	Vector3D linear1(float amount){
		Vector3D res = Vector3D();
		res.x = v1.x*amount+v2.x*(1-amount);
		res.y = v1.y*amount+v2.y*(1-amount);
		res.z = v1.z*amount+v2.z*(1-amount);
		return res;
	}

	Vector3D linear2(float amount){
		Vector3D res = Vector3D();
		res.x = v2.x*amount+v3.x*(1-amount);
		res.y = v2.y*amount+v3.y*(1-amount);
		res.z = v2.z*amount+v3.z*(1-amount);
		return res;
	}

	Vector3D linear3(float amount){
		Vector3D res = Vector3D();
		res.x = v3.x*amount+v1.x*(1-amount);
		res.y = v3.y*amount+v1.y*(1-amount);
		res.z = v3.z*amount+v1.z*(1-amount);
		return res;
	}

	Vector3D centroid(){
		Vector3D res = Vector3D();
		res.x = (v1.x+v2.x+v3.x)/3;
		res.y = (v1.y+v2.y+v3.y)/3;
		res.z = (v1.z+v2.z+v3.z)/3;

		res = (res*normal())*(sqrt(2./3.)*edgeDistance());
		return res;
	}

	Vector3D normal(){
		Vector3D res = Vector3D();

		//res = (v1-v3).cross(v2-v1);
		res = (v2-v1).cross(v3-v1);
		res = res.normalize();
		return res;
	}

};