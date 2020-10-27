#ifndef EDGE3D_H
#define EDGE3D_H
#endif

class Edge3D{

public:
	Vector3D v1,v2;
	Vector3D n;
	bool valid;

	Edge3D(Vector3D v1,Vector3D v2,Vector3D n){
		this->v1=v1;
		this->v2=v2;
		this->n=n;
		this->valid=true;
	}

	Edge3D(Vector3D v1,Vector3D v2){
		this->v1=v1;
		this->v2=v2;
		this->valid=true;
	}

	bool equals(Edge3D e){
		if((e.getV1().equals(this->v1) && e.getV2().equals(this->v2)) || (e.getV1().equals(this->v2) && e.getV2().equals(this->v1)))return true;
		return false;
	}

	void setInvalid(){
		this->valid = false;
	}

	bool isValid(){
		return this->valid;
	}

	void setV1(Vector3D v1){
		this->v1=v1;
	}

	void setV2(Vector3D v2){
		this->v2=v2;
	}

	Vector3D getV1(){
		return this->v1;
	}

	Vector3D getV2(){
		return this->v2;
	}

	Vector3D getNorm(){
		return this->n;
	}
};