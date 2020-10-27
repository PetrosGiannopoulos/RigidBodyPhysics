#ifndef CONTACT_H
#define CONTACT_H
#endif

class Contact{

public:
	Vector3 point,normal;
	Vector3 tang1,tang2;
	float depth;
	Vector3 pointA,pointB;
	Vector3 penetrationD;
	bool found;

	Contact(){
		point = Vector3(0,0,0);
		normal = Vector3(0,0,0);
		depth = 0;
		found=false;
	}
	
};