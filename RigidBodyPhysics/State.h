#ifndef STATE_H
#define STATE_H
#endif

class State{

public:
	Vector3 pos;
	Quaternion q;
	Vector3 P;
	Vector3 L;

	State(){}

	State(Vector3 pos,Quaternion q,Vector3 P,Vector3 L){
		this->pos = pos;
		this->q = q;
		this->P = P;
		this->L = L;
	}

	void raise(State s, float step){
		this->pos += s.pos*step;
		this->q = this->q+step*s.q;
		this->P += s.P*step;
		this->L += s.L*step;

	}

	void combine(State s0,State s1,State s2,State s3,float step){
		this->pos += step*(s0.pos+2.0*s1.pos+2.0*s2.pos+s3.pos);
		this->q = this->q+step*(s0.q+2.0*s1.q+2.0*s2.q+s3.q);
		this->P += step*(s0.P+2.0*s1.P+2.0*s2.P+s3.P);
		this->L += step*(s0.L+2.0*s1.L+2.0*s2.L+s3.L);
	}

};