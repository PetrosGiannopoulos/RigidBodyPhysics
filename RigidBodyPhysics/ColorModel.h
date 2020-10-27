#ifndef ColorModel_H
#define ColorModel_H
#endif

using namespace std;

class ColorModel{

public:

	float h,s,v;
	float r,g,b;

	ColorModel(){}
	//h:hue, s:saturation,v: brightness value
	void displayHSV(float h,float s,float v){

		this->h=h;
		this->s=s;
		this->v=v;

		float c,x,m;
		float fr,fg,fb;
		float hh;
		hh = h*360.;

		float j = fmod(hh/60.,2.);
		c = v*s;
		x = c*(1-std::abs(j-1));
		m = v-c;

		if(hh<60){r=c;g=x;b=0;}
		else if(hh<120){r=x;g=c;b=0;}
		else if(hh<180){r=0;g=c;b=x;}
		else if(hh<240){r=0;g=x;b=c;}
		else if(hh<300){r=x;g=0;b=c;}
		else if(hh<360){r=c;g=0;b=x;}

		fr = (r+m);
		fg = (g+m);
		fb = (b+m);

		this->r=fr;
		this->g=fg;
		this->b=fb;

		//printf("red: %f,green: %f,blue: %f\n",r,g,b);
		//glColor3f(fr,fg,fb);
	}

	void displayLAB(float L,float A,float B){
		float Xr = 95.047;
		float Yr = 100;
		float Zr = 108.883;

		float e = 0.008856;
		float k = 903.3;
		
		float fy = (L+16)/116.;
		float fx = A/500.+fy;
		float fz = fy-b/200.;

		float xr,yr,zr;

		if(pow(fx,3)>e)xr = pow(fx,3);
		else xr = (116.*fx-16.)/k;

		if(L>k*e)yr = pow((L+16.)/116.,3);
		else yr = L/k;

		if(pow(fz,3)>e) zr = pow(fz,3);
		else zr = (116.*fz-16.)/k;

		float X = xr*Xr,Y = yr*Yr,Z = zr*Zr;


		this->r = 2.0413690*X-0.5649464*Y-0.3446944*Z;
		this->g =-0.9692660*X+1.8760108*Y+0.0415560*Z;
		this->b = 0.0134474*X-0.1183897*Y+1.0154096*Z;
	}

	void displayRYB(float R,float Y,float B){
		
		// remove the whiteness from the color
		float w = fmin(fmin(R,Y),B);
		float nr = R-w;
		float ny = Y-w;
		float nb = B-w;

		float my = fmax(fmax(nr, ny), nb);

		// get the green out of the yellow and blue
		float ng = fmin(ny, nb);
		ny -= ng;
		nb -= ng;
		
		if(nb != 0 && ng != 0){
			nb *= 2.0;
			ng *= 2.0;
		}

		// redistribute the remaining yellow
		nr += ny;
		ng += ny;
		
		// normalize to values
		float mg = fmax(fmax(nr, ng), nb);
		if(mg != 0){
			float n = my / mg;
			nr *= n;
			ng *= n;
			nb *= n;
		}
		
		// add the white back in
		nr += w;
		ng += w;
		nb += w;

		this->r = nr;
		this->g = ng;
		this->b = nb;

	}

	float getHue(){
		return this->h;
	}

	float getSaturation(){
		return this->s;
	}

	float getBrightness(){
		return this->v;
	}

	float getRed(){
		return this->r;
	}

	float getGreen(){
		return this->g;
	}

	float getBlue(){
		return this->b;
	}

};