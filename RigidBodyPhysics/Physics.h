#ifndef PHYSICS_H
#define PHYSICS_H
#endif

#define PI 3.1415927

class Physics{

private:
	vector<Sphere*> spheres;
	vector<Cube*> cubes;
	vector<RigidBody*> objects;
	float dt;
	float e;
	float *total_t;
	float tolerance;

public:
	
	Physics(){
		e = 0.4;
		tolerance = 0.0001f;
	}

	void addSphere(Sphere *sphere){
		this->spheres.push_back(sphere);
		this->objects.push_back(sphere);
	}

	void addCube(Cube *cube){
		this->cubes.push_back(cube);
		this->objects.push_back(cube);
	}

	void clearAll(){
		spheres.clear();
		cubes.clear();
		objects.clear();
	}

	Vector3 support(RigidBody *rig,Vector3 d){
		
		if(d.length()!=0)d = d.normalize();

		Vector3 fur = (rig->offset_v[0]).toVec3();
		float dotP = -FLT_MAX;
		Matrix3 R = rig->q.rotationMatrix();
		for(int i=0;i<rig->offset_v.size();i++){
			//local
			Vector3D v_ = rig->offset_v[i];
			//global
			//Vector3 RV = rig->q.rotatedVector(v_.toVec3());
			Vector3D v = v_;//Vector3D(rig->pos+R*(v_.toVec3()*rig->size));
			//Vector3D v = *rig->vertices[i];

			if(v.dot(d)>dotP){
				dotP = v.dot(d);
				fur = v.toVec3();
			}
		}

		
		//global
		//fur = rig->pos+R*fur*rig->size;

		return fur;
	}

	bool Simplex4Assist(vector<Vector3> &simplex,Vector3 &direction){
	
		Vector3 a = simplex[2];
		Vector3 b = simplex[1];
		Vector3 c = simplex[0];

		Vector3 ab = b-a;
		Vector3 ac = c-a;
		Vector3 ao = -a;
		Vector3 abc = ab.cross(ac);

		if((ab.cross(abc)).dot(ao)>0){
			simplex.clear();
			simplex.push_back(b);
			simplex.push_back(a);
			direction = (ab.cross(ao)).cross(ab);
			return false;
		}

		if((abc.cross(ac)).dot(ao)>0){
			simplex.clear();
			simplex.push_back(c);
			simplex.push_back(a);
			direction = (ac.cross(ao)).cross(ac);
			return false;
		}

		simplex.clear();
		simplex.push_back(c);
		simplex.push_back(b);
		simplex.push_back(a);
		direction = abc;

		return false;

	}

	bool doSimplex(vector<Vector3> &simplex,Vector3 &direction){
		
		int size = simplex.size();
		Vector3 ao = -simplex[size-1];
		if(size==2){
			Vector3 a = simplex[1];
			Vector3 b = simplex[0];
			Vector3 ab = b-a;
			
			direction  = (ab.cross(ao)).cross(ab);
			return false;
		}
		else if(size==3){
			Vector3 a = simplex[2];
			Vector3 b = simplex[1];
			Vector3 c = simplex[0];

			Vector3 ab = (b-a);
			Vector3 ac = (c-a);
			Vector3 abc = ab.cross(ac);
			if((ab.cross(abc)).dot(ao)>0){
				simplex.clear();
				simplex.push_back(b);
				simplex.push_back(a);
				direction = (ab.cross(ao)).cross(ab);
				return false;
			}
			if((abc.cross(ac)).dot(ao)>0){
				simplex.clear();
				simplex.push_back(c);
				simplex.push_back(a);
				direction = (ac.cross(ao)).cross(ac);
				return false;
			}
			if(abc.dot(ao)>0){
				direction = abc;
				return false;
			}

			simplex.clear();
			simplex.push_back(b);
			simplex.push_back(c);
			simplex.push_back(a);
			direction = -abc;
			return false;
		}
		else if(size==4){
			Vector3 a = simplex[3];
			Vector3 b = simplex[2];
			Vector3 c = simplex[1];
			Vector3 d = simplex[0];

			Vector3 ab = b-a;
			Vector3 ac = c-a;

			if((ab.cross(ac)).dot(ao)>0){
				Simplex4Assist(simplex,direction);
				return false;
			}

			Vector3 ad = d-a;
			if((ac.cross(ad)).dot(ao)>0){
				simplex.clear();
				simplex.push_back(d);
				simplex.push_back(c);
				simplex.push_back(a);
				Simplex4Assist(simplex,direction);
				return false;
			}
			if((ad.cross(ab)).dot(ao)>0){
				simplex.push_back(b);
				simplex.push_back(d);
				simplex.push_back(a);
				Simplex4Assist(simplex,direction);
				return false;
			}

			return true;
		}

		return false;
	}

	bool gjkCollision(RigidBody *rigI,RigidBody *rigJ,Contact &contact){
		Vector3 D = (rigJ->offset_v[0]-rigI->offset_v[0]).toVec3();
		Vector3 A = support(rigI,D)-support(rigJ,-D);
		vector<Vector3> simplex;
		simplex.push_back(A);
		D = -A;
		
		bool intersect = true;
		for(int i=0;i<50;i++){
			
			if(D.lengthSq()<0.0001f){intersect=false;break;}
			D = D.normalize();
			Vector3 supA = support(rigI,D);
			Vector3 supB = support(rigJ,-D);
			A = supA-supB;
			if(A.dot(D)<=0) {intersect = false;break;}
			A.sup_aX = supA.x;A.sup_aY = supA.y;A.sup_aZ = supA.z;
			A.sup_bX = supB.x;A.sup_bY = supB.y;A.sup_bZ = supB.z;
			simplex.push_back(A);
			if(doSimplex(simplex,D)){intersect = true;break;}
		}

		if(intersect){
			//if(simplex.size()<4)return false;
			//cout << simplex.size() << endl;
			EPA_ComputePenetration(rigI,rigJ,simplex,contact);
			//cout << "pen_depth: " << contact.depth << endl;
		}

		return intersect;

	}

	void EPA_ComputePenetration(RigidBody *rigI,RigidBody *rigJ,vector<Vector3> &s,Contact &contact){
		

		vector<Triangle3D> list_triangles;
		vector<Edge3D> list_edges;

		list_triangles.emplace_back(Triangle3D(s[0],s[1],s[2]));
		list_triangles.emplace_back(Triangle3D(s[0],s[2],s[3]));
		list_triangles.emplace_back(Triangle3D(s[0],s[3],s[1]));
		list_triangles.emplace_back(Triangle3D(s[1],s[3],s[2]));

		for(int j=0;j<75;j++){
			
			float min_dist=FLT_MAX;
			float min_i=0;
			for(int i=0;i<list_triangles.size();i++){
				Triangle3D tri = list_triangles[i];
				Vector3D norm = tri.normal().normalize();
				float dist = fabs(norm.dot(tri.getV1()));
				if(dist<min_dist){
					min_dist = dist;
					min_i = i;
				}
			}
			
			Triangle3D curTri = list_triangles[min_i];
			Vector3 n = curTri.normal().toVec3().normalize();
			Vector3 next_point = (support(rigI,n)-support(rigJ,-n)).normalize();
			if(((n.dot(next_point)-min_dist)<tolerance)){
				
				float bary_u,bary_v,bary_w;
				barycentric(n * min_dist,curTri.getV1().toVec3(),curTri.getV2().toVec3(),curTri.getV3().toVec3(),&bary_u,&bary_v,&bary_w);
				/*bary_u = curTri.centroid().x;
				bary_v = curTri.centroid().y;
				bary_w = curTri.centroid().z;*/

				Vector3 supA_1 = Vector3(curTri.getV1().sup_aX,curTri.getV1().sup_aY,curTri.getV1().sup_aZ);
				Vector3 supA_2 = Vector3(curTri.getV2().sup_aX,curTri.getV2().sup_aY,curTri.getV2().sup_aZ);
				Vector3 supA_3 = Vector3(curTri.getV3().sup_aX,curTri.getV3().sup_aY,curTri.getV3().sup_aZ);

				Vector3 supB_1 = Vector3(curTri.getV1().sup_bX,curTri.getV1().sup_bY,curTri.getV1().sup_bZ);
				Vector3 supB_2 = Vector3(curTri.getV2().sup_bX,curTri.getV2().sup_bY,curTri.getV2().sup_bZ);
				Vector3 supB_3 = Vector3(curTri.getV3().sup_bX,curTri.getV3().sup_bY,curTri.getV3().sup_bZ);

				//collision points of A,B-world
				Vector3 collision_pointA = bary_u*supA_1+bary_v*supA_2+bary_w*supA_3;
				Vector3 collision_pointB = bary_u*supB_1+bary_v*supB_2+bary_w*supB_3;

				//collision points of A,B-local
				Vector3 T_A = rigI->pos;
				Vector3 T_B = rigJ->pos;
				//Vector3 collision_pointA_Local = (rigI->q.inverse())*(collision_pointA-T_A);
				//Vector3 collision_pointB_Local = (rigJ->q.inverse())*(collision_pointB-T_B);

				//collision normal
				Vector3 collision_normal = n;

				//penetration depth
				float penetration_depth = min_dist;
				//cout << "pen_depth_: " << penetration_depth << endl;

				if(rigJ->isStatic==false)contact.point = collision_pointA;
				else contact.point = collision_pointB;
				//contact.point = collision_pointA;
				contact.normal = collision_normal;
				contact.depth = penetration_depth;
				contact.found=true;
				Vector3 tang1,tang2;
				computeTangents(collision_normal,tang1,tang2);
				contact.tang1 = tang1;
				contact.tang2 = tang2;
				contact.penetrationD = (collision_pointB-collision_pointA)*collision_normal;
				//contact.pointA = collision_pointA_Local;
				//contact.pointB = collision_pointB_Local;
				
				break;
			}
			
			for(int i=0;i<list_triangles.size();i++){
				Triangle3D tri = list_triangles[i];
				// can this face be 'seen' by entry_cur_support?
				if((tri.normal().toVec3()).dot(next_point-tri.getV1().toVec3())>0){
					Edge3D e1 = Edge3D(tri.getV1(),tri.getV2());
					Edge3D e2 = Edge3D(tri.getV2(),tri.getV3());
					Edge3D e3 = Edge3D(tri.getV3(),tri.getV1());

					EPA_addEdge(list_edges,e1);
					EPA_addEdge(list_edges,e2);
					EPA_addEdge(list_edges,e3);
					list_triangles.erase(list_triangles.begin()+i);
					continue;
				}
			}
			
			for(int i=0;i<list_edges.size();i++){
				Edge3D e_ = list_edges[i];
				list_triangles.emplace_back(Triangle3D(Vector3D(next_point),e_.getV1(),e_.getV2()));
			}

			list_edges.clear();
		}
	}

	void EPA_addEdge(vector<Edge3D> &edges,Edge3D new_edge){
		for(int i=0;i<edges.size();i++){
			Edge3D edge = edges[i];
			if(edge.getV1().equals(new_edge.getV2()) && edge.getV2().equals(new_edge.getV1())){
				edges.erase(edges.begin()+i);
				return;
			}
		}
		edges.push_back(new_edge);
	}

	void barycentric(Vector3 &p,Vector3 &a,Vector3 &b,Vector3 &c,float *u,float *v,float *w) {
		// code from Crister Erickson's Real-Time Collision Detection
		Vector3 v0 = b - a,v1 = c - a,v2 = p - a;
		float d00 = v0.dot(v0);
		float d01 = v0.dot(v1);
		float d11 = v1.dot(v1);
		float d20 = v2.dot(v0);
		float d21 = v2.dot(v1);
		float denom = d00 * d11 - d01 * d01;
		*v = (d11 * d20 - d01 * d21) / denom;
		*w = (d00 * d21 - d01 * d20) / denom;
		*u = 1.0f - *v - *w;
	}

	void computeTangents(Vector3 n,Vector3 &tang1,Vector3 &tang2){
		if (abs(n.x) >= 0.57735f)
			 tang1 = Vector3(n.y, -n.x, 0.0f);
		else
			 tang1 = Vector3(0.0f, n.z, -n.y);
 
		tang1 = tang1.normalize();
		tang2 = n.cross(tang1);
	}

	void checkObjectCollision(){
		if(objects.empty())return;
		dt = objects[0]->getDT();

		int collision_num=0;
		
		for(int i=0;i<objects.size();i++){
			RigidBody *rigI = objects[i];
			Vector3 posI = rigI->getPos();
			Vector3 velI = rigI->getVelocity();
			Vector3 n_posI = posI+velI*dt;
			float rI = rigI->getRadius();
			float im1 = 1./rigI->getMass();
			float m1 = rigI->getMass();
			Vector3 n_velI = Vector3();
			for(int j=0;j<objects.size();j++){
				if(i==j)continue;
				RigidBody *rigJ = objects[j];
				Vector3 posJ = rigJ->getPos();	
				Vector3 velJ = rigJ->getVelocity();
				Vector3 n_posJ = posJ+velJ*dt;
				float rJ = rigJ->getRadius();
				float im2 = 1./rigJ->getMass();
				float m2 = rigJ->getMass();
				Vector3 n_velJ=Vector3();

				bool sphereCollision = n_posI.distance(n_posJ)<=(rI+rJ);
				if(sphereCollision==false)continue;
				bool cubeCollision=false;
				Vector3D intersectPoint,i_norm;
				float col_dist;
				
				Contact contact = Contact();
				//cubeCollision = gjkCollision(rigI,rigJ,contact);

				bool collision=false;
				if(rigI->type==0 || rigJ->type==0)collision=true;
				else collision = cubeCollision;

				if(collision){
					//cout << "collision occured!!!" << endl;
					collision_num++;

					Vector3 impactPointI,impactPointJ;
					Vector3 n_difJI,n_difIJ;
				
					n_difJI = (posI-posJ).normalize();
					n_difIJ = (posJ-posI).normalize();

					impactPointI = posI+n_difIJ*rI;
					impactPointJ = posJ+n_difJI*rJ;
					
					Vector3 n = n_difIJ;
					float depth;
					Vector3 p;
					if(contact.found==true){
						n = contact.normal;
						depth = contact.depth;
						p = contact.point;
					}

					rigI->setImpactPoint(impactPointI);
					rigJ->setImpactPoint(impactPointJ);

					Vector3 rAP = impactPointI-n_posI;
					Vector3 rBP = impactPointJ-n_posJ;

					if(contact.found==true){
						rAP = p-n_posI;
						rBP = p-n_posJ;
						rigI->setImpactPoint(p);
						rigJ->setImpactPoint(p);
					}

					rigI->setImpactTimeCounter(1.0);
					rigJ->setImpactTimeCounter(1.0);
					
					//fix position
					Vector3 delta = n_posI-n_posJ;
					float d = delta.length();
					Vector3 mtd = (delta)*((rI+rJ-d)/d);
					if(contact.found==true){
						mtd = depth*n;
					}

					n_posI = n_posI+((mtd)*(im1/(im1+im2)));
					n_posJ = (n_posJ)-((mtd)*(im2/(im1+im2)));
				
					rigI->setPos(n_posI);
					rigJ->setPos(n_posJ);
				
					//if(rigI->type==1 && rigJ->type==1){
						//calculate impulse
						float impulseNom = (rigJ->vel+rigJ->rot.cross(rBP)-rigI->vel-rigI->rot.cross(rAP)).dot(n);
						//float impulseNom = (rigJ->vel-rigI->vel).dot(n_difIJ);
					
						float impulseDen = im1+im2+(rigI->I_inv*(rAP.cross(n)).cross(rAP)+rigJ->I_inv*(rBP.cross(n)).cross(rBP)).dot(n);
						float impulseM = -(1+e)*impulseNom/impulseDen;
						Vector3 impulse = n*impulseM;

						rigI->vel = rigI->vel-impulse*im1;
						rigJ->vel = rigJ->vel+impulse*im2;

						rigI->rot = rigI->rot - rigI->I_inv*impulse*(n.cross(rAP));
						rigJ->rot = rigJ->rot + rigJ->I_inv*impulse*(n.cross(rBP));

						rigI->L = rigI->I*rigI->rot;
						rigJ->L = rigJ->I*rigJ->rot;
					
					//}
					rigI->P = rigI->mass*rigI->vel;
					rigJ->P = rigJ->mass*rigJ->vel;
				}
			}
		}

		//cout << "Collision Num: " << collision_num << endl;
	}

	void checkSphereCollision(){
		
		
		dt = spheres[0]->getDT();
		for(int i=0;i<spheres.size();i++){
			Sphere *sI = spheres[i];
			Vector3 posI = sI->getPos();
			Vector3 velI = sI->getVelocity();
			Vector3 n_posI = posI+velI*dt;
			float rI = sI->getRadius();
			float im1 = 1./sI->getMass();
			float m1 = sI->getMass();
			Vector3 n_velI = Vector3();
			for(int j=0;j<spheres.size();j++){
				if(i==j)continue;
				Sphere *sJ = spheres[j];
				Vector3 posJ = sJ->getPos();	
				Vector3 velJ = sJ->getVelocity();
				Vector3 n_posJ = posJ+velJ*dt;
				float rJ = sJ->getRadius();
				float im2 = 1./sJ->getMass();
				float m2 = sJ->getMass();
				Vector3 n_velJ=Vector3();
				
				if(n_posI.distance(n_posJ)<=(rI+rJ)){
					//cout << "Sphere Collision!" << endl;
					//sphere collision!

					Vector3 n_difJI = (n_posI-n_posJ).normalize();
					Vector3 n_difIJ = (n_posJ-n_posI).normalize();

					Vector3 difJI = (posI-posJ).normalize();
					Vector3 difIJ = (posJ-posI).normalize();

					Vector3 impI = posI+difIJ*rI;//impI.z -=difIJ.z*2;
					Vector3 impJ = posJ-difIJ*rJ;//impJ.z +=difIJ.z*2;

					//find collision points
					sI->setImpactPoint(impI);
					sJ->setImpactPoint(impJ);
					/*if(){
						sI->setImpactPoint(n_posI-n_difIJ*rI);
						sJ->setImpactPoint(n_posJ-n_difJI*rJ);
					}
					else{
						sI->setImpactPoint(n_posI+n_difIJ*rI);
						sJ->setImpactPoint(n_posJ+n_difJI*rJ);
					}*/

					sI->setImpactTimeCounter(1.0);
					sJ->setImpactTimeCounter(1.0);

					//Way 1
					Vector3 delta = n_posI-n_posJ;
					float d = delta.length();
					Vector3 mtd = (delta)*((rI+rJ-d)/d);

					n_posI = n_posI+((mtd)*(im1/(im1+im2)));
					n_posJ = (n_posJ)-((mtd)*(im2/(im1+im2)));
					sI->setPos(n_posI);
					sJ->setPos(n_posJ);

					/*Vector3D *v = (*velI)-(*velJ);
					float vn = v->dot(mtd->normalize());
					if(vn>0)continue;

					float imp = (-(1-e)*vn)/(im1+im2);
					Vector3D *impulse = (*mtd->normalize())*imp;
					sI->setVelocity((*velI)+*((*impulse)*im1));
					sJ->setVelocity((*velJ)-*((*impulse)*im2));*/
					
					
					//Way 2
					n_velI = (((velI)*(m1-m2))+((velJ)*(2*m2)))/(m1+m2);
					n_velJ = (((velJ)*(m2-m1))+((velI)*(2*m1)))/(m1+m2);
					
					sI->setVelocity(n_velI);
					sJ->setVelocity(n_velJ);

					break;

				}
				

			}
			
		}
	}

	void checkSphereWallCollision(){

		dt = spheres[0]->getDT();
		for(int i=0;i<spheres.size();i++){
			Sphere *s = spheres[i];
			Vector3 pos = s->getPos();
			Vector3 vel = s->getVelocity();
			
			//Up Wall Collision
			if((pos.y+vel.y*dt)>=(20-s->getRadius())){
				

				vel.x = vel.x;
				vel.y = -vel.y;
				vel.z = vel.z;
				
			}

			//Down Wall Collision
			if((pos.y+vel.y*dt)<=(0+s->getRadius())){
				
				vel.x = vel.x;
				vel.y = -vel.y;
				vel.z = vel.z;
				
			}

			//Right Wall Collision
			if((pos.x+vel.x*dt)>=(20-s->getRadius())){
				
				vel.x = -vel.x;
				vel.y = vel.y;
				vel.z = vel.z;
				
			}

			//Left Wall Collision
			if((pos.x+vel.x*dt)<=(0+s->getRadius())){
				
				vel.x = -vel.x;
				vel.y = vel.y;
				vel.z = vel.z;
				
			}

			//Front Wall Collision
			if((pos.z+vel.z*dt)<=(s->getRadius())){
				
				vel.x = vel.x;
				vel.y = vel.y;
				vel.z = -vel.z;
				
			}

			//Back Wall Collision
			if((pos.z+vel.z*dt)>=(20-s->getRadius())){
				
				vel.x = vel.x;
				vel.y = vel.y;
				vel.z = -vel.z;
				
			}

			s->setVelocity(vel);

		}
	}

	void checkCubeWallCollision(){
	
		float dt = cubes[0]->getDT();
		for(int i=0;i<cubes.size();i++){
			Cube *c = cubes[i];
			Vector3 pos = c->getPos();
			Vector3 vel = c->getVelocity();

			//Up Wall Collision
			if((pos.y+vel.y*dt)>=(20-c->getRadius())){
				

				vel.x = vel.x;
				vel.y = -vel.y;
				vel.z = vel.z;
				
			}

			//Down Wall Collision
			if((pos.y+vel.y*dt)<=(0+c->getRadius())){
				
				vel.x = vel.x;
				vel.y = -vel.y;
				vel.z = vel.z;
				
			}

			//Right Wall Collision
			if((pos.x+vel.x*dt)>=(20-c->getRadius())){
				
				vel.x = -vel.x;
				vel.y = vel.y;
				vel.z = vel.z;
				
			}

			//Left Wall Collision
			if((pos.x+vel.x*dt)<=(0+c->getRadius())){
				
				vel.x = -vel.x;
				vel.y = vel.y;
				vel.z = vel.z;
				
			}

			//Front Wall Collision
			if((pos.z+vel.z*dt)<=(c->getRadius())){
				
				vel.x = vel.x;
				vel.y = vel.y;
				vel.z = -vel.z;
				
			}

			//Back Wall Collision
			if((pos.z+vel.z*dt)>=(20-c->getRadius())){
				
				vel.x = vel.x;
				vel.y = vel.y;
				vel.z = -vel.z;
				
			}

			c->setVelocity(vel);
			c->q = Quaternion(c->rot,1);
			c->L = c->I*c->rot;
			c->P = c->mass*c->vel;

		}
	}

	void checkCubeCollision(){
		dt = cubes[0]->getDT();
		for(int i=0;i<cubes.size();i++){
			Cube *cI = cubes[i];
			Vector3 posI = cI->getPos();
			Vector3 velI = cI->getVelocity();
			Vector3 n_posI = posI+velI*dt;
			float rI = cI->getRadius();
			float im1 = 1./cI->getMass();
			float m1 = cI->getMass();
			Vector3 n_velI = Vector3();
			vector<Triangle3D> trianglesI = cI->getTriangles();
			vector<Triangle3D> offsetI = cI->getOffset();

			for(int j=0;j<cubes.size();j++){
				if(i==j)continue;
				Cube *cJ = cubes[j];
				Vector3 posJ = cJ->getPos();	
				Vector3 velJ = cJ->getVelocity();
				Vector3 n_posJ = posJ+velJ*dt;
				float rJ = cJ->getRadius();
				float im2 = 1./cJ->getMass();
				float m2 = cJ->getMass();
				Vector3 n_velJ=Vector3();
				vector<Triangle3D> trianglesJ = cJ->getTriangles();
				vector<Triangle3D> offsetJ = cJ->getOffset();

				bool sphereCollision = n_posI.distance(n_posJ)<=(rI+rJ);
				if(sphereCollision==false)continue;
				
				bool cubeCollision = false;
				Vector3 impactPointI,nI;
				Vector3 impactPointJ,nJ;

				/*for(int k=0;k<trianglesI.size();k++){
					Triangle3D triI = offsetI[k];
					Vector3 vI1 = n_posI+triI.getV1().toVec3();
					Vector3 vI2 = n_posI+triI.getV2().toVec3();
					Vector3 vI3 = n_posI+triI.getV3().toVec3();
					Triangle3D ftriI = Triangle3D(vI1,vI2,vI3);

					for(int l=0;l<trianglesJ.size();l++){
						Triangle3D triJ = offsetJ[l];
						Vector3 vJ1 = n_posJ+triJ.getV1().toVec3();
						Vector3 vJ2 = n_posJ+triJ.getV2().toVec3();
						Vector3 vJ3 = n_posJ+triJ.getV3().toVec3();

						Triangle3D ftriJ = Triangle3D(vJ1,vJ2,vJ3);

						if(ftriI.intersects(ftriJ)){
							impactPointI = ftriI.getIntersectPoint().toVec3();
							nI = ftriI.normal().toVec3();
							nJ = ftriJ.normal().toVec3();
							cubeCollision=true;
						}

						if(ftriJ.intersects(ftriI)){
							impactPointJ = ftriJ.getIntersectPoint().toVec3();
							nI = ftriI.normal().toVec3();
							nJ = ftriJ.normal().toVec3();
							cubeCollision=true;
						}

						if(cubeCollision)break;
					}

					if(cubeCollision)break;
				}*/
				
				if(sphereCollision){
				//if(cubeCollision){
					//simplified cube collision
					//cout << "cube collision!" << endl;
					Vector3 n_difJI,n_difIJ;
					//n_difJI = nI;
					//n_difIJ = nJ;

					n_difJI = (posI-posJ).normalize();
					n_difIJ = (posJ-posI).normalize();
					impactPointI=posI+n_difIJ*rI;
					impactPointJ=posJ+n_difJI*rJ;
					
					
					Vector3 rAP = impactPointI-n_posI;
					Vector3 rBP = impactPointJ-n_posJ;

					Vector3 delta = n_posI-n_posJ;
					float d = delta.length();
					Vector3 mtd = (delta)*((rI+rJ-d)/d);

					n_posI = n_posI+((mtd)*(im1/(im1+im2)));
					n_posJ = (n_posJ)-((mtd)*(im2/(im1+im2)));
					cI->setPos(n_posI);
					cJ->setPos(n_posJ);
					/*n_posI = n_posI+((mtd)*(im1/(im1+im2)));
					n_posJ = (n_posJ)-((mtd)*(im2/(im1+im2)));
					cI->setPos(n_posI);
					cJ->setPos(n_posJ);*/
					

					//calculate impulse
					float impulseNom = (cJ->vel-cI->vel).dot(n_difIJ);//+(rAP.cross(n_difIJ)).dot(cI->rot)-(rBP.cross(n_difIJ)).dot(cJ->rot);
					//float impulseDen = im1+im2+(rAP.cross(n_difIJ)).dot(cI->I_inv*(rAP.cross(n_difIJ)))+(rBP.cross(n_difIJ)).dot(cJ->I_inv*(rBP.cross(n_difIJ)));
					float impulseDen = im1+im2+(cI->I_inv*(rAP.cross(n_difIJ)).cross(rAP)+cJ->I_inv*(rBP.cross(n_difIJ)).cross(rBP)).dot(n_difIJ);
					float impulseM = -2*impulseNom/impulseDen;
					Vector3 impulse = n_difIJ*impulseM;

					
					cI->vel = cI->vel-impulse*im1;
					cJ->vel = cJ->vel+impulse*im2;

					cI->rot = cI->rot - cI->I_inv*impulse*(n_difIJ.cross(rAP));
					cJ->rot = cJ->rot + cJ->I_inv*impulse*(n_difIJ.cross(rBP));

					/*if(cI->vel.length()>cJ->vel.length()){

						cI->vel = cI->vel-impulse*im1;
						cJ->vel = cJ->vel+impulse*im2;

						cI->rot = cI->rot - cI->I_inv*impulse*(n_difIJ.cross(rAP));
						cJ->rot = cJ->rot + cJ->I_inv*impulse*(n_difIJ.cross(rBP));
					}
					else{
						cI->vel = cI->vel+impulse*im1;
						cJ->vel = cJ->vel-impulse*im2;

						cI->rot = cI->rot + cI->I_inv*impulse*(n_difIJ.cross(rAP));
						cJ->rot = cJ->rot - cJ->I_inv*impulse*(n_difIJ.cross(rBP));
					}*/
					
					/*Quaternion w1_hat = Quaternion(cI->rot, 1);
					Quaternion w2_hat = Quaternion(cJ->rot, 1);
					Quaternion q1_dot = (w1_hat * cI->q) / 2;
					Quaternion q2_dot = (w2_hat * cJ->q) / 2;
					cI->q = q1_dot;
					cJ->q = q2_dot;*/
					//cI->q = Quaternion(cI->rot,1);
					//cJ->q = Quaternion(cJ->rot,1);

					

					cI->L = cI->I*cI->rot;
					cJ->L = cJ->I*cJ->rot;

					cI->P = cI->mass*cI->vel;
					cJ->P = cJ->mass*cJ->vel;
					//*/
				}

			}
		}
	}

	void calculateMinDist(int i,int type){
		float minDist=400;
		float dist;
		float rI,rJ;
		Vector3 minPoint;
		if(type==0){
			Vector3 posI = spheres[i]->getPos();
			rI = spheres[i]->getRadius();
			for(int j=0;j<spheres.size();j++){
					if(i==j)continue;
					Vector3 posJ = spheres[j]->getPos();
					rJ = spheres[j]->getRadius();
					dist = posI.distance(posJ)-(rI+rJ);
				
					if(dist<minDist){
						minDist=dist;
						//delete minPoint;
						minPoint = posJ;//new Vector3D(posJ->x,posJ->y,posJ->z);
					}
				}
				if(spheres.size()>1)spheres[i]->setMinDistPoint(minPoint);
				else spheres[i]->setMinDistPoint(posI);
		}
		else{
			Vector3 posI = cubes[i]->getPos();
			rI = cubes[i]->getRadius();
			for(int j=0;j<cubes.size();j++){
					if(i==j)continue;
					Vector3 posJ = cubes[j]->getPos();
					rJ = cubes[j]->getRadius();
					dist = posI.distance(posJ)-(rI+rJ);
				
					if(dist<minDist){
						minDist=dist;
						//delete minPoint;
						minPoint = posJ;//new Vector3D(posJ->x,posJ->y,posJ->z);
					}
				}
				if(cubes.size()>1)cubes[i]->setMinDistPoint(minPoint);
				else cubes[i]->setMinDistPoint(posI);
		}
	}

	void calculateMinDist(){
		float minDist;
		float dist;
		float rI,rJ;
		Vector3 minPoint;
		//delete minPoint;
		for(int i=0;i<spheres.size();i++){
			minDist = 400;
			Vector3 posI = spheres[i]->getPos();
			rI = spheres[i]->getRadius();
			for(int j=0;j<spheres.size();j++){
				if(i==j)continue;
				Vector3 posJ = spheres[j]->getPos();
				rJ = spheres[j]->getRadius();
				dist = posI.distance(posJ)-(rI+rJ);
				
				if(dist<minDist){
					minDist=dist;
					//delete minPoint;
					minPoint = posJ;//new Vector3D(posJ->x,posJ->y,posJ->z);
				}
			}
			if(spheres.size()>1)spheres[i]->setMinDistPoint(minPoint);
			else spheres[i]->setMinDistPoint(posI);
			//delete minPoint;
		}
	}


};