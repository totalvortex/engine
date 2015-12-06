#include <iostream>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include <GL/freeglut.h>
#define NPI 3.14159265358979323846

class llama{
public:
	float longitud;
	btVector4 posi;
	btVector4 dir;



	llama(btVector4 p, float l){
		posi=p;
		longitud=l;
		dir = btVector4(0,0,0,0);
	} 
	llama(btVector4 p, float l,btVector4 d){
		posi=p;
		longitud=l;
		dir=d;
	} 
public:
	float getlong(){
		return longitud;
	}

	btScalar getX(){
		return posi.getX();
	}
	btScalar getY(){
		return posi.getY();
	}
	btScalar getZ(){
		return posi.getZ();
	}
	btScalar getW(){
		return posi.getW();
	}
};