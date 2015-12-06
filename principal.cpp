#include <GL/freeglut.h>
#include <iostream>
#include <jpeglib.h>
#include <jerror.h>
#include <btBulletDynamicsCommon.h>
//#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <BulletCollision/btBulletCollisionCommon.h>//CollisionShapes/btHeightfieldTerrainShape.h>
//#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>


#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBody.h"

//#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
//#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
//#include "LinearMath/btQuickprof.h"
//#include "LinearMath/btIDebugDraw.h"
//#include "LinearMath/btConvexHull.h"

//#include <irrlicht/irrlicht.h>





#include <SOIL/SOIL.h>
#include <png.h>
#include <string>
#include <queue>
//#include <mutex>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <cstring>


#include "circuito.h"

#include <sstream>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <noise/include/noise.h>
#include "noiseutils.h"

#include <iomanip>
#include <list>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "obj.c"

#include <string.h>


//#include "dungeon.cpp"
//#include "machango.cpp"

GLuint texturaex[19];
#include "grass.cpp"
#include "palmera.cpp"
#include "torpedo.cpp"
#include "variables.h"
#define TRUE 1
#define FALSE 0

#define RENDER_WIDTH 800.0
#define RENDER_HEIGHT 800.0

#define NPI 3.14159265358979323846
#include "lodepng.h"


  #define CALLBACK(_id) callback##_id

//_fun##_id
  // break
//#define CALLBACKK(_fun,_id) _fun##_id()
 
//struct obj_model_t hojas[12];
//grass cesped(0.0,0.0,0.0);
//red
typedef boost::shared_ptr<boost::asio::ip::tcp::socket> socket_ptr;

void decodeTwoSteps(const char* filename)
{
  std::vector<unsigned char> png;
  std::vector<unsigned char> image; //the raw pixels
  unsigned width, height;

  //load and decode
  lodepng::load_file(png, filename);
  unsigned error = lodepng::decode(image, width, height, png);

  //if there's an error, display it
  if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;

  //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...
}

using namespace noise;

bool LoadJPEG(const char* FileName) {
	bool Fast = TRUE;
	FILE* file = fopen(FileName, "rb");  //open the file
	struct jpeg_decompress_struct info;  //the jpeg decompress info
	struct jpeg_error_mgr err;           //the error handler

	info.err = jpeg_std_error(&err); //tell the jpeg decompression handler to send the errors to err
	jpeg_create_decompress(&info);       //sets info to all the default stuff

	//if the jpeg file didnt load exit
	if (!file) {
		fprintf(stderr, "Error reading JPEG file %s!!!", FileName);
		// LoadBlackWhiteBorder();
		return false;
	}

	jpeg_stdio_src(&info, file);    //tell the jpeg lib the file we'er reading

	jpeg_read_header(&info, TRUE);   //tell it to start reading it

	//if it wants to be read fast or not
	if (Fast) {
		info.do_fancy_upsampling = FALSE;
	}

	jpeg_start_decompress(&info);    //decompress the file

	//set the x and y
	x = info.output_width;
	y = info.output_height;
	channels = info.num_components;

	type = GL_RGB;

	if (channels == 4) {
		type = GL_RGBA;
	}

	bpp = channels * 8;

	int Sizef = x * y * 3;

	//read turn the uncompressed data into something ogl can read
	data = new unsigned char[Sizef]; //setup data for the data its going to be handling

	unsigned char* p1 = data;
	unsigned char** p2 = &p1;
	int numlines = 0;

	while (info.output_scanline < info.output_height) {
		numlines = jpeg_read_scanlines(&info, p2, 1);
		*p2 += numlines * 3 * info.output_width;
	}

	jpeg_finish_decompress(&info);   //finish decompressing this file

	fclose(file);                    //close the file

	return true;
}



bool LoadTextureplano(const char * filename, int width, int height, GLuint *textura) {

//    GLuint texture;

	LoadJPEG(filename);

	glGenTextures(1, textura);
	glBindTexture( GL_TEXTURE_2D, *textura);
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1);
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

//    // when texture area is small, bilinear filter the closest mipmap
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
//                    GL_LINEAR_MIPMAP_NEAREST );
//    // when texture area is large, bilinear filter the original
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
//    
//    // the texture wraps over at the edges (repeat)
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
//    
//    //Generate the texture
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 2, 0,GL_RGB, GL_UNSIGNED_BYTE, data);

// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	GL_LINEAR_MIPMAP_NEAREST);
// when texture area is large, bilinear filter the first mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//    // the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height,
	GL_RGB, GL_UNSIGNED_BYTE, data);

	free(data);

	return TRUE; //return whether it was successful

}
int getRand(int min, int max)
    {
        time_t seed;
        seed = time(NULL) + oldseed;
        oldseed = seed;
 
        srand(seed);
 
        int n = max - min + 1;
        int i = rand() % n;
 
        if(i < 0)
            i = -i;
 
        return min + i;
    }

int piramide(int ancho, int x, int y) {
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	
	if (ancho > 0) {

		for (int i = 0; i < ancho; i++) {
			for (int j = 0; j < ancho; j++) {
				cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,1.0, 1.0));
				cuboMotionState[nocajas] = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i + desp+x, altura*2 + 1.01, j + desp+y)));
				cuboShapecoll[nocajas]->calculateLocalInertia(mass, fallInertia);
				btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
						cuboMotionState[nocajas], cuboShapecoll[nocajas], fallInertia);
				cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
				dynamicsWorld->addRigidBody(cuborb[nocajas]);
				texturascajas[nocajas]=rand()%4;
				nocajas++;
			}
		}
		altura++;
		desp = desp + .5;
		return piramide(ancho - 1,x,y);
	} else {

		return altura;

	}

}

void coloreamapa(){
	float max=0,min=0;
	for (int i = 0; i < ANCHOMAPA-1; i++){
    	for (int j = 0; j < ANCHOMAPA-1; j++){
    		if(mmapa[i][j]<min) min=mmapa[i][j];
    		if(mmapa[i][j]>max) max=mmapa[i][j];
    	}
    }


	for (int i = 0; i < ANCHOMAPA-1; i++){
    	for (int j = 0; j < ANCHOMAPA-1; j++){

    	
    	//  if(mmapa[i][j]<-1.0*ESCALA){
    	if(mmapa[i][j]<-4.0){
    		colores[i][j][0]= 0.0;
    		colores[i][j][1]= 0.0;
    		colores[i][j][2]=(float) 128/255;
    	  }else
    	//  if(mmapa[i][j]<-.55*ESCALA){
    	  if(mmapa[i][j]<-3.0){
    		colores[i][j][0]= 0.0;
    		colores[i][j][1]= 0.0;
    		colores[i][j][2]=(float)  200/255;
    	  }else
    	  //if(mmapa[i][j]<-.26*ESCALA){
    	  if(mmapa[i][j]<-1.5){
    		colores[i][j][0]= 0.0;
    		colores[i][j][1]= 0.0;
    		colores[i][j][2]=(float)  255/255;
    	  }else
    	 // if(mmapa[i][j]<.40*ESCALA){
    	  if(mmapa[i][j]<-0.0){
    		colores[i][j][0]=(float)  240/255;
    		colores[i][j][1]=(float)  240/255;
    		colores[i][j][2]=(float)  64/255;
    	  }else 
    	  if(mmapa[i][j]==0.1){
    		colores[i][j][0]= (float) 32/255;
    		colores[i][j][1]=(float)  32/255;
    		colores[i][j][2]=(float)  32/255;
    	  }else
    	  //if(mmapa[i][j]<2.0*ESCALA){
    	  
    	  if(mmapa[i][j]<1.0){
    	 // if(mmapa[i][j]<4.5*ESCALA){
    		colores[i][j][0]= (float) 190/255;
    		colores[i][j][1]=(float)  108/255;
    		colores[i][j][2]= (float) 50/255;
    	  }else
    	 // if(mmapa[i][j]<5.5*ESCALA){
    	  if(mmapa[i][j]<2.0){
    		colores[i][j][0]= (float) 140/255;
    		colores[i][j][1]=(float)  58/255;
    		colores[i][j][2]= (float) 10/255;
    	  }else
    	if(mmapa[i][j]<5.0){
    		colores[i][j][0]= (float) 32/255;
    		colores[i][j][1]=(float)  160/255;
    		colores[i][j][2]= 0.0;
    	  
    	  }else{
    	    colores[i][j][0]= (float)24/255;
    		colores[i][j][1]= (float)100/255;
    		colores[i][j][2]= (float)0/255;
    	  }
    	}
    }
}
void creacaja(float x, float y , float altura){
btScalar mass = 10;
	btVector3 fallInertiaa(0, 0, 0);

				cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,1.0, 1.0));
				cuboMotionState[nocajas] = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(x, altura + 1.01, y)));
				cuboShapecoll[nocajas]->calculateLocalInertia(mass, fallInertiaa);
				btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
						cuboMotionState[nocajas], cuboShapecoll[nocajas], fallInertiaa);
				cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
				dynamicsWorld->addRigidBody(cuborb[nocajas]);
				texturascajas[nocajas]=rand()%4;
				nocajas++;
}

void dibujacircuito()

{
	char *filename="circuito.png";

  std::vector<unsigned char> png;
  std::vector<unsigned char> image; //the raw pixels
  unsigned width, height;
  int nocaj=0;
  //load and decode
  lodepng::load_file(png, filename);
  unsigned error = lodepng::decode(image, width, height, png);

  //if there's an error, display it
  if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;

  //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...

 for (int i=4; i<sqrt(image.size())-4; i+=2 )  
 	for (int j=4; j<sqrt(image.size())-4; j+=2 )  {                                  
//int j=i%(int) sqrt(image.size());
 	if(image[i+j*sqrt(image.size())]!=image[0]) { mmapa[j/2][i/2]=-0.3;  
 //	if(image[i+j*sqrt(image.size())]==image[2]) {
 			//mmapa[j/2][i/2]=-0.3; 
			//creacaja(j/2, i/2 , .6);
 		nocaj++;
 		}
 	}                             
 	std::cout << "cajas: "<< nocaj << std::endl;
  //Convert BGR to RGB and store                      

//if(header_data[j+i*ANCHOMAPA]!=header_data[0])                             
               
 	png.clear();
 	image.clear();



}


void creaescenario(){
	printf("\nGenerando mapa.");


  module::Perlin modulo;

	modulo.SetFrequency (frecuencia);
	modulo.SetOctaveCount (oct);
    utils::NoiseMap heightMap;
    utils::NoiseMapBuilderPlane heightMapBuilder;
    heightMapBuilder.SetSourceModule (modulo);
    heightMapBuilder.SetDestNoiseMap (heightMap);
    heightMapBuilder.SetDestSize (ANCHOMAPA, ANCHOMAPA);
    heightMapBuilder.SetBounds (0.0, 1.0, 0.0, 1.0);
    heightMapBuilder.Build ();
    heightMapBuilder.Build ();
	
//zxcv

	
	for(int n=0;n<ANCHOMAPA;n++)
		for(int m=0;m<ANCHOMAPA;m++)
	//	if(n>3 && m >3 && n<ANCHOMAPA-3 && m<ANCHOMAPA-3 ){
			//if(n>ANCHOMAPA/2-anchod/2-2 && n<ANCHOMAPA/2+anchod/2+2 && m>ANCHOMAPA/2-altod/2-2&&m<ANCHOMAPA/2+altod/2+2)
		//	if (circuitobmp[n + m * ANCHOMAPA] == 0)
		//		 mmapa[n][m]=.75;
		//	else
			mmapa[n][m]=heightMap.GetValue(n,m)+.7;
			//mmapa[n][m]= 1.0;
	//	} 
	

	float valla=10.0;

	for(int n=0;n<ANCHOMAPA;n++){
		mmapa[n][0]=valla;
		mmapa[n][ANCHOMAPA-1]=valla;
	}
	for(int m=0;m<ANCHOMAPA;m++){
		mmapa[0][m]=valla;
		mmapa[ANCHOMAPA-1][m]=valla;
	}
		mmapa[0][0]=valla;
		mmapa[ANCHOMAPA][ANCHOMAPA]=valla;
		mmapa[0][ANCHOMAPA]=valla;
		mmapa[ANCHOMAPA][0]=valla;

	//dibujacircuito();

	
float max=0.0,min=0.0;
	for(int n=1;n<ANCHOMAPA-1;n++) //suaviza el mapa
		for(int m=1;m<ANCHOMAPA-1;m++){
		//if(n>1 && m >1 && n<ANCHOMAPA-1 && m<ANCHOMAPA-1 && !(n>ANCHOMAPA/2-anchod/2-2 && n<ANCHOMAPA/2+anchod/2+2 && m>ANCHOMAPA/2-altod/2-2&&m<ANCHOMAPA/2+altod/2+2) ){
			for(int z=0;z<2;z++)
			mmapa[n][m]=(mmapa[n][m]*suavizado+mmapa[n+1][m]+mmapa[n-1][m]+mmapa[n][m+1]+mmapa[n][m+1]+mmapa[n][m]+mmapa[n+1][m]+mmapa[n-1][m]+mmapa[n][m+1]+mmapa[n][m+1])/(4+suavizado);
		//}


		if(max<mmapa[n][m]) max=mmapa[n][m];
		if(min>mmapa[n][m]) min=mmapa[n][m];
      }
    coloreamapa();
	printf("\nMapa generado. Max: %f, Min: %f\n",max,min);





}


void initvehiculo(){
btTransform t;  //position and rotation
t.setIdentity();
//edit here (1/3):
t.setOrigin(btVector3(0, 0, 0));
	// The vehicle
btScalar chassisMass(1024.0f);
btVector3 chassisInertia(0.0f, 0.0f, 0.0f);
 chassisShape = new btBoxShape(btVector3(2.4f, 1.0f, 4.07f));
btCollisionShape* torretaShape = new btSphereShape(1.5f);
//tuboShape = new btCylinderShape(btVector3(1.0,4.0,1.0));
btQuaternion rotacion=btQuaternion(0, 1, 1,1 );
btQuaternion rotaciont=btQuaternion(0, 0, 1,1 );

rotacion.setRotation(btVector3(0.0f,1.0f,0.0f), NPI);
rotaciont.setRotation(btVector3(0.0f,0.0f,1.0f), NPI);	

tanqueshape = new btCompoundShape();

tanqueshape->addChildShape(t,chassisShape);
t.setOrigin(btVector3(0, 1, 0));
tanqueshape->addChildShape(t,torretaShape);
//t.setOrigin(btVector3(0, 2, 0));
//t.setRotation(btQuaternion(1, 0, 0, -1));
//tanqueshape->addChildShape(t,tuboShape);
			

btDefaultMotionState* chassisMotionState = new btDefaultMotionState(btTransform(rotacion, btVector3(
	
235.560562, 2.807614, 219.59
	)));

				/*	tuboMotionState = new btDefaultMotionState(btTransform(t));
					tuboShape->calculateLocalInertia(0, chassisInertia);
					btRigidBody::btRigidBodyConstructionInfo tuboRigidBodyCI( 0, 
							tuboMotionState, tuboShape);
					tuborb = new btRigidBody(tuboRigidBodyCI);
					dynamicsWorld->addRigidBody(tuborb);

*/







tanqueshape->calculateLocalInertia(chassisMass, chassisInertia);
btRigidBody::btRigidBodyConstructionInfo chassisRigidBodyCI(chassisMass, chassisMotionState, tanqueshape, chassisInertia);
chassisRigidBody = new btRigidBody(chassisRigidBodyCI);
chassisRigidBody->setActivationState(DISABLE_DEACTIVATION);
// Be sure to add the chassis of the vehicle into the world as a rigid body
dynamicsWorld->addRigidBody(chassisRigidBody);

//colisiones


btRaycastVehicle::btVehicleTuning tuning;
raycaster = new btDefaultVehicleRaycaster(dynamicsWorld);
vehicle = new btRaycastVehicle(tuning, chassisRigidBody, raycaster);
vehicle->setCoordinateSystem(0, 1, 2);


// Be sure to attach the wheels not higher than the upper bounds of the hull of the vehicle chassis


//vehicle->addWheel(btVector3(-1.75f, -1.1f, 4.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);
//vehicle->addWheel(btVector3(1.75f, -1.1f, 4.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(-2.0f, -.79f, 3.15f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(2.0f, -.79f, 3.15f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);

//vehicle->addWheel(btVector3(-2.0f, -.79f, 0.1f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
//vehicle->addWheel(btVector3(2.0f, -.79f,  0.1f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);

vehicle->addWheel(btVector3(-2.0f, -.79f, -3.15f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
vehicle->addWheel(btVector3(2.0f, -.79f, -3.15f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);

/*
vehicle->addWheel(btVector3(-2.0f, -1.1f, 2.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(2.0f, -1.1f, 2.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);

vehicle->addWheel(btVector3(-2.0f, -1.1f, 0.1f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
vehicle->addWheel(btVector3(2.0f,- 1.1f,  0.1f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);

vehicle->addWheel(btVector3(-2.0f, -1.1f, -2.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
vehicle->addWheel(btVector3(2.0f, -1.1f, -2.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
*/
for (int i=0;i<vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}

	
dynamicsWorld->addAction(vehicle);

}


/*
void dibujadungeon(Dungeon d){
	btVector3 fallInertia(0, 0, 0);
	//cuerpos rigidos
	btDefaultMotionState* fallMotionState;
	btDefaultMotionState* fallMotionState1;

	altura=1.5;
	int z=1;
	
	for (int i = 0; i < d.xsize; i++) {

					paredcol[nobloques] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 ,  z*2 +altura, 0 )));
					paredcol[nobloques]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, paredcol[nobloques]);
					paredrb[nobloques] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(paredrb[nobloques]);
				
					nobloques++;


					paredcol[nobloques] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 ,  z*2 +altura, (d.ysize-1)*2 )));
					paredcol[nobloques]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState, paredcol[nobloques]);
					paredrb[nobloques] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(paredrb[nobloques]);
				
					nobloques++;




	}	


	for (int j = 0; j < d.ysize; j++) {

					paredcol[nobloques] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(0 ,  z*2 +altura, j*2 )));
					paredcol[nobloques]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, paredcol[nobloques]);
					paredrb[nobloques] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(paredrb[nobloques]);
				
					nobloques++;


					paredcol[nobloques] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3((d.xsize-1)*2 ,  z*2 +altura, j*2 )));
					paredcol[nobloques]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState, paredcol[nobloques]);
					paredrb[nobloques] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(paredrb[nobloques]);
				
					nobloques++;




	}	
	
	d.showDungeon();
	for (int i = 1; i < d.xsize-1; i++) {
		for (int j = 1; j < d.ysize-1; j++) {





			int kk=d.traduce(i,j);
			
			switch(kk){
			case 0:{
				
					pisoShapecoll[nopiso] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 ,  altura, j*2 )));
					pisoShapecoll[nopiso]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, pisoShapecoll[nopiso]);
					pisorb[nopiso] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(pisorb[nopiso]);
				
					nopiso++;


				
				break;
			}
			case 1:{
				
					pisoShapecoll[nopiso] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 ,  altura, j*2 )));
					pisoShapecoll[nopiso]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, pisoShapecoll[nopiso]);
					pisorb[nopiso] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(pisorb[nopiso]);
				
					nopiso++;


				
				break;
			}
				
			case 2:{
				
					pisoShapecoll[nopiso] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 ,  altura, j*2 )));
					pisoShapecoll[nopiso]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, pisoShapecoll[nopiso]);
					pisorb[nopiso] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(pisorb[nopiso]);
				
					nopiso++;


				
				break;
			}
			case 6:{
				is = i;
				js = j; 
					


				
				break;
			}
			case 7:{
				ie = i;
				je = j; 
				break;
			}
				case 11:{
				es1Shapecoll[noes1] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es1Shapecoll[noes1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, es1Shapecoll[noes1]);
					es1rb[noes1] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(es1rb[noes1]);
				
					noes1++;
				break;
				}
				case 12:{
				cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cuboShapecoll[nocajas]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cuboShapecoll[nocajas]);
					cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cuborb[nocajas]);
				
					nocajas++;
				break;
				}
				case 13:{
					es2Shapecoll[noes2] = new btBoxShape(btVector3(1.0,0.1, 0.5));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es2Shapecoll[noes2]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, es2Shapecoll[noes2]);
					es2rb[noes2] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(es2rb[noes2]);
				
					noes2++;
				break;
				}
				case 14:{
					cubo1Shapecoll[nocajas1] = new btBoxShape(btVector3(0.5,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cubo1Shapecoll[nocajas1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cubo1Shapecoll[nocajas1]);
					cubo1rb[nocajas1] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cubo1rb[nocajas1]);
				
					nocajas1++;
				


					es1Shapecoll[noes1] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState1 = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es1Shapecoll[noes1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState1, es1Shapecoll[noes1]);
					es1rb[noes1] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(es1rb[noes1]);
				
					noes1++;
				
				break;
				}
				case 15:{
					cuboShapecoll[nocajas] = new btBoxShape(btVector3(0.5,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cuboShapecoll[nocajas]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cuboShapecoll[nocajas]);
					cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cuborb[nocajas]);
				
					nocajas++;
				


					es1Shapecoll[noes1] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState1 = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es1Shapecoll[noes1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState1, es1Shapecoll[noes1]);
					es1rb[noes1] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(es1rb[noes1]);
				
					noes1++;
				
				break;
				}
				case 16:{
					cubo1Shapecoll[nocajas1] = new btBoxShape(btVector3(0.5,2.0, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cubo1Shapecoll[nocajas1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cubo1Shapecoll[nocajas1]);
					cubo1rb[nocajas1] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cubo1rb[nocajas1]);
				
					nocajas1++;
					break;
				}
				case 17:{

					cubo2Shapecoll[nocruz] = new btBoxShape(btVector3(0.5,2.0, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cubo2Shapecoll[nocruz]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cubo2Shapecoll[nocruz]);
					cubo2rb[nocruz] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cubo2rb[nocruz]);
				
					nocruz++;
					break;

					cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState1 = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cuboShapecoll[nocajas]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState1, cuboShapecoll[nocajas]);
					cuborb[nocajas] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(cuborb[nocajas]);
				
					nocajas++;

					break;
				}
				case 19:{

					cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cuboShapecoll[nocajas]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cuboShapecoll[nocajas]);
					cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cuborb[nocajas]);
				
					nocajas++;

					es3Shapecoll[noes3] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState1 = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es3Shapecoll[noes3]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState1, es3Shapecoll[noes3]);
					es3rb[noes3] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(es3rb[noes3]);
				
					noes3++;

					break;
				}
				case 20:{
					cubo1Shapecoll[nocajas1] = new btBoxShape(btVector3(0.5,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					cubo1Shapecoll[nocajas1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, cubo1Shapecoll[nocajas1]);
					cubo1rb[nocajas1] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(cubo1rb[nocajas1]);
				
					nocajas1++;
					
					
					es2Shapecoll[noes2] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState1 = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es2Shapecoll[noes2]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBody1CI( 0, 
							fallMotionState1, es2Shapecoll[noes2]);
					es2rb[noes2] = new btRigidBody(fallRigidBody1CI);
					dynamicsWorld->addRigidBody(es2rb[noes2]);
				
					noes2++;

					break;
				}
				case 21:{
					es3Shapecoll[noes3] = new btBoxShape(btVector3(1.0,0.1, 0.5));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es3Shapecoll[noes3]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, es3Shapecoll[noes3]);
					es3rb[noes3] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(es3rb[noes3]);
				
					noes3++;
				break;
				}

				case 23:{
					
				es4Shapecoll[noes4] = new btBoxShape(btVector3(1.0,0.1, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					es4Shapecoll[noes4]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
							fallMotionState, es4Shapecoll[noes4]);
					es4rb[noes4] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(es4rb[noes4]);
				
					noes4++;
				break;
				}
				case 5:{
				if(d.traduce(i-1,j)>2 && d.traduce(i+1,j)>2) {
					
					puertas1Shapecol[nopuertas1] = new btBoxShape(btVector3(0.5,2.0, 1.0));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					puertas1Shapecol[nopuertas1]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
						fallMotionState, puertas1Shapecol[nopuertas1]);
					puertas1rb[nopuertas1] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(puertas1rb[nopuertas1]);
				Fini
					nopuertas1++;
				
				}else{
					puertasShapecol[nopuertas] = new btBoxShape(btVector3(1.0,2.0, 0.5));
					fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 , z*2 + altura, j*2 )));
					puertasShapecol[nopuertas]->calculateLocalInertia(0, fallInertia);
					btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI( 0, 
						fallMotionState, puertasShapecol[nopuertas]);
					puertasrb[nopuertas] = new btRigidBody(fallRigidBodyCI);
					dynamicsWorld->addRigidBody(puertasrb[nopuertas]);
				
					nopuertas++;
				}
				break;
			}
			default:
			break;

			}
		}
	}

}*/


void movertorreta(){


}





void movervehiculo()
{
		
		int wheelIndex = 4;
		vehicle->applyEngineForce(gEngineForce,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 5;
		vehicle->applyEngineForce(gEngineForce,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);		
	/*	wheelIndex = 4;
		vehicle->applyEngineForce(gEngineForce,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 5;
		vehicle->applyEngineForce(gEngineForce,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
*/

for (int z=0;z<2;z+=1){
	vehicle->applyEngineForce(gEngineForce,z);
	vehicle->setBrake(gBreakingForce,z);
	vehicle->setSteeringValue(gVehicleSteering,z);
	
}

/*
for (int z=2;z<4;z++){
	vehicle->applyEngineForce(gEngineForce,z);
	vehicle->setBrake(gBreakingForce,z);
	vehicle->setSteeringValue(gVehicleSteering/2,z);

}*/
/*
		int wheelIndex = 0;
		vehicle->applyEngineForce(gEngineForce,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 1;
		vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);

		wheelIndex = 2;
		vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 3;
		vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 4;
		vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 5;
		vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		vehicle->setBrake(gBreakingForce,wheelIndex);
		*/
}

void fin() {
	for (int j = 0; j < nobolas; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete fallRigidBody[j]->getMotionState();
		delete fallShapecoll[j];
		delete fallRigidBody[j];
	}
/*
	for (int j = 0; j < nocruz; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete cubo2rb[j]->getMotionState();
		delete cubo2Shapecoll[j];
		delete cubo2rb[j];
	}
	for (int j = 0; j < nobloques; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete paredrb[j]->getMotionState();
		delete paredcol[j];
		delete paredrb[j];
	}
	for (int j = 0; j < nopiso; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete pisorb[j]->getMotionState();
		delete pisoShapecoll[j];
		delete pisorb[j];
	}
	for (int j = 0; j < noes1; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete es1rb[j]->getMotionState();
		delete es1Shapecoll[j];
		delete es1rb[j];
	}
	for (int j = 0; j < noes2; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete es2rb[j]->getMotionState();
		delete es2Shapecoll[j];
		delete es2rb[j];
	}
	for (int j = 0; j < noes3; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete es3rb[j]->getMotionState();
		delete es3Shapecoll[j];
		delete es3rb[j];
	}
	for (int j = 0; j < noes4; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete es4rb[j]->getMotionState();
		delete es4Shapecoll[j];
		delete es4rb[j];
	}
	for (int j = 0; j < nocajas; j++) {
		dynamicsWorld->removeRigidBody(cuborb[j]);
		delete cuborb[j]->getMotionState();
		delete cuboShapecoll[j];
		delete puertasShapecol[j];
		delete cuborb[j];
	}
	for (int j = 0; j < nocajas1; j++) {
		dynamicsWorld->removeRigidBody(cuborb[j]);
		delete cubo1rb[j]->getMotionState();
		delete cubo1Shapecoll[j];
		delete cubo1rb[j];
	}
	for (int j = 0; j < nopuertas; j++) {
		dynamicsWorld->removeRigidBody(puertasrb[j]);
		delete puertasrb[j]->getMotionState();
		delete puertasrb[j];
	}
	for (int j = 0; j < nopuertas1; j++) {
		dynamicsWorld->removeRigidBody(puertas1rb[j]);
		delete puertas1rb[j]->getMotionState();
		delete puertas1rb[j];
	}*/
	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

		
	delete suelocoll;

	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

	FreeModel(&tile);
	FreeModel(&puerta);
	FreeModel(&muro);
	FreeModel(&puerta1);
	FreeModel(&muro1);
	FreeModel(&piso);
	FreeModel(&esq);
	FreeModel(&cruz);
	FreeModel(&bloque);
	for(int kk=0;kk<27;kk++) FreeModel(&modelo[kk]);
	FreeModel(&escenario);





}

void dibujabloque(){
		glBindTexture( GL_TEXTURE_2D, texturacubo); //bind the texture
								glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
								glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
								RenderOBJModel(&bloque);
}



void generacesped(GLfloat a, GLfloat b, GLfloat c){
	cesped.clear();
		 for(float i=0;i<ESCALA;i=i+1)
			for(float j=ESCALA-i;j>=0;j=j-1){
				float a=(rand()%10)/10-.5f;
				float b=0.0f;
				float c=(rand()%10)/10-.5f;
        		
        		if(cesped.size()==0) {
        			//it=cesped.begin();
        			grass ces(a+j, b, c+i);
        			cesped.push_back(ces);
        		}	
        		else {
        		    grass ces(a+j, b, c+i);
        		    cesped.push_back(ces);
        					
        					
        		}
        	
       
        		
        	}
        	
	//cesped.push_back(grass(784.0,-6.5, 872.0));
	//std::cout << "." ;
cespedi.clear();
		 for(float i=ESCALA-1;i>=0;i=i-1)
			for(float j=0;j<ESCALA;j=j+1){
				float a=(rand()%10)/10-.5f;
				float b=0.0f;
				float c=(rand()%10)/10-.5f;
        		
        		if(cespedi.size()==0) {
        			//it=cesped.begin();
        			grass ces(a+j, b, c+i);
        			cespedi.push_back(ces);
        		}	
        		else {
        		    grass ces(a+j, b, c+i);
        		    cespedi.push_back(ces);
        					
        					
        		}
        	
       
        		
        	}

}

void creaseta(float x, float y, float z){
/*
 btVector3* btVerts = new btVector3[setaobj.num_verts];
for(int i=0;i<setaobj.num_verts;++i)
    btVerts[i] = (btVector3(setaobj.vertices[i].xyzw[0]+x,setaobj.vertices[i].xyzw[1]+y,setaobj.vertices[i].xyzw[2]+z));
const int iCount = setaobj.num_faces;
const int tCount = iCount/3;
int* btInd = new int[iCount];
for(int i=0;i<iCount;++i){
    btInd[i] = (int) setaobj.faces[i].vert_indices[0];
}

	setasoftBody = btSoftBodyHelpers::CreateFromTriMesh(
        ((btSoftRigidDynamicsWorld*)dynamicsWorld)->getWorldInfo()
        ,(btScalar*)btVerts
        ,&btInd[0],tCount
        ,false
    );
dynamicsWorld->addSoftBody(setasoftBody);*/
}
/*
void initsombra(){


 glGenFramebuffers(1, &FramebufferName);
 glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
 
 // Depth texture. Slower than a depth buffer, but you can sample it later in your shader
 glGenTextures(1, &depthTexture);
 glBindTexture(GL_TEXTURE_2D, depthTexture);
 glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT16, 1024, 1024, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
 
 glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);
 
 glDrawBuffer(GL_NONE); // No color buffer is drawn to.
 
 // Always check that our framebuffer is ok
 if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
 std::cout << "chungo" << std::endl;



 glm::vec3 lightInvDir = glm::vec3(313.0971f, 131.096f, 144.58f);
 
 // Compute the MVP matrix from the light's point of view
 glm::mat4 depthProjectionMatrix = glm::ortho<float>(-500,500,-500,500,-500,1000);
 glm::mat4 depthViewMatrix = glm::lookAt(lightInvDir, glm::vec3(0,0,0), glm::vec3(0,1,0));
 glm::mat4 depthModelMatrix = glm::mat4(1.0);
 glm::mat4 depthMVP = depthProjectionMatrix * depthViewMatrix * depthModelMatrix;
 
 // Send our transformation to the currently bound shader,
 // in the "MVP" uniform
 glUniformMatrix4fv(depthMatrixID, 1, GL_FALSE, &depthMVP[0][0])












}*/











void inicio() {

	glEnable(GL_TEXTURE_2D);

	/* //Setup cube vertex data. 
	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -.5;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = .5;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -.5;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = .5;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = .5;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -.5;*/

//asdf
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	bzero(activob,NUMERO);
	//suelocoll = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	//groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
/*
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState, suelocoll, btVector3(0, 0, 0));
	groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);
*/

	creaescenario();
/*
	  heightFieldShape = new btHeightfieldTerrainShape(128, 128, mmapa, 1.0, 0, ANCHOMAPA, 1, PHY_FLOAT, true);
      heightFieldShape->setUseDiamondSubdivision(true);
      heightFieldShape->setLocalScaling(btVector3(0,0,0));
      escenarioRigidBody = new btRigidBody(0, 0, heightFieldShape, btVector3(0, 0, 0));   
      escenarioRigidBody->setFriction(0.8);
      escenarioRigidBody->setHitFraction(0.8);
      escenarioRigidBody->setRestitution(0.6);
    
      escenarioRigidBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
      escenarioRigidBody->setActivationState(DISABLE_DEACTIVATION);
      dynamicsWorld->addRigidBody(escenarioRigidBody);
*/
   btTriangleMesh* gTriangleMesh = new btTriangleMesh();

   /*
for (int j = 0; j < escenario.num_faces; j++){
   			   int index0 = escenario.faces[j].vert_indices[0];
               int index1 = escenario.faces[j].vert_indices[1];
               int index2 = escenario.faces[j].vert_indices[2];



		btVector3 vertex0(escenario.vertices[index0].xyzw[0],escenario.vertices[index0].xyzw[1],escenario.vertices[index0].xyzw[2]);
         btVector3 vertex1(escenario.vertices[index1].xyzw[0],escenario.vertices[index1].xyzw[1],escenario.vertices[index1].xyzw[2]);
         btVector3 vertex2(escenario.vertices[index2].xyzw[0],escenario.vertices[index2].xyzw[1],escenario.vertices[index2].xyzw[2]);
         
         
         
         
         gTriangleMesh->addTriangle(vertex0,vertex1,vertex2);
}*/
         /*
p0 = Point3(-10, -10, 0)
p1 = Point3(-10, 10, 0)
p2 = Point3(10, -10, 0)
p3 = Point3(10, 10, 0)
mesh = BulletTriangleMesh()
mesh.addTriangle(p0, p1, p2)
mesh.addTriangle(p1, p2, p3)
shape = BulletTriangleMeshShape(mesh, dynamic=False)
*/

					
				int inicioi=0;
				int inicioj=0;
				int fini=CAMPOVISION*2;
				int finj=CAMPOVISION*2;

				if((int)(eyePos[X]/ESCALA-CAMPOVISION)>0) inicioi= (int)(eyePos[X]/ESCALA-CAMPOVISION);
				if((int)(eyePos[Z]/ESCALA-CAMPOVISION)>0) inicioj= (int)(eyePos[Z]/ESCALA-CAMPOVISION);

				if((int)(eyePos[X]/ESCALA+CAMPOVISION)<ANCHOMAPA) fini= (int)(eyePos[X]/ESCALA+CAMPOVISION);
				if((int)(eyePos[Z]/ESCALA+CAMPOVISION)<ANCHOMAPA) finj= (int)(eyePos[Z]/ESCALA+CAMPOVISION);
		


    for (int i = inicioi; i < fini; i++)
    	for (int j = inicioj; j < finj; j++){


			btVector3 v0((GLfloat)     i*ESCALA, (GLfloat) mmapa[i][j]     *ESCALA, (GLfloat)      j*ESCALA ); //i,j


			btVector3 v1((GLfloat) (i+1)*ESCALA, (GLfloat) mmapa[i+1][j]   *ESCALA, (GLfloat)      j*ESCALA );//i+1,j

		
			btVector3 v2((GLfloat)   (i)*ESCALA, (GLfloat) mmapa[i][j+1]   *ESCALA, (GLfloat) (j+1) *ESCALA );//i,j+1
            btVector3 v3((GLfloat) (i+1)*ESCALA, (GLfloat) mmapa[i+1][j+1] *ESCALA, (GLfloat) (j+1) *ESCALA );//i+1,j+1    
                        
                        
			gTriangleMesh->addTriangle(v0,v2,v3,false);
 			gTriangleMesh->addTriangle(v0,v1,v3,false);


			
			//printf(".");
		}



    
printf("\n\nMapa cargado...");
    gTriMeshShape = new btBvhTriangleMeshShape(gTriangleMesh,false);
    escenarioMotionState =   new btDefaultMotionState(btTransform( btQuaternion(0,0,0,1),btVector3(0,0,0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyescenarioCI(0,escenarioMotionState,gTriMeshShape);
    
    escenarioRigidBody = new btRigidBody(groundRigidBodyescenarioCI);
    dynamicsWorld->addRigidBody(escenarioRigidBody);




 
	 




initvehiculo();

altura=11.5f;
//piramide(6,597,570);
//dynamicsWorld->contactTest(escenarioRigidBody, callbackj  );
//dynamicsWorld->contactTest(chassisRigidBody, callbackj);

piramide(4,277,279);

creaseta(277.017395, 10.073755, 279.540);


/*

for(int x=10;x<ANCHOMAPA-10;x+=5){
	for(int y=10;y<ANCHOMAPA-10;y+=5){
		float desplazamiento=0;
			int j=nobolas;
				 btVector3 fallInertia=btVector3(0,0,0);
				 fallShapecoll[j] = new btSphereShape(anchobola);
				 fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3((x+desplazamiento)*ESCALA, (ANCHOMAPA/2)*ESCALA,( y+desplazamiento)*ESCALA)));
				 fallShapecoll[j]->calculateLocalInertia(5.0, fallInertia);
				 btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(2.0,fallMotionState, fallShapecoll[j], fallInertia);
				 fallRigidBody[j] = new btRigidBody(fallRigidBodyCI);
				 dynamicsWorld->addRigidBody(fallRigidBody[j]);
				 nobolas++;
	}
}*/













	//seleccionar masa e inercia
	btScalar mass = 75;
	

   // LoadTextureplano("Hex_Plating.jpg", 2048, 2048,&texturasuelo); //load the texture

	
	LoadTextureplano("crate difuse.jpg", 894, 894,&texturecaja[0]); //load the texture
	LoadTextureplano("box-wood-3-diffuse.jpg", 512, 512,&texturecaja[1]); //load the texture
	LoadTextureplano("crate.jpg", 512, 512,&texturecaja[2]); //load the texture
	LoadTextureplano("crate_1.jpg", 512, 512,&texturecaja[3]); //load the texture

	
	anchotextura=64;
	altotextura=32;
	bool alpha=true;
		
	texturasuelo = SOIL_load_OGL_texture("terrain1.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturaventana = SOIL_load_OGL_texture("edificio.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturacolumna = SOIL_load_OGL_texture("texturacolumna.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturapuerta = SOIL_load_OGL_texture("texturapuerta.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturamuro = SOIL_load_OGL_texture("texturamuro.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturapiso = SOIL_load_OGL_texture("piso.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturaesquina = SOIL_load_OGL_texture("texturaesquina.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturacruz = SOIL_load_OGL_texture("texturacruz.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturabloque = SOIL_load_OGL_texture("texturabloque.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturamodelo = SOIL_load_OGL_texture("./modelos/orco/texturaorco.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//texturaaracnido = SOIL_load_OGL_texture("./modelos/aracnido/texturaaracnido.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaagua = SOIL_load_OGL_texture("agua.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaskysphere = SOIL_load_OGL_texture("skysphere.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturatanque = SOIL_load_OGL_texture("modelos/cutretanque/camo.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturarueda = SOIL_load_OGL_texture("modelos/vehiculo/texturaruedavehiculo.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturapalmera = SOIL_load_OGL_texture("imagenes/palmera.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturacubo = SOIL_load_OGL_texture("imagenes/estela/estela0.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturacircuito = SOIL_load_OGL_texture("imagenes/circuito.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	

	texturaex[0] = SOIL_load_OGL_texture("imagenes/onda.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[1] = SOIL_load_OGL_texture("imagenes/estela1/explosion1.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[2] = SOIL_load_OGL_texture("imagenes/estela1/explosion2.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[3] = SOIL_load_OGL_texture("imagenes/estela1/explosion3.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[4] = SOIL_load_OGL_texture("imagenes/estela1/explosion4.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[5] = SOIL_load_OGL_texture("imagenes/estela1/explosion5.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[6] = SOIL_load_OGL_texture("imagenes/estela1/explosion6.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[7] = SOIL_load_OGL_texture("imagenes/estela1/explosion7.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[8] = SOIL_load_OGL_texture("imagenes/estela1/explosion8.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[9] = SOIL_load_OGL_texture("imagenes/estela1/explosion9.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[10] = SOIL_load_OGL_texture("imagenes/estela1/explosion10.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[11] = SOIL_load_OGL_texture("imagenes/estela1/explosion11.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[12] = SOIL_load_OGL_texture("imagenes/estela1/explosion12.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[13] = SOIL_load_OGL_texture("imagenes/estela1/explosion13.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[14] = SOIL_load_OGL_texture("imagenes/estela1/explosion14.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[15] = SOIL_load_OGL_texture("imagenes/estela1/explosion15.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[16] = SOIL_load_OGL_texture("imagenes/estela1/explosion16.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[17] = SOIL_load_OGL_texture("imagenes/estela1/explosion17.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[18] = SOIL_load_OGL_texture("imagenes/estela1/explosion18.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaex[19] = SOIL_load_OGL_texture("imagenes/estela1/explosion19.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	circuito = SOIL_load_OGL_texture("circuito.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	//circuitobmp = SOIL_load_image("circuito.bmp", &anchoc, &anchoc, 0, SOIL_LOAD_RGB);

   texturahoja = SOIL_load_OGL_texture("imagenes/espigatronco/espiga.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturatronco = SOIL_load_OGL_texture("imagenes/espigatronco/tronco.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	
	ReadOBJModel ("modelos/explosion/explo.obj", &exploobj);
	ReadOBJModel ("modelos/palmera/tronco.obj", &tronco);
	ReadOBJModel ("modelos/palmera/pina.obj", &pina);
	ReadOBJModel ("modelos/palmera/cabeza.obj", &cabeza);
 	ReadOBJModel ("modelos/seta/seta.obj", &setaobj);

//	ReadOBJModel ("mapa/piso.obj", &piso);
//	ReadOBJModel ("mapa/muro.obj", &muro);
//	ReadOBJModel ("mapa/esquina.obj", &esq);
//	ReadOBJModel ("mapa/cruz.obj", &cruz);
	//ReadOBJModel ("modelos/cubo/cubo.obj", &bloque);
	ReadOBJModel ("modelos/cutretanque/cutre1.obj", &tanqueobj);
	//ReadOBJModel ("modelos/vehiculo/torreta.obj", &tanquetobj);
	//ReadOBJModel ("modelos/tanque/tanquetorreta.obj", &tanquetobj);
	ReadOBJModel ("modelos/skysphere/skysphere.obj", &skysphere);
	ReadOBJModel ("modelos/vehiculo/rueda.obj", &ruedaobj);
//	ReadOBJModel ("modelos/quad/quad.obj", &quadobj);

/*	ReadOBJModel ("modelos/cesped/cesped0.obj", &hojas[0]);
	ReadOBJModel ("modelos/cesped/cesped1.obj", &hojas[1]);
	ReadOBJModel ("modelos/cesped/cesped2.obj", &hojas[2]);
	ReadOBJModel ("modelos/cesped/cesped3.obj", &hojas[3]);
	ReadOBJModel ("modelos/cesped/cesped4.obj", &hojas[4]);
	ReadOBJModel ("modelos/cesped/cesped5.obj", &hojas[5]);
	ReadOBJModel ("modelos/cesped/cesped6.obj", &hojas[6]);
	ReadOBJModel ("modelos/cesped/cesped7.obj", &hojas[7]);
	ReadOBJModel ("modelos/cesped/cesped8.obj", &hojas[8]);
	ReadOBJModel ("modelos/cesped/cesped9.obj", &hojas[9]);
	ReadOBJModel ("modelos/cesped/cesped10.obj", &hojas[10]);
	ReadOBJModel ("modelos/cesped/cesped11.obj", &hojas[11]);*/
	//ReadOBJModel ("modelos/cesped/cesped13.obj", &hojas[12]);
/*	
	ReadOBJModel ("modelos/modelo/modelo_000001.obj", &modelo[0]);
	ReadOBJModel ("modelos/modelo/modelo_000002.obj", &modelo[1]);
	ReadOBJModel ("modelos/modelo/modelo_000003.obj", &modelo[2]);
	ReadOBJModel ("modelos/modelo/modelo_000004.obj", &modelo[3]);
	ReadOBJModel ("modelos/modelo/modelo_000005.obj", &modelo[4]);
	ReadOBJModel ("modelos/modelo/modelo_000006.obj", &modelo[5]);
	ReadOBJModel ("modelos/modelo/modelo_000007.obj", &modelo[6]);
	ReadOBJModel ("modelos/modelo/modelo_000008.obj", &modelo[7]);
	ReadOBJModel ("modelos/modelo/modelo_000009.obj", &modelo[8]);
	ReadOBJModel ("modelos/modelo/modelo_000010.obj", &modelo[9]);
	ReadOBJModel ("modelos/modelo/modelo_000011.obj", &modelo[10]);
	ReadOBJModel ("modelos/modelo/modelo_000012.obj", &modelo[11]);
	ReadOBJModel ("modelos/modelo/modelo_000013.obj", &modelo[12]);
	ReadOBJModel ("modelos/modelo/modelo_000014.obj", &modelo[13]);
	ReadOBJModel ("modelos/modelo/modelo_000015.obj", &modelo[14]);
	ReadOBJModel ("modelos/modelo/modelo_000016.obj", &modelo[15]);
	ReadOBJModel ("modelos/modelo/modelo_000017.obj", &modelo[16]);
	ReadOBJModel ("modelos/modelo/modelo_000018.obj", &modelo[17]);
	ReadOBJModel ("modelos/modelo/modelo_000019.obj", &modelo[18]);
	ReadOBJModel ("modelos/modelo/modelo_000020.obj", &modelo[19]);
	ReadOBJModel ("modelos/modelo/modelo_000021.obj", &modelo[20]);
	ReadOBJModel ("modelos/modelo/modelo_000022.obj", &modelo[21]);
	ReadOBJModel ("modelos/modelo/modelo_000023.obj", &modelo[22]);
	ReadOBJModel ("modelos/modelo/modelo_000024.obj", &modelo[23]);
	ReadOBJModel ("modelos/modelo/modelo_000025.obj", &modelo[24]);
	ReadOBJModel ("modelos/modelo/modelo_000026.obj", &modelo[25]);
	ReadOBJModel ("modelos/modelo/modelo_000027.obj", &modelo[26]);

	ReadOBJModel ("modelos/orco/ORCO_000001.obj", &modelo[0]);
	ReadOBJModel ("modelos/orco/ORCO_000002.obj", &modelo[1]);
	ReadOBJModel ("modelos/orco/ORCO_000003.obj", &modelo[2]);
	ReadOBJModel ("modelos/orco/ORCO_000004.obj", &modelo[3]);
	ReadOBJModel ("modelos/orco/ORCO_000005.obj", &modelo[4]);
	ReadOBJModel ("modelos/orco/ORCO_000006.obj", &modelo[5]);
	ReadOBJModel ("modelos/orco/ORCO_000007.obj", &modelo[6]);
	ReadOBJModel ("modelos/orco/ORCO_000008.obj", &modelo[7]);
	ReadOBJModel ("modelos/orco/ORCO_000009.obj", &modelo[8]);
	ReadOBJModel ("modelos/orco/ORCO_000010.obj", &modelo[9]);
	ReadOBJModel ("modelos/orco/ORCO_000011.obj", &modelo[10]);
	ReadOBJModel ("modelos/orco/ORCO_000012.obj", &modelo[11]);
	ReadOBJModel ("modelos/orco/ORCO_000013.obj", &modelo[12]);
	ReadOBJModel ("modelos/orco/ORCO_000014.obj", &modelo[13]);
	ReadOBJModel ("modelos/orco/ORCO_000015.obj", &modelo[14]);
	ReadOBJModel ("modelos/orco/ORCO_000016.obj", &modelo[15]);
	ReadOBJModel ("modelos/orco/ORCO_000017.obj", &modelo[16]);
	ReadOBJModel ("modelos/orco/ORCO_000018.obj", &modelo[17]);
	ReadOBJModel ("modelos/orco/ORCO_000019.obj", &modelo[18]);
	//manchango(2.0,2.0,.1,modelo,27);
	*/

/*	ReadOBJModel ("modelos/aracnido/aracnido_000003.obj", &modelo[0]);
	ReadOBJModel ("modelos/aracnido/aracnido_000004.obj", &modelo[1]);
	ReadOBJModel ("modelos/aracnido/aracnido_000005.obj", &modelo[2]);
	ReadOBJModel ("modelos/aracnido/aracnido_000006.obj", &modelo[3]);
	ReadOBJModel ("modelos/aracnido/aracnido_000007.obj", &modelo[4]);
	ReadOBJModel ("modelos/aracnido/aracnido_000008.obj", &modelo[5]);
	ReadOBJModel ("modelos/aracnido/aracnido_000009.obj", &modelo[6]);

	ReadOBJModel ("modelos/aracnido/aracnido_000010.obj", &modelo[7]);
	ReadOBJModel ("modelos/aracnido/aracnido_000011.obj", &modelo[8]);
	ReadOBJModel ("modelos/aracnido/aracnido_000012.obj", &modelo[9]);
	ReadOBJModel ("modelos/aracnido/aracnido_000013.obj", &modelo[10]);
	ReadOBJModel ("modelos/aracnido/aracnido_000014.obj", &modelo[11]);
	ReadOBJModel ("modelos/aracnido/aracnido_000015.obj", &modelo[12]);
	ReadOBJModel ("modelos/aracnido/aracnido_000016.obj", &modelo[13]);
	ReadOBJModel ("modelos/aracnido/aracnido_000017.obj", &modelo[14]);
	ReadOBJModel ("modelos/aracnido/aracnido_000018.obj", &modelo[15]);
	ReadOBJModel ("modelos/aracnido/aracnido_000019.obj", &modelo[16]);

	ReadOBJModel ("modelos/aracnido/aracnido_000020.obj", &modelo[17]);
	ReadOBJModel ("modelos/aracnido/aracnido_000021.obj", &modelo[18]);
	ReadOBJModel ("modelos/aracnido/aracnido_000022.obj", &modelo[19]);
	ReadOBJModel ("modelos/aracnido/aracnido_000023.obj", &modelo[20]);
	ReadOBJModel ("modelos/aracnido/aracnido_000024.obj", &modelo[21]);
	ReadOBJModel ("modelos/aracnido/aracnido_000025.obj", &modelo[22]);
	ReadOBJModel ("modelos/aracnido/aracnido_000026.obj", &modelo[23]);
	ReadOBJModel ("modelos/aracnido/aracnido_000027.obj", &modelo[24]);
	ReadOBJModel ("modelos/aracnido/aracnido_000028.obj", &modelo[25]);*/


//ReadOBJModel ("mapa/muro1.obj", &muro1);
	/*ReadOBJModel ("./mapa/pieza1.obj", &tiles[0]);
	ReadOBJModel ("./mapa/pieza2.obj", &tiles[1]);
	ReadOBJModel ("./mapa/pieza3.obj", &tiles[2]);
	ReadOBJModel ("./mapa/pieza4.obj", &tiles[3]);
	ReadOBJModel ("./mapa/pieza5.obj", &tiles[4]);
	ReadOBJModel ("./mapa/pieza6.obj", &tiles[5]);
	ReadOBJModel ("./mapa/pieza7.obj", &tiles[6]);
	ReadOBJModel ("./mapa/pieza8.obj", &tiles[7]);
	ReadOBJModel ("./mapa/pieza9.obj", &tiles[8]);
	ReadOBJModel ("./mapa/pieza10.obj", &tiles[9]);
	ReadOBJModel ("./mapa/pieza11.obj", &tiles[10]);
	ReadOBJModel ("./mapa/pieza12.obj", &tiles[11]);
	ReadOBJModel ("./mapa/pieza13.obj", &tiles[12]);
	ReadOBJModel ("./mapa/pieza14.obj", &tiles[13]);
	ReadOBJModel ("./mapa/pieza15.obj", &tiles[14]);*/


//printf("\nStart[%d,%d], \n",is,js);
//printf("\nEnd[%d,%d], \n",ie,je);

//
std::cout << "Creando cesped." << std::endl;
	//for(int h=0;h<10;h++){
 	//generacesped(417.5, -4.147475, 661.0);
/*
float a=0.0f;
float b=0.0f;
float c=0.0f;

 cesped.clear();
		 for(float i=0;i<ESCALA;i=i+1)
			for(float j=ESCALA-i;j>=0;j=j-1){
				float a=(rand()%10)/10-.5f;
				float b=0.0f;
				float c=(rand()%10)/10-.5f;
        		
        		if(cesped.size()==0) {
        			//it=cesped.begin();
        			grass ces(a+j, b, c+i);
        			cesped.push_back(ces);
        		}	
        		else {
        		    grass ces(a+j, b, c+i);
        		    cesped.push_back(ces);
        					
        					
        		}
        	
       
        		
        	}
        	
	//cesped.push_back(grass(784.0,-6.5, 872.0));
	//std::cout << "." ;
cespedi.clear();
		 for(float i=ESCALA-1;i>=0;i=i-1)
			for(float j=0;j<ESCALA;j=j+1){
				float a=(rand()%10)/10-.5f;
				float b=0.0f;
				float c=(rand()%10)/10-.5f;
        		
        		if(cespedi.size()==0) {
        			//it=cesped.begin();
        			grass ces(a+j, b, c+i);
        			cespedi.push_back(ces);
        		}	
        		else {
        		    grass ces(a+j, b, c+i);
        		    cespedi.push_back(ces);
        					
        					
        		}
        	
       
        		
        	}
u
*/
/*
for(int u=5;u<ANCHOMAPA-5;u=u+3){
	for(int o=5;o<ANCHOMAPA-5;o=o+3){
		if(mmapa[u][o]>3.5){
			palmera pa((float)u, (mmapa[u][o]-3.0),(float) o);
		
			arbol.push_back(pa);
		}
	}
}
*/

std::cout << "Cesped creado. Palmeras: " << arbol.size() << std::endl;

}



void initluces() {
//luces
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHT1);
	//glMatrixMode(GL_MODELVIEW);
	//luces
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);
/*
	glLightfv(GL_LIGHT1, GL_POSITION, lightSPos);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spotDir);
	glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, exponent);
	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, cutoff);
	*/
}

void puntodevista() {

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if(!freelook){
		tp.setIdentity();
		fallRigidBody[nobolas-1]->getMotionState()->getWorldTransform(tp);
		eyePos[0]=tp.getOrigin().getX()-vectorvision[X];
		eyePos[1]=tp.getOrigin().getY()-vectorvision[Y]+.75;
		eyePos[2]=tp.getOrigin().getZ()-vectorvision[Z];
	}
	/*	btQuaternion vector=vt.getRotation();
		btQuaternion vectorn=vector.normalized();
		btVector3 mu=btVector3(0.0,0.0,1.0);
		mu=vt*mu;
		vectorvision[X]=vectorn[X];
		vectorvision[Y]=vectorn[Y];
		vectorvision[Z]=vectorn[Z];*/
	
	if(vistavala){
	//	tp.setIdentity();
	//	GLfloat matrix[16];
		//vehicle->getChassisWorldTransform().getOrigin().getX();
		//chassisRigidBody->getMotionState()->getWorldTransform(tp);
		float distan=2.1;

		eyePos[0]=vehicle->getChassisWorldTransform().getOrigin().getX();//+cos(vehicle->getChassisWorldTransform().getRotation().getZ());
		eyePos[1]=vehicle->getChassisWorldTransform().getOrigin().getY()+2.25;
		eyePos[2]=vehicle->getChassisWorldTransform().getOrigin().getZ();


  // get chassis and turret transforms
  // btTransform chassisTransform = vehicle->getChassisWorldTransform();

 // vectorvision[0] = chassisTransform.getRotation().normalized().getAxis().getX();
 // vectorvision[1] = chassisTransform.getRotation().normalized().getAxis().getZ();
 //eeeeeeeeeee vectorvision[2] = chassisTransform.getRotation().normalized().getAxis().getY();
  
	
		//alfatorreta=tp.getRotation().getZ();
	}
		//puntodevista();
	// Set the camera
	gluLookAt( eyePos[X], eyePos[Y], eyePos[Z],
	           eyePos[X]+vectorvision[X], eyePos[Y]+vectorvision[Y], eyePos[Z]+vectorvision[Z],
	           0.0, 1.0, 0.0);
}


static void text(GLuint x, GLuint y, GLfloat scale, char* format, ...)
{
  va_list args;
  char buffer[255], *p;
  GLfloat font_scale = 119.05 + 33.33;

  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT));

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glTranslatef(x, y, 0.0);

  glScalef(scale/font_scale, scale/font_scale, scale/font_scale);

  for(p = buffer; *p; p++)
    glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
  
  glPopAttrib();

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

void drawBox(int i) {
	glColor3f(1.0,1.0,1.0);
	glBindTexture( GL_TEXTURE_2D, texturecaja[texturascajas[i]]); //bind the texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	GL_LINEAR_MIPMAP_NEAREST);
	glBegin(GL_QUADS);
	// Front Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Left Of The Texture and Quad
	// Back Face
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Left Of The Texture and Quad
	// Top Face
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Texture and Quad
	// Bottom Face
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	// Right face
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	// Left Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glEnd();

}







void dibujamapa(){

				glBindTexture(GL_TEXTURE_2D, texturasuelo);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			//glScalef(.2,1.0,.2);
			//glTranslatef(-ANCHOMAPA/2,0.0,-ANCHOMAPA/2);
			// for (int i = ((int)(eyePos[X]-ANCHOMAPA/2)<1)?1:(int)(eyePos[X]-ANCHOMAPA/2)+1; i < (int)(eyePos[X]+ANCHOMAPA/2>ANCHOMAPA-1)?ANCHOMAPA-1:(int)(eyePos[X]-ANCHOMAPA/2)-1; i++){
   			 // for (int j = ((int)(eyePos[Y]-ANCHOMAPA/2)<1)?1:(int)(eyePos[Y]+ANCHOMAPA/2)+1; j < (int)(eyePos[Y]+ANCHOMAPA/2>ANCHOMAPA-1)?ANCHOMAPA-1:(int)(eyePos[Y]-ANCHOMAPA/2)-1; j++){
		
				int inicioi=0;
				int inicioj=0;
				int fini=ANCHOMAPA;
				int finj=ANCHOMAPA;

				if((int)(eyePos[X]/ESCALA-CAMPOVISION)>0) inicioi= (int)(eyePos[X]/ESCALA-CAMPOVISION);
				if((int)(eyePos[Z]/ESCALA-CAMPOVISION)>0) inicioj= (int)(eyePos[Z]/ESCALA-CAMPOVISION);

				if((int)(eyePos[X]/ESCALA+CAMPOVISION)<ANCHOMAPA) fini= (int)(eyePos[X]/ESCALA+CAMPOVISION);
				if((int)(eyePos[Z]/ESCALA+CAMPOVISION)<ANCHOMAPA) finj= (int)(eyePos[Z]/ESCALA+CAMPOVISION);

				if(((int)(eyePos[X]/ESCALA))%CAMPOVISION>CAMPOVISION-CAMBIOCUADRO || ((int)(eyePos[Z]/ESCALA))%CAMPOVISION>CAMPOVISION-CAMBIOCUADRO ){
	 
 
	 			 dynamicsWorld->removeCollisionObject(escenarioRigidBody);
	 			 delete Cuadro;
	 			 delete escenarioRigidBody;

					 Cuadro = new btTriangleMesh();	
	
					for (int i = inicioi; i < fini; i++){
    					for (int j = inicioj; j < finj; j++){

  

						btVector3 v0((GLfloat)     i*ESCALA, (GLfloat) mmapa[i][j]     *ESCALA, (GLfloat)      j*ESCALA ); //i,j


						btVector3 v1((GLfloat) (i+1)*ESCALA, (GLfloat) mmapa[i+1][j]   *ESCALA, (GLfloat)      j*ESCALA );//i+1,j

		
						btVector3 v2((GLfloat)   (i)*ESCALA, (GLfloat) mmapa[i][j+1]   *ESCALA, (GLfloat) (j+1) *ESCALA );//i,j+1
        			    btVector3 v3((GLfloat) (i+1)*ESCALA, (GLfloat) mmapa[i+1][j+1] *ESCALA, (GLfloat) (j+1) *ESCALA );//i+1,j+1    
                        
                        
						Cuadro->addTriangle(v0,v2,v3,false);
 						Cuadro->addTriangle(v0,v1,v3,false);
 						}
 					}

			
			//printf(".");
				
				inii=inicioi;
				inij=inicioj;
				finni=fini;
				finnj=finj;


    
		printf("\n\nCuadro");
		delete gTriMeshShape;
		delete escenarioMotionState;
  			  gTriMeshShape = new btBvhTriangleMeshShape(Cuadro,false);
  			  escenarioMotionState =   new btDefaultMotionState(btTransform( btQuaternion(0,0,0,1),btVector3(0,0,0)));
  			  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyescenarioCI(0,escenarioMotionState,gTriMeshShape);
    
   			  escenarioRigidBody = new btRigidBody(groundRigidBodyescenarioCI);
  			  dynamicsWorld->addRigidBody(escenarioRigidBody);
		printf(" cargado.");


				}

		for (int i = inicioi; i < fini; i++){
    		for (int j = inicioj; j < finj; j++){
/*
			glBegin(GL_TRIANGLES);

				glColor3f(colores[i+1][j+1][0],colores[i+1][j+1][1],colores[i+1][j+1][2]);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(i+1, mmapa[i+1][j+1], j+1);  
				glColor3f(colores[i][j][0],colores[i][j][1],colores[i][j][2]);
				glTexCoord2f(0.0f, 0.0f);
				glVertex3f(i, mmapa[i][j], j);  
				glColor3f(colores[i][j+1][0],colores[i][j+1][1],colores[i][j+1][2]);
				glTexCoord2f(0.0f, 0.1f);
				glVertex3f(i, mmapa[i][j+1], j+1);  

			glEnd();
		
			glBegin(GL_TRIANGLES);

				glColor3f(colores[i][j][0],colores[i][j][1],colores[i][j][2]);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(i, mmapa[i][j], j);
				glTexCoord2f(0.0f, 0.0f);
				glColor3f(colores[i+1][j+1][0],colores[i+1][j+1][1],colores[i+1][j+1][2]);

				glVertex3f(i+1, mmapa[i+1][j+1], j+1);  
			 	  
			 	glColor3f(colores[i+1][j][0],colores[i+1][j][1],colores[i+1][j][2]);
				glTexCoord2f(0.0f, 1.0f);		 	
				glVertex3f(i+1, mmapa[i+1][j], j); 

			glEnd();*/
				glBegin(GL_QUADS);
				glColor3f(colores[i][j][0],colores[i][j][1],colores[i][j][2]);
				glTexCoord2f(0.0f, 1.0f);
				glVertex3f(i, mmapa[i][j], j);    // Top Left Of The Texture and Quad
				
				glColor3f(colores[i][j+1][0],colores[i][j+1][1],colores[i][j+1][2]);
				glTexCoord2f(0.0f, 0.0f);
				glVertex3f(i, mmapa[i][j+1], j+1); // Bottom Left Of The Texture and Quad
				
				glColor3f(colores[i+1][j+1][0],colores[i+1][j+1][1],colores[i+1][j+1][2]);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(i+1, mmapa[i+1][j+1], j+1);    // Bottom Right Of The Texture and Quad
				
				glColor3f(colores[i+1][j][0],colores[i+1][j][1],colores[i+1][j][2]);
				glTexCoord2f(1.0f, 1.0f);
				glVertex3f(i+1, mmapa[i+1][j], j);   // Top Right Of The Texture and Quad
				glEnd();

/*
			glBegin(GL_LINES);

				glColor3f(0.0f, 0.0f, 0.0f);

				glVertex3f(i+1, mmapa[i+1][j+1], j+1); 
				glVertex3f(i, mmapa[i][j], j); 

			glEnd();
			glBegin(GL_LINES);

				glColor3f(0.0f, 0.0f, 0.0f);

				glVertex3f(i, mmapa[i][j], j);
				glVertex3f(i, mmapa[i][j+1], j+1);

			glEnd();
			glBegin(GL_LINES);

				glColor3f(0.0f, 0.0f, 0.0f);

				glVertex3f(i, mmapa[i][j+1], j+1);
				glVertex3f(i+1, mmapa[i+1][j+1], j+1);

			glEnd();

		//	glBegin(GL_LINES);

		//		glColor3f(0.0f, 0.0f, 0.0f);

		//		glVertex3f(i, mmapa[i][j], j);
		//		glVertex3f(i+1, mmapa[i+1][j+1], j+1); 

		//	glEnd();
			glBegin(GL_LINES);

				glColor3f(0.0f, 0.0f, 0.0f);

				glVertex3f(i+1, mmapa[i+1][j+1], j+1); 
				glVertex3f(i+1, mmapa[i+1][j], j);

			glEnd();
			glBegin(GL_LINES);

				glColor3f(0.0f, 0.0f, 0.0f);

				glVertex3f(i+1, mmapa[i+1][j], j);
				glVertex3f(i, mmapa[i][j], j);

			glEnd();
		*/	
		}
	
	}
	
}





void renderScene(void) {
	btScalar m[16];

//ContactSensorCallback callbacksuelo(*escenarioRigidBody);
//ContactSensorCallback callbackchasis(*chassisRigidBody);


	

	if(fisica){
		dynamicsWorld->stepSimulation(1 / 60.f, 10);
	}
	// (Serv) escribe();
	initluces();
	puntodevista();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
 
 	if(niebla){
 	glClearColor(0.5f,0.5f,0.5f,1.0f);          // We'll Clear To The Color Of The Fog ( Modified )
 
glFogi(GL_FOG_MODE, fogMode[fogfilter]);        // Fog Mode
glFogfv(GL_FOG_COLOR, fogColor);            // Set Fog Color
glFogf(GL_FOG_DENSITY, 0.35f);              // How Dense Will The Fog Be
glHint(GL_FOG_HINT, GL_DONT_CARE);          // Fog Hint Value
glFogf(GL_FOG_START, 615.0f);             // Fog Start Depth
glFogf(GL_FOG_END, 640.0f);               // Fog End Depth
glEnable(GL_FOG);                   // Enables GL_FOG
 	}




/*
 GLfloat fogColor[] = {0.21f, 0.21f, 0.21f, 1.0};
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, 900.0f);
    glFogf(GL_FOG_END, 960.0f);
glEnable(GL_FOG);   
*/                // Enables GL_FOG
//suelo
glEnable(GL_TEXTURE_2D);
	glPushMatrix();
	//groundRigidBody->getMotionState()->getWorldTransform(t1);
	escenarioRigidBody->getMotionState()->getWorldTransform(t1);
	t1.getOpenGLMatrix(m);
	glMultMatrixf((GLfloat*) m);


	

	
	
	glScalef(ESCALA,ESCALA, ESCALA);
	
	dibujamapa();	
	
	//glDisable(GL_TEXTURE_2D);
		
	//RenderOBJModel(&escenario);

	glPopMatrix();
	
		
				//glTranslatef((GLfloat)explosiones[j])->getX(),(GLfloat)explosiones[j]->getY(),(GLfloat)explosiones[j]->getZ());
			
			
	glDisable(GL_TEXTURE_2D);



	for (int j = 0; j < nobolas; j++)
		{
			if(activob[j]==0) {



		glPushMatrix();

		glColor3f(1.0, 0.0, 0.0);
		trans.setIdentity();
		if(fallRigidBody[j]!=NULL)
		fallRigidBody[j]->getMotionState()->getWorldTransform(trans);
		trans.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		glColor3f(1.0, 0.0, 0.0);
				//	glBindTexture( GL_TEXTURE_2D, texturaventana); //bind the texture
				//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
				//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		
			ContactSensorCallback callbackbola(*fallRigidBody[j],j);
			dynamicsWorld->contactTest(fallRigidBody[j],callbackbola);
				glutSolidSphere(anchobola, 10, 10);
				//glRotatef(agrados(a)+90,0.0,1.0,0.0);
				//RenderOBJModel(&objfile);
		glPopMatrix();

			}
		
		}
		glEnable(GL_TEXTURE_2D);




glColor4f(.7,.7, .7,1.0);
glPushMatrix();
glBindTexture( GL_TEXTURE_2D, texturaventana); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
glScalef(ESCALA,ESCALA,ESCALA);
glTranslatef(ANCHOMAPA/2,0.75,ANCHOMAPA/2);
RenderOBJModel(&tile);

glPopMatrix();






/*	for (int j = 0; j < noes1; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		es1rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glBindTexture( GL_TEXTURE_2D, texturaesquina); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&esq);
		glPopMatrix();

	}
	for (int j = 0; j < nobloques; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		paredrb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glBindTexture( GL_TEXTURE_2D, texturabloque); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&bloque);
		glPopMatrix();

	}
	
	for (int j = 0; j < noes2; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		es2rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glBindTexture( GL_TEXTURE_2D, texturaesquina); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			glRotatef(-90,0.0,1.0,0.0);
		RenderOBJModel(&esq);
		glPopMatrix();

	}
	for (int j = 0; j < noes3; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		es3rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glRotatef(90,0.0,1.0,0.0);
		glBindTexture( GL_TEXTURE_2D, texturaesquina); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&esq);
		glPopMatrix();

	}
	for (int j = 0; j < noes4; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		es4rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glRotatef(180,0.0,1.0,0.0);
		glBindTexture( GL_TEXTURE_2D, texturaesquina); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&esq);
		glPopMatrix();

	}
	for (int j = 0; j < nopiso; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tpi.setIdentity();
		pisorb[j]->getMotionState()->getWorldTransform(tpi);
		tpi.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glBindTexture( GL_TEXTURE_2D, texturapiso); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&piso);
		glPopMatrix();

	}
	for (int j = 0; j < nocajas1; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		cubo1rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glRotatef(90,0.0,1.0,0.0);
		glBindTexture( GL_TEXTURE_2D, texturamuro); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&muro);
		glPopMatrix();

	}
	for (int j = 0; j < nocruz; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		cubo2rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		
		glBindTexture( GL_TEXTURE_2D, texturacruz); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&cruz);
		glPopMatrix();

	}
	for (int j = 0; j < nopuertas; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		puertasrb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glRotatef(90,0.0,1.0,0.0);
		glBindTexture( GL_TEXTURE_2D, texturapuerta); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&puerta);
		glPopMatrix();

	}
	for (int j = 0; j < nopuertas1; j++) {
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		puertas1rb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		//glRotatef(90,0.0,1.0,0.0);
		glBindTexture( GL_TEXTURE_2D, texturapuerta); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		RenderOBJModel(&puerta);
		glPopMatrix();

	}
	*/
for (int j = 0; j < nocajas; j++) { //dibuja cajas
		
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc.setIdentity();
		cuborb[j]->getMotionState()->getWorldTransform(tc);
		tc.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		//drawBox(j);
		//glTranslatef(15.95,-1.101,4.95);
		glBindTexture( GL_TEXTURE_2D, texturacaja[3]); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	//	RenderOBJModel(&muro);
			drawBox(j);
		glPopMatrix();

	}
	//vehiculo 
	glColor3f(1.0, 1.0, 1.0);
		glBindTexture( GL_TEXTURE_2D, texturarueda); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

		for(int l=0;l<vehicle->getNumWheels();l++){
		glPushMatrix(); //ruedas
			GLfloat mat[16];
			vehicle->getWheelInfo(l).m_worldTransform.getOpenGLMatrix(&mat[0]);
			glMultMatrixf(mat);
			/*glRotatef(-90,0.0,0.0,1.0);
			glRotatef(-90,1.0,0.0,0.0);
			glTranslatef(0.0,0.0,-.5);
			glScalef(.5,.5,1.0);
			glutSolidCylinder(wheelRadius,wheelWidth,12,12);*/
			glColor3f(1.0, 1.0, 1.0);
		glBindTexture( GL_TEXTURE_2D, texturarueda); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			if(l%2==1) glRotatef(180,0.0,1.0,0.0);
			RenderOBJModel(&ruedaobj);
		glPopMatrix();
		}

// CALLBACK(callback,j)(*fallRigidBody[j],j);
	//world->contactTest(tgtBody,callback);
// _elemento##_id##"("##_id##")"
	glColor3f(.5, 1.0, 0.5);
		glPushMatrix(); //chasis
			GLfloat matrix[16];
			vehicle->getChassisWorldTransform().getOpenGLMatrix(&matrix[0]);
			glMultMatrixf( matrix);
	//qwerruedaobj
	



// btBoxShape(btVector3(2.5f, 1.5f, 6.5f));
//btCollisionShape* torretaShape = new btSphereShape(2.0f);
//btCollisionShape* tuboShape = new btCylinderShape(btVector3(1.0,2.0,1.0));
	//glEnable(GL_TEXTURE_2D);

		
		glPushMatrix();
		if(!vistavala) {
			glColor3f(1.0, 1.0, 1.0);
			
		}
		else{
			glEnable (GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA,GL_ONE);     
				glColor4f(1.0f,1.0f,1.0f,.7f);
			//GLEnable(GL_TEXTURE_2D);
		}

	glBindTexture( GL_TEXTURE_2D, texturatanque); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			RenderOBJModel(&tanqueobj);
		//GLDisable(GL_TEXTURE_2D);

		glPopMatrix();

	/*	glPushMatrix();
			glColor3f(1.0, 1.0, 1.0);
			
	glBindTexture( GL_TEXTURE_2D, texturatanque); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			glTranslatef(0.0,1.0,0.0);
			glutSolidSphere(1.5,12,12);
			glTranslatef(0.0,1.0,0.0);
			
			glutSolidCylinder(.5,4.0,12,12);
		glPopMatrix();
*/
			//RenderOBJModel(&quadobj);
/*
	
		glTranslatef(0.0,-0.75,0.0);
		RenderOBJModel(&tanqueobj);
		glTranslatef(0.0,2.0,-0.5);
		glRotatef(alfatorreta,0.0,1.0,0.0);
		RenderOBJModel(&tanquetobj);
		*/
		if(vistavala){
				glDisable (GL_BLEND);
			
				glColor4f(1.0f,1.0f,1.0f,1.0f);
		}

/*
 	if (!Serv){
         //mtx.unlock();
     	glPushMatrix();
     	//glLoadIdentity();
     	
		  //glMultMatrixf(matrizene);
     		  // glScalef(2.5,1.5,6.5);
     			glTranslatef(matrizene[0],matrizene[1],matrizene[2]);
     		  	glutSolidCube(1);
		glPopMatrix();	

		}
*/



	glPopMatrix();	
//qwer
	//glDisable(GL_TEXTURE_2D);

	



		//objetos de red
		
		
		 // mtx.trylock();
      



/*

	glPushMatrix();


		glColor3f(1.0, 1.0, 1.0);
		glBindTexture( GL_TEXTURE_2D, texturatanque); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		
	

	//	RenderOBJModel(&tanqueobj);

		//float a = atan2(vectorvision[X] - eyePos[X], vectorvision[Y] - eyePos[Y]);


		glRotatef(alfatorreta,0.0,1.0,0.0);
		RenderOBJModel(&tanquetobj);
		if(vistavala)glRotatef(-agrados(sin(vectorvision[Y])),1.0,.0,.0);
		RenderOBJModel(&tanquetuobj);
	
	
		glPopMatrix();
	//machango.update();
	*/
/*
		glPushMatrix();

			glColor3f(1.0, 1.0, 1.0);
			glBindTexture( GL_TEXTURE_2D, texturaaracnido); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			glTranslatef(machango.casx,machango.altura,machango.casy);
			glRotatef(agrados(machango.alfa),0.0,1.0,0.0);
			glScalef(.5,0.5,0.5);
			RenderOBJModel(&(modelo[machango.anima]));
			
		glPopMatrix();
	//machango.render();
*/
glEnable(GL_TEXTURE_2D);
glColor4f(0.0f,0.3f,0.0f,1.0f);
for (int j = 0; j < arbol.size(); j++) {
			btVector3 po=btVector3(0.0,0.0,0.0);  
			glPushMatrix();

					glScalef(ESCALA,ESCALA, ESCALA);
					if(arbol[j].getx()>inii && arbol[j].getz()>inij && arbol[j].getx()<finni && arbol[j].getz()<finnj )
  					arbol[j].drawgrass(po,eyePos, CAMPOVISION,vectorvision,&tronco,&cabeza,&pina,texturapalmera,cos(viento)*7);
  					
			glPopMatrix();
		}	

glEnable (GL_BLEND);
glBlendFunc(GL_SRC_ALPHA,GL_ONE);     









glPushMatrix();
	glTranslatef(ANCHOMAPA*ESCALA/2,16.0,ANCHOMAPA*ESCALA/2);
	glScalef(ESCALA,ESCALA,ESCALA);
	glBindTexture( GL_TEXTURE_2D, texturacircuito); //bind the texture
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

	//glScalef(10.0f,10.0f,10.0f);
		RenderOBJModel(&circuitoobj);
glPopMatrix();

glPushMatrix();
glColor4f(0.0f,0.3f,1.0f,1.0f);
glBindTexture( GL_TEXTURE_2D, texturaskysphere); //bind the texture
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
glTranslatef(eyePos[X], eyePos[Y],eyePos[Z]);
float esca=615;
glScalef(esca,esca,esca);
int i, j;
struct obj_model_t mdl=skysphere;
  for (i = 0; i < mdl.num_faces; ++i)
    {
      glBegin (mdl.faces[i].type);
	for (j = 0; j < mdl.faces[i].num_elems; ++j)
	  {
	    if (mdl.has_texCoords)
	      glTexCoord3fv (mdl.texCoords[mdl.faces[i].uvw_indices[j]].uvw);

	    if (mdl.has_normals)
	     glNormal3f (-mdl.normals[mdl.faces[i].norm_indices[j]].ijk[0],-mdl.normals[mdl.faces[i].norm_indices[j]].ijk[1],-mdl.normals[mdl.faces[i].norm_indices[j]].ijk[2]);
		//glNormal3fv (mdl.normals[mdl.faces[i].norm_indices[j]].ijk);

	    glVertex4fv (mdl.vertices [mdl.faces[i].vert_indices[j]].xyzw);
	  }
	glEnd();
    }

glPopMatrix();









glColor4f(0.0f,0.0f,1.0f,.6f); 	
/*
for(int j=0;j<nobolas;j++){
	trans.setIdentity();
	if(fallRigidBody[j]!=NULL){
		fallRigidBody[j]->getMotionState()->getWorldTransform(trans);

		if(explosiones[j].activ()){
			explosiones[j].dibuja(btVector3(trans.getOrigin()),j,eyePos,vectorvision,texturaex);
		}else{
			//dynamicsWorld->removeRigidBody(fallRigidBody[j]);
				
		}
	}
}
*/
for(int j=0;j<ne;j++)
	if(explosion[j].activ()){
			if(!explosion[j].dibujaexplosion(0,eyePos,vectorvision,texturaex)){
				dynamicsWorld->removeRigidBody(fallRigidBody[explosion[j].nrb]);
			}
	
			//if(!explosion[j].activ()){
			//	dynamicsWorld->removeRigidBody(fallrb[j]);
				
			//}
	}
//}




viento=viento+.03;
//else viento=0.0;
glColor4f(0.0f,1.0f,1.0f,1.0f); 	



//for(int z=ANCHOMAPA/8;z<ANCHOMAPA/5;z++)
//	for(int e=ANCHOMAPA/8;e<ANCHOMAPA/5;e++){
		
		//if(mmapa[e][z]>3.95){
//std::vector <grass> temp;

/**			

	for(int j=0;j<cesped.size();j++){//ordena las llamas segun el punto de vista
				
        		float longitud = sqrt((eyePos[0]-cesped[j].getx())*(eyePos[0]-cesped[j].getx())+(eyePos[1]-cesped[j].gety())*(eyePos[1]-cesped[j].gety())+(eyePos[2]-cesped[j].getz())*(eyePos[2]-cesped[j].getz()));
				//inserta(vectort[j],longitud);
        	//	llama flame(vectort[j],longitud);
        		//if(getw()>0) vectort.setW(getw()-6.0);

        		it=temp.begin();
        		if(temp.size()==0) 	temp.insert(it,  cesped[j]);
        		else for (it=cesped.begin(); it<cesped.end(); it++){
        				if(longitud>it->getlon(eyePos)){
        					temp.insert(it,  cesped[j]);
        					
        					break;
        				}
        		}
        	
       
        		
        	}

*/

int e=18;
int z=13;




			btVector3 po=btVector3(e*ESCALA, mmapa[e][z]*ESCALA-.3, z*ESCALA);  
			
			for (int j = 0; j < cesped.size(); j++) {
		//glScalef(cesped[j].getw(),cesped[j].getw(),cesped[j].getw());
				glPushMatrix();

				//  glRotatef(90-agrados(atan2(vectorv[2],vectorv[0])),0.0,1.0,0.0); //rotacion hacia el punto de vista
			          
  					float gx=(90-agrados(atan2(ESCALA,(mmapa[e][z+1]-mmapa[e][z])*ESCALA)));
  					float gz=(90-agrados(atan2(ESCALA,(mmapa[e+1][z]-mmapa[e][z])*ESCALA)));
  					//std::cout << "gx: " << gx << std::endl;
					cesped[j].drawgrass(po,eyePos,gx,gz, vectorvision,hojas,texturahoja,texturatronco,cos(viento)*7);
		//	cesped[j].drawgrass(eyePos,hojas,texturahoja);
				glPopMatrix();
			}


		//temp.clear();
			//btVector3 po=btVector3(e*ESCALA, mmapa[e][z]*ESCALA-.3, z*ESCALA);  
			
			for (int j = 0; j < cespedi.size(); j++) {
		//glScalef(cesped[j].getw(),cesped[j].getw(),cesped[j].getw());
				glPushMatrix();

				//  glRotatef(90-agrados(atan2(vectorv[2],vectorv[0])),0.0,1.0,0.0); //rotacion hacia el punto de vista
			          
  					float gx=(90-agrados(atan2(ESCALA,(mmapa[e][z+1]-mmapa[e][z])*ESCALA)));
  					float gz=(90-agrados(atan2(ESCALA,(mmapa[e+1][z]-mmapa[e][z])*ESCALA)));
  					//std::cout << "gx: " << gx << std::endl;
					cespedi[j].drawgrass(po,eyePos,gx,gz, vectorvision,hojas,texturahoja,texturatronco,cos(viento)*7);
		//	cesped[j].drawgrass(eyePos,hojas,texturahoja);
				glPopMatrix();
			}
glDisable(GL_BLEND); 



	//	}
	//}






 //sombras
/*
glm::vec3 lightInvDir = glm::vec3(	769.8f, 30.3f, 773.0f);
 
 // Compute the MVP matrix from the light's point of view
 glm::mat4 depthProjectionMatrix = glm::ortho<float>(-10,10,-10,10,-10,20);
 glm::mat4 depthViewMatrix = glm::lookAt(lightInvDir, glm::vec3(0,0,0), glm::vec3(0,1,0));
 glm::mat4 depthModelMatrix = glm::mat4(1.0);
 glm::mat4 depthMVP = depthProjectionMatrix * depthViewMatrix * depthModelMatrix;
 



 // Send our transformation to the currently bound shader,
 // in the "MVP" uniform
 glUniformMatrix4fv(depthMatrixID, 1, GL_FALSE, &depthMVP[0][0])
//GLuint FramebufferName = 0;
 glGenFramebuffers(1, &FramebufferName);
 glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
 
 // Depth texture. Slower than a depth buffer, but you can sample it later in your shader
 //GLuint depthTexture;
 glGenTextures(1, &depthTexture);
 glBindTexture(GL_TEXTURE_2D, depthTexture);
 glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT16, 1024, 1024, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
 
 glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);
 
 glDrawBuffer(GL_NONE); // No color buffer is drawn to.
 
 // Always check that our framebuffer is ok
 if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
 //return false;
*/
/*

glPushMatrix();

glEnable (GL_BLEND);
	// glColor4f(0.0f,0.0f,200/255,0.5f);                 // Full Brightness, 50% Alpha ( NEW )
        glBlendFunc(GL_SRC_ALPHA,GL_ONE);               // Blending Function For Translucency Based On Source Alpha Value ( NEW )

		glColor4f(0.0f,0.0f,1.0f,.6f); 		
		glTranslatef(ANCHOMAPA*ESCALA/2,0.0,ANCHOMAPA*ESCALA/2);


		glBindTexture( GL_TEXTURE_2D, texturaagua); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	



 		glBegin(GL_TRIANGLES); //vista desde arriba
       			glTexCoord2f(1, 0); glVertex3f(-ANCHOMAPA*ESCALA/2, -ESCALA, -ANCHOMAPA*ESCALA/2 );
      			 glTexCoord2f(0, 0); glVertex3f(-ANCHOMAPA*ESCALA/2, -ESCALA, ANCHOMAPA*ESCALA/2 );
      			 glTexCoord2f(0, 1); glVertex3f(ANCHOMAPA*ESCALA/2, -ESCALA, ANCHOMAPA*ESCALA/2 );
			
				 glTexCoord2f(1, 0); glVertex3f(-ANCHOMAPA*ESCALA/2, -ESCALA, -ANCHOMAPA*ESCALA/2 );
				 glTexCoord2f(0, 0); glVertex3f(ANCHOMAPA*ESCALA/2, -ESCALA, ANCHOMAPA*ESCALA/2 );
      			 glTexCoord2f(0, 1); glVertex3f(ANCHOMAPA*ESCALA/2, -ESCALA, -ANCHOMAPA*ESCALA/2 );
      			 
    		glEnd();


		glBegin(GL_TRIANGLES);
       			glTexCoord2f(1, 0); glVertex3f(ANCHOMAPA*ESCALA/2, -ESCALA, ANCHOMAPA*ESCALA/2 );
      			 glTexCoord2f(0, 0); glVertex3f(-ANCHOMAPA*ESCALA/2, -ESCALA, ANCHOMAPA*ESCALA/2 );
      			 glTexCoord2f(0, 1); glVertex3f(-ANCHOMAPA*ESCALA/2, -ESCALA, -ANCHOMAPA*ESCALA/2 );
			
				 glTexCoord2f(1, 0); glVertex3f(ANCHOMAPA*ESCALA/2, -ESCALA, -ANCHOMAPA*ESCALA/2 );
				 glTexCoord2f(0, 0); glVertex3f(ANCHOMAPA*ESCALA/2,- ESCALA, ANCHOMAPA*ESCALA/2 );
      			 glTexCoord2f(0, 1); glVertex3f(-ANCHOMAPA*ESCALA/2, -ESCALA, -ANCHOMAPA*ESCALA/2 );
      			 
    	glEnd();

	glColor4f(1.0f,1.0f,1.0f,1.0f);  





glDisable(GL_BLEND);            // Turn Blending Off

glPopMatrix();

*/








	glColor3f(1.0,1.0,1.0);
	const char hola[4]={'h','o','l','a'};
	//const char hola[1]={'+'};
	text(cruzx,cruzy,20,"+");

	//glEnable(GL_TEXTURE_2D);
	

	glutSwapBuffers();
}


void changeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;

	float ratio = w * 1.0 / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(60.0f, ratio, 0.1f, 1000.0f);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
	cruzx=w/2;
	cruzy=h/2;
}

void motion(int x, int y) {
   if(moving){
	if (is_first_time) {
        //is_first_time = false;
        // Check for zooming mode, but do not allow the model
        // jump off the screen when the mouse enters the window from an outside
        // region of screen. Thus, to activate the mouse, you need to click the
        // mouse to zoom the view AND move it a little:
        is_first_time = false;
        beginx = x;
        beginy = y;
        return;
    }
    GLint deltaX = beginx-x;
    GLint deltaY = beginy-y;
	
 
    if ( deltaX < -1.0 ) { deltaX = -1.0; }
    if ( deltaX >  1.0 ) { deltaX =  1.0; }
    if ( deltaY < -1.0 ) { deltaY = -1.0; }
    if ( deltaY >  1.0 ) { deltaY =  1.0; }
	alfax=alfax-deltaX*(NPI/180);
	alfay=alfay+deltaY*(NPI/180);

	if(alfay>NPI/2) alfay=NPI/2;
	if(alfay<-NPI/2) alfay=-NPI/2;
	vectorvision[X]=cos(alfax);
	vectorvision[Y]=sin(alfay);
	vectorvision[Z]=sin(alfax);

	/*if(vistavala){
		btTransform t;  //position and rotation
		t.setIdentity();

		t.setOrigin(btVector3(vehicle->getChassisWorldTransform().getOrigin().getX(),vehicle->getChassisWorldTransform().getOrigin().getY()+2, vehicle->getChassisWorldTransform().getOrigin().getZ()));
		t.setRotation(btQuaternion(vectorvision[X], vectorvision[Y], vectorvision[Z], 1));
		tuborb->setCenterOfMassTransform(t);
	}*/

	

	beginx = x;
    beginy = y;

	glutPostRedisplay();
   }
}


void mouse(int button, int state, int x, int y)
{
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
    moving = 1;
    beginx = x;
	beginy = y;
	
  }
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
    moving = 0;
  }
	 if (button==3) {
	  	 eyePos[0]=eyePos[0]+vectorvision[0];
		 eyePos[1]=eyePos[1]+vectorvision[1];
	     eyePos[2]=eyePos[2]+vectorvision[2];
	}
	if (button==4){
		eyePos[0]=eyePos[0]-vectorvision[0];
		 eyePos[1]=eyePos[1]-vectorvision[1];
	     eyePos[2]=eyePos[2]-vectorvision[2];
	}

if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
	if(freelook){
				GLfloat matrix[16];
			vehicle->getChassisWorldTransform().getOpenGLMatrix(&matrix[0]);
				 int j=nobolas;
				 btVector3 fallInertia=btVector3(vectorvision);
				fallShapecoll[j] = new btSphereShape(anchobola);
				 btDefaultMotionState *fallMotionState;
				 if(vistavala) fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(eyePos[X],eyePos[Y],eyePos[Z])));
				 else  fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(eyePos[X], eyePos[Y], eyePos[Z])));

				 fallShapecoll[j]->calculateLocalInertia(1000.0, fallInertia);
				 btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(2.0,fallMotionState, fallShapecoll[j], fallInertia);
				 fallRigidBody[j] = new btRigidBody(fallRigidBodyCI);
				 dynamicsWorld->addRigidBody(fallRigidBody[j]);
				
				 nobolas++;
				
				fallRigidBody[j]->setLinearVelocity(fallRigidBody[j]->getLinearVelocity()+vectorvision*128);
				 
				
				//ContactSensorCallback callbackbola[j](&fallRigidBody[j]);
				
					//colisiones
				
			}

	}

	
}


void specialkeyboard(int key, intf x, int y) {
	//printf("\ntecla %d", key);
//fallRigidBody[0]->applyTorqueImpulse(btVector3(-1.0,0.0,0.0));
	btVector3 vm=btVector3(0.0,0.0,0.0);
	if (key==100){ //izquierda
	//	vm=vm+btVector3(1.0, 0.0, 0.0);
		//machango.alfa=machango.alfa-0.010;
		}
	if (key==103) {//abajo
		//vm=vm+btVector3(0.0, 0.0, 1.0);
		if(freelook){
			eyePos[0]=eyePos[0]-vectorvision[0]*ESCALA;
			eyePos[1]=eyePos[1]-vectorvision[1]*ESCALA;
			eyePos[2]=eyePos[2]-vectorvision[2]*ESCALA;
		}else{
			//personajerb->translate( btVector3(vectorvision[0]/8, 0.0f,vectorvision[2])/8 );
		}
			
	}
	if (key==101){ //arriba
		//machago.avanza();
	//	vm=vm+btVector3(0.0, 0.0, -1.0);
		if(freelook){
		 eyePos[0]=eyePos[0]+vectorvision[0]*ESCALA;
		 eyePos[1]=eyePos[1]+vectorvision[1]*ESCALA;
	     eyePos[2]=eyePos[2]+vectorvision[2]*ESCALA;
		}else{
			//personajerb->translate( btVector3(-vectorvision[0]/8, 0.0f,-vectorvision[2])/8 );
		}
			//machango.avanza();
			
	}
	if (key==102){ //derecha
	//	vm=vm+btVector3(-1.0, 0.0, 0.0);
		//machango.alfa=machango.alfa+0.010;
	}
	if (key== 27){ //esc

		//fin();
		exit(0);
	}
	
}


void specialkeyboard(unsigned char key, int x, int y) {
	bool mover=false;
	float fuerzai=0.0,fuerzad=0.0;
	//printf("\ntecla %uc", key);
//fallRigidBody[0]->applyTorqueImpulse(btVector3(0.0,-1.0,0.0));
	switch (key) {
	case 'e': //arrib	
		//machango.casx=machango.casx+sin(machango.alfa)/20;
		//machango.casy=machango.casy+cos(machango.alfa)/20;
		//machango.update(true);
			if(gEngineForce<maxEngineForce){
				gEngineForce += acel;
				gBreakingForce = 0.f;
				movervehiculo();
			}
		//naverb->setLinearVelocity(fallRigidBody[0]->getLinearVelocity()+vectorvision);
		//eeeenaverb[0]->applyCentralImpulse(vectorvision);
		//personajerb->translate(btVector3(vectorvision[X],0.0,vectorvision[Z]));
	break;
		case 'd': //atras
			//machango.casx=machango.casx-sin(machango.alfa)/20;
			//machango.casy=machango.casy-cos(machango.alfa)/20;
			//machango.update(false);
			if(gEngineForce>-maxEngineForce){
				gEngineForce -= acel;
				gBreakingForce = 0.f;
				movervehiculo();
			}
		
	break;
		case 's':
			if(gVehicleSteering<(NPI/5)){
				gVehicleSteering += steeringIncrement;
				movervehiculo();
			}
			/*if(gEngineForced<maxEngineForce){
				gEngineForced += acel;
				gBreakingForce = 0.f;
				movervehiculo();
			}*/
			//machango.alfa=machango.alfa+0.1;
			//movervehiculo();	
		//fallRigidBody[0]->setLinearVelocity(fallRigidBody[0]->getLinearVelocity()+btVector3(+sin(vectorvision[2]), 0.0, +cos(vectorvision[2])));
		break;
	case 'f':
			//machango.alfa=machango.alfa-0.1;
			if(gVehicleSteering>-(NPI/5)){
				gVehicleSteering -= steeringIncrement;
				movervehiculo();
			}
			/*if(gEngineForced>-maxEngineForce){
				gEngineForced -= acel;
				gBreakingForce = 0.f;
				movervehiculo();
			}*/
			
		//fallRigidBody[0]->setLinearVelocity(fallRigidBody[0]->getLinearVelocity()+btVector3(-sin(vectorvision[2]), 0.0,-cos(vectorvision[2])));
		break;
	case 'a':
			gBreakingForce = maxBreakingForce; 
			gEngineForce = 0.f;
			gEngineForced = 0.f;
		    movervehiculo();
		    break;

	case 't':

		break;

	case 'l':
		freelook=!freelook;
		break;
	case 'p':
		 creaescenario();
		break;
	case 'o':
		/*	movervehiculo();*/
		vara=vara-1.0;
		break;
	case 'r':
		viento++;
		break;
	case 'w':
		viento--;
	  	//tanqueshape->getChildShape(2)->setRotation();
		//initpersonaje();
		break;
	
	case ' ':
		printf("\neyePos[]={%f, %f, %f}\t vectorvision[]={%f, %f, %f}",eyePos[X],eyePos[Y],eyePos[Z], vectorvision[X],vectorvision[Y],vectorvision[Z]);	
		printf("\n motor: %f",gEngineForce);
		break;	
	case 'v':
		pintacircuito=!pintacircuito;
		
		break;
	case 'g':
		if(niebla){
		 
    	fogfilter+=1;                   // Increase fogfilter By One
   			 if (fogfilter>2)             // Is fogfilter Greater Than 2?
   			 {
		    		    fogfilter=0;                // If So, Set fogfilter To Zero
		    }
   			 glFogi (GL_FOG_MODE, fogMode[fogfilter]);   // Fog Mode
		}
		else
		{
				niebla=true;
		}
		break;
	

		case 'z':
		fisica=!fisica;
		
		break;	
	case 'q':
			
			servidor=false;
			fin();
			exit(1);
			break;
	case '0':
			vistavala=!vistavala;
	default:
		break;
	}

	
	
}


void session(socket_ptr sock)
{
	using boost::asio::ip::tcp;
   try
  {
    while(servidor)
    {
    
	  char data[max_length];
      boost::system::error_code error;
      size_t length = sock->read_some(boost::asio::buffer(data), error);
      std::cout << "recibido: \"" <<data << "\""<<std::endl;
      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.
  		
    		GLfloat matrix[16];
			matrix[0]=0.1f;
			matrix[1]=0.2f;
			matrix[2]=0.3f;
			

 				std::ostringstream archive_stream;
				boost::archive::text_oarchive archive(archive_stream);
				archive << matrix;
				
				std::string outbound_data_ = archive_stream.str();
				 boost::asio::write(*sock, boost::asio::buffer(outbound_data_, outbound_data_.length()));
				// std::cout << "enviado: \"" <<outbound_data_<< "\"";

     	 		bzero(data,1024);
     	 		

            }
       }
  		catch (std::exception& e)
  		{
 		   std::cerr << "Exception in thread: " << e.what() << "\n";
 		}
	
}

 boost::asio::io_service io_service;


void server(int puerto)
{
	using boost::asio::ip::tcp;
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), puerto));
   while (servidor)
  {
  	std::cout << "Servidor conectado:." << std::endl;
    socket_ptr sock(new tcp::socket(io_service));
    a.accept(*sock);
    boost::thread t(boost::bind(session, sock));
    std::cout << "Nuevo Cliente." << std::endl;
  }
}



//CLIENTE____________________________________________________________________________
void client(char* argv[])
{
	using boost::asio::ip::tcp;
    char reply[max_length];
	try
	{
	boost::asio::io_service io_service;

    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), "127.0.0.1", "4444");
    tcp::resolver::iterator iterator = resolver.resolve(query);
 	tcp::iostream stream;
    tcp::socket s(io_service);
    boost::asio::connect(s, iterator);
 	std::cout << "Cliente conectado al sevidor.";
    using namespace std; // For strlen.
      char request[max_length];
  	    bzero(request,max_length);
  	    request[0]=':';
  	    request[1]='P';
  	 // std::cin.getline(request, max_length);
  	  size_t request_length = strlen(request);
  	  boost::asio::write(s, boost::asio::buffer(request, request_length));
  while(servidor){
  	//  std::cout << "Enter message: ";
  	

  	 
  	 
  	//  size_t lon=16 * (sizeof(GLfloat));
  	   bzero(reply,max_length*sizeof(char));
  		size_t reply_length = boost::asio::read(s, boost::asio::buffer(reply, max_length));
 	   //std::cout << "Reply is: ";
    //std::cout<<reply[0]<<std::endl;
  		//GLfloat *p = &reply;
	//stringstream ss(reply);
		GLfloat msg[16];
  		 bzero(msg,16 * (sizeof(GLfloat)));
  		 int f=0;
	for(int i=0; i<reply_length; i+=4) {
		char flotante[4];
		flotante[0]=reply[i];
		flotante[1]=reply[i+1];
		flotante[2]=reply[i+2];
		flotante[3]=reply[i+3];
		msg[f]=std::atof(flotante);
		f++;
	}
	//std::stof(fs);
  	 std::cout << "(" << msg[0]<< ")" << std::endl;

  	//	memcpy(matrizene, reply, 16*sizeof(GLfloat));
  	//	  std::cout << "(" << matrizene[0] << ", " << matrizene[1] << ", "<< matrizene[2] << ")" << std::endl;


  char request[max_length];
  	    bzero(request,max_length);
  	    request[0]=':';
  	    request[1]='P';
  	 // std::cin.getline(request, max_length);
  	  size_t request_length = strlen(request);
  	  boost::asio::write(s, boost::asio::buffer(request, request_length));


  	 
	}
	}
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

 
//}

}
/*
void escribe(){

	 char request[max_length];
     bzero(request,max_length);
     vehicle->getChassisWorldTransform().getOpenGLMatrix((float*)request[0]);
    //equest[0]=(char) contador;
     contador++;

    size_t request_length = std::strlen(request);
    std::cout << "Escribiendo["<<contador<<"]: \"" << request << "\""<<std::endl;
    boost::asio::write(s, boost::asio::buffer(request, request_length));
}
*/

void initmapa() {
	initluces();
	gluLookAt( 0.0, 0.0, 1.0,
	           0.0, 0.0, 1,
	           0.0, 1.0, 0.0);

	
 glColor3f(1.0, 1.0, 1.0);
	//glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);

/*
	// register callbacks
	glutIgnoreKeyRepeat(1);
	glutKeyboardFunc(processNormalKeys);
	glutSpecialFunc(pressKey);
	glutSpecialUpFunc(releaseKey);
	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMove);*/
}

void rendermap(){
	glColor3f(1.0,1.0,1.0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture( GL_TEXTURE_2D, texturecaja[texturascajas[0]]); //bind the texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	GL_LINEAR_MIPMAP_NEAREST);
	glBegin(GL_QUADS);
	// Front Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Left Of The Texture and Quad
	glEnd();
	glDisable(GL_TEXTURE_2D);
}
int main(int argc, char **argv) {

	// init GLUT and create window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(RENDER_WIDTH, RENDER_HEIGHT);
	mainwindow=glutCreateWindow("Proyecto");

	// register callbacks

	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutMotionFunc(motion);
    glutMouseFunc(mouse); 
	glutSpecialFunc(specialkeyboard);
	glutKeyboardFunc(specialkeyboard);


	float border=2.0;
	float w=150.0;
	float h=150.0;
	//mapwindow = glutCreateSubWindow(mainwindow, border,border,w-2*border, h - 2*border);
	//glutDisplayFunc(rendermap);
	//initmapa();





//genera el dungeon
	//Dungeon D(3,3,1);
	
//inicia la fisica y pos objetos
	
if (argc != 3)
    {
  try
  {
   
    Serv=true;

   boost::thread t(boost::bind(server, 4444));
   //server(4444);
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }
} 
inicio();


if (argc == 3) boost::thread t(boost::bind(client, &argv[2]));






	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glCullFace(GL_BACK);

	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearDepth(1.0f);								// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	//glDepthFunc(GL_LEQUAL);					// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);// Really Nice Perspective Calculations

	glClearColor(0.15, .15, 0.15, 1.0); //selecciona color de fondo
	std::cout << "Iniciando loop del juego...";
	glutMainLoop();
	
	return 1;
}
