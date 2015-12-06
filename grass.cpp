#include <iostream>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include <GL/freeglut.h>


#define NPI 3.14159265358979323846



#define NOHOJAS 1


#ifndef EXTRUCTURASVERTICES

/* Vectors */
typedef float vec3_t[3];
typedef float vec4_t[4];


struct obj_vertex_t
{
  vec4_t xyzw;
};

/* Texture coordinates */
struct obj_texCoord_t
{
  vec3_t uvw;
};

/* Normal vector */
struct obj_normal_t
{
  vec3_t ijk;
};

/* Polygon */
struct obj_face_t
{
  GLenum type;        /* primitive type */
  int num_elems;      /* number of vertices */

  int *vert_indices;  /* vertex indices */
  int *uvw_indices;   /* texture coordinate indices */
  int *norm_indices;  /* normal vector indices */
};

/* OBJ model structure */
struct obj_model_t
{
  int num_verts;                     /* number of vertices */
  int num_texCoords;                 /* number of texture coords. */
  int num_normals;                   /* number of normal vectors */
  int num_faces;                     /* number of polygons */

  int has_texCoords;                 /* has texture coordinates? */
  int has_normals;                   /* has normal vectors? */

  struct obj_vertex_t *vertices;     /* vertex list */
  struct obj_texCoord_t *texCoords;  /* tex. coord. list */
  struct obj_normal_t *normals;      /* normal vector list */
  struct obj_face_t *faces;          /* model's polygons */
};




#endif

	class grass{



 public:
	btVector4 vectort;
	int text[NOHOJAS];
	int rot[NOHOJAS];
	int mod[NOHOJAS];
	time_t oldseedw;
	int num;
	btVector3 ramo[NOHOJAS];
	
	grass(){
		num=0;
		bzero(vectort,sizeof(btVector4)*NOHOJAS);
		bzero(ramo,sizeof(btVector3)*NOHOJAS);
	}

	grass(float x,float y,float z){
		num=0;
		
		time(&oldseedw);
		bzero(vectort,sizeof(btVector4)*NOHOJAS);

		//std::cout << "nueva explosion" << std::endl;
		vectort=( btVector4(x,y,z,.25));
		int maxi=NOHOJAS;
		for(int j=0;j<maxi;j++){
		
			add();
		}
		
	}
	/*grass(btVector3 v4){
		num=0;
		bzero(vectort,sizeof(btVector4)*NOHOJAS);
		//std::cout << "nueva explosion" << std::endl;
		vectort=( btVector4(v4.getX(),v4.getY(),v4.getZ(),(btScalar) (rand()%10/10+.2)));
		int maxi=rand()%32+32;
		for(int j=0;j<maxi;j++){
			add();
		}
	}*/
	
	void add(){
		ramo[num]=btVector3 (getRand(0,10)/10, gety(), getRand(0,10)/10);
		text[num]=0;//rand()%4;
		rot[num]=getRand(-15,15);
		mod[num]=getRand(0,7);
	//	oldseedw=time(num);
		//std::cout << mod[num] << ", ";
		num++;
		//std::cout << ".,";
	}
	inline double agrados(double radianes) {
    return radianes*(180.0/NPI);
}
	
	btScalar getx(){
		return vectort.getX();
	}
	btScalar gety(){
		return vectort.getY();
	}
	btScalar getz(){
		return vectort.getZ();
	}
	btScalar getw(){
		return vectort.getW();
	}

float getlon(float eyePos[3]){
	float l = sqrt((eyePos[0]-getx() )*(eyePos[0]-getx() )+(eyePos[1]-gety() )*(eyePos[1]-gety() )+(eyePos[2]-getz() )*(eyePos[2]-getz() ));
				
	return l;
}

	bool drawgrass(btVector3 pos,float eyePos[3],float gx, float gz ,float vectorv[3],obj_model_t hojas[12], GLuint texturahoja, GLuint texturatronco,int viento){

		glTranslatef(pos.getX(),pos.getY(),pos.getZ());
		glRotatef(-gx ,1.0,0.0,0.0);
		glRotatef(-gz ,0.0,0.0,1.0);
		glTranslatef(getx(),gety(),getz());
		
		
			
		//glScalef(.8,getw(),.8);
			for(int d=0;d<num;d++){




			glPushMatrix();
				glTranslatef(ramo[d].getX(),ramo[d].getY(),ramo[d].getZ());
					glRotatef(viento%30,1.0,0.0,0.0);
					glTranslatef(num/9,0.0,num/9);
				  
			
			
				 	  glRotatef(90-agrados(atan2(vectorv[2],vectorv[0])),0.0,1.0,0.0); //rotacion hacia el punto de vista
			          
			          glRotatef(rot[d],0.0,1.0,0.0); //rotacion aleatoria
		
				 glScalef(2.0,16.0,2.0);
			   float f=getw()/10;
   			   float nudo=0.0;
			   glColor4f(1.0,1.0,1.0,1.0);
                  	glBindTexture( GL_TEXTURE_2D, texturatronco); //bind the texture
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
					         
                glBegin(GL_QUADS);
					glTexCoord2f(1.0f, 0.0f);
					glVertex3f(-f, nudo, - f);  // Bottom Right Of The Texture and Quad
					glTexCoord2f(1.0f, 1.0f);
					glVertex3f(- f,  f, - f);  // Top Right Of The Texture and Quad
					glTexCoord2f(0.0f, 1.0f);
					glVertex3f( f,  f, - f);  // Top Left Of The Texture and Quad
					glTexCoord2f(0.0f, 0.0f);
					glVertex3f(f, nudo, - f);  // Bottom Left Of The Texture and Quad
				glEnd();

				glTranslatef(0.0,f,0.0);
					glRotatef(viento%14/2,vectorv[0],0.0,vectorv[2]);

				glBegin(GL_QUADS);
					glTexCoord2f(1.0f, 0.0f);
					glVertex3f(-f, nudo, - f);  // Bottom Right Of The Texture and Quad
					glTexCoord2f(1.0f, 1.0f);
					glVertex3f(- f,  f, - f);  // Top Right Of The Texture and Quad
					glTexCoord2f(0.0f, 1.0f);
					glVertex3f( f,  f, - f);  // Top Left Of The Texture and Quad
					glTexCoord2f(0.0f, 0.0f);
					glVertex3f(f, nudo, - f);  // Bottom Left Of The Texture and Quad
				glEnd();
                 
               glTranslatef(0.0,f,0.0);
					glRotatef(viento%7,vectorv[0],0.0,vectorv[2]);
					glColor3f(1.0,1.0,1.0);

					glBindTexture( GL_TEXTURE_2D, texturahoja); //bind the texture
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

               glBegin(GL_QUADS);
					glTexCoord2f(1.0f, 0.0f);
					glVertex3f(-f, nudo, - f);  // Bottom Right Of The Texture and Quad
					glTexCoord2f(1.0f, 1.0f);
					glVertex3f(- f,  f, - f);  // Top Right Of The Texture and Quad
					glTexCoord2f(0.0f, 1.0f);
					glVertex3f( f,  f, - f);  // Top Left Of The Texture and Quad
					glTexCoord2f(0.0f, 0.0f);
					glVertex3f(f, nudo, - f);  // Bottom Left Of The Texture and Quad
				glEnd();
			glPopMatrix();
           	}

					/*struct obj_model_t *mdl=&hojas[mod[d]];
  for (int i = 0; i < mdl->num_faces; ++i)
    {
      glBegin (mdl->faces[i].type);
	for (int j = 0; j < mdl->faces[i].num_elems; ++j)
	  {
	    if (mdl->has_texCoords)
	      glTexCoord3fv (mdl->texCoords[mdl->faces[i].uvw_indices[j]].uvw);

	    if (mdl->has_normals)
	    // glNormal3f (-mdl->normals[mdl->faces[i].norm_indices[j]].ijk[0],-mdl->normals[mdl->faces[i].norm_indices[j]].ijk[1],-mdl->normals[mdl->faces[i].norm_indices[j]].ijk[2]);
		glNormal3fv (mdl->normals[mdl->faces[i].norm_indices[j]].ijk);

	    glVertex4fv (mdl->vertices [mdl->faces[i].vert_indices[j]].xyzw);
	  }
	glEnd();
	 
    }
*/


		




}







int rdtsc()
{
    __asm__ __volatile__("rdtsc");
}

int getRand(int min, int max)
    {
        
        srand(rdtsc());
 
        int n = max - min + 1;
        int i = rand() % n;
 
        if(i < 0)
            i = -i;
 
        return min + i;
    }

};
