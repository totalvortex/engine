#include <iostream>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include <GL/freeglut.h>

#include "llama.cpp"
#define NPI 3.14159265358979323846

#ifndef EXTRUCTURASVERTICES

typedef float vec4_t[4];

	struct obj_vertex_t
	{
 	 vec4_t xyzw;
	};
	
#endif

 class torpedo{
 	public:
	btVector4 vectort[500];
	bool activo;
	int num;
//	int rot[1000];
	std::vector<llama> temp;
	int nrb;

	
	std::vector<llama>::iterator it;
	

   


	torpedo(){
		num=0;
		activo=true;
		bzero(vectort,sizeof(btVector4)*500);
		
		//bzero(tf,sizeof(float)*100);
//		bzero(rot,sizeof(int)*1000);
	}

	torpedo(btScalar x,btScalar y,btScalar z){
		//std::cout << "nueva explosion" << std::endl;
		vectort[num]=( btVector4(x,y,z,(btScalar) 50.0f));
		num++;
		
	}
	torpedo(btVector3 v4){
		//std::cout << "nueva explosion" << std::endl;
		vectort[num]=( btVector4(v4.getX(),v4.getY(),v4.getZ(),(btScalar)19*4));
		num++;
	}
	 void explota(int numrb,btVector3 v4,bool init,obj_vertex_t *vertices,int num_verts){
		//std::cout << "nueva explosion" << std::endl;
		if(init){
				  activo=true;
		          num=0;
		          nrb=numrb;
	    }
	    for(int k=0;k<num_verts;k++){
			vectort[num]=( btVector4(v4.getX()+( vertices[k].xyzw[0]),v4.getY()+( vertices[k].xyzw[1]),v4.getZ()+( vertices[k].xyzw[2]),(btScalar)(rand()%70) ));
			num++;
			//std::cout << ".";
		}
	}
	void add(btVector3 v4){
		vectort[num]=( btVector4(v4.getX(),v4.getY(),v4.getZ(),(btScalar)19*4));
		num++;
		//std::cout << ".,";
	}
	void adde(btVector3 v4){
		vectort[num]=( btVector4(v4.getX(),v4.getY(),v4.getZ(),(btScalar)19*5));
		num++;
		//std::cout << "*,";
		activo=true;
	}
public:
	void desactiva(){
		activo=false;
	}
	bool activ(){
		return activo;
	}
	btScalar getx(int j){
		return vectort[j].getX();
	}
	btScalar gety(int j){
		return vectort[j].getY();
	}
	btScalar getz(int j){
		return vectort[j].getZ();
	}
	btScalar getw(int j){
		return vectort[j].getW();
	}

	void avanza(btRigidBody *rb,int j){
				
		btTransform vat;
		vat.setIdentity();
		
		rb->getMotionState()->getWorldTransform(vat);
		
			if(vectort[j].getW()>0){
				vectort[j].setW(vectort[j].getW()-1.0);
			}
			
			
		
	}


    
	





bool dibujaexplosion(int j,GLfloat eyePos[3],GLfloat vectorv[3],GLuint texturaex[19]){
    btScalar m[16];
    btTransform tran;
  	bool quedan=false;
        if(activo){
			
		   temp.clear();
		   int cont=0;
			for(int j=0;j<num;j++){//ordena las llamas segun el punto de vista
			
        		float longitud = sqrt((eyePos[0]-vectort[j].getX())*(eyePos[0]-vectort[j].getX())+(eyePos[1]-vectort[j].getY())*(eyePos[1]-vectort[j].getY())+(eyePos[2]-vectort[j].getZ())*(eyePos[2]-vectort[j].getZ()));
				//inserta(vectort[j],longitud);
        	//	llama flame(vectort[j],longitud);
        		if(vectort[j].getW()>0) vectort[j].setW(vectort[j].getW()-6.0);

        		it=temp.begin();
        		if(temp.size()==0) 	temp.insert(it,  llama(vectort[j],longitud));
        		else for (it=temp.begin(); it<temp.end(); it++){
        				if(longitud>it->getlong()){
        					temp.insert(it,  llama(vectort[j],longitud));
        					cont++;
        					break;
        				}
        		}
        	
       
        		
        	}




		

		
        for(int j=0;j<temp.size();j++){
      // for (it=temp.begin(); it<temp.end(); it++){
         glColor4f(1.0,1.0,1.0,1.0);
        	
            glPushMatrix();
        		glTranslatef(temp[j].getX(),temp[j].getY(),temp[j].getZ());

            	if(vectorv[2]<0){
              		  float ix=(temp[j].getX()-eyePos[0]);///longtemp[j]ud;
             	 	  float iy=(temp[j].getY()-eyePos[1]);///longtemp[j]ud;
				      float iz=(temp[j].getZ()-eyePos[2]);///longtemp[j]ud;

			

				 	  glRotatef(agrados(atan(iy/ix)),0.0,0.0,1.0);
				 	  glRotatef(180+agrados(atan(ix/iz)),0.0,1.0,0.0);

				      int n=rand()%5;
				      glRotatef(3-n,1.0,0.0,0.0);

				} else {
					  float ix=(temp[j].getX()-eyePos[0]);///longtemp[j]ud;
             	 	  float iy=(temp[j].getY()-eyePos[1]);///longtemp[j]ud;
				      float iz=(eyePos[2]-temp[j].getZ());///longtemp[j]ud;

 				 glRotatef(180+agrados(atan(iy/ix)),0.0,0.0,1.0);
				 glRotatef(agrados(atan(ix/iz)),0.0,1.0,0.0);
				}

			   float f=temp[j].getW()/(10);

			   int n=(int)temp[j].getW()/4;
		if(n>0){
			quedan=true;
			      glBindTexture( GL_TEXTURE_2D, texturaex[20-n]); //bind the texture
                          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
                          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			   glColor4f(1.0,1.0,1.0,1.0);
                           
                glBegin(GL_QUADS);
					glTexCoord2f(1.0f, 0.0f);
					glVertex3f(-f, -f, - f);  // Bottom Right Of The Texture and Quad
					glTexCoord2f(1.0f, 1.0f);
					glVertex3f(- f,  f, - f);  // Top Right Of The Texture and Quad
					glTexCoord2f(0.0f, 1.0f);
					glVertex3f( f,  f, - f);  // Top Left Of The Texture and Quad
					glTexCoord2f(0.0f, 0.0f);
					glVertex3f(f, - f, - f);  // Bottom Left Of The Texture and Quad
				glEnd();
		}
              

             
            glPopMatrix();
	
			}

        }
        if(!quedan){
        	 activo=false;

        }
        return quedan;
    }


void dibuja(btVector3 nuevo,int j,GLfloat eyePos[3],GLfloat vectorv[3],GLuint texturaex[19]){
    btScalar m[16];
    btTransform tran;
    
    //for (int j=0;j<vectort->size();j++) {
//  std::cout<< num << "(" << vectort[num-1].getX() << "," << vectort[num-1].getY() << "," << vectort[num-1].getZ() << ")" << std::endl; 
        if(activo){
			add(nuevo);
         //add(nuevo);
        
    //(btScalar) trans.getX(),(btScalar)trans.getY(),(btScalar)trans.getZ()));
        //trans.setIdentity();
        for(int j=num-1;j>=0;j--){
        
        	 glColor4f(1.0,1.0,1.0,1.0);
        	
            glPushMatrix();
       
   				glTranslatef(vectort[j].getX(),vectort[j].getY(),vectort[j].getZ());

               // float longitud = sqrt((eyePos[0]-vectort[j].getX())*(eyePos[0]-vectort[j].getX())+(eyePos[1]-vectort[j].getY())*(eyePos[1]-vectort[j].getY())+(eyePos[2]-vectort[j].getZ())*(eyePos[2]-vectort[j].getZ()));
				if(vectorv[2]<0){
          //      if(longitud !=0.0){
              		  float ix=(vectort[j].getX()-eyePos[0]);///longitud;
             	 	  float iy=(vectort[j].getY()-eyePos[1]);///longitud;
				      float iz=(vectort[j].getZ()-eyePos[2]);///longitud;

				   

				

				 	  glRotatef(agrados(atan(iy/ix)),0.0,0.0,1.0);
			
				 glRotatef(180+agrados(atan(ix/iz)),0.0,1.0,0.0);

				 int n=rand()%5;
				 glRotatef(3-n,1.0,0.0,0.0);


				} else {
					  float ix=(vectort[j].getX()-eyePos[0]);///longitud;
             	 	  float iy=(vectort[j].getY()-eyePos[1]);///longitud;
				      float iz=(eyePos[2]-vectort[j].getZ());///longitud;

 				 glRotatef(180+agrados(atan(iy/ix)),0.0,0.0,1.0);
			//	else     glRotatef(agrados(atan(ix/iy)),0.0,0.0,1.0);
				 glRotatef(agrados(atan(ix/iz)),0.0,1.0,0.0);

				
				}

			   float f=vectort[j].getW()/(19*3);

			   int n=(int)vectort[j].getW()/4;
			   // 
			    if(n>=0 && n<20){
			      glBindTexture( GL_TEXTURE_2D, texturaex[0]); //bind the texture
                          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
                          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			   glColor4f(1.0,1.0,1.0,(float)n/19);
                           
                glBegin(GL_QUADS);
					glTexCoord2f(1.0f, 0.0f);
					glVertex3f(-f, -f, - f);  // Bottom Right Of The Texture and Quad
					glTexCoord2f(1.0f, 1.0f);
					glVertex3f(- f,  f, - f);  // Top Right Of The Texture and Quad
					glTexCoord2f(0.0f, 1.0f);
					glVertex3f( f,  f, - f);  // Top Left Of The Texture and Quad
					glTexCoord2f(0.0f, 0.0f);
					glVertex3f(f, - f, - f);  // Bottom Left Of The Texture and Quad
				glEnd();

                 
               

                  if(vectort[j].getW()>0) vectort[j].setW(vectort[j].getW()-1.0);

             
            glPopMatrix();
			}
 		
 
			}

        }

    }
	

	inline double agrados(double radianes) {
    return radianes*(180.0/NPI);
}
 };