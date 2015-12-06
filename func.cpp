	void dibuja(btVector3 nuevo,GLfloat vectorv[3]){
	btScalar m[16];
	btTransform tran;
	
	//for (int j=0;j<vectort->size();j++) {
//	std::cout<< num << "(" << vectort[num-1].getX() << "," << vectort[num-1].getY() << "," << vectort[num-1].getZ() << ")" << std::endl; 
		if(activo) add(nuevo);
		
	//(btScalar) trans.getX(),(btScalar)trans.getY(),(btScalar)trans.getZ()));
		//trans.setIdentity();
		for(int j=0;j<num;j++){
			glPushMatrix();
				tran.setIdentity();
				tran=btTransform(btQuaternion(0, 0, 0, 1), vectort[j]);
				tran.setRotation(btQuaternion(vectorv[0], vectorv[1], vectorv[2],1));
				tran.getOpenGLMatrix(m);
				glMultMatrixf((GLfloat*) m);
			//	glEnable (GL_BLEND);
				//	glLoadIdentity();
				//	glTranslatef( vectort[j].getX(), vectort[j].getY(), vectort[j].getZ());
			//		glColor3f(vectort[j].getW()/17*4,vectort[j].getW()/17*4,vectort[j].getW()/17*4);
						//glRotatef(tan(vectorv[2]/vectorv[3]),1.0,0.0,0.0);
							//glutSolidCube(1);//vectort[j].getW()/(17*8));
      							glBindTexture( GL_TEXTURE_2D, texturacubo); //bind the texture
								glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
								glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
								RenderOBJModel(&bloque);
						float f=vectort[j].getW()/(17*8);
					/*	glBegin(GL_QUADS);
							glTexCoord2f(0.0f, 0.0f);
							glVertex3f(-f, -f, f);  // Bottom Left Of The Texture and Quad
							glTexCoord2f(1.0f, 0.0f);
							glVertex3f(f, -f, f);  // Bottom Right Of The Texture and Quad
							glTexCoord2f(1.0f, 1.0f);
							glVertex3f(f, f, f);  // Top Right Of The Texture and Quad
							glTexCoord2f(0.0f, 1.0f);
							glVertex3f(-f, f, f);  // Top Left Of The Texture and Quad
						glEnd();*/ 

					 	if(vectort[j].getW()>0) vectort[j].setW(vectort[j].getW()-1.0);
					
					//glTranslatef( -vectort[j].getX(), -vectort[j].getY(), -vectort[j].getZ());
				//	glColor4f(1.0f,1.0f,1.0f,1.0f);    
			//	glDisable(GL_BLEND); 
			glPopMatrix();

		}

	}