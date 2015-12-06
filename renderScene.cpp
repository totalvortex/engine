void renderScene(void) {
	btScalar m[16];
	if(fisica){
		dynamicsWorld->stepSimulation(1 / 60.f, 10);
	}
	initluces();
	puntodevista();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	
	dibujaskybox();
/*
glPushMatrix();
	glColor3f(1.0,1.0,1.0);
	glBindTexture(GL_TEXTURE_2D, texturaobjeto);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	glScalef(.1,.1,.1);
	glTranslatef(17.0,0.0,-80);
	RenderOBJModel(&beetleobj);
glPopMatrix();
*/	


	
//suelo
	glPushMatrix();
	groundRigidBody->getMotionState()->getWorldTransform(t1);
	t1.getOpenGLMatrix(m);
	glMultMatrixf((GLfloat*) m);
	//glutSolidCube(.5);
	//glTranslatef(ESCALA/4,0.0,-ESCALA/2); //dibuja el plano -5 hacia abajo
	glTranslatef(0.0,1.0,0.0);
	glScalef(ESCALA, 1.0, ESCALA);
	//glRotatef(-90, 1.0, 0.0, 0.0);
	float ancho=150.0;
	
	glBindTexture(GL_TEXTURE_2D, texturasuelo);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	
	glBegin(GL_QUADS);
       glTexCoord2f(1, 0); glVertex3f(-ancho, 0.0, -ancho );
       glTexCoord2f(0, 0); glVertex3f(-ancho, 0.0, ancho );
       glTexCoord2f(0, 1); glVertex3f(ancho, 0.0, ancho );
       glTexCoord2f(1, 1); glVertex3f(ancho, 0.0, -ancho );
    glEnd();
	
	
	if(pintamuros){
		
		glPushMatrix();
		glColor3f(1.0,1.0,1.0);
		groundRigidBody->getMotionState()->getWorldTransform(t1);
		t1.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		
		glScalef(100,0.0,100.0);
		glTranslatef(0.0,1.0,0.0);
		drawCheck(200, 200, 1, 0);
	}
	glPopMatrix();
	
//Circuito
	if(pintacircuito){
	glPushMatrix();
		groundRigidBody->getMotionState()->getWorldTransform(t1);
		t1.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		
		glTranslatef(0.0,.4,0.0);
		glColor3f(1.0,1.0,1.0);
		glBindTexture( GL_TEXTURE_2D, texturatecho); //bind the texture
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		glTranslatef(0.0,0.0,0.0);
		RenderOBJModel(&objfilelow); 
		renderizaobj(&objfilelow,true);

		
		//modelacircuito(&objfilelow);
	glPopMatrix();
	}

	glPushMatrix();
		glColor3f(1.0,1.0,1.0);
		tp.setIdentity();
		personajerb->getMotionState()->getWorldTransform(tp);
		tp.getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);

		glBindTexture( GL_TEXTURE_2D, texturacabeza); //bind the texture
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		
		float a,ax;

		if(vectorvision[X]>=0){
			
			if(vectorvision[Z]>=0.0){
				a=-asin(vectorvision[Z]/2);
								 
			}
			else{
				a=acos(vectorvision[X]/2);
			
							
			}
		}
		else{
			girogradoscuerpo=180;
			if(vectorvision[Z]>=0.0){
				a=-NPI+asin(vectorvision[Z]/2);
								
			}
			else{
				a=acos(vectorvision[X]/2);
							
			}
		}
	//	girogradoscuerpo=((int)agrados(a)%90)*90;
		glTranslatef(0.0, -1.5,0.0);
		glRotatef(girogradoscuerpo,0.0,1.0,0.0);
	
		RenderOBJModel(&cuerpoobj);
		glRotatef(agrados(a)+90+vara-girogradoscuerpo,0.0,1.0,0.0);
		
		
		RenderOBJModel(&cabezaobj);
		a=0.0;
		

	glPopMatrix();

	
	//pintacol(&objfilelow);

		
//calculos circuito (prueba)
	//modelacircuito(&objfilelow);
	
		//for(int z=0;z<objfile.num_faces;z++){
		//	glPushMatrix();
		//	//printf("\n%f,%f,%f",posver[z*3][0],posver[z*3][1],posver[z*3][2]);
		//	glTranslatef(
		//	glutSolidSphere(.5,8,8);
		//	glPopMatrix();	
		//}
//muros circuito
	if(pintamuros){
	for(int j = 0; j < indice; j++) {
	glPushMatrix();
		tm[j].setIdentity();
		murorb[j]->getMotionState()->getWorldTransform(tm[j]);
		tm[j].getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		glColor3f(1.0, 1.0, 1.0);
	/*	float *no=	objfile.normals[objfile.faces[j].norm_indices[2]].ijk;	
			glBegin(GL_LINES);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(no[X],no[Y],no[Z]);
			glEnd();
		*/
		
		//anchomuro[indice],1.0f,0.5
		dibujamuro(anchomuro[j]*2,4.3f,.1f);
		//drawBox();
	glPopMatrix();
	}



	}
	
	
	for (int j = 0; j < nobolas; j++) {

		glPushMatrix();
		trans[j].setIdentity();
		fallRigidBody[j]->getMotionState()->getWorldTransform(trans[j]);
		trans[j].getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		glColor3f(1.0, 1.0, 1.0);
			glBindTexture( GL_TEXTURE_2D, texturaventana); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		//glutSolidSphere(0.5, 15, 15);
		//glRotatef(agrados(a)+90,0.0,1.0,0.0);
		RenderOBJModel(&objfile);
		glPopMatrix();

	}
	for (int j = 0; j < nocajas; j++) {
		glColor3f(1.0, 1.0, 1.0);
		glPushMatrix();
		tc[j].setIdentity();
		cuborb[j]->getMotionState()->getWorldTransform(tc[j]);
		tc[j].getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		drawBox(j);
		glPopMatrix();

	}
	for (int j = 0; j < nofichas; j++) {
		glColor3f(.3, .3, .3);
		glBindTexture( GL_TEXTURE_2D, texturadomino); //bind the texture
		glPushMatrix();
		tf[j].setIdentity();
		ficharb[j]->getMotionState()->getWorldTransform(tf[j]);
		tf[j].getOpenGLMatrix(m);
		glMultMatrixf((GLfloat*) m);
		glTranslatef(0.0,-1.0,0.0);
		RenderOBJModel(&dominoobj);
		glPopMatrix();

	}

//glDisable(GL_TEXTURE_2D);
//vehiculo 
	glColor3f(.7,.7,.7);
		glBindTexture(GL_TEXTURE_2D, texturaobjeto);
		glPushMatrix(); //chasis
			GLfloat matrix[16];
			vehicle->getChassisWorldTransform().getOpenGLMatrix(&matrix[0]);
			glMultMatrixf( matrix);
	//qwer
	/*		glScalef(1.5,0.5,4.0);
	glTranslatef(-.125,0.0,0.0);
	glColor3f(0.3, .3, .3);
			glutSolidCube(1);*/
		
		
		
	

		RenderOBJModel(&beetleobj);
	
	
		glPopMatrix();
//ruedas
	
	for (int i=0;i<vehicle->getNumWheels();i++)
	{
		glColor3f(0.8, .8, .8);
		glBindTexture( GL_TEXTURE_2D, texturarueda); //bind the texture
	glPushMatrix();
		
		GLfloat matrix[16];
		vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(&matrix[0]);
		
		glMultMatrixf(matrix);
		
		
		
	
		//glRotatef(-90,0.0,0.0,1.0);
		//glRotatef(-90,1.0,0.0,0.0);
		//glTranslatef(0.0,0.0,-.5);
		//glScalef(.5,.5,1.0);
		//glutSolidCylinder(wheelRadius,wheelWidth,12,12);
		RenderOBJModel(&ruedaobj);
	glPopMatrix();
	}

	
	glColor3f(1.0,1.0,1.0);
	//const char hola[4]={'h','o','l','a'};
	const char *hola="hola mundo";
	text(cruzx,cruzy,20,"+");

glEnable(GL_TEXTURE_2D);
	
	//glPushMatrix();
	//	modela();
//	glPopMatrix();
//	glPushMatrix();
	//	modelacircuito(&objfile);
//	glPopMatrix();
	glutSwapBuffers();
}
