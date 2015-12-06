CC=g++
LIBS= -I/usr/include/bullet/ -L/usr/local/lib -lBulletDynamics -lBulletCollision -lLinearMath -lX11 -lGL -lGLU -lglut -ljpeg -lpng -lSOIL -lnoise


LIBSBOOST=-lboost_system -lboost_thread -lpthread  -lboost_date_time -lboost_serialization

all: clean main

main:

	$(CC) principal.cpp noiseutils.cpp torpedo.cpp llama.cpp lodepng.cpp grass.cpp palmera.cpp -o main $(LIBS) $(LIBSBOOST) -fpermissive -g
	
server:
	$(CC) server.cpp -o server $(LIBSBOOST)

client:
	$(CC) clienteprueba.cpp -o client $(LIBSBOOST)
clean:
	rm -f *.o main

example:


	$(CC) softbody.cpp -o soft $(LIBS) -lirrlicht
