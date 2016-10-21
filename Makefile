all:
	g++ -std=c++11 -o robotest robotest.cc irobot-create.cc -pthread -L/opt/vc/lib -lserial -lmmal -lmmal_core -lmmal_util 

clean:
	rm robotest
