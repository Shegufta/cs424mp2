all:
	g++ -std=c++11 -o robotest robotest.cc irobot-create.cc -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

clean:
	rm robotest
