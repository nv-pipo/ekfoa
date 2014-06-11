CXXFLAGS=-O3 -Wall -DEIGEN_NO_DEBUG -D__ARM_NEON__ -fPIC -march=armv7-a -mfloat-abi=hard -mfpu=neon -ftree-vectorize -mvectorize-with-neon-quad -mcpu=cortex-a9 -mtune=cortex-a9 -frounding-math

INCLUDES=-I/opt/local/include/ -I/opt/local/include/eigen3 -I/usr/include/eigen3/ -I/usr/include/ -I/opt/ros/hydro/include/ -I/usr/include/libdrm

LIB_FOLDERS=-L/opt/local/lib -L/opt/ros/hydro/lib/

LIBS=-lopencv_core -lopencv_calib3d -lopencv_contrib -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lglfw3 -lpthread -lX11 -lXi -lXrandr -lXcursor -lGL -lGLU -lXxf86vm -lboost_thread -lboost_system -lCGAL -lgmp -lmpfr

all:
	g++ $(CXXFLAGS) $(INCLUDES) $(LIB_FOLDERS) -o ~/ekfoa ./src/gui.cpp ./src/ekfoa.cpp ./src/camera.cpp ./src/feature.cpp ./src/kalman.cpp ./src/main.cpp ./src/motion_model.cpp ./src/motion_tracker_of.cpp src/opengl_utils/arcball.cpp ./src/print.cpp $(LIBS)
