CXX=g++
CXXFLAGS+=-W -fexceptions -O2 -DNDEBUG -Dlinux
LIBS+=-lHD -lHDU -lglut -lGL -lGLU -lrt -lncurses -lstdc++ -lm

TARGET=main
HDRS=
SRCS= \
	helper.cpp \
	mainAirSim.cpp
OBJS=$(SRCS:.cpp=.o)

.PHONY: all
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS) $(LIBS)

.PHONY: clean
clean:
	-rm -f $(OBJS) $(TARGET)
