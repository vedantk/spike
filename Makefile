CXX = g++
CXXFLAGS = -I./eigen/ -Wall -Wextra -DNDEBUG -O3
LDFLAGS = -lglut -lm -lstdc++ -lGL -lGLU -L/usr/lib -L/usr/lib/x86_64-linux-gnu

all: test spike

test: test.cc
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

spike: spike.cc
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)
