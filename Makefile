CXX = g++
CXXFLAGS = -I./eigen/ -Wall -Wextra -DNDEBUG -O3
LDFLAGS = -lglut -lm -lstdc++ -lGL -lGLU -L/usr/lib -L/usr/lib/x86_64-linux-gnu

spike: spike.cc
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f spike
