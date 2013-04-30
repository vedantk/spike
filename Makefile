CXX = g++
CXXFLAGS = -I./eigen/ -I./FreeImage/Source/ -std=c++11 -Wall -Wextra -DNDEBUG -O3
LDFLAGS = -lglut -lm -lstdc++ -lGL -lGLU -L/usr/lib -L/usr/lib/x86_64-linux-gnu

spike: spike.cc util.hh scene.hh arm.hh 
	$(CXX) $(CXXFLAGS) spike.cc -o $@ $(LDFLAGS) ./FreeImage/libfreeimage.a

.PHONY: test
test: spike
	./spike

.PHONY: test
clean:
	rm -f spike
