CXX = "D:/murIDE/devkit-x86/sysroots/x86_64-pokysdk-mingw32/usr/bin/i586-poky-linux/i586-poky-linux-g++.exe"
CXXFLAGS = -std=c++11 -Wall -O2 --sysroot="D:/murIDE/devkit-x86/sysroots/i586-poky-linux"

INCLUDES = -I.
LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lzmq

cxxs = $(wildcard *.cpp)
objs = $(patsubst %.cpp,%.o,$(cxxs))
ds = $(wildcard *.d)

all: Real

include $(ds)

Real: $(objs)

	$(CXX) $(CXXFLAGS) $^ $(INCLUDES) $(LIBS) -o Real.bin

 %.o: %.cpp

	$(CXX) $(CXXFLAGS) -c -MD $< $(INCLUDES) -o $@

clean:
	rm -r *.d
	rm -r *.o
	rm Real
