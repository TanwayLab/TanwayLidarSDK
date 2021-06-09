CC=g++ -std=c++11

SRCS=demo/Demo_UseSDK.cpp\
	src/NetworkReader.cpp\
	src/PackageCache.cpp\
	src/PcapReader.cpp\
	src/TWException.cpp

OBJS=$(SRCS:.cpp=.o)


EXEC=run_demo

start:$(OBJS)
	
	$(CC) -o $(EXEC) $(OBJS) -lpthread

.cpp.o:

	$(CC) -o $@ -c $< -D__linux__

clean:
	-rm -rf $(OBJS) run_demo
