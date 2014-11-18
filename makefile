CPlPlElev=-std=gnu++11
extension=um7
%: %.cpp
	$(CXX) $(CPlPlElev) -o $*.$(extension) $*.cpp
#run code
run:
	sudo ./*.$(extension)