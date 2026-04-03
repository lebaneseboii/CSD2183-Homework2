CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

TARGET = simplify
SRC = simplify.cpp

all:
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)