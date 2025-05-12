# Makefile for compiling a simple C++ project

# Variables
CXX = g++
CXXFLAGS = -Wall -O2
TARGET = my_program
SOURCES = main.cpp other_file.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Default target to build the executable
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

# Rule to compile .cpp to .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

# Clean up generated files
clean:
	del *.o $(TARGET)
