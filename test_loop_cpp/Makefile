# Define the compiler
CC=g++

# Manually set Python include path
PYTHON_INCLUDES=-I/usr/include/python3.8

# Define any additional include paths
INCLUDES=$(PYTHON_INCLUDES)

# Compiler flags
CFLAGS=-Wall -pthread $(INCLUDES)

# Linker flags
LDFLAGS=-lpython3.8

# Define the source files
SOURCES=ControlLoopPython.cpp

# Define the executable file 
EXECUTABLE=ControlLoopPython

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) -o $@ $(LDFLAGS)

clean:
	rm -f $(EXECUTABLE)
