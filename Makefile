CC = g++
CXX_STD = -std=c++17
WARN_FLAGS = -Wall -Wextra -pedantic
SRC_DIR = src
INC_DIR = include
TARGET = main
SRC = $(SRC_DIR)/main.cc $(SRC_DIR)/parser.cc $(SRC_DIR)/treer.cc
INC = $(INC_DIR)/common.h $(INC_DIR)/parser.h $(INC_DIR)/treer.h
CXXFLAGS = $(CXX_STD) $(WARN_FLAGS) -I$(INC_DIR)

all: $(TARGET)

$(TARGET): $(SRC) $(INC)
	$(CC) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
