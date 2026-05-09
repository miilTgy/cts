CC = g++
CXX_STD = -std=c++17
WARN_FLAGS = -Wall -Wextra -pedantic
SRC_DIR = src
INC_DIR = include
TARGET = cts
EVAL_TARGET = evaluate
SAMPLE ?= samples/sample1.txt
SAMPLE_BASE = $(basename $(notdir $(SAMPLE)))
SOLUTION = result/$(SAMPLE_BASE)_solution.txt
SRC = $(SRC_DIR)/main.cc $(SRC_DIR)/parser.cc $(SRC_DIR)/partitioner.cc $(SRC_DIR)/treer.cc $(SRC_DIR)/bu.cc $(SRC_DIR)/td.cc $(SRC_DIR)/writer.cc
INC = $(INC_DIR)/common.h $(INC_DIR)/parser.h $(INC_DIR)/partitioner.h $(INC_DIR)/treer.h $(INC_DIR)/bu.h $(INC_DIR)/td.h $(INC_DIR)/writer.h
CXXFLAGS = $(CXX_STD) $(WARN_FLAGS) -I$(INC_DIR)

.PHONY: all run eval clean

all: $(TARGET)

$(TARGET): $(SRC) $(INC)
	$(CC) $(CXXFLAGS) $(SRC) -o $(TARGET)

$(EVAL_TARGET): evaluate.cpp
	$(CC) $(CXXFLAGS) evaluate.cpp -o $(EVAL_TARGET)

run: $(TARGET)
	./$(TARGET) $(SAMPLE)

eval: $(TARGET) $(EVAL_TARGET)
	./$(EVAL_TARGET) $(SAMPLE) $(SOLUTION)

clean:
	rm -f $(TARGET) $(EVAL_TARGET)
