SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
BIN_DIR = bin

CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic -I$(INC_DIR)
LDFLAGS =

SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRC_FILES))
TARGETS   = $(patsubst $(SRC_DIR)/%.cpp, $(BIN_DIR)/%, $(SRC_FILES))

all: $(TARGETS)

$(BIN_DIR)/%: $(BUILD_DIR)/%.o | $(BIN_DIR)
	$(CXX) $< -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR) $(BIN_DIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

.SECONDARY: $(OBJ_FILES)
.PHONY: all clean
