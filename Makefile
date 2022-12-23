##############################################################################
# @author: GaoKong
# @date:   15/09/2017
##############################################################################

TOOL_DIR =
MODULE	 = ak-flash
OPTIMIZE = -g -Os
CXX	 = g++
CC	 = gcc
OBJ_DIR	 = build_$(MODULE)

CXXFLAGS += -I/usr/local/include
CXXFLAGS += -I/usr/include

CXXFLAGS += -Isources

VPATH += sources

OBJ += $(OBJ_DIR)/uart_boot.o
OBJ += $(OBJ_DIR)/firmware.o

# CXX compiler option
CXXFLAGS	+=$(OPTIMIZE)		\
		-std=c++11		\
		-Wall			\
		-Winline		\
		-Wno-unused-result	\
		-pipe			\
		-g			\

# Library paths
LDFLAGS	+= -L/usr/local/lib
LDFLAGS	+= -L/usr/include
LDFLAGS	+= -Wl,-Map=$(OBJ_DIR)/$(MODULE).map

#Library libs
LDLIBS	+=	-lpthread		\
		-lrt			\

all: create $(OBJ_DIR)/$(MODULE)

create:
	@echo mkdir -p $(OBJ_DIR)
	@mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: %.cpp
	@echo CXX $<
	@$(CXX) -c -o $@ $< $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.c
	@echo CXX $<
	@$(CC) -c -o $@ $< $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/$(MODULE): $(OBJ)
	@echo ---------- START LINK PROJECT ----------
	@echo $(CXX) -o $@ $^ $(CXXFLAGS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)

flash:
	@$(OBJ_DIR)/$(MODULE)

install:
	@cp $(OBJ_DIR)/$(MODULE) /usr/local/bin/

clean:
	@echo rm -rf $(OBJ_DIR)
	@rm -rf $(OBJ_DIR)
