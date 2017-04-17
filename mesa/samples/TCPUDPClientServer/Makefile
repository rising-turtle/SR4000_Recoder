# Following calls are provided:
#
# make                             -> builds all executables for the target
# make clean                       -> cleans and removes /target directory


#---------------- Variables --------------------

ifdef HOST
  DST=host
  CPPFLAGS = -fPIC -MMD -MF $(@:%.o=%.d)
  LDFLAGS = -fPIC

else
  DST=target
  CC  = gcc
  CXX = g++

  LINK.o = $(CXX) $(TARGET_ARCH)

  INCLUDE   = ./ /usr/include 
  CPPFLAGS  += $(INCLUDE:%=-I%) $(DEFINES:%=-D%) -MMD -MF $(@:%.o=%.d)
endif

OUT       =  TCPUDPClient

SRC_APP= \
 TCPUDPClient.cpp

OBJ_APP := $(SRC_APP:%.cpp=./$(DST)/%.o)
DEP_APP := $(SRC_APP:%.cpp=./$(DST)/%.d)

$(DST)/%.o: %.cpp
	@if [ ! -d $(DST) ]; then mkdir $(DST); fi
	$(COMPILE.cpp) $(OUTPUT_OPTION) $<

SRC := $(SRC_APP)
OBJ := $(OBJ_APP)
DEP := $(DEP_APP)

#---------------- Dependency Rules --------------------

ifneq ($(strip $(DEP)),)
  -include $(DEP)
endif


#---------------- Main Rule --------------------

.DEFAULT_GOAL := all

#---------------- Rules --------------------

all: $(OUT:%=$(DST)/%)

$(DST)/$(OUT): $(OBJ_APP)

clean:
	$(RM) -r host target
#	$(RM) $(OUT) *.o *.d *.gdb rc.local host

.PHONY : debug clean links deploy

#---------------- debug and tests --------------------

debug:
	@echo .VARIABLES: '$(.VARIABLES)'
	@echo .FEATURES: '$(.FEATURES)'
	@echo .SUFFIXES: '$(.SUFFIXES)'
	@echo COMPILE.c $(COMPILE.c)
	@echo COMPILE.cpp $(COMPILE.cpp)
	@echo LINK.o $(LINK.o)

	@echo CXX $(CXX)
	@echo CXXFLAGS $(CXXFLAGS)

	@echo CC $(CC)
	@echo CFLAGS $(CFLAGS)

	@echo CPPFLAGS $(CPPFLAGS)
	@echo TARGET_ARCH $(TARGET_ARCH)

	@echo LDFLAGS $(LDFLAGS)

	@echo CURDIR $(CURDIR)

	@echo AR $(AR)
	@echo ARFLAGS $(ARFLAGS)

	@echo SRC_APP $(SRC_APP)
	@echo OBJ_APP $(OBJ_APP)
	@echo DEP_APP $(DEP_APP)
	@echo OUT $(OUT:%=$(DST)/%)

