CCC = g++ -O0

DIR_CPLEX      = /opt/ibm/ILOG/CPLEX_Studio_Community1262/cplex
DIR_CONCERT    = /opt/ibm/ILOG/CPLEX_Studio_Community1262/concert

DIR_LIB_CPLEX   = $(DIR_CPLEX)/lib/x86-64_linux/static_pic
DIR_LIB_CONCERT = $(DIR_CONCERT)/lib/x86-64_linux/static_pic

DIR_INC_CPLEX   = $(DIR_CPLEX)/include
DIR_INC_CONCERT = $(DIR_CONCERT)/include

OPTIONS   = #-std=c++11
FLAGS     = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD
DIR_INCS  = -I$(DIR_INC_CPLEX) -I$(DIR_INC_CONCERT)
DIR_LIBS  = -L$(DIR_LIB_CPLEX) -L$(DIR_LIB_CONCERT)
LIBS      = -lconcert -lilocplex -lcplex -lm -lpthread -llapack -lcblas -lblas
DIR_SRC   = src
DIR_BIN = build

all: $(DIR_BIN)/main

# method 1: (2 steps)
#$(DIR_BIN)/ilolpex1: $(DIR_BIN)/ilolpex1.o
#	$(CCC) $(FLAGS) $(DIR_INCS) $(DIR_LIBS) -o $(DIR_BIN)/ilolpex1 $(DIR_BIN)/ilolpex1.o $(LIBS)
#$(DIR_BIN)/ilolpex1.o: $(DIR_SRC)/ilolpex1.cpp#
#	$(CCC) -c $(FLAGS) $(DIR_INCS) $(DIR_SRC)/ilolpex1.cpp -o $(DIR_BIN)/ilolpex1.o

# method 1: (1 step)
#$(DIR_BIN)/ilolpex1: $(DIR_SRC)/ilolpex1.cpp
#	$(CCC) $(FLAGS) $(DIR_INCS) $(DIR_LIBS) -o $(DIR_BIN)/ilolpex1 $(DIR_SRC)/ilolpex1.cpp $(LIBS)

$(DIR_BIN)/main: $(DIR_SRC)/*.cpp
	$(CCC) $(FLAGS) $(DIR_INCS) $(DIR_LIBS) $(LIBS) -o $(DIR_BIN)/main $(DIR_SRC)/*.cpp $(OPTIONS)
clean:
	/bin/rm -rf $(DIR_BIN)/*

