CCC = clang++ -O0

DIR_CPLEX      = /Applications/IBM/ILOG/CPLEX_Studio_Community1262/cplex
DIR_CONCERT    = /Applications/IBM/ILOG/CPLEX_Studio_Community1262/concert

DIR_LIB_CPLEX   = $(DIR_CPLEX)/lib/x86-64_osx/static_pic
DIR_LIB_CONCERT = $(DIR_CONCERT)/lib/x86-64_osx/static_pic

DIR_INC_CPLEX   = $(DIR_CPLEX)/include
DIR_INC_CONCERT = $(DIR_CONCERT)/include

OPTIONS   = -stdlib=libstdc++ -std=c++11 -stdlib=libc++
#-framework CoreFoundation -framework IOKit
FLAGS     = -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD
DIR_INCS  = -I$(DIR_INC_CPLEX) -I$(DIR_INC_CONCERT)
DIR_LIBS  = -L$(DIR_LIB_CPLEX) -L$(DIR_LIB_CONCERT)
LIBS      = -lconcert -lilocplex -lcplex -m64 -lm -lpthread -llapack -lcblas -lblas
DIR_SRC   = src
DIR_BIN = build

all: $(DIR_BIN)/ilolpex1


$(DIR_BIN)/% : $(DIR_SRC)/%.cpp
	$(CCC) $(FLAGS) $(DIR_INCS) $(DIR_LIBS) $(LIBS) $< -o $@ $(OPTIONS)


#$(DIR_BIN)/main: $(DIR_SRC)/*.cpp
#	$(CCC) $(FLAGS) $(DIR_INCS) $(DIR_LIBS) $(DIR_SRC)/*.cpp -o $(DIR_BIN)/main $(OPTIONS)

clean:
	/bin/rm -rf $(DIR_BIN)/*

