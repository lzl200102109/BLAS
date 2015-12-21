SYSTEM     = x86-64_osx
LIBFORMAT  = static_pic

#------------------------------------------------------------
#
# When you adapt this makefile to compile your CPLEX programs
# please copy this makefile and set CPLEXDIR and CONCERTDIR to
# the directories where CPLEX and CONCERT are installed.
#
#------------------------------------------------------------

CPLEXDIR      = /Applications/IBM/ILOG/CPLEX_Studio_Community1262/cplex
CONCERTDIR    = /Applications/IBM/ILOG/CPLEX_Studio_Community1262/concert

# ---------------------------------------------------------------------
# Compiler selection 
# ---------------------------------------------------------------------

CCC = clang++ -O0
CC  = clang -O0

# ---------------------------------------------------------------------
# Compiler options 
# ---------------------------------------------------------------------

CCOPT = -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD -stdlib=libstdc++
COPT  = -m64 -fPIC

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR)
CLNDIRS   = -L$(CPLEXLIBDIR)
CCLNFLAGS = -lconcert -lilocplex -lcplex -m64 -lm -lpthread -framework CoreFoundation -framework IOKit -stdlib=libstdc++
CLNFLAGS  = -lcplex -m64 -lm -lpthread -framework CoreFoundation -framework IOKit

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

EXINC         = include
EXDATA        = data
EXSRCC        = src/c
EXSRCCX       = src/c_x
EXSRCCPP      = src/cpp

BINDIR_C   = build/c
BINDIR_CX  = build/c_x
BINDIR_CPP = build/cpp

SRC_C = $(wildcard $(EXSRCC)/*.c)
SRC_CX = $(wildcard $(EXSRCCX)/*.c)
SRC_CPP = $(wildcard $(EXSRCCPP)/*.cpp)

C_EX   = $(SRC_C:$(EXSRCC)/%.c=$(BINDIR_C)/%)
CX_EX  = $(SRC_CX:$(EXSRCCX)/%.c=$(BINDIR_CX)/%)
CPP_EX = $(SRC_CPP:$(EXSRCCPP)/%.cpp=$(BINDIR_CPP)/%)

CFLAGS  = $(COPT)  -I$(CPLEXINCDIR)
CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) 

#------------------------------------------------------------
#  make all      : to compile the examples. 
#  make execute  : to compile and execute the examples.
#------------------------------------------------------------

all:
	make all_c
	make all_cx
	make all_cpp

execute: all
	make execute_c
	make execute_cpp

all_c: $(C_EX)
all_cx: $(CX_EX)
all_cpp: $(CPP_EX)

$(BINDIR_C)/% : $(BINDIR_C)/%.o
	$(CC) $(CFLAGS) $(CLNDIRS) $< -o $@ $(CLNFLAGS)
	
$(BINDIR_C)/%.o : $(EXSRCC)/%.c
	$(CC) -c $(CFLAGS) $< -o $@
	
$(BINDIR_CX)/% : $(BINDIR_CX)/%.o
	$(CC) $(CFLAGS) $(CLNDIRS) $< -o $@ $(CLNFLAGS)
	
$(BINDIR_CX)/%.o : $(EXSRCCX)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BINDIR_CPP)/% : $(BINDIR_CPP)/%.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) $< -o $@ $(CCLNFLAGS)

$(BINDIR_CPP)/%.o : $(EXSRCCPP)/%.cpp
	$(CCC) -c $(CCFLAGS) $< -o $@

clean :
	/bin/rm -rf *.o *~ *.class
	/bin/rm -rf $(CX_EX) 
	/bin/rm -rf *.mps *.ord *.sos *.lp *.sav *.net *.msg *.log *.clp

execute_c: $(C_EX) $(CX_EX)
	 ./indefqpex1
	 ./globalqpex1 $(EXDATA)/nonconvexqp.lp g
	 ./globalmiqpex1 $(EXDATA)/nonconvexmiqp.lp
	 ./lpex1 -r
	 ./lpex2 $(EXDATA)/example.mps p
	 ./lpex3
	 ./lpex4
	 ./lpex5
	 ./lpex6
	 ./lpex7 $(EXDATA)/afiro.mps p
	 ./lpex8
	 ./mipex1
	 ./mipex2 $(EXDATA)/mexample.mps
	 ./mipex3
	 ./mipex4 $(EXDATA)/p0033.mps l
	 ./miqpex1
	 ./netex1
	 ./netex2 $(EXDATA)/infnet.net
	 ./qcpex1
	 ./qpex1
	 ./qpex2 $(EXDATA)/qpex.lp o
	 ./steel -r
	 ./steel -c
	 ./diet -r $(EXDATA)/diet.dat
	 ./fixnet
	 ./foodmanu
	 ./populate $(EXDATA)/location.lp
	 ./tuneset $(EXDATA)/p0033.mps
	 ./bendersatsp 1 $(EXDATA)/atsp.dat
	 ./socpex1
	 ./qcpdual
	 ./adpreex1
	 ./admipex1 $(EXDATA)/mexample.mps
	 ./admipex2 $(EXDATA)/p0033.mps
	 ./admipex3 $(EXDATA)/sosex3.lp
	 ./admipex4
	 ./admipex5
	 ./admipex6 $(EXDATA)/mexample.mps
	 ./admipex7 $(EXDATA)/mexample.mps
	 ./xindefqpex1
	 ./xglobalqpex1 $(EXDATA)/nonconvexqp.lp g
	 ./xglobalmiqpex1 $(EXDATA)/nonconvexmiqp.lp
	 ./xlpex1 -r
	 ./xlpex2 $(EXDATA)/example.mps p
	 ./xlpex3
	 ./xlpex4
	 ./xlpex5
	 ./xlpex6
	 ./xlpex7 $(EXDATA)/afiro.mps p
	 ./xlpex8
	 ./xmipex1
	 ./xmipex2 $(EXDATA)/mexample.mps
	 ./xmipex3
	 ./xmipex4 $(EXDATA)/p0033.mps l
	 ./xmiqpex1
	 ./xnetex1
	 ./xnetex2 $(EXDATA)/infnet.net
	 ./xqcpex1
	 ./xqpex1
	 ./xqpex2 $(EXDATA)/qpex.lp o
	 ./xsteel -r
	 ./xsteel -c
	 ./xdiet -r $(EXDATA)/diet.dat
	 ./xfixnet
	 ./xfoodmanu
	 ./xpopulate $(EXDATA)/location.lp
	 ./xtuneset $(EXDATA)/p0033.mps
	 ./xbendersatsp 1 $(EXDATA)/atsp.dat
	 ./xsocpex1
	 ./xqcpdual
	 ./xadpreex1
	 ./xadmipex1 $(EXDATA)/mexample.mps
	 ./xadmipex2 $(EXDATA)/p0033.mps
	 ./xadmipex3 $(EXDATA)/sosex3.lp
	 ./xadmipex4
	 ./xadmipex5
	 ./xadmipex6 $(EXDATA)/mexample.mps
	 ./xadmipex7 $(EXDATA)/mexample.mps

execute_cpp: $(CPP_EX)
	 ./blend
	 ./cutstock
	 ./etsp
	 ./facility
	 ./fixcost1
	 ./foodmanufact
	 ./iloadmipex1 $(EXDATA)/mexample.mps
	 ./iloadmipex2 $(EXDATA)/p0033.mps
	 ./iloadmipex3 $(EXDATA)/sosex3.lp
	 ./iloadmipex4
	 ./iloadmipex5
	 ./iloadmipex6 $(EXDATA)/mexample.mps
	 ./ilodiet
	 ./ilogoalex1 $(EXDATA)/mexample.mps
	 ./ilogoalex2
	 ./ilogoalex3 $(EXDATA)/mexample.mps
	 ./iloindefqpex1
	 ./iloglobalqpex1 $(EXDATA)/nonconvexqp.lp g
	 ./iloglobalqpex1 $(EXDATA)/nonconvexmiqp.lp g
	 ./ilolpex1 -r
	 ./ilolpex2 $(EXDATA)/example.mps p
	 ./ilolpex3
	 ./ilolpex4
	 ./ilolpex6
	 ./ilolpex7 $(EXDATA)/afiro.mps p
	 ./ilomipex1
	 ./ilomipex2 $(EXDATA)/mexample.mps
	 ./ilomipex3
	 ./ilomipex4 $(EXDATA)/p0033.mps l
	 ./ilomiqpex1
	 ./ilopopulate $(EXDATA)/location.lp
	 ./iloqcpex1
	 ./iloqpex1
	 ./iloqpex2 $(EXDATA)/qpex.lp o
	 ./iloqpex3
	 ./ilotuneset $(EXDATA)/p0033.mps
	 ./ilobendersatsp 1 $(EXDATA)/atsp.dat
	 ./ilosocpex1
	 ./iloqcpdual
	 ./inout1
	 ./inout3
	 ./mixblend
	 ./rates
	 ./ilosteel
	 ./transport 1
	 ./warehouse
	 