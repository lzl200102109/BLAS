# define the C compiler to use
CC = g++

# define any compile-time flags
CFLAGS = -Wall -g

# define any directories containing header files other than /usr/include
#
INCLUDES = 

# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
LFLAGS = 

# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname 
#   option, something like (this will link in libmylib.so and libm.so:
LIBS = -lmgl-qt -lmgl-glut -lmgl

# define source and binary directories
SRCDIR = src
BINDIR = bin

# define the C source files
SRCS := $(wildcard $(SRCDIR)/*.cpp)

# define the C object files 
#
OBJS = 

# define the executable file 
#   $(name:string1=string2)
#         For each word in 'name' replace 'string1' with 'string2'
# Below we are replacing the suffix .c of all words in the macro SRCS
# with the .o suffix
#
MAIN = $(SRCS:$(SRCDIR)/%.cpp=$(BINDIR)/%.out)

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#
.PHONY: all depend clean

all: $(MAIN)


# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
$(BINDIR)/%.out : $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(OBJS) $(LFLAGS) $(LIBS) $< -o $@

clean:
	rm -rf $(wildcard $(BINDIR)/*.out) $(wildcard $(BINDIR)/*.dSYM)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it