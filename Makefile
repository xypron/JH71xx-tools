#PROG_FILE=vf2-recover.c
PROG_NAME=vf2-recover

CFLAGS := -Wall

#if you need to enable debug info, just uncomment below
#CFLAGS += -DDEBUG

EXEF := $(PROG_NAME)

.PHONY : all clean
all : $(EXEF)

clean:
	$(RM) *.o *.d $(EXEF)
