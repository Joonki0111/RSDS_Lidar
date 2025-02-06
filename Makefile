DEBUG ?= 0

CFLAGS = -Wall

ifeq ($(DEBUG), 0)
    CFLAGS    += -O3
else ifeq ($(DEBUG), 1)
	CFLAGS    += -g
endif

CC = gcc
DEF_CFLAGS += -DLINUX -D_GNU_SOURCE

CFLAGS += $(DEF_CFLAGS)

include /opt/ipg/carmaker/linux64-11.1/include/MakeDefs.linux64 #HJK_250206
LD_LIBS =		$(CAR_LIB) \
			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB)

OBJS = rsds-client.o sUDP.o RSDS_LidarRSI_Model.o 

ifeq ($(OSTYPE), msys)
EXT =	.exe
LIBS =	-lws2_32
endif

rsds-client$(EXT):	$(OBJS)
	$(CC) -o $@ $(CFLAGS) $(OBJS) $(LD_LIBS) $(LD_LIBS_OS) $(LIBS)

clean:
	rm -f *.o rsds-client
