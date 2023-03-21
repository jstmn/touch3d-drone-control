CC=gcc
CFLAGS+=-W -g -DNDEBUG -Dlinux
LIBS = -lHL -lHLU -lHDU -lHD -lrt -lstdc++ -lm

TARGET=CannedForceEffect
HDRS=
SRCS=CannedForceEffect.c conio.c
OBJS=$(SRCS:.c=.o)

.PHONY: all
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LIBS)

.PHONY: clean
clean:
	-rm -f $(OBJS) $(TARGET)

