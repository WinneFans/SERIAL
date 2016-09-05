#common makefile header

DIR_BIN = ./

TARGET	= com
BIN_TARGET = $(DIR_BIN)/$(TARGET)

CFLAGS = -pthread
CC = g++

OBJECTS := com.cpp

$(BIN_TARGET) : $(OBJECTS)
	$(CC) $(CFLAGS) $^ -o $@

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@
clean:
	@rm -f *.o $(BIN_TARGET)
	@rm -f com

.PHONY:clean

#common makefile foot
