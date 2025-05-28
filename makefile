CFLAGS=-lpthread -lncurses -g -O0
FILES=hw5

all: hw5

hw5: main.c hw5.c
        gcc $^ -o $@ $(CFLAGS)

clean:
        rm -f $(FILES) *.o
