# PWLink MAVLink 联通测试 —— 上位机(Linux) / MinGW 通用 Makefile
CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=c99
TARGET  = pwlink_test
SRC     = main.c mavlink2.c
OBJ     = $(SRC:.c=.o)

# 上位机(CM5/RDK X5 等 Linux ARM)：直接 make
# Windows(MinGW)：make LDFLAGS="-lws2_32"

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LDFLAGS)

%.o: %.c mavlink2.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: clean
