# STM32 Sensor Monitor - Windows Build
# Requires: MinGW-w64 GCC

CC = gcc
CFLAGS = -Wall -O2
LDFLAGS = -lgdi32 -lcomctl32 -mwindows

TARGET = sensor_monitor.exe
SRC = sensor_monitor.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	del /f $(TARGET) 2>nul

.PHONY: all clean
