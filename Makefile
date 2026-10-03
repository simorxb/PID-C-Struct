CC = gcc
CFLAGS = -Wall -Wextra -O2
LDLIBS = -lm

ifeq ($(OS),Windows_NT)
EXE = .exe
else
EXE =
endif

.PHONY: all clean

all: Test_PID$(EXE) Test_PID_anti_windup_comparison$(EXE)

Test_PID$(EXE): Test_PID.c pid.c object.c pid.h object.h
	$(CC) $(CFLAGS) -o $@ Test_PID.c pid.c object.c $(LDLIBS)

Test_PID_anti_windup_comparison$(EXE): Test_PID_anti_windup_comparison.c pid.c object.c pid.h object.h
	$(CC) $(CFLAGS) -o $@ Test_PID_anti_windup_comparison.c pid.c object.c $(LDLIBS)

clean:
ifeq ($(OS),Windows_NT)
	-cmd /c del /Q Test_PID.exe Test_PID_anti_windup_comparison.exe
else
	rm -f Test_PID Test_PID_anti_windup_comparison
endif
