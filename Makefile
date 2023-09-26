CFLAGS=-Wall

all: main.o mpu6050.o pid.o
	${CC} $(CFLAGS) $^ -o balbot $(LDFLAGS)
test: main.o mpu6050.o pid.o
	${CC} $(CFLAGS) $^ -o balbot -lpigpio -lm
main.o: src/main.c src/common.h src/mpu6050.h
	${CC} $(CFLAGS) -c $<
mpu6050.o: src/mpu6050.c src/mpu6050.h
	${CC} $(CFLAGS) -c $<
pid.o: src/PID-library/pid.c src/PID-library/pid.h
	${CC} $(CFLAGS) -c $<
s: src/server.c
	${CC} src/server.c -o server
c: src/client.c
	${CC} src/client.c -lcurses -o client
clean:
	rm -f main.o mpu6050.o pid.o balbot server client
