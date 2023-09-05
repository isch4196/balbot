all: main.o mpu6050.o
	${CC} $(CFLAGS) main.o mpu6050.o -o balbot $(LDFLAGS)
test: main.o mpu6050.o
	${CC} main.o mpu6050.o -o balbot -lpigpio -lm
main.o: main.c
	${CC} -c main.c
mpu6050.o: mpu6050.c mpu6050.h
	${CC} -c mpu6050.c
clean:
	rm -f main.o mpu6050.o balbot
