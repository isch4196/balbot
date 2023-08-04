all:
	${CC} $(CFLAGS) mpu6050_driver.c -o mpu6050_driver $(LDFLAGS)
clean:
	rm -f mpu6050_driver
