CC = cc
CFLAGS = -Wall -Wextra -I../Core/Inc -Iunity

.PHONY: test clean

test: test_nmea_parser
	./test_nmea_parser

test_nmea_parser: test_nmea_parser.c unity/unity.c ../Core/Src/nmea_parser.c
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f test_nmea_parser
