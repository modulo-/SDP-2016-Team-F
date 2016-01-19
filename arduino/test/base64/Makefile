.PHONY: clean test

test: base64_tests
	./base64_tests

base64_tests: base64.o base64_tests.o
	gcc $^ -o $@

base64.o: ../../libraries/base64/base64.c
	gcc -c -o $@ $<

base64_tests.o: base64_tests.c
	gcc -I../../libraries/base64 -c -o $@ $<

clean:
	rm -f *.o
	rm -f base64_tests
