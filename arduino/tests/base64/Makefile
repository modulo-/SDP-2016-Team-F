.PHONY: clean test

test: base64_tests
	./base64_tests

base64_tests: base64.o base64_tests.o
	g++ $^ -o $@

base64.o: ../../libraries/comms/base64.cpp
	g++ -c -o $@ $<

base64_tests.o: base64_tests.cpp
	g++ -I../../libraries/comms -c -o $@ $<

clean:
	rm -f *.o
	rm -f base64_tests
