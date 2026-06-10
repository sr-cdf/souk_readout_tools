CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra

devmem_profile: devmem_profile.c
	$(CC) $(CFLAGS) -o $@ $<

.PHONY: clean
clean:
	rm -f devmem_profile devmem_backing.bin devmem_profile_c.txt devmem_profile_py.txt
