gcc simple_test.c -I../include -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__ -lnative -lrtdm -L/usr/xenomai/lib -lxenomai -lpthread -lrt -Wall -pipe  -o simple_test
