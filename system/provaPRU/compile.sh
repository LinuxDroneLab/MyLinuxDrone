g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I/usr/local/include/  -I/usr/include/glib-2.0/  -I/usr/include/gio-unix-2.0  -I/lib/arm-linux-gnueabihf/glib-2.0/include  -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -fPIC -MMD -MP  -MF"./provaPRU.d"  -MT"./provaPRU.o" -o "./provaPRU.o" "./provaPRU.c"

g++  -L/usr/local/lib  -o "provaPRU"  ./provaPRU.o -lpthread -lpthread_nonshared -lgio-2.0 -lgobject-2.0 -lglib-2.0 -lboost_system -lboost_filesystem -lboost_log -lboost_log_setup -lboost_thread -lboost_timer -lprussdrv -lprussdrvd


