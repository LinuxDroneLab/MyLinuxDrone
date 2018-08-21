//============================================================================
// Name        : MyDrone.cpp
// Author      : Andrea Lambruschini
// Version     :
// Copyright   : hi!
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <agents/MyPIDCntrllr.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <cfenv>
using namespace std;

#define DAEMON_NAME "mydrone"

int main() {
#pragma STDC FENV_ACCESS ON
    std::fesetround(FE_TONEAREST);
    openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);

    syslog(LOG_INFO, "Entering Daemon");
    pid_t pid, sid;

   //Fork the Parent Process
    pid = fork();

    if (pid < 0) { exit(EXIT_FAILURE); }

    //We got a good pid, Close the Parent Process
    if (pid > 0) { exit(EXIT_SUCCESS); }

    //Change File Mask
    umask(0);

    //Create a new Signature Id for our child
    sid = setsid();
    if (sid < 0) { exit(EXIT_FAILURE); }

    //Change Directory
    //If we cant find the directory we exit with failure.
    // if ((chdir("/")) < 0) { exit(EXIT_FAILURE); }

    //Close Standard File Descriptors
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);

    syslog(LOG_INFO, "mydrone Daemon started");

	MyPIDCntrllr pidControlledAgent;
	pidControlledAgent.initialize();
    syslog(LOG_INFO, "mydrone MyPIDCntrllr created");

    while(1) {
        pidControlledAgent.pulse();
    }

    syslog(LOG_INFO, "mydrone Daemon bye bye");

	closelog();
	return 0;
}
