#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ncurses.h>
#include "client.h"

// to comply with unity
#ifndef TEST
#define MAIN main
#else
#define MAIN testable_main
#endif

#define PORT "2781" // the port client will be connecting to 

#define MAXDATASIZE 100 // max number of bytes we can get at once 

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

int MAIN(int argc, char *argv[])
{
    int sockfd, numbytes;
    char buf[MAXDATASIZE];
    struct addrinfo hints, *servinfo, *p;
    int rv, ch;
    char s[INET6_ADDRSTRLEN];
    
    if (argc != 2) {
        fprintf(stderr,"usage: client hostname\n");
        exit(1);
    }

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if ((rv = getaddrinfo(argv[1], PORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and connect to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("client: connect");
            continue;
        }
        break;
    }

    if (p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        return 2;
    }

    // ncurses initialization
    initscr();
    keypad(stdscr, TRUE);
    noecho();
    
    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof s);
    printw("client: connecting to %s\n", s);

    freeaddrinfo(servinfo); // all done with this structure

    /* // receive something from the server */
    /* if ((numbytes = recv(sockfd, buf, MAXDATASIZE-1, 0)) == -1) { */
    /*     perror("recv"); */
    /*     exit(1); */
    /* } */
    /* buf[numbytes] = '\0'; */
    /* printw("client: received '%s'\n",buf); */

    // getch() isn't standardized for, so arrow key codes may vary from compiler
    // to compiler, so use ncurses to take care of that
    int num_to_send;
    while ((ch = getch()) != '#') {
	switch (ch) {
	case KEY_UP:
	case KEY_DOWN:
	case KEY_RIGHT:
	case KEY_LEFT:
	    num_to_send = htonl(ch);
	    goto send;
	default:
	    printw("random key: %d\n", ch);
	    break;
	}
    default_action:
	continue;
    send:
	send(sockfd, &num_to_send, sizeof(num_to_send), 0);
    }
    
    endwin();
    
    printf("Close socket\n");
    close(sockfd);
    return 0;

}
