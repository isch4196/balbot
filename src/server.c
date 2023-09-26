#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include "server.h"

#define PORT "2781"  // the port users will be connecting to
#define BACKLOG 1   // how many pending connections queue will hold

/**
 * get_in_addr()
 * @sa: sockaddr
 * 
 * Get the sockaddr, IPv4 or IPv6
 *
 * Return: void ptr to sockaddr struct
 */
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

/**
 * sockfd_setup() - set up socket for clients to connect to
 * 
 * Return: int - socket file descriptor
 */
int sockfd_setup(void)
{
    int sockfd, rv;
    int yes = 1;
    struct addrinfo hints, *servinfo, *p;

    // set up socket info
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // don't care IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM; // tcp
    hints.ai_flags = AI_PASSIVE; // use my IP

    // return 1+ addrinfo structs containing an internet address that can be
    // binded or connected to
    if ((rv = getaddrinfo(NULL, PORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
	exit(1);
    }

    // loop through all the results and bind to the first we can
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
			     p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
		       sizeof(int)) == -1) {
            perror("setsockopt");
            exit(1);
        }

        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("server: bind");
            continue;
        }
        break;
    }
    freeaddrinfo(servinfo); // all done with this structure

    if (p == NULL)  {
        fprintf(stderr, "server: failed to bind\n");
        exit(1);
    }

    // market socket as passive socket (used to accept incoming connection requests)
    if (listen(sockfd, BACKLOG) == -1) {
        perror("listen");
        exit(1);
    }
    return sockfd;
}

int MAIN(void)
{
    int sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
    char ip_str[INET6_ADDRSTRLEN];
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
    int num_bytes, recv_int;
    
    sockfd = sockfd_setup();

    printf("server: waiting for client to connect...\n");
    sin_size = sizeof(their_addr);
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
    if (new_fd == -1) {
	perror("accept");
    }
    
    if (!inet_ntop(their_addr.ss_family,
		   get_in_addr((struct sockaddr *)&their_addr), ip_str, sizeof(ip_str))) {
	perror("inet_ntop");
    }
    printf("server: got connection from %s\n", ip_str);
    
#warning replace recv_int with a circular queue?
    recv_int = 0;
    while(1) {
	if ((num_bytes = recv(new_fd, &recv_int, sizeof(recv_int), 0)) == -1) {
	    perror("recv");
	    exit(1);
	} else if (!num_bytes) {
	    printf("Socket peer has shutdown\n");
	    break;
	}
	printf("server: received %d\n", ntohl(recv_int));
    }
    return 0;
}

