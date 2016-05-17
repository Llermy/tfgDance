#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h> // For exit function
#include <signal.h> // For catching signals
#include <sys/socket.h>
#include <arpa/inet.h>

#include "osc_float_receiver.hpp"

#define BUFLEN 16
#define ADDR_PATTERN_LENGTH 8
#define TYPE_TAG_LENGTH 4
#define OSC_FLOAT_LENGTH 4
#define ADDR_PATTERN_ONSET "/onset"
#define TYPE_TAG_FLOAT ",f"
#define POW_2_TO_MINUS_23 0.00000011920928955078125

OscFloatReceiver::OscFloatReceiver(int port)
{
    // Create socket and bind socket to port
    listener_d = open_listener();
    bind_to_port(listener_d, port);
}

int OscFloatReceiver::open_listener()
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(s == -1)
    {
        printf("Can't open socket\n");
        exit(-1);
    }
    return s;
}

void OscFloatReceiver::bind_to_port(int socket, int port)
{
    struct sockaddr_in host_sockaddr;
    host_sockaddr.sin_family = AF_INET;
    host_sockaddr.sin_port = (in_port_t)htons(port);
    host_sockaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    
    int reuse = 1;
    if (setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(int)) == -1)
    {
        printf("Can't set the reuse option on the socket\n");
        exit(-1);
    }
    
    int c = bind (socket, (struct sockaddr *) &host_sockaddr, sizeof(host_sockaddr));
    if (c == -1)
    {
        printf("Can't bind to socket\n");
        exit(-1);
    }
}

float OscFloatReceiver::parse_float32(char *float_string)
{
    float f;
    // DEBUG: printf("Float bits: %x %x %x %x\n", float_string[0], float_string[1], float_string[2], float_string[3]);
    
    // Parse each part of float32
    int significand =
    (float_string[1] & 0x7f)*65536 +
    ((unsigned char)float_string[2])*256 +
    ((unsigned char)float_string[3]);
    if (significand == 0)
    {
        return 0;
    }
    int sign = (float_string[0] & 0x80) >> 7;
    int exp =
    (float_string[0] & 0x7f)*2 +
    ((float_string[1] & 0x80) >> 7);
    
    f = (-sign*2 + 1)*(1 + significand*POW_2_TO_MINUS_23)*pow(2, exp - 127);
    
    /* DEBUG
     printf("Float sign: %d\n", sign);
     printf("Encoded exponent: %d\n", exp);
     printf("Encoded significand: %d\n", significand);
     printf("Decoded significand: %f\n", 1+significand*POW_2_TO_MINUS_23);
     printf("Float: %f\n", f);
     */
    return f;
}

float OscFloatReceiver::read_float()
{
    struct sockaddr_storage client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    //printf("Reading float...\n");
    char buffer[BUFLEN];
    ssize_t count = 0;
    count = recvfrom(listener_d, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
    if(count == -1)
    {
        printf("Error whith recvfrom.\n");
        exit(-1);
    } else if (count < BUFLEN)
    {
        printf("Non proper message.\n");
        return 0;
    }
    /* DEBUG
     printf("Return: %li\n", count);
     printf("Length: %lu\n", sizeof(buffer));
     printf("%c%c%c%c%c%c%c%c\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
     printf("%c%c%c%c\n", buffer[8], buffer[9], buffer[10], buffer[11]);
     printf("%c%c%c%c\n", buffer[12], buffer[13], buffer[14], buffer[15]);
     */
    
    char addr_pattern[ADDR_PATTERN_LENGTH];
    char type_tag[TYPE_TAG_LENGTH];
    char float_string[OSC_FLOAT_LENGTH];
    strncpy(addr_pattern, buffer, ADDR_PATTERN_LENGTH);
    strncpy(type_tag, buffer + ADDR_PATTERN_LENGTH, TYPE_TAG_LENGTH);
    strncpy(float_string, buffer + ADDR_PATTERN_LENGTH + TYPE_TAG_LENGTH, OSC_FLOAT_LENGTH);
    
    /* DEBUG
     printf("Address pattern: %s\n", addr_pattern);
     printf("Type tag: %s\n", type_tag);
     printf("Float32: %s\n", float_string);
     */
    
    if(strcmp(addr_pattern, ADDR_PATTERN_ONSET) || strcmp(type_tag, TYPE_TAG_FLOAT))
    {
        printf("Address pattern: %s\n", addr_pattern);
        printf("Type tag: %s\n", type_tag);
        printf("Non proper message");
        return 0;
    }
    return parse_float32(float_string);
}

/*
int main()
{
    OscFloatReceiver *osc_rec = new OscFloatReceiver(7000);
    
    while(1)
    {
        osc_rec->read_float();
    }
    
    return 0;
}*/