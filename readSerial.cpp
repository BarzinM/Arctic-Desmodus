#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

using namespace std;

int main()
{   
    // int iteration = 1000;
    // int length = 1;
    // char * testvar = new char [5];
    // testvar[0]='0';
    // testvar[1]='1';
    // testvar[2]='2';
    // testvar[3]='3';
    // testvar[4]='4';

    // cout<<testvar<<endl;
    // cout<<testvar[2]<<endl;

    // printf(testvar,"\r");


    // char * buffer = new char [length];
    // fstream serialCom ("/dev/ttyUSB0");
    // for (int i=0; i<iteration; i++){
    //     serialCom.read(buffer, length);
    //     printf("%s\n",buffer);
    // }
    // delete[] buffer;
   
/* Open File Descriptor */
/* Open File Descriptor */
    int serialCom = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
/* Error Handling */
    if ( serialCom < 0 )
    {
        cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;
    }

/* *** Configure Port *** */
    struct termios tty;
    //memset (&tty, 0, sizeof tty);

/* Error Handling */
    if ( tcgetattr ( serialCom, &tty ) != 0 )
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    }

/* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B115200);
    cfsetispeed (&tty, (speed_t)B115200);

/* Setting other Port Stuff */
tty.c_cflag     &=  ~PARENB;        // Make 8n1
tty.c_cflag     &=  ~CSTOPB;
tty.c_cflag     &=  ~CSIZE;
tty.c_cflag     |=  CS8;

tty.c_cflag     &=  ~CRTSCTS;       // no flow control
tty.c_cc[VMIN]      =   1;                  // read doesn't block
tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

/* Make raw */
cfmakeraw(&tty);

/* Flush Port, then applies attributes */
tcflush( serialCom, TCIFLUSH );
if ( tcsetattr ( serialCom, TCSANOW, &tty ) != 0)
{
    cout << "Error " << errno << " from tcsetattr" << endl;
}

/* *** WRITE *** */
// unsigned char cmd[] = "INIT \r";
// int n_written = 0;

// do {
//     n_written += write( serialCom, &cmd[n_written], 1 );
// }
// while (cmd[n_written-1] != '\r' && n_written > 0);

/* *** READ *** */
int n = 0;

/* Whole response*/
std::string response;
char * buff = new char [256];

for (int i = 0; i<1000; i++){

    n = read( serialCom, buff, sizeof(buff) );
    cout<<buff<<endl;
    if(n<0){
        cout<<"Reading Error"<<strerror(errno)<<endl;
        break;
    }
    if(n==0){
        cout<<"Nothing to read";
        break;
    }
    if(*buff=='\r'){
        cout<<endl<<"Done"<<endl;
        break;
    }
}

close(serialCom);

//     //int serial;
//     char * buff = new char [1];
//     int rd,nbytes,tries;

//     //serial=open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
//     fcntl(serial, F_SETFL,0);
//     for (int i = 0; i<iteration; i++){
//         rd=read(serial,buff,1);
// //cout<<buff;
//         printf("%s",buff);
//         cout<<endl;
//     }
//     close(serial);




return 0;
}