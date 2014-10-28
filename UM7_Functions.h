// UM7 Functions
// UM7 classes header
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>
#include <cmath>
#include <bitset>
#include <cstdint>
#include <time.h>
#include <sstream>
#include <vector>


class SerialCom;

SerialCom::SerialCom()
{
    /* Open File Descriptor */
    handler = open ( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );

    /* Error Handling */
    if ( handler < 0 )
    {
        cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (
                 errno ) << endl;
    }

    /* *** Configure Port *** */
    struct termios tty;
    //memset ( &tty, 0, sizeof tty );

    /* Error Handling */
    if ( tcgetattr ( handler, &tty ) != 0 )
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror ( errno ) << endl;
    }

    /* Set Baud Rate */
    cfsetospeed ( &tty, ( speed_t ) B115200 );
    cfsetispeed ( &tty, ( speed_t ) B115200 );
    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;      // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;     // no flow control
    tty.c_cc[VMIN] = 1;                // read doesn't block
    tty.c_cc[VTIME] = 5;                // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL;    // turn on READ & ignore ctrl lines
    /* Make raw */
    cfmakeraw ( &tty );
    /* Flush Port, then applies attributes */
    tcflush ( handler, TCIOFLUSH );

    if ( tcsetattr ( handler, TCSANOW, &tty ) != 0 )
    {
        cout << "Error " << errno <<
             " from tcsetattr. Refer to http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html"
             << endl;
    }
}

SerialCom::~SerialCom()
{
    close ( handler );
    cout << "Communication Terminated" << endl;
}



int SerialCom::echoPacketsInDeveloperMode ( int serialCommunicationHandler )
{
    int address;
    int packet_type;
    unsigned char decimal;
    int length;
    uint16_t computer_checksum;
    string binary;
    uint16_t received_checksum;
    /* *** READ *** */
    int n = 0;
    int iter = 10;
    /* Whole response*/
    char *buffer = new char [30];
    tcflush ( serialCommunicationHandler, TCIOFLUSH );

    for ( int i = 0; i < iter; i++ )
    {
        length = 0;
        computer_checksum = 's' + 'n' + 'p';
        // Find 's' character
        n = read ( serialCommunicationHandler, buffer, 1 );

        if ( *buffer == 's' )
        {
            // Check to see if it is followed by 'n' and 'p'
            n = read ( serialCommunicationHandler, buffer, 1 );

            if ( *buffer == 'n' )
            {
                n = read ( serialCommunicationHandler, buffer, 1 );

                if ( *buffer == 'p' )
                {
                    n = read ( serialCommunicationHandler, buffer, 1 );
                    packet_type = int ( uint8_t ( buffer[0] ) );
                    decimal = buffer[0];
                    // packet_type
                    computer_checksum += uint8_t ( buffer[0] );
                    binary = bitset<8> ( packet_type ).to_string(); //to binary

                    //char *binary_char = &binary[0];
                    if ( binary[0] == '1' )
                    {
                        length = 1;
                    }

                    if ( binary[1] == '1' )
                    {
                        length = 0;
                    }

                    if ( binary[2] == '1' )
                    {
                        length += 8;
                    }

                    if ( binary[3] == '1' )
                    {
                        length += 4;
                    }

                    if ( binary[4] == '1' )
                    {
                        length += 2;
                    }

                    if ( binary[5] == '1' )
                    {
                        length += 1;
                    }

                    n = read ( serialCommunicationHandler, buffer, 1 );
                    address = int ( uint8_t ( buffer[0] ) );
                    computer_checksum += uint8_t ( buffer[0] );
                    cout << endl <<
                         "===========================================================================" <<
                         endl;

                    if ( binary[7] == '1' )
                    {
                        cout << "******COMMUNICATION FAILED******";
                    }

                    cout << packet_type << ": " << binary << ": ";

                    if ( binary[0] == '1' )
                    {
                        cout << "has data";
                    }

                    if ( binary[1] == '1' )
                    {
                        cout << ", is batch";
                    }

                    printf ( "\nAddress is %i = %#x", address, address );
                    cout << " = " << bitset<8> ( address ).to_string();
                    cout << "\nData length = " << length << " registers";
                    // else{cout<<endl<<"Skipped";}
                    // if( address==86||address==112||address==97||address==85 ){continue;}//else{i=iter;}
                    binary = bitset<8> ( address ).to_string(); //to binary
                    length = length * 4;

                    for ( int i = 0; i < length; i++ )
                    {
                        if ( i % 4 == 0 )
                        {
                            printf ( "\nReg %3i:  | ", address + i / 4 );
                        }

                        n = read ( serialCommunicationHandler, buffer, 1 );
                        decimal = buffer[0];
                        computer_checksum += uint8_t ( buffer[0] );
                        binary = bitset<8> ( int ( decimal ) ).to_string(); //to binary
                        printf ( "%3i: ", int ( decimal ) );
                        cout << binary << " | ";
                    }

                    n = read ( serialCommunicationHandler, buffer, 2 );
                    received_checksum = uint16_t ( buffer[0] ) << 8 | uint8_t ( buffer[1] );

                    if ( computer_checksum == received_checksum )
                    {
                        cout << endl << computer_checksum << "   " << received_checksum << endl;
                        printf ( "\n" ANSI_COLOR_GREEN "Healthy\n" ANSI_COLOR_RESET );
                    }
                    else
                    {
                        printf ( "\n" ANSI_COLOR_RED "Corrupted" ANSI_COLOR_RESET );
                    }
                }
            }
        }

        if ( n < 0 )
        {
            cout << "Reading Error" << strerror ( errno ) << endl;
            break;
        }

        if ( n == 0 )
        {
            cout << "Nothing to read";
            break;
        }
    }

    cout << endl;
    delete[] ( buffer );
    return 0;
}