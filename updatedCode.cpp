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

#define ANSI_COLOR_RED     "\x1b[31;1m"
#define ANSI_COLOR_GREEN   "\x1b[32;1m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35;1m"
#define ANSI_COLOR_CYAN    "\x1b[36;1m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace std;

int serialSetup()
{
    /* Open File Descriptor */
    int serialCommunicationHandler = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
    /* Error Handling */
    if ( serialCommunicationHandler < 0 )
    {
        cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;
    }

    /* *** Configure Port *** */
    struct termios tty;
    //memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( serialCommunicationHandler, &tty ) != 0 )
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B115200);
    cfsetispeed (&tty, (speed_t)B115200);

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
    cfmakeraw(&tty);
    /* Flush Port, then applies attributes */
    tcflush( serialCommunicationHandler, TCIOFLUSH );
    if ( tcsetattr ( serialCommunicationHandler, TCSANOW, &tty ) != 0)
    {
        cout << "Error " << errno << " from tcsetattr. Refer to http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html" << endl;
    }
    return serialCommunicationHandler;
}
//  showRegister(string &path)
// {
//     vector<string> bytes_in_binary;
//     string single_byte;
//     // serialWrite()
//     for (int i = 0; i < 4; i++)
//     {
//         ///get single bytes
//         bytes_in_binary.push_back(single_byte);
//     }
//     return bytes_in_binary;
// }
int readRegister(int serialCommunicationHandler, int requested_address,vector<string> * bytes_in_binary)
{
    
    string single_byte;
    int address;
    int packet_type;
    bool found = 0;
    unsigned char decimal;
    int length;
    uint16_t computer_checksum;
    string binary;
    uint16_t received_checksum;
    /* *** READ *** */
    int n = 0;
    int iter;
    /* Whole response*/
    char *buffer = new char [100];
    iter = 10;
    
    for (int i = 0; i < iter; i++)
    {
        length = 0;
        computer_checksum = 's' + 'n' + 'p';

        // Find 's' character
        n = read( serialCommunicationHandler, buffer, 1);
        if (*buffer == 's')
        {

            // Check to see if it is followed by 'n' and 'p'
            n = read( serialCommunicationHandler, buffer, 1);
            if (*buffer == 'n')
            {
                n = read( serialCommunicationHandler, buffer, 1);
                if (*buffer == 'p')
                {
                    n = read( serialCommunicationHandler, buffer, 1);
                    packet_type = int(uint8_t(buffer[0]));
                    decimal = buffer[0];
                    // packet_type
                    computer_checksum += uint8_t(buffer[0]);
                    binary = bitset<8>(packet_type).to_string(); //to binary
                    //char *binary_char = &binary[0];
if (binary[0] == '1')
                    {
                        length = 1;
                    }
                    if (binary[1] == '1')
                    {
                        length = 0;
                    }
                    if (binary[2] == '1')
                    {
                        length += 8;
                    }
                    if (binary[3] == '1')
                    {
                        length += 4;
                    }
                    if (binary[4] == '1')
                    {
                        length += 2;
                    }
                    if (binary[5] == '1')
                    {
                        length += 1;
                    }

                    n = read( serialCommunicationHandler, buffer, 1);
                    address = int(uint8_t(buffer[0]));
                    cout << address << ", ";
                    computer_checksum += uint8_t(buffer[0]);
                    length=length*4;
                    // else{cout<<endl<<"Skipped";}
                    if (requested_address != address){
                    	read( serialCommunicationHandler, buffer, length+2);
                    	continue;
                    }
                    // if(address==86||address==112||address==97||address==85){continue;}//else{i=iter;}
                    
                    binary = bitset<8>(address).to_string(); //to binary
                    printf("\nReg %3i:  | ", address );
                    for (int i = 0; i < length; i++)
                    {
                        n = read( serialCommunicationHandler, buffer, 1);
                        decimal = buffer[0];
                        computer_checksum += uint8_t(buffer[0]);
                        single_byte = bitset<8>(int(decimal)).to_string(); //to binary
                        printf("%3i: ", int(decimal));
                        
                        (*bytes_in_binary).push_back(single_byte);
                        cout << (*bytes_in_binary).at(i) << " | ";
                    }
                    n = read( serialCommunicationHandler, buffer, 2);
                    received_checksum = uint16_t(buffer[0]) << 8 | uint8_t(buffer[1]);
                    if (computer_checksum == received_checksum)
                    {
                        printf("\n" ANSI_COLOR_GREEN "Healthy" ANSI_COLOR_RESET);
                        if (requested_address == address)
                        {
                            found = 1;
                            return 0;
                        }

                    }
                    else
                    {
                        cout << endl << computer_checksum << "   " << received_checksum << endl;
                        printf("\n" ANSI_COLOR_RED "Corrupted" ANSI_COLOR_RESET);
                    }
                }
            }
        }
        if (n < 0)
        {
            cout << "Reading Error" << strerror(errno) << endl;
            return -1;
        }
        if (n == 0)
        {
            cout << "Nothing to read";
            return -2;
        }
    }
    if (!found)printf( ANSI_COLOR_RED "Did NOT get the requested register information" ANSI_COLOR_RESET);
    cout << endl;
    delete[](buffer);

    return -3;
}

int readPacketsInDeveloperMode(int serialCommunicationHandler)
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
    tcflush( serialCommunicationHandler, TCIOFLUSH );
    for (int i = 0; i < iter; i++)
    {
        length = 0;
        computer_checksum = 's' + 'n' + 'p';

        // Find 's' character
        n = read( serialCommunicationHandler, buffer, 1);
        if (*buffer == 's')
        {
            // Check to see if it is followed by 'n' and 'p'
            n = read( serialCommunicationHandler, buffer, 1);
            if (*buffer == 'n')
            {
                n = read( serialCommunicationHandler, buffer, 1);
                if (*buffer == 'p')
                {
                    n = read( serialCommunicationHandler, buffer, 1);
                    packet_type = int(uint8_t(buffer[0]));
                    decimal = buffer[0];
                    // packet_type
                    computer_checksum += uint8_t(buffer[0]);
                    binary = bitset<8>(packet_type).to_string(); //to binary
                    //char *binary_char = &binary[0];
                    if (binary[0] == '1')
                    {
                        length = 1;
                    }
                    if (binary[1] == '1')
                    {
                        length = 0;
                    }
                    if (binary[2] == '1')
                    {
                        length += 8;
                    }
                    if (binary[3] == '1')
                    {
                        length += 4;
                    }
                    if (binary[4] == '1')
                    {
                        length += 2;
                    }
                    if (binary[5] == '1')
                    {
                        length += 1;
                    }

                    n = read( serialCommunicationHandler, buffer, 1);
                    address = int(uint8_t(buffer[0]));
                    computer_checksum += uint8_t(buffer[0]);

                    cout << endl << "===========================================================================" << endl;
                    if (binary[7] == '1')
                    {
                        cout << "******COMMUNICATION FAILED******";
                    }
                    cout << packet_type << ": " << binary << ": ";
                    if (binary[0] == '1')
                    {
                        cout << "has data";
                    }
                    if (binary[1] == '1')
                    {
                        cout << ", is batch";
                    }
                    printf("\nAddress is %i = %#x", address, address);
                    cout << " = " << bitset<8>(address).to_string();
                    cout << "\nData length = " << length << " registers";


                    // else{cout<<endl<<"Skipped";}

                    // if(address==86||address==112||address==97||address==85){continue;}//else{i=iter;}
                    binary = bitset<8>(address).to_string(); //to binary
                    length = length * 4;
                    for (int i = 0; i < length; i++)
                    {
                        if (i % 4 == 0)
                        {
                            printf("\nReg %3i:  | ", address + i / 4);
                        }
                        n = read( serialCommunicationHandler, buffer, 1);
                        decimal = buffer[0];
                        computer_checksum += uint8_t(buffer[0]);
                        binary = bitset<8>(int(decimal)).to_string(); //to binary
                        printf("%3i: ", int(decimal));
                        cout << binary << " | ";
                    }
                    n = read( serialCommunicationHandler, buffer, 2);
                    received_checksum = uint16_t(buffer[0]) << 8 | uint8_t(buffer[1]);
                    if (computer_checksum == received_checksum)
                    {
                        cout << endl << computer_checksum << "   " << received_checksum << endl;
                        printf("\n" ANSI_COLOR_GREEN "Healthy" ANSI_COLOR_RESET);
                    }
                    else
                    {
                        printf("\n" ANSI_COLOR_RED "Corrupted" ANSI_COLOR_RESET);
                    }
                }
            }
        }
        if (n < 0)
        {
            cout << "Reading Error" << strerror(errno) << endl;
            break;
        }
        if (n == 0)
        {
            cout << "Nothing to read";
            break;
        }
    }
    cout << endl;
    delete[](buffer);

    return 0;
}

int registerWrite(int serialCommunicationHandler)
{
    /* *** WRITE *** */
    // Start of packet
    char cmd[11];
    uint16_t computer_checksum;

    // Set packet type
    char packet_type = (bitset<8>("10000000").to_ulong());
    unsigned int address = 2;
    // Construct the command
    cmd[0] = 's';
    cmd[1] = 'n';
    cmd[2] = 'p';
    computer_checksum = 's' + 'n' + 'p';
    cmd[3] = packet_type;
    computer_checksum += uint8_t(cmd[3]);
    cmd[4] = char(address);
    computer_checksum += uint8_t(cmd[4]);
    cmd[5] = (bitset<8>("01100000").to_ulong());
    computer_checksum += uint8_t(cmd[5]);
    cmd[6] = (bitset<8>("00000000").to_ulong());
    computer_checksum += uint8_t(cmd[6]);
    cmd[7] = (bitset<8>("00000000").to_ulong());
    computer_checksum += uint8_t(cmd[7]);
    cmd[8] = (bitset<8>("00000000").to_ulong());
    computer_checksum += uint8_t(cmd[8]);
    cmd[9] = uint8_t(computer_checksum >> 8);
    cmd[10] = uint8_t(computer_checksum);

    // for(int n_written=0;n_written<11;){

    // n_written += write( serialCommunicationHandler, cmd+n_written, 11 );
    // }

    write( serialCommunicationHandler, cmd, 30 );

    return 0;
}

int packetRequest(int serialCommunicationHandler, int requested_address)
{
    // Start of packet
    char cmd[11];
    uint16_t computer_checksum;
    vector<string> bytes_in_binary;
    bool error;

    // Set packet type
    char packet_type = (bitset<8>("00000000").to_ulong());
    // Construct the command
    cmd[0] = 's';
    cmd[1] = 'n';
    cmd[2] = 'p';
    computer_checksum = 's' + 'n' + 'p';
    cmd[3] = packet_type;
    computer_checksum += uint8_t(cmd[3]);
    cmd[4] = char(requested_address);
    computer_checksum += uint8_t(cmd[4]);
    cmd[5] = uint8_t(computer_checksum >> 8);
    cmd[6] = uint8_t(computer_checksum);
    while(true)
    {
    	tcflush( serialCommunicationHandler, TCIOFLUSH );
        write( serialCommunicationHandler, cmd, 7 );

        error = readRegister(serialCommunicationHandler, requested_address, &bytes_in_binary);
        if(error==0)break;
    }
    cout<<endl<<bytes_in_binary.at(0);
    cout<<endl<<bytes_in_binary.at(1);
    cout<<endl<<bytes_in_binary.at(2);
    cout<<endl<<bytes_in_binary.at(3)<<endl;
    return 0;
}

void serialClose(int serialCommunicationHandler)
{
    close(serialCommunicationHandler);
}

int getIntegerInput(int *input)
{
    string line;
    cin >> line;
    istringstream s(line);
    if (!(s >> *input))
    {
        printf("\n" ANSI_COLOR_RED "Input was not a number" ANSI_COLOR_RESET);
        return -1;
    }
    char c;
    if (s >> c)
    {
        printf("\n" ANSI_COLOR_RED "Input included non-numerical characters" ANSI_COLOR_RESET);
        return -2;
    }
    return 0;
}

vector<string> showRegister(string &path)
{
    // vector<string> bytes_in_binary;
    // string single_byte;
    // // serialWrite()
    // for (int i = 0; i < 4; i++)
    // {
    //     ///get single bytes
    //     bytes_in_binary.push_back(single_byte);
    // }
    // return bytes_in_binary;
}

int developerMode(int serialCommunicationHandler)
{
    bool exitMode = 0;
    string temp;
    int input = -1;
    unsigned int address;
    int raw_rate = 0;
    while (exitMode == 0)
    {
        cout << "\nYou are in developer mode. Choose an option" << endl;
        cout << "\t1. Set raw data transmittion rate." << endl;
        cout << "\t2. Set processed data transmittion rate." << endl;
        cout << "\t3. Set raw data transmittion rate." << endl;
        cout << "\t0. EXIT." << endl << ">> ";
        int error = getIntegerInput(&input);
        if (error != 0)
        {
            cout << "\nBad input try again" << endl;
            continue;
        }
        switch (input)
        {
        default:
            cout << "\nBad input try again" << endl;
            break;
        case 0:
            exitMode = 1;
            break;
        case 1:
            cout << "Enter desired raw data rate (0 - 255)[Hz]: " << endl << ">> ";
            error = getIntegerInput(&raw_rate);
            if (raw_rate > 255)
            {
                raw_rate = 255;
            }
            else
            {
                if (raw_rate < 0)
                {
                    raw_rate = 1;
                }
            }
            address = 2;
            cout << endl << "Transmition rate will be set to " << raw_rate << " Hz" << endl;
            packetRequest(serialCommunicationHandler, address);
            // serialWrite(serialCommunicationHandler,address,raw_rate)

            // case 2:

            // case 3:
        }
        sleep(1);
        // exitMode=1;
    }
    cout << "You are exiting developer mode in 3 seconds.\n3..." << endl;
    sleep(1);
    cout << "2..." << endl;
    sleep(1);
    cout << "1..." << endl;
    sleep(1);
    return 0;
}

int main()
{
    printf( ANSI_COLOR_CYAN "######################## UM7 Communication, BarzinM #######################" ANSI_COLOR_RESET );
    cout << endl;
    int serialCommunicationHandler = serialSetup();

    developerMode(serialCommunicationHandler);
    // packetRequest(serialCommunicationHandler, 4);

    registerWrite(serialCommunicationHandler);

    readPacketsInDeveloperMode(serialCommunicationHandler);

    serialClose(serialCommunicationHandler);

    return 0;
}