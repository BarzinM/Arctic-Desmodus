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
#include <limits.h>
#include <math.h>
#include <algorithm>

#define ANSI_COLOR_RED     "\x1b[31;1m"
#define ANSI_COLOR_GREEN   "\x1b[32;1m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35;1m"
#define ANSI_COLOR_CYAN    "\x1b[36;1m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace std;

class Packet
{
private:
public:
    int packet_type;
    int address;
    int length = 0;
    char *data = new char [0];
    uint16_t checksum;

    Packet();
    ~Packet();
    void getLength();
    void checkHealth();
    uint16_t generateChecksum();
};

class UM7
{
private:
    int serial_handler;
    int error;

    int overwriteRegister ( Packet *config_packet , int start_bit,
                            int bits_length );
    int packetRequest ( Packet *config_packet );
    int registerWrite ( Packet *config_packet );
    int confirmWrite ( int requested_address );     //TODO
    int getRegister ( Packet *config_packet );
    void findBeginning();
    void setError ( int err );

public:
    int registers_32[70] = {9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 88, 91, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 111, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130};
    int registers_16[20] = {86, 87, 89, 90, 92, 93, 109, 110, 112, 113, 114, 115};
    int registers_quat[2] = {109, 110};
    int registers_eulr[2] = {112, 113};
    int registers_eulr_rates[2] = {114, 115};
    float position_x = 0, position_y = 0, position_z = 0;
    float velocity_x = 0, velocity_y = 0, velocity_z = 0;

    UM7();
    ~UM7();
    int setPosition ( float , float , float );
    int setVelocity ( float , float , float );
    float translate ( int address, bitset<8> *array_of_bytes );
    void config();
    int getFrequency();
    void echoPacket ( int iterations );
};

////////////////////////////////
///////////////////////////// //
////////////////////////// // //
// Functions Definition // // //
////////////////////////// // //
///////////////////////////// //
////////////////////////////////

int UM7::setPosition ( float X, float Y, float Z )
{
    position_x = X;
    position_y = Y;
    position_z = Z;
    return 0;
}

int UM7::setVelocity ( float Xdot, float Ydot, float Zdot )
{
    velocity_x = Xdot;
    velocity_y = Ydot;
    velocity_z = Zdot;
    return 0;
}

int getIntegerInput ( int *input )
{
    string line;
    cin >> line;
    istringstream s ( line );
    char c;

    if ( ! ( s >> *input ) )
    {
        printf ( "\n" ANSI_COLOR_RED "Input was not a number" ANSI_COLOR_RESET );
        return -1;
    }

    if ( s >> c )
    {
        printf ( "\n" ANSI_COLOR_RED "Input included non-numerical characters"
                 ANSI_COLOR_RESET );
        return -2;
    }

    return 0;
}

int GetFloat16 ( bitset<8> *array_of_bytes )
{
    int value;
    value=(short)((array_of_bytes[0].to_ulong()<<8)|array_of_bytes[1].to_ulong());
    return value;
}
// Convert the 32-bit binary into the decimal
float GetFloat32 ( bitset<8> *array_of_bytes )
{
    string Binary = array_of_bytes[0].to_string() + array_of_bytes[1].to_string() +
                    array_of_bytes[2].to_string() + array_of_bytes[3].to_string();
    bitset<32> set ( Binary );
    int HexNumber = set.to_ulong();
    bool negative  = !! ( HexNumber & 0x80000000 );
    int  exponent  = ( HexNumber & 0x7f800000 ) >> 23;
    int sign = negative ? -1 : 1;
    // Subtract 127 from the exponent
    exponent -= 127;
    // Convert the mantissa into decimal using the
    // last 23 bits
    int power = -1;
    float total = 0.0;

    for ( int i = 0; i < 23; i++ )
    {
        int c = Binary[ i + 9 ] - '0';
        total += ( float ) c * ( float ) pow ( 2.0, power );
        power--;
    }

    total += 1.0;
    float value = sign * ( float ) pow ( 2.0, exponent ) * total;
    cout << value;
    return value;
}

float UM7::translate ( int address, bitset<8> *array_of_bytes )
{
    float value;

    if ( any_of ( begin ( registers_32 ), end ( registers_32 ), [&] ( int i )
{
    return i == address;
} ) )
    {
        value = GetFloat32 ( array_of_bytes );
        return value;
    }

    if ( any_of ( begin ( registers_16 ), end ( registers_16 ), [&] ( int i )
{
    return i == address;
} ) )
    {
        bitset<8> temp[2];
        int divide_by = 1;
        temp[0] = array_of_bytes[0];
        temp[1] = array_of_bytes[1];
        value = GetFloat16 ( temp );

        if ( address == registers_quat[0] )
        {
            cout << "Quat: ";
            divide_by = 29789.09091;
            value = value / divide_by;
            printf ( "%6.2f, ", value );
            temp[0] = array_of_bytes[2];
            temp[1] = array_of_bytes[3];
            value = GetFloat16 ( temp );
            value = value / divide_by;
            printf ( "%6.2f", value );
            return 0;
        }

        if ( address == registers_quat[1] )
        {
            cout << "Quat: ";
            divide_by = 29789.09091;
            value = value / divide_by;
            printf ( "%6.2f", value );
            return 0;
        }

        if ( address == registers_eulr[0] )
        {
            cout << "Euler: ";
            divide_by = 91.02222;
            value = value / divide_by;
            printf ( "%3.2f, ", value );
            temp[0] = array_of_bytes[2];
            temp[1] = array_of_bytes[3];
            value = GetFloat16 ( temp );
            value = value / divide_by;
            printf ( "%3.2f", value );
            return 0;
        }

        if ( address ==  registers_eulr[1] )
        {
            cout << "Euler: ";
            divide_by = 91.02222;
            value = value / divide_by;
            printf ( "%3.2f", value );
            return 0;
        }

        if ( address == registers_eulr_rates[0] )
        {
            cout << "Euler Rates: ";
            divide_by = 16;
            value = value / divide_by;
            printf ( "%6.2f, ", value );
            temp[0] = array_of_bytes[2];
            temp[1] = array_of_bytes[3];
            value = GetFloat16 ( temp );
            value = value / divide_by;
            printf ( "%6.2f", value );
            return 0;
        }

        if ( address ==  registers_eulr_rates[1] )
        {
            cout << "Euler Rates: ";
            divide_by = 16;
            value = value / divide_by;
            printf ( "%6.2f", value );
            return 0;
        }

        printf ( "%6.2f, ", value );
        temp[0] = array_of_bytes[2];
        temp[1] = array_of_bytes[3];
        value = GetFloat16 ( temp );
        printf ( "%6.2f", value );
    }
}

void UM7::setError ( int err )
{
    error = err;

    if ( error != 0 )
    {
        exit ( error );
    }
}




Packet::Packet() {};

Packet::~Packet()
{
    // if ( length != 0 )
    {
        delete[] ( data );
    }
}

uint16_t Packet::generateChecksum()
{
    uint16_t computed_checksum;
    computed_checksum = 's' + 'n' + 'p';
    computed_checksum += packet_type + address;

    for ( int i = 0; i < length * 4; i++ )
    {
        computed_checksum += int ( uint8_t ( data[i] ) );
    }

    return computed_checksum;
}

void Packet::checkHealth()
{
    uint16_t computed_checksum = generateChecksum();

    if ( computed_checksum == checksum )
    {
        printf ( "\n" ANSI_COLOR_GREEN "Healthy\n" ANSI_COLOR_RESET );
    }
    else
    {
        printf ( "\n" ANSI_COLOR_RED "Corrupted" ANSI_COLOR_RESET );
    }
}

void Packet::getLength ()
{
    if ( length != 0 )
    {
        delete[] ( data );
    }

    length = 0;
    string binary = bitset<8> ( packet_type ).to_string();

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

    data = new char [length * 4];
    cout << "==========================================================================="
         << endl;

    if ( binary[7] == '1' )
    {
        cout << "******COMMUNICATION FAILED******";
    }

    cout << "Packet type: " << packet_type << "= " << binary << ": ";

    if ( binary[0] == '1' )
    {
        cout << "has data.";
    }

    if ( binary[1] == '1' )
    {
        cout << "\b and is batch";
    }
}
UM7::UM7()
{
    printf ( ANSI_COLOR_CYAN
             "######################## UM7 Communication, BarzinM #######################\n"
             ANSI_COLOR_RESET );
    /* Open File Descriptor */
    serial_handler = open ( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );

    /* Error Handling */
    if ( serial_handler < 0 )
    {
        cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (
                 errno ) << endl;
    }

    /* *** Configure Port *** */
    struct termios tty;
    //memset ( &tty, 0, sizeof tty );

    /* Error Handling */
    if ( tcgetattr ( serial_handler, &tty ) != 0 )
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
    tcflush ( serial_handler, TCIOFLUSH );

    if ( tcsetattr ( serial_handler, TCSANOW, &tty ) != 0 )
    {
        cout << "Error " << errno <<
             " from tcsetattr. Refer to http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html"
             << endl;
    }

    setError ( 0 );
}

UM7::~UM7()
{
    close ( serial_handler );
    printf ( ANSI_COLOR_CYAN
             "++++++++++++++++++++++++ Exiting IMU Communication Program ++++++++++++++++\n"
             ANSI_COLOR_RESET );
}

void UM7::findBeginning()
{
    int n = 0;
    char buffer[10];

    while ( true )
    {
        // Find 's' character
        n = read ( serial_handler, buffer, 1 );

        if ( *buffer == 's' )
        {
            // Check to see if it is followed by 'n' and 'p'
            n = read ( serial_handler, buffer, 1 );

            if ( *buffer == 'n' )
            {
                n = read ( serial_handler, buffer, 1 );

                if ( *buffer == 'p' )
                {
                    return;
                }
            }
        }
    }
}

void UM7::echoPacket ( int iter )
{
    Packet received_pkt;
    uint16_t received_checksum;
    unsigned char decimal;
    string binary;
    bitset<8> array_of_bytes[4];
    char buffer[100];
    /* *** READ *** */
    int n, reg_address;
    int buffer_value;
    /* Whole response*/
    tcflush ( serial_handler, TCIOFLUSH );

    for ( int i = 0; i < iter; i++ )
    {
        // computer_checksum = 's' + 'n' + 'p';
        findBeginning();
        n = read ( serial_handler, buffer, 1 );
        int temp = int ( uint8_t ( buffer[0] ) );
        received_pkt.packet_type = temp;
        // packet_type
        // computer_checksum += uint8_t ( buffer[0] );
        received_pkt.getLength ( );
        //char *binary_char = &binary[0];
        n = read ( serial_handler, buffer, 1 );
        received_pkt.address =  int ( uint8_t ( buffer[0] ) );
        printf ( "\nAddress is: %i = %#x", received_pkt.address,
                 received_pkt.address );
        cout << "\nData length: " << received_pkt.length << " register(s)";
        binary = bitset<8> ( received_pkt.address ).to_string(); //to binary

        for ( int i = 0; i < received_pkt.length * 4; i++ )
        {
            if ( i % 4 == 0 )
            {
                reg_address = received_pkt.address + i / 4;
                printf ( "\nReg %3i:  | ", reg_address );
            }

            n = read ( serial_handler, buffer, 1 );
            received_pkt.data[i] =  int ( uint8_t ( buffer[0] ) );
            array_of_bytes[i % 4] = bitset<8> ( buffer[0] );
            binary = bitset<8> ( buffer[0] ).to_string(); //to binary
            printf ( "%3u: ", ( uint8_t ( received_pkt.data[i] ) ) );
            cout << binary << " | ";

            if ( i % 4 == 3 )
            {
                translate ( reg_address, array_of_bytes );
            }
        }

        n = read ( serial_handler, buffer, 2 );
        received_pkt.checksum = uint16_t ( buffer[0] ) << 8 | uint8_t ( buffer[1] );
        received_pkt.checkHealth();
    }

    cout << endl;
    setError ( 0 );
}
int UM7::getFrequency()
{
    int  frequency;
    getIntegerInput ( &frequency );

    if ( frequency > 255 )
    {
        frequency = 255;
    }
    else
    {
        if ( frequency < 0 )
        {
            frequency = 0;
        }
    }

    return frequency;
}
void UM7::config()
{
    bool exitMode = 0;
    string binary;
    int input = -1;
    unsigned int address;
    int start_bit, bits_length;
    int raw_rate = 0;
    Packet config_packet;

    while ( exitMode == 0 )
    {
        cout << "\nYou are in configuration mode. Thou shall choose an option" << endl
             << endl;
        cout << "        ____________________________________________________________"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        |  1 | Set ALL RAW SENSOR data transmission rate.          |"
             << endl;
        cout << "        |  2 | Set ALL PROCESSESED SENSOR data transmission rate.  |"
             << endl;
        cout << "        |  3 | Set ALL POSE data transmission rate.                |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        |  4 | Set QUATERNION data transmission rate.              |"
             << endl;
        cout << "        |  5 | Set EULER data transmission rate.                   |"
             << endl;
        cout << "        |  6 | Set POSITION data transmission rate.                |"
             << endl;
        cout << "        |  7 | Set VELOCITY data transmission rate.                |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        |  8 | Set RAW ACCELEROMETER data transmission rate.       |"
             << endl;
        cout << "        |  9 | Set RAW GYROSCOPE data transmission rate.           |"
             << endl;
        cout << "        | 10 | Set RAW MAGNETOMETER data transmission rate.        |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        | 11 | Set PROCESSED ACCELEROMETER data transmission rate. |"
             << endl;
        cout << "        | 12 | Set PROCESSED GYROSCOPE data transmission rate.     |"
             << endl;
        cout << "        | 13 | Set PROCESSED MAGENTOMETER data transmission rate.  |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        | 14 | Set TEMPERATURE data transmission rate.             |"
             << endl;
        cout << "        | 15 | Set HEALTH monitoring rate.                         |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        | 99 | Show IMU outputs.                                   |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << "        |    |                                                     |"
             << endl;
        cout << "        |  0 |EXIT.                                                |"
             << endl;
        cout << "        |____|_____________________________________________________|"
             << endl;
        cout << endl << ">> ";
        int error = getIntegerInput ( &input );

        if ( error != 0 )
        {
            cout << "\nTry again or enter '0' to exit." << endl;
            continue;
        }

        switch ( input )
        {
        default:
            cout << "\nBad input try again" << endl;

        case 0:
            exitMode = 1;
            break;

        case 1:
            address = 2;
            start_bit = 7;
            bits_length = 8;
            break;

        case 2:
            address = 4;
            start_bit = 7;
            bits_length = 8;
            break;

        case 3:
            address = 6;
            start_bit = 31;
            bits_length = 8;
            break;

        case 4:
            address = 5;
            start_bit = 31;
            bits_length = 8;
            break;

        case 5:
            address = 5;
            start_bit = 23;
            bits_length = 8;
            break;

        case 6:
            address = 5;
            start_bit = 15;
            bits_length = 8;
            break;

        case 7:
            address = 5;
            start_bit = 7;
            bits_length = 8;
            break;

        case 8:
            address = 1;
            start_bit = 31;
            bits_length = 8;
            break;

        case 9:
            address = 1;
            start_bit = 23;
            bits_length = 8;
            break;

        case 10:
            address = 1;
            start_bit = 15;
            bits_length = 8;
            break;

        case 11:
            address = 3;
            start_bit = 31;
            bits_length = 8;
            break;

        case 12:
            address = 3;
            start_bit = 23;
            bits_length = 8;
            break;

        case 13:
            address = 3;
            start_bit = 15;
            bits_length = 8;
            break;

        case 14:
            address = 2;
            start_bit = 31;
            bits_length = 8;
            break;

        case 15:
            cout << "Coming soon" << endl;
            break;

        case 99:
            cout << "\nHow many outputs does my lord desire?" << endl << ">> ";
            echoPacket ( getFrequency() );
        }

        if ( ( input != 0 ) && ( input != 99 ) )
        {
            cout << "Enter Desired Frequency:" << endl << ">> ";
            int raw_rate = getFrequency();
            cout << "this is raw input " << raw_rate << endl;
            // Setup configuration packet
            config_packet.packet_type = ( bitset<8> ( "10000000" ).to_ulong() );
            config_packet.address = address;
            config_packet.getLength();
            start_bit = 31 - start_bit;
            config_packet.data[start_bit / 4] = raw_rate;
            cout << "data length is " << config_packet.length << endl;
            cout << "Got to line: " << __LINE__ << ", File: " << __FILE__ << endl;
            cout << "560 start: " << start_bit << endl;
            // overwrite new values in register
            error = overwriteRegister ( &config_packet ,  start_bit,
                                        bits_length );
        }
    }

    cout << "You are exiting configuration mode in 3 seconds.\n3..." << endl;
    sleep ( 1 );
    cout << "2..." << endl;
    sleep ( 1 );
    cout << "1..." << endl;
    sleep ( 1 );
    return;
}

int UM7::packetRequest ( Packet *config_packet )
{
    // Start of packet
    char cmd[7];
    uint16_t ckecksum;
    // vector<string> bytes_in_binary;
    int error = 1;
    // Set packet type
    char packet_type = ( bitset<8> ( "00000000" ).to_ulong() );
    // Construct the command
    cmd[0] = 's';
    cmd[1] = 'n';
    cmd[2] = 'p';
    cmd[3] = packet_type;
    cmd[4] = char ( config_packet->address );
    ckecksum = 's' + 'n' + 'p' + uint8_t ( cmd[3] ) + uint8_t ( cmd[4] );
    cmd[5] = uint8_t ( ckecksum >> 8 );
    cmd[6] = uint8_t ( ckecksum );
    config_packet->packet_type = 128;
    ( *config_packet ).getLength();

    while ( error != 0 )
    {
        tcflush ( serial_handler, TCIOFLUSH );
        write ( serial_handler, cmd, 7 );
        error = getRegister ( config_packet );
        cout << "GOT IT" << endl;
    }

    return 0;
}

int UM7::getRegister ( Packet *config_packet )
{
    Packet temp_packet;
    string byte;
    bool found = 0;
    uint16_t received_checksum;
    //     /* *** READ *** */
    int n = 0;
    int iter;
    //     /* Whole response*/
    char *buffer = new char [100];
    iter = 10;

    // Try this many times
    for ( int i = 0; i < iter; i++ )
    {
        // Find 's' character
        findBeginning();
        n = read ( serial_handler, buffer, 1 );
        temp_packet.packet_type = int ( uint8_t ( buffer[0] ) );
        temp_packet.getLength();
        n = read ( serial_handler, buffer, 1 );
        temp_packet.address = int ( uint8_t ( buffer[0] ) );

        if ( config_packet->address != temp_packet.address
                || config_packet->length != temp_packet.length )
        {
            read ( serial_handler, buffer, ( temp_packet.length * 4 ) + 2 );
            continue;
        }

        printf ( "\nReg %3i:  | ", config_packet->address );

        for ( int i = 0; i < ( config_packet->length * 4 ); i++ )
        {
            n = read ( serial_handler, buffer, 1 );
            config_packet->data[i] = *buffer;
            byte = bitset<8> ( int ( config_packet->data[i] ) ).to_string(); //to binary
            printf ( "%3i: ", int ( uint8_t ( buffer[0] ) ) );
            cout << byte << " | ";
        }

        n = read ( serial_handler, buffer, 2 );
        received_checksum = uint16_t ( buffer[0] ) << 8 | uint8_t ( buffer[1] );

        if ( ( *config_packet ).generateChecksum() ==
                received_checksum ) // use checkhealth funciton
        {
            printf ( "\n" ANSI_COLOR_GREEN "Healthy\n" ANSI_COLOR_RESET );

            if ( config_packet->address == temp_packet.address )
            {
                found = 1;
                delete[] ( buffer );
                return 0;
            }
        }
        else
        {
            printf ( "\n" ANSI_COLOR_RED "Corrupted" ANSI_COLOR_RESET );
        }

        if ( n < 0 )
        {
            cout << "Reading Error" << strerror ( errno ) << endl;
            delete[] ( buffer );
            return -1;
        }

        if ( n == 0 )
        {
            cout << "Nothing to read";
            delete[] ( buffer );
            return -2;
        }
    }

    if ( !found )
    {
        printf ( ANSI_COLOR_RED "Did NOT get the requested register information"
                 ANSI_COLOR_RESET );
    }

    cout << endl;
    delete[] ( buffer );
    return -3;
}

int UM7::registerWrite ( Packet *config_packet )
{
    /* *** WRITE *** */
    // Start of packet
    uint16_t computer_checksum = config_packet->generateChecksum();
    int data_length = ( config_packet->length ) * 4;
    char *cmd = new char [data_length + 7];
    // Set packet type
    char packet_type = ( bitset<8> ( "10000000" ).to_ulong() );
    //unsigned int address = 2;
    // Construct the command
    cmd[0] = 's';
    cmd[1] = 'n';
    cmd[2] = 'p';
    cmd[3] = config_packet->packet_type;
    cmd[4] = config_packet->address;

    for ( int i = 0; i < data_length; i++ )
    {
        cmd[5 + i] = config_packet->data[i];
        cout << "data is " << int ( cmd[5 + i] ) << endl;
    }

    cmd[5 + data_length] = uint8_t ( computer_checksum >> 8 );
    cmd[6 + data_length] = uint8_t ( computer_checksum );
    // for( int n_written=0;n_written<11; ){
    // n_written += write( serialCommunicationHandler, cmd+n_written, 11 );
    // }
    write ( serial_handler, cmd, data_length + 7 );
    // int error = confirmWrite ( serialCommunicationHandler, address );
    // cout << endl << "error is " << error ;
    // packetRequest( serialCommunicationHandler, address );
    return 0;
}

int UM7::overwriteRegister ( Packet *config_packet , int start_bit,
                             int bits_length )
{
    bitset<8> byte;
    bitset<32> bits_form_user;
    bitset<32> bits_from_register;
    char *data2 = new char [bits_length];
    int error = 0;

    // Save data in a temp variable
    for ( int i = 0; i < 4; i++ )
    {
        byte = bitset<8> ( config_packet->data[i] );

        for ( int j = 0; j < 8; j++ )
        {
            bits_form_user[i * 8 + j] = byte[j];
        }
    }

    cout << "user input: " << bits_form_user << endl;
    // Get register data
    error = packetRequest ( config_packet );

    for ( int i = 0; i < 4; i++ )
    {
        byte = bitset<8> ( config_packet->data[i] );

        for ( int j = 0; j < 8; j++ )
        {
            bits_from_register[i * 8 + j] = byte[j];
        }
    }

    cout << "start" << start_bit << endl;

    // Substitute configuration request parts of register data with user data
    for ( int i = start_bit; i < start_bit + bits_length; i++ )
    {
        cout << "i is: " << i << "," << bits_from_register[i] << bits_form_user[i] <<
             endl;
        bits_from_register[i] = bits_form_user[i];
    }

    cout << "substituted: " << bits_from_register << endl;

    // Convert configuration package bits to data bytes
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 8; j++ )
        {
            byte[j] = bits_from_register[i * 8 + j];
        }

        config_packet->data[i] = byte.to_ulong();
        cout << "sent byte is " << byte << endl;
    }

    // write register
    registerWrite ( config_packet );
    // confirm write
}

