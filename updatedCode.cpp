#include </home/naslab/BRZN/SensorFusion/UM7.h>

int main()
{
    // Packet test;
    // test.packet_type = 0;
    // test.address = 85;
    UM7 imu;
    // imu.packetRequest(&test);
    // test.address=23;
    // imu.packetRequest(&test);
    imu.echoPacket ( 10 );
    imu.config();
    printf ( ANSI_COLOR_CYAN
             "######################## UM7 Communication, BarzinM #######################\n"
             ANSI_COLOR_RESET );
    cout << "Got to line: " << __LINE__ << ", File: " << __FILE__ << endl;
    // int serialCommunicationHandler = serialSetup();
    // configurationMode ( serialCommunicationHandler );
    // echoPacketsInconfigurationMode ( serialCommunicationHandler );
    // serialClose ( serialCommunicationHandler );
    return 0;
}

