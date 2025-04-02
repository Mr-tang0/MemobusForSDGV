#include "memobusClient.h"

int main(int argc, char *argv[])
{
    memobusClient client;
    client.set_config("COM2", 9600,EVENPARITY, 8, ONESTOPBIT, 100,100);

    if (client.memobus_connect())
    {
        std::cout << "Connected" << std::endl;
        vector<uint16_t> reply = client.memobus_read(0x02, 0x0383, 0x01);
        for (int i = 0; i < reply.size(); i++)
        {
            printf("%04X ", reply[i]);
        }
    }
    else
    {
        std::cout << "Failed to connect" << std::endl;
    }

    return 0;
}