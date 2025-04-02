#pragma once

#include <windows.h>
#include <iostream>
#include <vector>

using namespace std;

struct serial_config
{
	string port_name;
	int buandrate;
	int parity;
	int byte_size;
	int stop_bits;
	int read_timeout;
	int write_timeout;
};


class memobusClient
{
	public:
		memobusClient(const std::string& port_name, int buandrate, int parity, int byte_size, int stop_bits, int read_timeout, int write_timeout );
		memobusClient();
        ~memobusClient();

		void set_config(const std::string& port_name, int buandrate, int parity, int byte_size, int stop_bits, int read_timeout, int write_timeout);
		bool memobus_connect(const std::string& port_name, int buandrate, int parity, int byte_size, int stop_bits, int read_timeout, int write_timeout);

		void set_port_name(const std::string& port_name);
        void set_buandrate(int buandrate);
        void set_parity(int parity);
        void set_byte_size(int byte_size);
        void set_stop_bits(int stop_bits);
        void set_read_timeout(int read_timeout);
		void set_write_timeout(int write_timeout);

		bool memobus_connect();
		bool memobus_disconnect();
		bool is_connected();

		bool memobus_write(uint16_t id, uint16_t address, const vector<uint16_t>& data);
		vector<uint16_t> memobus_read(uint16_t id, uint16_t address, uint16_t offset);


	protected:
		uint16_t calculateCRC(const vector<uint8_t>& data);
		bool write_data(vector<uint8_t> data);
		vector<uint8_t> read_data();
        bool check_data(vector<uint8_t> data);

	private:
		serial_config config_;
		HANDLE hSerial;
		bool is_connected_ = false;
};

