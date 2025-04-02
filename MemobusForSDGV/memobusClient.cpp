#include "memobusClient.h"


memobusClient::memobusClient(const std::string& port_name, int buandrate, int parity, int byte_size, int stop_bits, int read_timeout, int write_timeout)
{
    set_config(port_name, buandrate, parity, byte_size, stop_bits, read_timeout, write_timeout);
}

memobusClient::memobusClient()
{
	set_config("", CBR_19200, NOPARITY, 8, ONESTOPBIT,1000,1000);
}

memobusClient::~memobusClient()
{
    memobus_disconnect();
}

void memobusClient::set_config(const std::string& port_name, int buandrate, int parity, int byte_size, int stop_bits, int read_timeout, int write_timeout)
{
    config_.port_name = port_name;
    config_.buandrate = buandrate;
    config_.parity = parity;
    config_.byte_size = byte_size;
    config_.stop_bits = stop_bits;
    config_.read_timeout = read_timeout;
	config_.write_timeout = write_timeout;

}

bool memobusClient::memobus_connect(const std::string& port_name, int buandrate, int parity, int byte_size, int stop_bits, int read_timeout, int write_timeout)
{
	set_config(port_name, buandrate, parity, byte_size, stop_bits, read_timeout, write_timeout);
    return memobus_connect();
}

void memobusClient::set_port_name(const std::string& port_name){config_.port_name = port_name;}

void memobusClient::set_buandrate(int buandrate){config_.buandrate = buandrate;}

void memobusClient::set_parity(int parity){config_.parity = parity;}

void memobusClient::set_byte_size(int byte_size){config_.byte_size = byte_size;}

void memobusClient::set_stop_bits(int stop_bits){config_.stop_bits = stop_bits;}

void memobusClient::set_read_timeout(int read_timeout){ config_.read_timeout = read_timeout;}

void memobusClient::set_write_timeout(int write_timeout){ config_.write_timeout = write_timeout;}


bool memobusClient::memobus_connect()
{
	try {
		hSerial = CreateFileA(config_.port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (hSerial == INVALID_HANDLE_VALUE) {
			if (GetLastError() == ERROR_FILE_NOT_FOUND) {
				std::cerr << "Error: Handle was not attached. Reason: " << config_.port_name << " not available\n";
                return false;
			}else{
				std::cerr << "Error!\n";
				return false;
			}
		}else {
			DCB dcbSerialParams = { 0 };
			dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
			if (!GetCommState(hSerial, &dcbSerialParams)) {
				std::cerr << "Error getting state\n";
				CloseHandle(hSerial);
                return false;
			}
			else {
				dcbSerialParams.BaudRate = config_.buandrate;
				dcbSerialParams.ByteSize = config_.byte_size;
				dcbSerialParams.StopBits = config_.stop_bits;
				dcbSerialParams.Parity = config_.parity;
                

				dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;


                if (!SetCommState(hSerial, &dcbSerialParams))
                {
                    DWORD errorMessageID = GetLastError();
                    LPSTR messageBuffer = nullptr;
                    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                        NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

                    std::cerr << "设置串口状态失败。错误代码: " << errorMessageID << "\n错误信息: " << messageBuffer << "\n";

                    // Free the buffer allocated by the system.
                    LocalFree(messageBuffer);
                    CloseHandle(hSerial);
                    return false;
                }
                else {

                    // 设置读取和写入超时
                    COMMTIMEOUTS timeouts = { 0 };
                    timeouts.ReadIntervalTimeout = MAXDWORD;
                    timeouts.ReadTotalTimeoutConstant = config_.read_timeout;
                    timeouts.ReadTotalTimeoutMultiplier = 0;
                    timeouts.WriteTotalTimeoutConstant = config_.write_timeout;
                    timeouts.WriteTotalTimeoutMultiplier = 0;

                    if (!SetCommTimeouts(hSerial, &timeouts)) {
                        std::cerr << "设置串口超时失败" << std::endl;
                        CloseHandle(hSerial);
                        return false;
                    }

                    is_connected_ = true;
                    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
                    return true;
                }
			}
		}

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

bool memobusClient::memobus_disconnect()
{
    try {
        if (is_connected_) {
            CloseHandle(hSerial);
            is_connected_ = false;
            std::cout << "Disconnected from " << config_.port_name << std::endl;
            return true;
        }
        else {
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

bool memobusClient::is_connected()
{
    return is_connected_;
}

uint16_t memobusClient::calculateCRC(const vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (auto byte : data) {
        crc ^= static_cast<uint16_t>(byte);
        for (int i = 0; i < 8; ++i) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }
    return crc;
}


bool memobusClient::memobus_write(uint16_t id, uint16_t address, const vector<uint16_t>& data)
{
    std::vector<uint8_t> frame;

    // id1个字节
    frame.push_back(id & 0xFF);  
    //功能码
    frame.push_back(0x40);
    //子写功能码
    frame.push_back((0x0110 >> 8) & 0xFF);
    frame.push_back(0x0110 & 0xFF); 

    frame.push_back(0x00);
    frame.push_back(0x00);

	//地址两个字节
    frame.push_back((address >> 8) & 0xFF);
    frame.push_back(address & 0xFF); 

	//计算数据长度
    int data_length = data.size();
    if (data_length > 0 && data_length%2 ==0) {
        frame.push_back((data_length / 2 >> 8) & 0xFF);
        frame.push_back(data_length / 2  & 0xFF);

        frame.push_back((data_length >> 8) & 0xFF);
        frame.push_back(data_length & 0xFF); 

        for (auto d : data) {
           frame.push_back(d & 0xFF); 
        }
    }else{
        std::cout << "err: " << data_length << std::endl;
        return false;
    }
	
    uint16_t crc = calculateCRC(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);

    if (!this->write_data(frame))return false;

    vector<uint8_t> readData = this->read_data();
    if (readData.size() < 2) return false;

    if (check_data(readData)) {
        for (int i = 0; i < readData.size() - 2; i++)
        {
            if (readData[i] != frame[i]) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

vector<uint16_t> memobusClient::memobus_read(uint16_t id, uint16_t address, uint16_t offset)
{
    std::vector<uint8_t> frame;

    // id1个字节
    frame.push_back(id & 0xFF);
    //功能码
    frame.push_back(0x40);
    //子读功能码
    frame.push_back((0x0103 >> 8) & 0xFF);
    frame.push_back(0x0103 & 0xFF); 

    frame.push_back(0x00);
    frame.push_back(0x00);

    //地址两个字节
    frame.push_back((address >> 8) & 0xFF);
    frame.push_back(address & 0xFF); 

    frame.push_back((offset >> 8) & 0xFF);
    frame.push_back(offset & 0xFF);

    uint16_t crc = calculateCRC(frame);
    frame.push_back(crc & 0xFF);   
    frame.push_back((crc >> 8) & 0xFF); 

    if (!this->write_data(frame))return vector<uint16_t>();

    vector<uint8_t> readData = this->read_data();
    if (readData.size() < 2) return vector<uint16_t>();

    if (check_data(readData)) {
        if (readData[1] == 0x40) {
            uint16_t bytesLength = (readData[6] << 8) | readData[7];
            vector<uint16_t> reply;
            for (int i = 0; i < bytesLength; i+=2) {
                uint16_t combinedValue = static_cast<uint16_t>(readData[8 + i]<<8 | (readData[9 + i]));
                reply.push_back(combinedValue);
            }
            return reply;
        }
    }

    return vector<uint16_t>();
}

bool memobusClient::write_data(vector<uint8_t> data)
{
    // 发送
    if (!is_connected_) {
        std::cerr << "Error: Not connected to " << config_.port_name << std::endl;
        return false;
    }
    else {
        DWORD bytesWritten;
        if (!WriteFile(hSerial, data.data(), static_cast<DWORD>(data.size()), &bytesWritten, NULL)) {
            std::cerr << "写入串口失败!" << std::endl;
            return false;
        }
        // 检查是否所有数据都已发送
        if (bytesWritten != data.size()) {
            std::cerr << "未完全写入数据" << std::endl;
            return false;
        }
    }
    
    return true;

}

vector<uint8_t> memobusClient::read_data()
{
    // 接收返回的数据
    const size_t bufferSize = 256;
    uint8_t buffer[bufferSize];
    DWORD bytesRead;

	vector<uint8_t> data;
    if (ReadFile(hSerial, buffer, bufferSize, &bytesRead, NULL)) {
        for (DWORD i = 0; i < bytesRead; ++i) {
            data.push_back(buffer[i]);
        }
        return data;
    }
    else {
        std::cerr << "从串口读取数据失败!" << std::endl;
        return vector<uint8_t>();
    }  
}

bool memobusClient::check_data(vector<uint8_t> data)
{
    if (data.size() > 0) {
        vector<uint8_t> check;
        copy(data.begin(), data.end()-2, back_inserter(check));
        uint16_t crc = calculateCRC(check);
        if (crc == (data[data.size()-2] | data[data.size()-1] << 8)) {
            return true;
        }
    }
    return false;
}
