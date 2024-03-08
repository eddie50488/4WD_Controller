#ifndef OMNI_DRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define OMNI_DRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <boost/type_index.hpp>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = true)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    }

    if (print_output)
    {
      // std::cout << "Type of msg_to_send: " << boost::typeindex::type_id_with_cvr<decltype(msg_to_send)>().pretty_name() << std::endl;
      // std::cout << "Type of msg_to_send: " << typeid(msg_to_send).name() << std::endl;
      // std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
      std::cout << "Sent: " << msg_to_send << std::endl;
    }

    return response;
    // return "";
  }

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void set_motor_values(int val_1, int val_2, int val_3, int val_4)
  {
    std::stringstream ss;
    ss << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << "\n";
    send_msg(ss.str());
  }


private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // OMNI_DRIVE_ARDUINO_ARDUINO_COMMS_HPP