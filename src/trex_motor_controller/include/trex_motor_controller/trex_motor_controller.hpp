#pragma once

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>          //O_RDWR
#include "trex_status.hpp"
#include <stdexcept>
#include "trex_command.hpp"

namespace TRex {

  /*
    * Pointer to write function used for i2c communication.
    * Also see http://linux.die.net/man/2/write
    */
  ssize_t (* _linux_write)(int, const void *, size_t) = write;
  
  /*
    * Pointer to read function used for i2c communication.
    * Also see http://linux.die.net/man/2/read
    */
  ssize_t (* _linux_read)(int, void *, size_t) = read;

  /*
    * Pointer to open function used for i2c communication.
    * Also see https://linux.die.net/man/2/open
    */
  int (* _linux_open)(const char *pathname, int flags, ...) = open;

  /*
    * Pointer to close function used for i2c communication.
    * Also see https://linux.die.net/man/2/open
    */
  int (* _linux_close)(int fd) = close;


  class TRexMotorController {

    public:
      TRexMotorController(std::string device = "/dev/i2c-1", uint8_t address = 0x07) {
        this->device = device;
        this->address = address;
      }

      bool open(void) {
        mutex.lock();
        fileDescriptor = _linux_open(device.c_str(), O_RDWR);

        int result = -1;
        if (fileDescriptor >= 0) {
          result = ioctl(fileDescriptor, I2C_SLAVE, address);
        }
        mutex.unlock();

        return (fileDescriptor >= 0 && result >= 0);
      }

      Status status(void) {
        char buffer[STATUS_PACKET_SIZE] = { 0 };
        
        mutex.lock();
        int result = _linux_read(fileDescriptor, buffer, STATUS_PACKET_SIZE);
        mutex.unlock();

        if (result < 0) throw std::runtime_error("Failed to read TRex status");
        if (buffer[0] != STATUS_START_BYTE) throw std::runtime_error("Failed to read TRex status - wrong startbyte");

        return Status{
          (ErrorFlags)(buffer[1]),                      // Error flags
          (double)((buffer[3] << 8) + buffer[2])/100,   // Battery
          (double)((buffer[5] << 8) + buffer[4]),       // L Motor Current
          (double)((buffer[7] << 8) + buffer[6]),       // R Motor Current
          (OperationMode)(buffer[8]),                   // Mode
        };
      }

      void move(int16_t leftSpeed, int16_t rightSpeed) {
        currentCommand.leftMotorSpeed = leftSpeed;
        currentCommand.rightMotorSpeed = rightSpeed;
        currentCommand.leftMotorBrake = 0;
        currentCommand.rightMotorBrake = 0;
        command(currentCommand);
      }

      void stop(void) {
        move(0, 0);
      }

      bool close(void) {
        mutex.lock();
        int result = _linux_close(fileDescriptor);
        fileDescriptor = -1;
        mutex.unlock();
        return (result == 0);
      }

    private:
      void command(Command command) {
        char buffer[STATUS_PACKET_SIZE] = { 
          COMMAND_START_BYTE,
          (char)(command.leftMotorSpeed & 255),
          (char)((command.leftMotorSpeed >> 8) & 255),
          command.leftMotorBrake,
          (char)(command.rightMotorSpeed & 255),
          (char)((command.rightMotorSpeed >> 8) & 255),
          command.rightMotorBrake,
          (char)((uint16_t)command.batteryThreshold & 255),
          (char)(((uint16_t)command.batteryThreshold >> 8) & 255)
        };
        
        mutex.lock();
        int result = _linux_write(fileDescriptor, buffer, COMMAND_PACKET_SIZE);
        mutex.unlock();

        if (result < 0) throw std::runtime_error("Failed to write TRex command");
      }

    private:
      std::string device = "/dev/i2c-1";
      uint8_t address = 0x07;
      int fileDescriptor = -1;

      const static int STATUS_PACKET_SIZE = 9;
      const static int STATUS_START_BYTE = 0x0f;

      const static int COMMAND_PACKET_SIZE = 9;
      const static int COMMAND_START_BYTE = 0x0f;
      std::mutex mutex;

      Command currentCommand;
  };

}
